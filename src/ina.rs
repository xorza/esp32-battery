//! INA228 battery sensor — I²C @ 400 kHz on the board's I2C0 bus.
//!
//! The device is abstracted behind `InaDevice` so the thread loop runs
//! unchanged under the `ina-fake` feature. The fake still constructs
//! the I²C bus driver (so pin/mux/clock conflicts surface on the bench)
//! but never reads the chip — `read()` returns zeros.

use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use esp32_battery_logic::data::{Ina228Reading, SensorData};
use esp32_battery_logic::error_log::{Event, InaError};

use crate::board::I2cPins;
use crate::clock::EventRecorder;

const SAMPLES_PER_UPDATE: u32 = 10;
const SAMPLE_INTERVAL: Duration = Duration::from_millis(100);

#[derive(Default)]
struct ReadingAccum {
    voltage: f64,
    current: f64,
    power: f64,
}

impl ReadingAccum {
    fn add(&mut self, r: &Ina228Reading) {
        self.voltage += r.voltage as f64;
        self.current += r.current as f64;
        self.power += r.power as f64;
    }

    fn average(&self, n: u32) -> Ina228Reading {
        let n = n as f64;
        Ina228Reading {
            voltage: (self.voltage / n) as f32,
            current: (self.current / n) as f32,
            power: (self.power / n) as f32,
        }
    }
}

/// Read all three INA registers. Real impl returns the kind of the
/// *first* register that failed so the caller can record it; partial
/// reads are discarded because mixing fresh + stale fields would silently
/// bias the average.
trait InaDevice {
    fn read(&mut self) -> Result<Ina228Reading, InaError>;
}

// --- Real device ------------------------------------------------------------

#[cfg(not(feature = "ina-fake"))]
mod real {
    use std::thread;
    use std::time::Duration;

    use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
    use esp_idf_hal::units::Hertz;

    use esp32_battery_logic::data::Ina228Reading;
    use esp32_battery_logic::error_log::InaError;

    use super::InaDevice;
    use crate::board::I2cPins;

    const I2C_SPEED_HZ: u32 = 400_000;
    const BATTERY_INA_ADDR: u8 = 0x40;
    const SHUNT_RESISTANCE_OHM: f32 = 0.002;
    const MAX_CURRENT_A: f32 = 15.0;

    /// Oscillator/ADC settling window the INA228 needs after a soft reset.
    /// The datasheet asks for 300 us and `Ina228::reset` deliberately does
    /// not sleep on its own, so calibrating immediately would write
    /// SHUNT_CAL while the device is still coming up.
    const RESET_SETTLE: Duration = Duration::from_millis(1);

    type I2cDev = I2cDriver<'static>;

    pub struct Ina {
        ina: ina228::Ina228<I2cDev>,
    }

    impl Ina {
        /// Claims the I²C peripheral and pins, probes + calibrates the
        /// INA228. Panics on any init failure — the INA is soldered on,
        /// so a failure here is a hardware fault, not a runtime condition.
        pub fn new(pins: I2cPins) -> Self {
            let config = I2cConfig::new().baudrate(Hertz(I2C_SPEED_HZ));
            let dev = I2cDriver::new(pins.i2c, pins.sda, pins.scl, &config).expect("I2C init");
            // `Ina228::new` reads CONFIG, so this is also the presence probe.
            let mut ina = ina228::Ina228::new(dev, BATTERY_INA_ADDR)
                .expect("INA228 probe (sensor missing or wired wrong?)");
            ina.reset().expect("INA228 reset");
            thread::sleep(RESET_SETTLE);
            ina.calibrate(MAX_CURRENT_A, SHUNT_RESISTANCE_OHM)
                .expect("INA228 calibrate");
            Self { ina }
        }
    }

    impl InaDevice for Ina {
        fn read(&mut self) -> Result<Ina228Reading, InaError> {
            let voltage = self
                .ina
                .bus_voltage()
                .map_err(|_| InaError::BusVoltageRead)?;
            let current = self.ina.current().map_err(|_| InaError::CurrentRead)?;
            let power = self.ina.power().map_err(|_| InaError::PowerRead)?;
            Ok(Ina228Reading {
                voltage,
                current,
                power,
            })
        }
    }
}

// --- Fake device ------------------------------------------------------------

#[cfg(feature = "ina-fake")]
mod fake {
    use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
    use esp_idf_hal::units::Hertz;

    use esp32_battery_logic::data::Ina228Reading;
    use esp32_battery_logic::error_log::InaError;

    use super::InaDevice;
    use crate::board::I2cPins;

    const I2C_SPEED_HZ: u32 = 400_000;

    pub struct Ina {
        // Real I²C driver, constructed but never read. Held so the
        // peripheral and its GPIOs are genuinely configured and claimed
        // for the program lifetime — pin/mux/clock conflicts surface on
        // the bench just as they would in a real build.
        _i2c: I2cDriver<'static>,
    }

    impl Ina {
        pub fn new(pins: I2cPins) -> Self {
            let config = I2cConfig::new().baudrate(Hertz(I2C_SPEED_HZ));
            let i2c = I2cDriver::new(pins.i2c, pins.sda, pins.scl, &config).expect("I2C init");
            Self { _i2c: i2c }
        }
    }

    impl InaDevice for Ina {
        fn read(&mut self) -> Result<Ina228Reading, InaError> {
            Ok(Ina228Reading {
                voltage: 0.0,
                current: 0.0,
                power: 0.0,
            })
        }
    }
}

// --- Shared thread loop -----------------------------------------------------

fn run<D: InaDevice>(mut device: D, sensor_data: Arc<Mutex<SensorData>>, recorder: EventRecorder) {
    loop {
        let mut bat_acc = ReadingAccum::default();
        let mut count: u32 = 0;

        while count < SAMPLES_PER_UPDATE {
            thread::sleep(SAMPLE_INTERVAL);

            match device.read() {
                Ok(r) => {
                    bat_acc.add(&r);
                    count += 1;
                }
                Err(kind) => {
                    // Drop the failed sample, log it. Stale-reading
                    // detection in the supervisor is what catches a sensor
                    // that's truly dead.
                    recorder.record(Event::Ina(kind));
                }
            }
        }

        sensor_data
            .lock()
            .unwrap()
            .update_battery(bat_acc.average(SAMPLES_PER_UPDATE));
    }
}

// --- Public entry point -----------------------------------------------------

#[cfg(not(feature = "ina-fake"))]
fn make_device(pins: I2cPins) -> real::Ina {
    real::Ina::new(pins)
}

#[cfg(feature = "ina-fake")]
fn make_device(pins: I2cPins) -> fake::Ina {
    log::info!("INA: fake mode — claiming I²C but not driving it");
    fake::Ina::new(pins)
}

pub fn start(pins: I2cPins, sensor_data: Arc<Mutex<SensorData>>, recorder: EventRecorder) {
    thread::Builder::new()
        .name("ina".into())
        .stack_size(4096)
        .spawn(move || run(make_device(pins), sensor_data, recorder))
        .unwrap();
}
