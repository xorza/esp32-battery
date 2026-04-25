//! INA228 battery sensor — I²C @ 400 kHz on the board's I2C0 bus.
//!
//! The device is abstracted behind `InaDevice` so the thread loop runs
//! unchanged under the `ina-fake` feature (which substitutes a canned
//! in-memory device for the I²C INA228 — useful when developing without
//! the sensor wired up).

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
    use esp_idf_hal::i2c::{I2cBusDriver, I2cDriver, config::BusConfig, config::DeviceConfig};

    use esp32_battery_logic::data::Ina228Reading;
    use esp32_battery_logic::error_log::InaError;

    use super::InaDevice;
    use crate::board::I2cPins;

    const I2C_SPEED_HZ: u32 = 400_000;
    const BATTERY_INA_ADDR: u8 = 0x40;
    const SHUNT_RESISTANCE_OHM: f32 = 0.002;
    const MAX_CURRENT_A: f32 = 15.0;

    type I2cDev = I2cDriver<'static, &'static I2cBusDriver<'static>>;

    pub struct RealIna {
        ina: ina228::Ina228<I2cDev>,
    }

    impl RealIna {
        /// Claims the I²C peripheral and pins, probes + calibrates the
        /// INA228. Panics on any init failure — the INA is soldered on,
        /// so a failure here is a hardware fault, not a runtime condition.
        pub fn new(pins: I2cPins) -> Self {
            // Bus is leaked for `'static` — it lives for the whole process lifetime.
            let i2c_bus: &'static I2cBusDriver<'static> = Box::leak(Box::new(
                I2cBusDriver::new(pins.i2c, pins.sda, pins.scl, &BusConfig::new())
                    .expect("I2C bus init"),
            ));
            let dev_config = DeviceConfig::new().scl_speed_hz(I2C_SPEED_HZ);
            let dev =
                I2cDriver::new(i2c_bus, BATTERY_INA_ADDR, &dev_config).expect("I2C device init");
            let mut ina = ina228::Ina228::new(dev, BATTERY_INA_ADDR);
            ina.reset()
                .expect("INA228 reset (sensor missing or wired wrong?)");
            ina.calibrate(MAX_CURRENT_A, SHUNT_RESISTANCE_OHM)
                .expect("INA228 calibrate");
            Self { ina }
        }
    }

    impl InaDevice for RealIna {
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
    use esp32_battery_logic::data::Ina228Reading;
    use esp32_battery_logic::error_log::InaError;

    use super::InaDevice;

    pub struct FakeIna;

    impl InaDevice for FakeIna {
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
fn make_device(pins: I2cPins) -> real::RealIna {
    real::RealIna::new(pins)
}

#[cfg(feature = "ina-fake")]
fn make_device(pins: I2cPins) -> fake::FakeIna {
    // Burn the peripherals through black_box so I2cPins fields aren't
    // flagged dead — we still claim them at boot and just don't drive
    // the bus.
    let I2cPins { i2c, sda, scl } = pins;
    std::hint::black_box((i2c, sda, scl));
    log::info!("INA: fake mode — no I²C, in-memory device");
    fake::FakeIna
}

pub fn start(pins: I2cPins, sensor_data: Arc<Mutex<SensorData>>, recorder: EventRecorder) {
    thread::Builder::new()
        .name("ina".into())
        .stack_size(4096)
        .spawn(move || run(make_device(pins), sensor_data, recorder))
        .unwrap();
}
