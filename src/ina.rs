use std::sync::Arc;
use std::thread;
use std::time::Duration;

use esp_idf_hal::i2c::{I2cBusDriver, I2cDriver, config::BusConfig, config::DeviceConfig};

use esp32_battery_logic::data::Ina228Reading;

use crate::app_state::Shared;
use crate::board::I2cPins;

const I2C_SPEED_HZ: u32 = 400_000;
const BATTERY_INA_ADDR: u8 = 0x40;

const SHUNT_RESISTANCE_OHM: f32 = 0.002;
const MAX_CURRENT_A: f32 = 15.0;
const SAMPLES_PER_UPDATE: u32 = 10;
const SAMPLE_INTERVAL: Duration = Duration::from_millis(100);

#[cfg(feature = "ina-fake")]
const FAKE_READING: Ina228Reading = Ina228Reading {
    voltage: 0.0,
    current: 0.0,
    power: 0.0,
};

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

fn retry<T, E>(mut f: impl FnMut() -> Result<T, E>) -> Option<T> {
    for _ in 0..3 {
        if let Ok(v) = f() {
            return Some(v);
        }
    }
    None
}

type I2cDev = I2cDriver<'static, &'static I2cBusDriver<'static>>;

fn init_ina(dev: I2cDev, addr: u8) -> Option<ina228::Ina228<I2cDev>> {
    let mut ina = ina228::Ina228::new(dev, addr);
    if ina.reset().is_ok() && ina.calibrate(MAX_CURRENT_A, SHUNT_RESISTANCE_OHM).is_ok() {
        Some(ina)
    } else {
        log::warn!("INA228 at 0x{:02x} not found", addr);
        None
    }
}

fn read_ina(ina: &mut ina228::Ina228<I2cDev>) -> Option<Ina228Reading> {
    let voltage = retry(|| ina.bus_voltage())?;
    let current = retry(|| ina.current())?;
    let power = retry(|| ina.power())?;
    Some(Ina228Reading {
        voltage,
        current,
        power,
    })
}

pub fn start(pins: I2cPins, shared: Arc<Shared>) {
    thread::Builder::new()
        .name("ina".into())
        .stack_size(4096)
        .spawn(move || {
            // Bus is leaked for `'static` — it lives for the whole process lifetime.
            let i2c_bus: &'static I2cBusDriver<'static> = Box::leak(Box::new(
                I2cBusDriver::new(pins.i2c, pins.sda, pins.scl, &BusConfig::new()).unwrap(),
            ));
            let dev_config = DeviceConfig::new().scl_speed_hz(I2C_SPEED_HZ);
            let ina = I2cDriver::new(i2c_bus, BATTERY_INA_ADDR, &dev_config)
                .ok()
                .and_then(|dev| init_ina(dev, BATTERY_INA_ADDR));

            #[cfg(feature = "ina-fake")]
            let mut battery_ina = ina;
            #[cfg(not(feature = "ina-fake"))]
            let mut battery_ina = ina.expect(
                "battery INA228 did not initialize — enable `ina-fake` to run without sensor",
            );

            loop {
                let mut bat_acc = ReadingAccum::default();
                let mut count: u32 = 0;

                while count < SAMPLES_PER_UPDATE {
                    thread::sleep(SAMPLE_INTERVAL);

                    if let Some(bat_r) = read_battery(&mut battery_ina) {
                        bat_acc.add(&bat_r);
                        count += 1;
                    }
                }

                shared
                    .sensor_data
                    .lock()
                    .unwrap()
                    .update_battery(bat_acc.average(SAMPLES_PER_UPDATE));
            }
        })
        .unwrap();
}

#[cfg(feature = "ina-fake")]
fn read_battery(ina: &mut Option<ina228::Ina228<I2cDev>>) -> Option<Ina228Reading> {
    match ina {
        Some(ina) => read_ina(ina),
        None => Some(FAKE_READING),
    }
}

#[cfg(not(feature = "ina-fake"))]
fn read_battery(ina: &mut ina228::Ina228<I2cDev>) -> Option<Ina228Reading> {
    read_ina(ina)
}
