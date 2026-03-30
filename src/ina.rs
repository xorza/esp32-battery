use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use esp_idf_hal::i2c::{I2cBusDriver, I2cDriver, config::DeviceConfig};

const I2C_SPEED_HZ: u32 = 400_000;

use esp32_battery_logic::data::{Ina228Reading, Platform, SensorData};

const SHUNT_RESISTANCE_OHM: f32 = 0.002;
const MAX_CURRENT_A: f32 = 15.0;
const SAMPLES_PER_UPDATE: u32 = 10;
const SAMPLE_INTERVAL: Duration = Duration::from_millis(100);
const FAKE_READING: Ina228Reading = Ina228Reading {
    voltage: 0.0,
    current: 0.0,
    power: 0.0,
    charge: 0.0,
};

#[derive(Default)]
struct ReadingAccum {
    voltage: f64,
    current: f64,
    power: f64,
    last_charge: f64,
}

impl ReadingAccum {
    fn add(&mut self, r: &Ina228Reading) {
        self.voltage += r.voltage as f64;
        self.current += r.current as f64;
        self.power += r.power as f64;
        self.last_charge = r.charge;
    }

    fn average(&self, n: u32) -> Ina228Reading {
        let n = n as f64;
        Ina228Reading {
            voltage: (self.voltage / n) as f32,
            current: (self.current / n) as f32,
            power: (self.power / n) as f32,
            charge: self.last_charge,
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
        log::warn!("INA228 at 0x{:02x} not found, using fake data", addr);
        None
    }
}

fn read_ina(ina: &mut ina228::Ina228<I2cDev>) -> Option<Ina228Reading> {
    let voltage = retry(|| ina.bus_voltage())?;
    let current = retry(|| ina.current())?;
    let power = retry(|| ina.power())?;
    let charge = retry(|| ina.charge())?;
    Some(Ina228Reading {
        voltage,
        current,
        power,
        charge: charge / 3600.0,
    })
}

pub fn start_measurement_thread<P: Platform + Send + 'static>(
    i2c_bus: &'static I2cBusDriver<'static>,
    sensor_data: Arc<Mutex<SensorData<P>>>,
) {
    thread::Builder::new()
        .stack_size(4096)
        .spawn(move || {
            let dev_config = DeviceConfig::new().scl_speed_hz(I2C_SPEED_HZ);

            let mut battery_ina = I2cDriver::new(i2c_bus, 0x40, &dev_config)
                .ok()
                .and_then(|dev| init_ina(dev, 0x40));
            let mut ps_ina = I2cDriver::new(i2c_bus, 0x41, &dev_config)
                .ok()
                .and_then(|dev| init_ina(dev, 0x41));

            let mut max_charge = f64::MIN;
            let mut min_charge = f64::MAX;

            loop {
                let mut bat_acc = ReadingAccum::default();
                let mut ps_acc = ReadingAccum::default();
                let mut count: u32 = 0;
                let mut read_total: u32 = 0;
                let mut read_failures: u32 = 0;

                while count < SAMPLES_PER_UPDATE {
                    thread::sleep(SAMPLE_INTERVAL);

                    // Both must succeed — if either fails, discard the pair and retry.
                    let bat_r = battery_ina.as_mut().map_or(Some(FAKE_READING), read_ina);
                    let ps_r = ps_ina.as_mut().map_or(Some(FAKE_READING), read_ina);
                    read_total += 1;

                    if let (Some(bat_r), Some(ps_r)) = (bat_r, ps_r) {
                        bat_acc.add(&bat_r);
                        ps_acc.add(&ps_r);
                        count += 1;
                    } else {
                        read_failures += 1;
                    }
                }

                max_charge = max_charge.max(bat_acc.last_charge);
                min_charge = min_charge.min(bat_acc.last_charge);
                let charge_range = max_charge - min_charge;

                sensor_data.lock().unwrap().update(
                    bat_acc.average(SAMPLES_PER_UPDATE),
                    ps_acc.average(SAMPLES_PER_UPDATE),
                    read_total,
                    read_failures,
                    charge_range,
                );
            }
        })
        .unwrap();
}
