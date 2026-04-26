//! XY7025 programmable buck converter — Modbus-RTU over UART1 @ 115200 8N1.
//!
//! Slave addr 0x01 (default). Holding registers (fn 0x03), /100 scale:
//!   0x0000 V_set, 0x0001 I_set, 0x0002 V_out, 0x0003 I_out,
//!   0x0004 P_out, 0x0005 V_in.
//!
//! Wiring: ESP TX -> XY RX, ESP RX -> XY TX, common GND. No voltage divider
//! needed — both sides are 3.3 V TTL.
//!
//! The device is abstracted behind `XyDevice` so the thread loop +
//! charge-supervisor integration runs unchanged under the `xy-fake`
//! feature. The fake still constructs the UART driver (so pin/mux/baud
//! conflicts surface on the bench) but never transacts — setpoints are
//! tracked in-memory.

use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use log::{error, warn};

use esp32_battery_logic::charging::{
    Action, BatterySample, ChargeSupervisor, Chemistry, FaultReason, Profile, SafetyLimits,
};
use esp32_battery_logic::data::{PsReading, SensorData};
use esp32_battery_logic::error_log::{Event, XyError};
use esp32_battery_logic::modbus::RtuError;

use crate::board::XyPins;
use crate::clock::EventRecorder;

const POLL_INTERVAL: Duration = Duration::from_millis(1000);

/// This board's pack: 4S LiFePO4, 50 Ah. Daily-cycle setpoints — 14.4 V
/// absorb / 13.5 V float. Currents derive from capacity via the `*_C`
/// constants in `charging`: 0.2C = 10 A CC, 0.06C = 3 A enter, 0.05C = 2.5 A
/// exit (manufacturer-standard tail).
const PACK_PROFILE: Profile = Profile::for_pack(Chemistry::LiFePo4, 4, 50.0);
/// Hard trip thresholds programmed into the XY's protection registers.
/// Derived from the profile so a chemistry/cell-count change moves them
/// in lockstep — no chance the OVP ceiling drifts below the absorb target.
const SAFETY: SafetyLimits = PACK_PROFILE.safety_limits();

#[allow(dead_code)] // v_set/i_set/v_in will be surfaced via HTTP panel
struct XyStatus {
    v_set: f32,
    i_set: f32,
    v_out: f32,
    i_out: f32,
    p_out: f32,
    v_in: f32,
}

/// The set of operations the charging loop needs from the buck. Real
/// builds get the UART-backed implementation; `xy-fake` builds get an
/// in-memory canned device. The thread loop is identical for both.
/// `RtuError` is the trait error type because every failure mode the
/// supervisor cares about today is a Modbus-RTU transport failure.
trait XyDevice {
    fn read_status(&self) -> Result<XyStatus, RtuError>;
    fn read_protection(&self) -> Result<SafetyLimits, RtuError>;
    fn set_voltage(&self, volts: f32) -> Result<(), RtuError>;
    fn set_current_limit(&self, amps: f32) -> Result<(), RtuError>;
    fn set_protection(&self, limits: SafetyLimits) -> Result<(), RtuError>;
    fn set_output(&self, on: bool) -> Result<(), RtuError>;
    fn set_power_on_default_off(&self) -> Result<(), RtuError>;
}

/// Boot-time failure: either the Modbus transport gave up, or a register
/// read back a different value than we wrote (wrong slave, scale-divider
/// mismatch, write rejected by the device, etc.). Either way, we must not
/// enable output.
enum BootError {
    Rtu(RtuError),
    Verify {
        what: &'static str,
        expected: f32,
        actual: f32,
    },
}

impl From<RtuError> for BootError {
    fn from(e: RtuError) -> Self {
        Self::Rtu(e)
    }
}

impl std::fmt::Display for BootError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Rtu(e) => std::fmt::Display::fmt(e, f),
            Self::Verify {
                what,
                expected,
                actual,
            } => {
                write!(
                    f,
                    "{what} readback mismatch: expected {expected:.2}, got {actual:.2}"
                )
            }
        }
    }
}

// --- Real device ------------------------------------------------------------

#[cfg(not(feature = "xy-fake"))]
mod real {
    use std::time::Duration;

    use esp_idf_hal::uart::{UartDriver, config::Config};
    use esp_idf_hal::units::Hertz;

    use esp32_battery_logic::charging::SafetyLimits;
    use esp32_battery_logic::modbus::RtuError;

    use super::{XyDevice, XyStatus};
    use crate::board::XyPins;
    use crate::modbus_rtu::{ModbusRtu, RtuConfig};

    const SLAVE: u8 = 0x01;
    const REG_V_SET: u16 = 0x0000;
    const REG_I_SET: u16 = 0x0001;
    const REG_OUTPUT_EN: u16 = 0x0012;
    const REG_S_LVP: u16 = 0x0052;
    const REG_S_OVP: u16 = 0x0053;
    const REG_S_OCP: u16 = 0x0054;
    /// S-INI: power-on default for the output switch. 0 = OFF on power-up,
    /// 1 = ON. Persists in EEPROM. Set OFF so a brown-out / MCU crash never
    /// leaves the buck silently sourcing into the pack while we're not
    /// supervising.
    const REG_S_INI: u16 = 0x005D;
    const BAUD: u32 = 115200;

    pub struct Xy<'d> {
        modbus: ModbusRtu<'d>,
    }

    impl<'d> Xy<'d> {
        pub fn new(pins: XyPins) -> Self {
            let config = Config::new().baudrate(Hertz(BAUD));
            let uart = UartDriver::new(
                pins.uart,
                pins.tx,
                pins.rx,
                None::<esp_idf_hal::gpio::AnyIOPin>,
                None::<esp_idf_hal::gpio::AnyIOPin>,
                &config,
            )
            .expect("UART1 init");
            // 500 ms response window + 10 ms post-write quiet gap are
            // empirically what the XY7025 wants — tighter values cause
            // the slave to miss back-to-back writes.
            let modbus = ModbusRtu::new(
                uart,
                RtuConfig {
                    response_timeout: Duration::from_millis(500),
                    post_write_gap: Duration::from_millis(10),
                },
            );
            Self { modbus }
        }
    }

    impl XyDevice for Xy<'_> {
        fn read_status(&self) -> Result<XyStatus, RtuError> {
            let r = self.modbus.read_holding(SLAVE, 0x0000, 6)?;
            Ok(XyStatus {
                v_set: r[0] as f32 / 100.0,
                i_set: r[1] as f32 / 100.0,
                v_out: r[2] as f32 / 100.0,
                i_out: r[3] as f32 / 100.0,
                p_out: r[4] as f32 / 10.0,
                v_in: r[5] as f32 / 100.0,
            })
        }

        fn read_protection(&self) -> Result<SafetyLimits, RtuError> {
            // 0x0052 LVP, 0x0053 OVP, 0x0054 OCP — contiguous, one read.
            let r = self.modbus.read_holding(SLAVE, REG_S_LVP, 3)?;
            Ok(SafetyLimits {
                lvp_v: r[0] as f32 / 100.0,
                ovp_v: r[1] as f32 / 100.0,
                ocp_a: r[2] as f32 / 100.0,
            })
        }

        fn set_voltage(&self, volts: f32) -> Result<(), RtuError> {
            self.modbus
                .write_holding(SLAVE, REG_V_SET, (volts * 100.0).round() as u16)
        }

        fn set_current_limit(&self, amps: f32) -> Result<(), RtuError> {
            self.modbus
                .write_holding(SLAVE, REG_I_SET, (amps * 100.0).round() as u16)
        }

        fn set_protection(&self, limits: SafetyLimits) -> Result<(), RtuError> {
            self.modbus
                .write_holding(SLAVE, REG_S_OVP, (limits.ovp_v * 100.0).round() as u16)?;
            self.modbus
                .write_holding(SLAVE, REG_S_OCP, (limits.ocp_a * 100.0).round() as u16)?;
            self.modbus
                .write_holding(SLAVE, REG_S_LVP, (limits.lvp_v * 100.0).round() as u16)?;
            Ok(())
        }

        fn set_output(&self, on: bool) -> Result<(), RtuError> {
            self.modbus
                .write_holding(SLAVE, REG_OUTPUT_EN, if on { 1 } else { 0 })
        }

        fn set_power_on_default_off(&self) -> Result<(), RtuError> {
            self.modbus.write_holding(SLAVE, REG_S_INI, 0)
        }
    }
}

// --- Fake device ------------------------------------------------------------

#[cfg(feature = "xy-fake")]
mod fake {
    use std::cell::Cell;

    use esp_idf_hal::uart::{UartDriver, config::Config};
    use esp_idf_hal::units::Hertz;

    use esp32_battery_logic::charging::SafetyLimits;
    use esp32_battery_logic::modbus::RtuError;

    use super::{XyDevice, XyStatus};
    use crate::board::XyPins;

    const BAUD: u32 = 115200;

    /// In-memory stand-in for the buck. Tracks the last voltage/output
    /// state set by the supervisor so reads reflect what the supervisor
    /// last commanded — exercises the same control flow as the real path.
    pub struct Xy<'d> {
        v_set: Cell<f32>,
        i_set: Cell<f32>,
        protection: Cell<SafetyLimits>,
        output_on: Cell<bool>,
        // Real UART driver, constructed but never written to. Held so the
        // peripheral and its GPIOs are genuinely configured and claimed
        // for the program lifetime — pin/mux/baud conflicts surface on
        // the bench just as they would in a real build.
        _uart: UartDriver<'d>,
    }

    impl<'d> Xy<'d> {
        pub fn new(pins: XyPins) -> Self {
            let config = Config::new().baudrate(Hertz(BAUD));
            let uart = UartDriver::new(
                pins.uart,
                pins.tx,
                pins.rx,
                None::<esp_idf_hal::gpio::AnyIOPin>,
                None::<esp_idf_hal::gpio::AnyIOPin>,
                &config,
            )
            .expect("UART1 init");
            Self {
                v_set: Cell::new(13.5),
                i_set: Cell::new(0.0),
                protection: Cell::new(SafetyLimits {
                    ovp_v: 0.0,
                    ocp_a: 0.0,
                    lvp_v: 0.0,
                }),
                output_on: Cell::new(false),
                _uart: uart,
            }
        }
    }

    impl XyDevice for Xy<'_> {
        fn read_status(&self) -> Result<XyStatus, RtuError> {
            let v = if self.output_on.get() {
                self.v_set.get()
            } else {
                0.0
            };
            Ok(XyStatus {
                v_set: self.v_set.get(),
                i_set: self.i_set.get(),
                v_out: v,
                i_out: 0.0,
                p_out: 0.0,
                v_in: 24.0,
            })
        }
        fn read_protection(&self) -> Result<SafetyLimits, RtuError> {
            Ok(self.protection.get())
        }
        fn set_voltage(&self, volts: f32) -> Result<(), RtuError> {
            self.v_set.set(volts);
            Ok(())
        }
        fn set_current_limit(&self, amps: f32) -> Result<(), RtuError> {
            self.i_set.set(amps);
            Ok(())
        }
        fn set_protection(&self, limits: SafetyLimits) -> Result<(), RtuError> {
            self.protection.set(limits);
            Ok(())
        }
        fn set_output(&self, on: bool) -> Result<(), RtuError> {
            self.output_on.set(on);
            Ok(())
        }
        fn set_power_on_default_off(&self) -> Result<(), RtuError> {
            Ok(())
        }
    }
}

// --- Shared thread loop -----------------------------------------------------

/// Programs protection + setpoints, reads them back to confirm the device
/// accepted the writes, then enables output. Any failure short-circuits —
/// we never enable output with unverified setpoints. Readback catches
/// dropped Modbus writes, scale-divider mismatches, and wrong-slave wiring
/// before the buck can source into the pack.
fn boot_sequence<D: XyDevice>(xy: &D) -> Result<(), BootError> {
    xy.set_output(false)?;
    xy.set_power_on_default_off()?;
    xy.set_protection(SAFETY)?;
    xy.set_voltage(PACK_PROFILE.float_v)?;
    xy.set_current_limit(PACK_PROFILE.regulation_a)?;

    let s = xy.read_status()?;
    verify("V_SET", PACK_PROFILE.float_v, s.v_set)?;
    verify("I_SET", PACK_PROFILE.regulation_a, s.i_set)?;
    let p = xy.read_protection()?;
    verify("OVP", SAFETY.ovp_v, p.ovp_v)?;
    verify("OCP", SAFETY.ocp_a, p.ocp_a)?;
    verify("LVP", SAFETY.lvp_v, p.lvp_v)?;

    xy.set_output(true)?;
    Ok(())
}

/// One register quantum is 0.01 (V or A); allow up to two quanta for
/// IEEE-float round-trip slack on values like 14.4 V whose binary repr
/// isn't exact.
fn verify(what: &'static str, expected: f32, actual: f32) -> Result<(), BootError> {
    if (expected - actual).abs() < 0.02 {
        Ok(())
    } else {
        Err(BootError::Verify {
            what,
            expected,
            actual,
        })
    }
}

/// Cold-boot retry budget for `boot_sequence`. The XY7025's UART is
/// slower to come up than the ESP, especially on standalone (no-USB)
/// power where there's no CDC-enumeration delay to mask the gap. ~5 s
/// of retries swallows the race without delaying the supervisor loop
/// noticeably when the XY is actually unreachable.
const BOOT_RETRY_DELAY: Duration = Duration::from_millis(100);
const BOOT_RETRY_COUNT: u32 = 10;

fn run<D: XyDevice>(xy: D, sensor_data: Arc<Mutex<SensorData>>, recorder: EventRecorder) {
    let mut last_err = None;
    let booted = (0..BOOT_RETRY_COUNT).any(|attempt| {
        if attempt > 0 {
            thread::sleep(BOOT_RETRY_DELAY);
        }
        match boot_sequence(&xy) {
            Ok(()) => true,
            Err(e) => {
                last_err = Some(e);
                false
            }
        }
    });
    if !booted {
        let e = last_err.expect("retry loop ran at least once");
        error!(
            "XY boot failed after {BOOT_RETRY_COUNT} attempts: {e} — forcing output OFF, will keep polling"
        );
        recorder.record(Event::Xy(XyError::BootSequence));
        let _ = xy.set_output(false);
    }

    let mut supervisor = ChargeSupervisor::new(PACK_PROFILE);

    loop {
        let (modbus_ok, battery) = {
            let mut sd = sensor_data.lock().unwrap();
            let modbus_ok = match xy.read_status() {
                Ok(s) => {
                    sd.update_ps(PsReading {
                        voltage: s.v_out,
                        current: s.i_out,
                        power: s.p_out,
                    });
                    // Setpoint drift = buck is sourcing under unknown
                    // settings. Latch SettingsDrift; the next supervisor
                    // tick will emit DisableOutput.
                    let want_v = supervisor.target_voltage();
                    let want_i = PACK_PROFILE.regulation_a;
                    let v_drift = (s.v_set - want_v).abs() >= 0.02;
                    let i_drift = (s.i_set - want_i).abs() >= 0.02;
                    if v_drift || i_drift {
                        error!(
                            "XY setpoint drift: V_SET want {want_v:.2} got {:.2}, I_SET want {want_i:.2} got {:.2}",
                            s.v_set, s.i_set
                        );
                        supervisor.force_fault(FaultReason::SettingsDrift);
                    }
                    true
                }
                Err(e) => {
                    warn!("XY read_status: {e}");
                    recorder.record(Event::Xy(XyError::ReadStatus));
                    false
                }
            };

            let battery = sd.battery_reading().map(|b| BatterySample {
                voltage: b.voltage,
                current: b.current,
            });

            (modbus_ok, battery)
        };

        match supervisor.tick(modbus_ok, battery, POLL_INTERVAL) {
            Action::None => {}
            Action::SetVoltage(v) => {
                let phase = match supervisor.phase() {
                    esp32_battery_logic::charging::Phase::Float => "float",
                    esp32_battery_logic::charging::Phase::Absorb => "absorb",
                };
                log::info!("charge phase → {phase}: setting V_set = {v:.2} V");
                if let Err(e) = xy.set_voltage(v) {
                    warn!("XY set_voltage({v}): {e}");
                    recorder.record(Event::Xy(XyError::SetVoltage));
                }
            }
            Action::DisableOutput(reason) => {
                let reason_str = match reason {
                    FaultReason::BatterySensorStale => "battery sensor stale",
                    FaultReason::ModbusUnhealthy => "modbus link unhealthy",
                    FaultReason::Overvoltage => "pack overvoltage",
                    FaultReason::AbsorbTimeout => "absorb time cap reached",
                    FaultReason::SettingsDrift => "setpoint readback drift",
                };
                match xy.set_output(false) {
                    Ok(()) => {
                        error!("CHARGE FAULT ({reason_str}): PS output DISABLED");
                        supervisor.ack_disable();
                    }
                    Err(e) => {
                        error!(
                            "CHARGE FAULT ({reason_str}): set_output(false) failed: {e} — will retry"
                        );
                        recorder.record(Event::Xy(XyError::SetOutput));
                    }
                }
            }
        }
        thread::sleep(POLL_INTERVAL);
    }
}

// --- Public entry point -----------------------------------------------------

#[cfg(not(feature = "xy-fake"))]
fn make_device(pins: XyPins) -> real::Xy<'static> {
    real::Xy::new(pins)
}

#[cfg(feature = "xy-fake")]
fn make_device(pins: XyPins) -> fake::Xy<'static> {
    log::info!("XY: fake mode — claiming UART but not driving it");
    fake::Xy::new(pins)
}

pub fn start(pins: XyPins, sensor_data: Arc<Mutex<SensorData>>, recorder: EventRecorder) {
    thread::Builder::new()
        .name("xy".into())
        .stack_size(4096)
        .spawn(move || run(make_device(pins), sensor_data, recorder))
        .unwrap();
}
