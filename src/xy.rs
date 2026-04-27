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

use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use log::{error, info, warn};

use esp32_battery_logic::charging::{
    self, Action, BatterySample, BuckOutput, ChargeSupervisor, Chemistry, PollResult, Profile,
    SafetyLimits, Setpoints, XyProtectionStatus,
};
use esp32_battery_logic::data::{PsReading, SensorData};
use esp32_battery_logic::error_log::{Event, XyError};
use esp32_battery_logic::modbus::RtuError;

use crate::board::XyPins;
use crate::clock::EventRecorder;
use crate::reboot;
use crate::task_wdt;

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
#[allow(dead_code)] // protection-status methods wired in once the supervisor gates recovery on them
trait XyDevice {
    fn read_status(&self) -> Result<XyStatus, RtuError>;
    fn read_protection(&self) -> Result<SafetyLimits, RtuError>;
    fn read_protection_status(&self) -> Result<XyProtectionStatus, RtuError>;
    fn read_output_on(&self) -> Result<bool, RtuError>;
    fn set_voltage(&self, volts: f32) -> Result<(), RtuError>;
    fn set_current_limit(&self, amps: f32) -> Result<(), RtuError>;
    fn set_protection(&self, limits: SafetyLimits) -> Result<(), RtuError>;
    /// Write 0 to register 0x0010 (PROTECT) to clear a latched
    /// protection cause. Per the XY6020L doc, this is how the device
    /// stops blinking the front-panel backlight after a trip.
    fn clear_protection_status(&self) -> Result<(), RtuError>;
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
    /// OUTPUT_EN read back as 1 after we wrote 0 + S_INI=0. Either the
    /// disable didn't stick or the front panel re-enabled it. Refuse to
    /// hand off to the supervisor — we'd be entering Pending with the
    /// buck already sourcing.
    OutputOn,
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
            Self::OutputOn => f.write_str("OUTPUT_EN read back ON after disable"),
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

    use super::{XyDevice, XyProtectionStatus, XyStatus};
    use crate::board::XyPins;
    use crate::modbus_rtu::{ModbusRtu, RtuConfig};

    const SLAVE: u8 = 0x01;
    const REG_V_SET: u16 = 0x0000;
    const REG_I_SET: u16 = 0x0001;
    /// PROTECT register: latched protection cause, 0 = normal, 1–10 =
    /// specific trip (see `XyProtectionStatus`). Write 0 to clear.
    #[allow(dead_code)] // wired in once the supervisor gates recovery on it
    const REG_PROTECT: u16 = 0x0010;
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

    /// XY7025 holding registers store voltage / current as hundredths
    /// (e.g. 1440 = 14.40 V). 25 A worst case → 2500, well under u16::MAX.
    fn to_reg(v: f32) -> u16 {
        (v * 100.0).round() as u16
    }

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

        fn read_protection_status(&self) -> Result<XyProtectionStatus, RtuError> {
            let r = self.modbus.read_holding(SLAVE, REG_PROTECT, 1)?;
            Ok(XyProtectionStatus::from_register(r[0]))
        }

        fn read_output_on(&self) -> Result<bool, RtuError> {
            let r = self.modbus.read_holding(SLAVE, REG_OUTPUT_EN, 1)?;
            Ok(r[0] != 0)
        }

        fn set_voltage(&self, volts: f32) -> Result<(), RtuError> {
            self.modbus.write_holding(SLAVE, REG_V_SET, to_reg(volts))
        }

        fn set_current_limit(&self, amps: f32) -> Result<(), RtuError> {
            self.modbus.write_holding(SLAVE, REG_I_SET, to_reg(amps))
        }

        fn set_protection(&self, limits: SafetyLimits) -> Result<(), RtuError> {
            self.modbus
                .write_holding(SLAVE, REG_S_OVP, to_reg(limits.ovp_v))?;
            self.modbus
                .write_holding(SLAVE, REG_S_OCP, to_reg(limits.ocp_a))?;
            self.modbus
                .write_holding(SLAVE, REG_S_LVP, to_reg(limits.lvp_v))?;
            Ok(())
        }

        fn clear_protection_status(&self) -> Result<(), RtuError> {
            self.modbus.write_holding(SLAVE, REG_PROTECT, 0)
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

    use super::{XyDevice, XyProtectionStatus, XyStatus};
    use crate::board::XyPins;

    const BAUD: u32 = 115200;

    /// In-memory stand-in for the buck. Tracks the last voltage/output
    /// state set by the supervisor so reads reflect what the supervisor
    /// last commanded — exercises the same control flow as the real path.
    pub struct Xy<'d> {
        v_set: Cell<f32>,
        i_set: Cell<f32>,
        protection: Cell<SafetyLimits>,
        protection_status: Cell<XyProtectionStatus>,
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
                protection_status: Cell::new(XyProtectionStatus::Normal),
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
        fn read_protection_status(&self) -> Result<XyProtectionStatus, RtuError> {
            Ok(self.protection_status.get())
        }
        fn read_output_on(&self) -> Result<bool, RtuError> {
            Ok(self.output_on.get())
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
        fn clear_protection_status(&self) -> Result<(), RtuError> {
            self.protection_status.set(XyProtectionStatus::Normal);
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

/// Programs protection + setpoints and reads them back to confirm the
/// device accepted the writes. Output stays OFF — bringing up the buck
/// is the supervisor's job, conditional on a fresh, drift-free, in-range
/// first tick. Readback catches dropped writes, scale-divider mismatches,
/// and wrong-slave wiring before the supervisor can ask for output enable.
fn boot_sequence<D: XyDevice>(xy: &D) -> Result<(), BootError> {
    xy.set_output(false)?;
    // Wipe any latched protection cause from a prior session — power
    // outages and unrelated crashes leave 0x0010 set, and we don't want
    // that stale value contaminating the per-tick read in `poll`.
    xy.clear_protection_status()?;
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
    // Confirm the disable actually took. set_output(false) and S_INI=0
    // both happened above; if OUTPUT_EN still reads 1, we don't trust
    // the device enough to hand off to the supervisor.
    if xy.read_output_on()? {
        return Err(BootError::OutputOn);
    }
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

/// Task watchdog timeout for the xy supervisor thread. Must comfortably
/// exceed `POLL_INTERVAL` (1 s) plus the worst-case Modbus retry budget
/// (~500 ms response timeout × a few transactions per tick). 10 s gives
/// generous headroom — long enough that a single slow tick never trips
/// the WDT, short enough that a wedged loop reboots within ~10 s and
/// `S_INI=OFF` brings the buck back up disabled.
const WDT_TIMEOUT: Duration = Duration::from_secs(10);
const _: () = assert!(WDT_TIMEOUT.as_secs() > POLL_INTERVAL.as_secs() * 2);

/// Try `boot_sequence` up to `BOOT_RETRY_COUNT` times. On total failure,
/// best-effort disable output and reboot the MCU — the alternative is
/// falling through to a supervisor that immediately latches
/// `ModbusUnhealthy` and stays Tripped forever. A reboot might clear
/// transient causes (XY7025 still powering up, UART state, ESP IDF
/// driver wedged), and S_INI=OFF means the buck comes back disabled
/// even if our set_output call below failed.
fn boot_with_retries<D: XyDevice>(xy: &D, recorder: &EventRecorder) {
    for attempt in 0..BOOT_RETRY_COUNT {
        if attempt > 0 {
            thread::sleep(BOOT_RETRY_DELAY);
        }
        match boot_sequence(xy) {
            Ok(()) => return,
            Err(e) => warn!("XY boot attempt {}/{BOOT_RETRY_COUNT}: {e}", attempt + 1),
        }
    }
    error!("XY boot failed after {BOOT_RETRY_COUNT} attempts — rebooting MCU");
    recorder.record(Event::Xy(XyError::BootSequence));
    shutdown_or_reboot(xy, "pre-reboot", true, recorder);
}

fn run<D: XyDevice>(xy: D, sensor_data: Arc<Mutex<SensorData>>, recorder: EventRecorder) {
    // Subscribe once for the lifetime of this thread. Restarts of the
    // supervise loop (below) keep feeding the same WDT subscription.
    task_wdt::init_and_subscribe(WDT_TIMEOUT);

    // Outer loop = recovery restarts. Each iteration re-runs boot_sequence
    // (which reprograms + verifies OVP/OCP/LVP/V_SET/I_SET) and constructs
    // a fresh ChargeSupervisor. Bounded by OUTPUT_RECOVERY_MAX_ATTEMPTS;
    // exhaustion → leave the buck off and exit the thread.
    for restart in 0..=charging::OUTPUT_RECOVERY_MAX_ATTEMPTS {
        if restart > 0 {
            info!(
                "XY supervisor restart {restart}/{} (recovering from buck self-disable)",
                charging::OUTPUT_RECOVERY_MAX_ATTEMPTS
            );
        }
        boot_with_retries(&xy, &recorder);
        let mut supervisor = ChargeSupervisor::new(PACK_PROFILE);

        // Inner loop = supervise. Returns Ok(()) when tick emits
        // RestartSupervisor (caller's cue to tear down + redo
        // boot_sequence); panics propagate out as Err.
        let result = catch_unwind(AssertUnwindSafe(|| {
            loop {
                task_wdt::reset();
                let p = poll(&xy, &sensor_data, &recorder);
                let action = supervisor.tick(p, POLL_INTERVAL);
                if matches!(action, Action::RestartSupervisor) {
                    return;
                }
                apply_action(&xy, &mut supervisor, action, &recorder);
                thread::sleep(POLL_INTERVAL);
            }
        }));

        match result {
            Ok(()) => continue, // outer loop re-runs boot_sequence
            Err(panic) => {
                let msg = panic
                    .downcast_ref::<&'static str>()
                    .copied()
                    .or_else(|| panic.downcast_ref::<String>().map(String::as_str))
                    .unwrap_or("<non-string panic>");
                error!("XY supervisor thread PANICKED: {msg} — forcing output OFF");
                recorder.record(Event::Xy(XyError::SupervisorPanic));
                shutdown_or_reboot(&xy, "post-panic", false, &recorder);
                return;
            }
        }
    }

    error!(
        "XY recovery budget exhausted ({} attempts) — forcing output OFF",
        charging::OUTPUT_RECOVERY_MAX_ATTEMPTS
    );
    shutdown_or_reboot(&xy, "recovery-exhausted", false, &recorder);
}

/// Best-effort `set_output(false)` then either exit (buck is OFF) or
/// reboot. Reboot is forced when the caller can't continue regardless
/// (boot failure), or triggered by a failed disable — leaving the buck
/// sourcing with no supervisor and no WDT is the one thing we never want.
/// S_INI=0 ensures the buck comes back OFF after the reboot.
fn shutdown_or_reboot<D: XyDevice>(
    xy: &D,
    ctx: &'static str,
    force_reboot: bool,
    recorder: &EventRecorder,
) {
    let disable_ok = match xy.set_output(false) {
        Ok(()) => {
            error!("XY {ctx} set_output(false) succeeded — buck is OFF");
            true
        }
        Err(e) => {
            error!("XY {ctx} set_output(false) FAILED: {e}");
            recorder.record(Event::Xy(XyError::SetOutput));
            false
        }
    };
    // Releases the WDT subscription so the reboot grace period (2 s in
    // `reboot_after`) doesn't trip the deadman before the restart fires.
    task_wdt::unsubscribe();
    if force_reboot || !disable_ok {
        reboot::reboot_after("XY shutdown: rebooting now");
        loop {
            thread::park();
        }
    }
}

/// One read cycle: poll the buck, push readings into shared sensor data,
/// snapshot the latest battery sample, and return the supervisor's
/// per-tick view of the world.
fn poll<D: XyDevice>(
    xy: &D,
    sensor_data: &Mutex<SensorData>,
    recorder: &EventRecorder,
) -> PollResult {
    let mut sd = sensor_data.lock().unwrap();
    let setpoints = match xy.read_status() {
        Ok(s) => {
            sd.update_ps(PsReading {
                voltage: s.v_out,
                current: s.i_out,
                power: s.p_out,
            });
            Some(Setpoints {
                v_set: s.v_set,
                i_set: s.i_set,
            })
        }
        Err(e) => {
            warn!("XY read_status: {e}");
            recorder.record(Event::Xy(XyError::ReadStatus));
            None
        }
    };
    // Read OUTPUT_EN separately — it's at 0x0012, not contiguous with the
    // main readback block. Lets the supervisor catch buck self-disable
    // (hardware OVP/OCP/LVP, panel toggle) within one tick. When the
    // buck reports OFF, ask PROTECT (0x0010) why — that was wiped during
    // boot_sequence, so any non-Normal value here is from this session.
    // While output is on PROTECT is necessarily Normal so we skip the
    // round-trip.
    let output = match xy.read_output_on() {
        Ok(true) => Some(BuckOutput::On),
        Ok(false) => {
            let cause = match xy.read_protection_status() {
                Ok(status) => {
                    if status != XyProtectionStatus::Normal {
                        warn!("XY PROTECT latched: {status}");
                    }
                    Some(status)
                }
                Err(e) => {
                    warn!("XY read_protection_status: {e}");
                    None
                }
            };
            Some(BuckOutput::Off { cause })
        }
        Err(e) => {
            warn!("XY read_output_on: {e}");
            recorder.record(Event::Xy(XyError::ReadStatus));
            None
        }
    };
    let battery = sd.battery_reading().map(|b| BatterySample {
        voltage: b.voltage,
        current: b.current,
    });
    PollResult {
        setpoints,
        output,
        battery,
    }
}

fn apply_action<D: XyDevice>(
    xy: &D,
    supervisor: &mut ChargeSupervisor,
    action: Action,
    recorder: &EventRecorder,
) {
    match action {
        // Filtered out by `run`'s inner loop before this is called.
        Action::None | Action::RestartSupervisor => {}
        Action::EnableOutput => {
            info!("supervisor enabling output");
            match xy.set_output(true) {
                Ok(()) => supervisor.ack_enable(),
                Err(e) => {
                    warn!("XY set_output(true): {e} — supervisor stays Pending, will retry");
                    recorder.record(Event::Xy(XyError::SetOutput));
                }
            }
        }
        Action::UpdateVoltage { target_v } => match xy.set_voltage(target_v) {
            Ok(()) => {
                supervisor.ack_voltage_update();
                info!(
                    "charge phase → {}: V_set = {target_v:.2} V",
                    supervisor.phase().label()
                );
            }
            Err(e) => {
                warn!("XY set_voltage({target_v}): {e} — supervisor will retry next tick");
                recorder.record(Event::Xy(XyError::SetVoltage));
            }
        },
        Action::DisableOutput(reason) => match xy.set_output(false) {
            Ok(()) => {
                error!("CHARGE FAULT ({reason}): PS output DISABLED");
                supervisor.ack_disable();
            }
            Err(e) => {
                error!("CHARGE FAULT ({reason}): set_output(false) failed: {e} — will retry");
                recorder.record(Event::Xy(XyError::SetOutput));
            }
        },
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
        .stack_size(8192)
        .spawn(move || run(make_device(pins), sensor_data, recorder))
        .unwrap();
}
