//! XY7025 programmable buck converter — Modbus-RTU over UART1 @ 115200 8N1.
//!
//! Slave addr 0x01 (default). Wiring: ESP TX -> XY RX, ESP RX -> XY TX,
//! common GND. No voltage divider needed — both sides are 3.3 V TTL.
//!
//! The protocol/device layer is the external `xy_modbus` crate; this
//! module owns only the supervisor thread, the boot/recovery policy,
//! and (under `xy-fake`) an in-memory stand-in that drives the same
//! supervisor loop without touching the bus.

use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use log::{error, info, warn};

use esp32_battery_logic::SETPOINT_DRIFT_TOL;
use esp32_battery_logic::{
    Action, BatterySample, BuckOutput, BusError, ChargeSupervisor, PollResult, VoltageWriteOutcome,
    VoltageWriter, apply_update_voltage,
};
use esp32_battery_logic::{ChargeStatus, ProtectionPolicy, PsReading, SensorData};
use esp32_battery_logic::{Event, XyError};

use xy_modbus::{ModelCheck, ProtectionStatus, SafetyLimits, Status};

use crate::board::XyPins;
use crate::clock::{EventRecorder, LoopTimer};
use crate::reboot;
use crate::task_wdt;
use crate::{BUCK, PACK_PROFILE, PACK_TEMP};

const POLL_INTERVAL: Duration = Duration::from_millis(1000);

/// Quiet window between disabling output and writing the new V_SET on a
/// step-down. Lets the inductor current decay and the buck's internal
/// regulator state quiesce so the next register write lands in a clean
/// state. Modbus latency usually covers this on its own (~50 ms
/// inter-frame + ~500 ms response timeout), but the explicit sleep
/// removes the dependence on transport timing.
const STEP_DOWN_SETTLE: Duration = Duration::from_millis(100);

/// The set of operations the charging loop needs from the buck. Real
/// builds get the `xy_modbus`-backed implementation; `xy-fake` builds
/// get an in-memory canned device. The thread loop is identical.
///
/// Extends `charging::VoltageWriter` so `set_voltage` / `set_output`
/// are defined exactly once and `charging::apply_update_voltage` can
/// drive any `XyDevice` directly.
trait XyDevice: VoltageWriter {
    fn check_model(&mut self) -> Result<ModelCheck, BusError>;
    /// Live + control snapshot (regs 0x0000–0x0012). One Modbus
    /// round-trip per supervisor tick.
    fn read_status(&mut self) -> Result<Status, BusError>;
    fn read_safety_limits(&mut self) -> Result<SafetyLimits, BusError>;
    fn set_current_limit(&mut self, amps: f32) -> Result<(), BusError>;
    fn set_safety_limits(&mut self, limits: SafetyLimits) -> Result<(), BusError>;
    /// Write 0 to PROTECT (0x0010) to clear a latched protection cause.
    fn clear_protection_status(&mut self) -> Result<(), BusError>;
    /// Program S_INI (power-on default of OUTPUT_EN). We always pass
    /// `false` so a brown-out / unrelated reset brings the buck back
    /// disabled — the supervisor's bring-up is the only thing allowed
    /// to enable output.
    fn set_power_on_default(&mut self, on: bool) -> Result<(), BusError>;
}

/// Boot-time failure: either the Modbus transport gave up, or a register
/// read back a different value than we wrote (wrong slave, scale mismatch,
/// write rejected, etc.). Either way, we must not enable output.
enum BootError {
    Bus(BusError),
    Verify {
        what: &'static str,
        expected: f32,
        actual: f32,
    },
    /// OUTPUT_EN read back as 1 after we wrote 0 + S_INI=0. Either the
    /// disable didn't stick or the front panel re-enabled it. Refuse to
    /// hand off to the supervisor — we'd be entering its `Boot` state
    /// with the buck already sourcing.
    OutputOn,
    /// Device's MODEL register reports a code whose register scales are
    /// not the ones xy-modbus decodes with. Readings (especially I-OUT)
    /// would be off by 10×; refuse to proceed. Only the reported code is
    /// carried: which codes the driver's scales suit is `check_model`'s
    /// business, and naming one here would be this module's guess at it.
    ModelMismatch {
        device_code: u16,
    },
}

impl From<BusError> for BootError {
    fn from(e: BusError) -> Self {
        Self::Bus(e)
    }
}

impl std::fmt::Display for BootError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Bus(e) => std::fmt::Display::fmt(e, f),
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
            Self::ModelMismatch { device_code } => write!(
                f,
                "MODEL 0x{device_code:04X}: register scales are not the ones this driver decodes with"
            ),
        }
    }
}

#[cfg(not(feature = "xy-fake"))]
mod real {
    use esp_idf_hal::uart::{UartDriver, config::Config};
    use esp_idf_hal::units::Hertz;

    use xy_modbus::esp_idf::EspIdfTransport;
    use xy_modbus::{ModelCheck, SafetyLimits, Status};

    use esp32_battery_logic::{BusError, VoltageWriter};

    use super::XyDevice;
    use crate::board::XyPins;

    const BAUD: u32 = 115200;

    pub struct Xy<'d>(xy_modbus::Xy<EspIdfTransport<'d>>);

    impl Xy<'_> {
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
            // Default XY-series timing baked in by `from_esp_uart`:
            // 500 ms response window, 50 ms inter-frame gap.
            Self(xy_modbus::Xy::from_esp_uart(uart))
        }
    }

    impl VoltageWriter for Xy<'_> {
        fn set_voltage(&mut self, volts: f32) -> Result<(), BusError> {
            self.0.set_voltage(volts)
        }
        fn set_output(&mut self, on: bool) -> Result<(), BusError> {
            self.0.set_output(on)
        }
    }

    impl XyDevice for Xy<'_> {
        fn check_model(&mut self) -> Result<ModelCheck, BusError> {
            self.0.check_model()
        }

        fn read_status(&mut self) -> Result<Status, BusError> {
            self.0.read_status()
        }

        fn read_safety_limits(&mut self) -> Result<SafetyLimits, BusError> {
            self.0.read_safety_limits()
        }

        fn set_current_limit(&mut self, amps: f32) -> Result<(), BusError> {
            self.0.set_current_limit(amps)
        }

        fn set_safety_limits(&mut self, limits: SafetyLimits) -> Result<(), BusError> {
            self.0.set_safety_limits(limits)
        }

        fn clear_protection_status(&mut self) -> Result<(), BusError> {
            self.0.clear_protection_status()
        }

        fn set_power_on_default(&mut self, on: bool) -> Result<(), BusError> {
            self.0.set_power_on_output(on)
        }
    }
}

#[cfg(feature = "xy-fake")]
mod fake {
    use esp_idf_hal::uart::{UartDriver, config::Config};
    use esp_idf_hal::units::Hertz;

    use esp32_battery_logic::{BusError, VoltageWriter};
    use xy_modbus::{ModelCheck, ProtectionStatus, RegMode, SafetyLimits, Setpoints, Status};

    use super::XyDevice;
    use crate::board::XyPins;

    /// MODEL word the fake reports: the XY7025's, so `xy-fake` builds clear
    /// the same `check_model` gate a real board does rather than masking it.
    /// Nothing compares against this — whether the driver's scales suit the
    /// device is `ModelCheck::scales_match`, decided inside xy-modbus.
    const FAKE_MODEL_CODE: u16 = 0x6500;

    const BAUD: u32 = 115200;

    /// In-memory stand-in for the buck. Tracks the last voltage/output
    /// state set by the supervisor so reads reflect what the supervisor
    /// last commanded — exercises the same control flow as the real path.
    pub struct Xy<'d> {
        v_set: f32,
        i_set: f32,
        protection: SafetyLimits,
        protection_status: ProtectionStatus,
        output_on: bool,
        // Real UART driver, constructed but never written to. Held so the
        // peripheral and its GPIOs are genuinely configured and claimed
        // for the program lifetime — pin/mux/baud conflicts surface on
        // the bench just as they would in a real build.
        _uart: UartDriver<'d>,
    }

    impl Xy<'_> {
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
                v_set: 13.5,
                i_set: 0.0,
                protection: SafetyLimits {
                    lvp_v: 0.0,
                    ovp_v: 0.0,
                    ocp_a: 0.0,
                },
                protection_status: ProtectionStatus::Normal,
                output_on: false,
                _uart: uart,
            }
        }
    }

    impl VoltageWriter for Xy<'_> {
        fn set_voltage(&mut self, volts: f32) -> Result<(), BusError> {
            self.v_set = volts;
            Ok(())
        }
        fn set_output(&mut self, on: bool) -> Result<(), BusError> {
            self.output_on = on;
            Ok(())
        }
    }

    impl XyDevice for Xy<'_> {
        fn check_model(&mut self) -> Result<ModelCheck, BusError> {
            Ok(ModelCheck {
                device_code: FAKE_MODEL_CODE,
                scales_match: true,
                limits_match: true,
            })
        }

        fn read_status(&mut self) -> Result<Status, BusError> {
            let v = if self.output_on { self.v_set } else { 0.0 };
            Ok(Status {
                setpoints: Setpoints {
                    v_set: self.v_set,
                    i_set: self.i_set,
                },
                v_out: v,
                i_out: 0.0,
                p_out: 0.0,
                v_in: 24.0,
                protection: self.protection_status,
                reg_mode: RegMode::ConstantVoltage,
                output_on: self.output_on,
            })
        }
        fn read_safety_limits(&mut self) -> Result<SafetyLimits, BusError> {
            Ok(self.protection)
        }
        fn set_current_limit(&mut self, amps: f32) -> Result<(), BusError> {
            self.i_set = amps;
            Ok(())
        }
        fn set_safety_limits(&mut self, limits: SafetyLimits) -> Result<(), BusError> {
            self.protection = limits;
            Ok(())
        }
        fn clear_protection_status(&mut self) -> Result<(), BusError> {
            self.protection_status = ProtectionStatus::Normal;
            Ok(())
        }
        fn set_power_on_default(&mut self, _on: bool) -> Result<(), BusError> {
            Ok(())
        }
    }
}

/// Programs protection + setpoints and reads them back to confirm the
/// device accepted the writes. Output stays OFF — bringing up the buck
/// is the supervisor's job, conditional on a fresh, drift-free, in-range
/// first tick. Readback catches dropped writes, scale mismatches, and
/// wrong-slave wiring before the supervisor can ask for output enable.
fn boot_sequence<D: XyDevice>(xy: &mut D) -> Result<u16, BootError> {
    // Confirm we're talking to the device the register scales were
    // written for, before any writes go out — a scale mismatch silently
    // shifts every subsequent reading (notably I-OUT) by 10×.
    //
    // `scales_match` is false for an unrecognised MODEL code as well as a
    // known-incompatible one, and we refuse either way. That is stricter
    // than the old `Inconclusive`-passes behaviour: an undocumented code
    // now blocks bring-up rather than being trusted on the assumption the
    // scales happen to line up.
    let check = xy.check_model()?;
    if !check.scales_match {
        return Err(BootError::ModelMismatch {
            device_code: check.device_code,
        });
    }
    // Scales decode correctly but the ceilings `set_voltage` and friends
    // enforce were written for the XY7025. Worth saying out loud; not
    // worth refusing over, since our own trip thresholds are programmed
    // into the device's protection registers either way.
    if !check.limits_match {
        warn!(
            "XY MODEL 0x{:04X}: scales match but limit ceilings differ from XY7025",
            check.device_code
        );
    }
    let device_code = check.device_code;
    xy.set_output(false)?;
    // Wipe any latched protection cause from a prior session — power
    // outages and unrelated crashes leave 0x0010 set, and we don't want
    // that stale value contaminating the per-tick read in `poll`.
    xy.clear_protection_status()?;
    xy.set_power_on_default(false)?;
    xy.set_safety_limits(BUCK.limits)?;
    xy.set_voltage(PACK_PROFILE.float_v)?;
    xy.set_current_limit(BUCK.i_set_a)?;

    let s = xy.read_status()?;
    verify("V_SET", PACK_PROFILE.float_v, s.setpoints.v_set)?;
    verify("I_SET", BUCK.i_set_a, s.setpoints.i_set)?;
    let p = xy.read_safety_limits()?;
    verify("OVP", BUCK.limits.ovp_v, p.ovp_v)?;
    verify("OCP", BUCK.limits.ocp_a, p.ocp_a)?;
    verify("LVP", BUCK.limits.lvp_v, p.lvp_v)?;
    // Confirm the disable actually took. set_output(false) and S_INI=0
    // both happened above; if OUTPUT_EN still reads 1, we don't trust
    // the device enough to hand off to the supervisor.
    if s.output_on {
        return Err(BootError::OutputOn);
    }
    Ok(device_code)
}

/// Confirm a register read back what we wrote. Uses the supervisor's
/// `SETPOINT_DRIFT_TOL`: this is the same commanded-vs-reported comparison
/// its per-tick drift check makes, done once at boot, and the two must not
/// disagree about how close counts as equal.
fn verify(what: &'static str, expected: f32, actual: f32) -> Result<(), BootError> {
    if (expected - actual).abs() < SETPOINT_DRIFT_TOL {
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
const _: () = assert!(task_wdt::WDT_TIMEOUT.as_secs() > POLL_INTERVAL.as_secs() * 2);

/// Try `boot_sequence` up to `BOOT_RETRY_COUNT` times. On total failure,
/// best-effort disable output and reboot the MCU — the alternative is
/// falling through to a supervisor that immediately latches
/// `ModbusUnhealthy` and stays latched forever. A reboot might clear
/// transient causes (XY7025 still powering up, UART state, ESP IDF
/// driver wedged), and S_INI=OFF means the buck comes back disabled
/// even if our set_output call below failed.
fn boot_with_retries<D: XyDevice>(xy: &mut D, recorder: &EventRecorder) -> u16 {
    for attempt in 0..BOOT_RETRY_COUNT {
        if attempt > 0 {
            thread::sleep(BOOT_RETRY_DELAY);
        }
        match boot_sequence(xy) {
            Ok(device_code) => return device_code,
            Err(e) => warn!("XY boot attempt {}/{BOOT_RETRY_COUNT}: {e}", attempt + 1),
        }
    }
    error!("XY boot failed after {BOOT_RETRY_COUNT} attempts — rebooting MCU");
    recorder.record(Event::Xy(XyError::BootSequence));
    // Best-effort disable before the reboot. S_INI=0 means the buck
    // comes back up disabled even if this write fails, so we don't
    // gate the reboot on it.
    if let Err(e) = xy.set_output(false) {
        error!("XY pre-reboot set_output(false) FAILED: {e}");
        recorder.record(Event::Xy(XyError::SetOutput));
    }
    reboot::reboot_after("XY boot exhausted: rebooting now");
    loop {
        thread::park();
    }
}

/// Supervisor thread entry: boot the buck, then poll → tick → apply
/// forever. Transient buck protections (LVP/OTP) are handled in-place
/// by the supervisor; permanent faults latch in `Action::None` and the
/// loop keeps polling so observability and the LCD stay live until a
/// reboot. Panics propagate (the panic hook reboots the MCU).
fn run<D: XyDevice>(
    mut xy: D,
    sensor_data: Arc<Mutex<SensorData>>,
    charge_status: Arc<Mutex<ChargeStatus>>,
    recorder: EventRecorder,
) {
    // !Send token stays bound to this FreeRTOS task for its lifetime.
    let wdt = task_wdt::subscribe();

    let model_code = boot_with_retries(&mut xy, &recorder);
    charge_status.lock().unwrap().model_code = model_code;
    let mut supervisor = ChargeSupervisor::new(PACK_PROFILE, BUCK.i_set_a, PACK_TEMP);

    let mut protection = ProtectionLog::new();
    // Lapped after `poll`, so a tick is charged its own Modbus traffic as
    // well as `POLL_INTERVAL` — which is the whole span the supervisor's
    // windows are meant to cover.
    let mut timer = LoopTimer::start();
    loop {
        wdt.reset();
        let outcome = poll(&mut xy, &sensor_data, &recorder, &mut protection);
        let action = supervisor.tick(outcome.poll, timer.lap());
        apply_action(&mut xy, &mut supervisor, action, &recorder);
        // The supervisor has no clock, so it buffers latch transitions and
        // we timestamp them here.
        while let Some(t) = supervisor.pop_transition() {
            recorder.record(Event::Charge(t));
        }
        // One publish per tick, covering this cycle's buck reading and the
        // supervisor state derived from it — so /api and the LCD always see
        // a coherent pair rather than a half-updated one. `phase()` is only
        // Some while the buck is sourcing; bring-up and latched states
        // surface as None so dashboards distinguish "not yet charging"
        // from a real phase.
        {
            let mut status = charge_status.lock().unwrap();
            status.ps_offline = outcome.ps_offline;
            status.phase = supervisor.phase();
            status.fault = supervisor.fault();
            status.parked = supervisor.parked();
            status.inhibit = supervisor.inhibit();
        }
        thread::sleep(POLL_INTERVAL);
    }
}

/// What one read cycle produced: the supervisor's view of the world, and
/// the input-rail status the dashboard shows. Returned together so `run`
/// publishes the status in the same lock as the supervisor state it feeds.
#[derive(Debug)]
struct PollOutcome {
    poll: PollResult,
    ps_offline: bool,
}

/// Per-episode de-duplication for latched `PROTECT` causes.
///
/// A latched buck reports the same cause on every poll for as long as it
/// stays down, so the event log wants one entry per episode rather than one
/// per second. Owns the last-seen cause so the loop doesn't have to carry it.
#[derive(Debug)]
struct ProtectionLog {
    last: ProtectionStatus,
}

impl ProtectionLog {
    fn new() -> Self {
        Self {
            last: ProtectionStatus::Normal,
        }
    }

    /// Turn one status read into the supervisor's `BuckOutput`, recording any
    /// newly-latched protection cause on the way.
    fn classify(&mut self, s: &Status, recorder: &EventRecorder) -> BuckOutput {
        if s.output_on {
            // Sourcing again: whatever episode was running is over, so the
            // next latch of the same cause counts as new.
            self.last = ProtectionStatus::Normal;
            return BuckOutput::On;
        }
        // Input loss is surfaced as "PS offline" and recovers on its own —
        // don't pollute the event log with it. Every other cause records on
        // the rising edge (first poll, or a change of cause); the warn! stays
        // per-poll for log visibility.
        if !s.protection.is_input_loss()
            && let Some(ev) = XyError::from_protection(s.protection)
        {
            warn!("XY PROTECT latched: {}", s.protection);
            if self.last != s.protection {
                recorder.record(Event::Xy(ev));
            }
        }
        self.last = s.protection;
        BuckOutput::Off {
            cause: s.protection,
        }
    }
}

/// The PS half of a status read, as the sensor store wants it.
fn ps_reading(s: &Status) -> PsReading {
    PsReading {
        voltage: s.v_out,
        current: s.i_out,
        power: s.p_out,
        v_set: s.setpoints.v_set,
        i_set: s.setpoints.i_set,
    }
}

/// One read cycle: read the buck, classify what it reported, publish the PS
/// half into shared sensor data, snapshot the latest battery sample, and hand
/// back everything the tick needs.
///
/// The Modbus read runs *without* the `SensorData` lock held — UART
/// transactions can take up to `response_timeout` (500 ms) and we don't
/// want HTTP / LCD / INA blocked on that. Everything it yields is decoded
/// first, then published and read back under a single acquisition.
fn poll<D: XyDevice>(
    xy: &mut D,
    sensor_data: &Mutex<SensorData>,
    recorder: &EventRecorder,
    protection: &mut ProtectionLog,
) -> PollOutcome {
    // Single bulk read covers V_SET..V_IN, PROTECT, CVCC, OUTPUT_EN —
    // one Modbus round-trip instead of three. PROTECT is necessarily
    // Normal while OUTPUT_EN is on; non-Normal here means the buck
    // self-disabled this session (boot_sequence wiped 0x0010).
    let status = match xy.read_status() {
        Ok(s) => Some(s),
        Err(e) => {
            warn!("XY read_status: {e}");
            recorder.record(Event::Xy(XyError::ReadStatus));
            None
        }
    };
    let output = status.as_ref().map(|s| protection.classify(s, recorder));

    // The cycle's only `SensorData` acquisition: publish what the buck said
    // and take the battery snapshot the supervisor needs, together.
    let battery = {
        let mut sd = sensor_data.lock().unwrap();
        if let Some(s) = &status {
            sd.update_ps(ps_reading(s));
        }
        sd.battery_reading().map(|b| BatterySample {
            voltage: b.voltage,
            current: b.current,
        })
    };

    // Input loss with the output down = the DC supply was disconnected or
    // sagged. Benign and self-clearing, so the dashboard shows it as a status
    // rather than a fault. Read back off `output` rather than recomputed from
    // the raw status, so it cannot disagree with what the supervisor is handed
    // for the same poll.
    let ps_offline = matches!(output, Some(BuckOutput::Off { cause }) if cause.is_input_loss());

    PollOutcome {
        poll: PollResult {
            setpoints: status.as_ref().map(|s| s.setpoints),
            output,
            battery,
            // No sensor on this board — see `PACK_TEMP`.
            pack_temp_c: None,
        },
        ps_offline,
    }
}

fn apply_action<D: XyDevice>(
    xy: &mut D,
    supervisor: &mut ChargeSupervisor,
    action: Action,
    recorder: &EventRecorder,
) {
    match action {
        Action::None => {}
        Action::EnableOutput(ticket) => {
            info!(
                "supervisor enabling output (resume_absorb={})",
                ticket.resume_absorb()
            );
            match xy.set_output(true) {
                // Dropping the ticket instead is what makes a failed
                // write a retry: the supervisor stays in bring-up and
                // re-emits EnableOutput next tick.
                Ok(()) => supervisor.commit_enable(ticket),
                Err(e) => {
                    warn!("XY set_output(true): {e} — supervisor stays in bring-up, will retry");
                    recorder.record(Event::Xy(XyError::SetOutput));
                }
            }
        }
        Action::UpdateVoltage(ticket) => {
            let outcome = apply_update_voltage(
                xy,
                &ticket,
                || thread::sleep(STEP_DOWN_SETTLE),
                |err| recorder.record(Event::Xy(err)),
            );
            if outcome == VoltageWriteOutcome::Committed {
                supervisor.commit_voltage(ticket);
            }
        }
        Action::DisableOutput(ticket) => {
            let reason = ticket.reason();
            match xy.set_output(false) {
                Ok(()) => {
                    // The latch itself is already in the event log as
                    // `ChargeTransition::Latched`, recorded when the
                    // supervisor tripped rather than when the write landed.
                    error!("CHARGE FAULT ({reason}): PS output DISABLED");
                    supervisor.commit_disable(ticket);
                }
                Err(e) => {
                    error!("CHARGE FAULT ({reason}): set_output(false) failed: {e} — will retry");
                    recorder.record(Event::Xy(XyError::SetOutput));
                }
            }
        }
    }
}

#[cfg(not(feature = "xy-fake"))]
fn make_device(pins: XyPins) -> real::Xy<'static> {
    real::Xy::new(pins)
}

#[cfg(feature = "xy-fake")]
fn make_device(pins: XyPins) -> fake::Xy<'static> {
    log::info!("XY: fake mode — claiming UART but not driving it");
    fake::Xy::new(pins)
}

pub fn start(
    pins: XyPins,
    sensor_data: Arc<Mutex<SensorData>>,
    charge_status: Arc<Mutex<ChargeStatus>>,
    recorder: EventRecorder,
) {
    thread::Builder::new()
        .name("xy".into())
        .stack_size(8192)
        .spawn(move || run(make_device(pins), sensor_data, charge_status, recorder))
        .unwrap();
}
