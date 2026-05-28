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

use esp32_battery_logic::charging::{
    Action, BatterySample, BuckOutput, ChargeSupervisor, PollResult,
};
use esp32_battery_logic::data::{PsReading, SensorData};
use esp32_battery_logic::error_log::{Event, XyError};

use xy_modbus::{Model, ModelCheck, ProtectionStatus, RtuError, SafetyLimits, Setpoints, Status};

use crate::board::XyPins;
use crate::clock::EventRecorder;
use crate::reboot;
use crate::task_wdt;
use crate::{PACK_PROFILE, SAFETY};

const POLL_INTERVAL: Duration = Duration::from_millis(1000);

/// Quiet window between disabling output and writing the new V_SET on a
/// step-down. Lets the inductor current decay and the buck's internal
/// regulator state quiesce so the next register write lands in a clean
/// state. Modbus latency usually covers this on its own (~50 ms
/// inter-frame + ~500 ms response timeout), but the explicit sleep
/// removes the dependence on transport timing.
const STEP_DOWN_SETTLE: Duration = Duration::from_millis(100);

/// Buck variant on this board. Sets the per-register scales (I-OUT,
/// POWER, S-OCP, S-OPP) — wrong family silently shifts readings by 10×,
/// so this also drives the boot-time `verify_model` check and the fake
/// device's mock MODEL response.
const PACK_MODEL: Model = Model::Xy7025;

/// The set of operations the charging loop needs from the buck. Real
/// builds get the `xy_modbus`-backed implementation; `xy-fake` builds
/// get an in-memory canned device. The thread loop is identical.
trait XyDevice {
    fn verify_model(&mut self) -> Result<ModelCheck, RtuError>;
    /// Live + control snapshot (regs 0x0000–0x0012). One Modbus
    /// round-trip per supervisor tick.
    fn read_status(&mut self) -> Result<Status, RtuError>;
    fn read_protection(&mut self) -> Result<SafetyLimits, RtuError>;
    fn set_voltage(&mut self, volts: f32) -> Result<(), RtuError>;
    fn set_current_limit(&mut self, amps: f32) -> Result<(), RtuError>;
    fn set_protection(&mut self, limits: SafetyLimits) -> Result<(), RtuError>;
    /// Write 0 to PROTECT (0x0010) to clear a latched protection cause.
    fn clear_protection_status(&mut self) -> Result<(), RtuError>;
    fn set_output(&mut self, on: bool) -> Result<(), RtuError>;
    fn set_power_on_default_off(&mut self) -> Result<(), RtuError>;
}

/// Boot-time failure: either the Modbus transport gave up, or a register
/// read back a different value than we wrote (wrong slave, scale mismatch,
/// write rejected, etc.). Either way, we must not enable output.
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
    /// Device's MODEL register reports a code mapped to a different
    /// scale family than the configured `Model`. Readings (especially
    /// I-OUT) would be off by 10×; refuse to proceed.
    ModelMismatch {
        expected_code: u16,
        device_code: u16,
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
            Self::OutputOn => f.write_str("OUTPUT_EN read back ON after disable"),
            Self::ModelMismatch {
                expected_code,
                device_code,
            } => write!(
                f,
                "MODEL mismatch: configured family expects 0x{expected_code:04X}, device reports 0x{device_code:04X}"
            ),
        }
    }
}

// --- Real device ------------------------------------------------------------

#[cfg(not(feature = "xy-fake"))]
mod real {
    use esp_idf_hal::uart::{UartDriver, config::Config};
    use esp_idf_hal::units::Hertz;

    use xy_modbus::esp_idf::EspIdfTransport;
    use xy_modbus::{ModelCheck, RtuError, SafetyLimits, Status};

    use super::{PACK_MODEL, XyDevice};
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
            Self(xy_modbus::Xy::from_esp_uart(uart, PACK_MODEL))
        }
    }

    impl XyDevice for Xy<'_> {
        fn verify_model(&mut self) -> Result<ModelCheck, RtuError> {
            self.0.verify_model()
        }

        fn read_status(&mut self) -> Result<Status, RtuError> {
            self.0.read_status()
        }

        fn read_protection(&mut self) -> Result<SafetyLimits, RtuError> {
            self.0.read_protection()
        }

        fn set_voltage(&mut self, volts: f32) -> Result<(), RtuError> {
            self.0.set_voltage(volts)
        }

        fn set_current_limit(&mut self, amps: f32) -> Result<(), RtuError> {
            self.0.set_current_limit(amps)
        }

        fn set_protection(&mut self, limits: SafetyLimits) -> Result<(), RtuError> {
            self.0.set_protection(limits)
        }

        fn clear_protection_status(&mut self) -> Result<(), RtuError> {
            self.0.clear_protection_status()
        }

        fn set_output(&mut self, on: bool) -> Result<(), RtuError> {
            self.0.set_output(on)
        }

        fn set_power_on_default_off(&mut self) -> Result<(), RtuError> {
            self.0.set_power_on_output(false)
        }
    }
}

// --- Fake device ------------------------------------------------------------

#[cfg(feature = "xy-fake")]
mod fake {
    use esp_idf_hal::uart::{UartDriver, config::Config};
    use esp_idf_hal::units::Hertz;

    use xy_modbus::{ModelCheck, ProtectionStatus, RegMode, RtuError, SafetyLimits, Status};

    use super::{PACK_MODEL, XyDevice};
    use crate::board::XyPins;

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

    impl XyDevice for Xy<'_> {
        fn verify_model(&mut self) -> Result<ModelCheck, RtuError> {
            // Fake mirrors what a correctly-wired device would report:
            // the family code that matches `PACK_MODEL`. `expect`
            // rather than a fallback so switching `PACK_MODEL` to an
            // unpinned variant (SK family / Custom) fails loudly here
            // instead of silently masking the boot gate.
            Ok(ModelCheck::Match {
                device_code: PACK_MODEL
                    .expected_model_code()
                    .expect("PACK_MODEL must have a pinned family code"),
            })
        }

        fn read_status(&mut self) -> Result<Status, RtuError> {
            let v = if self.output_on { self.v_set } else { 0.0 };
            Ok(Status {
                v_set: self.v_set,
                i_set: self.i_set,
                v_out: v,
                i_out: 0.0,
                p_out: 0.0,
                v_in: 24.0,
                protection: self.protection_status,
                reg_mode: RegMode::ConstantVoltage,
                output_on: self.output_on,
            })
        }
        fn read_protection(&mut self) -> Result<SafetyLimits, RtuError> {
            Ok(self.protection)
        }
        fn set_voltage(&mut self, volts: f32) -> Result<(), RtuError> {
            self.v_set = volts;
            Ok(())
        }
        fn set_current_limit(&mut self, amps: f32) -> Result<(), RtuError> {
            self.i_set = amps;
            Ok(())
        }
        fn set_protection(&mut self, limits: SafetyLimits) -> Result<(), RtuError> {
            self.protection = limits;
            Ok(())
        }
        fn clear_protection_status(&mut self) -> Result<(), RtuError> {
            self.protection_status = ProtectionStatus::Normal;
            Ok(())
        }
        fn set_output(&mut self, on: bool) -> Result<(), RtuError> {
            self.output_on = on;
            Ok(())
        }
        fn set_power_on_default_off(&mut self) -> Result<(), RtuError> {
            Ok(())
        }
    }
}

// --- Shared thread loop -----------------------------------------------------

/// Programs protection + setpoints and reads them back to confirm the
/// device accepted the writes. Output stays OFF — bringing up the buck
/// is the supervisor's job, conditional on a fresh, drift-free, in-range
/// first tick. Readback catches dropped writes, scale mismatches, and
/// wrong-slave wiring before the supervisor can ask for output enable.
fn boot_sequence<D: XyDevice>(xy: &mut D) -> Result<u16, BootError> {
    // Confirm we're talking to the family we think we're talking to —
    // before any writes go out. Wrong-family configuration silently
    // corrupts every subsequent reading by 10×, so we refuse to proceed
    // on Mismatch. `Inconclusive` (SK family / Custom / undocumented
    // code) is allowed through; verification just isn't possible.
    let device_code = match xy.verify_model()? {
        ModelCheck::Mismatch {
            expected_code,
            device_code,
        } => {
            return Err(BootError::ModelMismatch {
                expected_code,
                device_code,
            });
        }
        ModelCheck::Match { device_code } | ModelCheck::Inconclusive { device_code } => device_code,
    };
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
    if s.output_on {
        return Err(BootError::OutputOn);
    }
    Ok(device_code)
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
const _: () = assert!(task_wdt::WDT_TIMEOUT.as_secs() > POLL_INTERVAL.as_secs() * 2);

/// Try `boot_sequence` up to `BOOT_RETRY_COUNT` times. On total failure,
/// best-effort disable output and reboot the MCU — the alternative is
/// falling through to a supervisor that immediately latches
/// `ModbusUnhealthy` and stays Tripped forever. A reboot might clear
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
fn run<D: XyDevice>(mut xy: D, sensor_data: Arc<Mutex<SensorData>>, recorder: EventRecorder) {
    // !Send token stays bound to this FreeRTOS task for its lifetime.
    let wdt = task_wdt::subscribe();

    let model_code = boot_with_retries(&mut xy, &recorder);
    sensor_data.lock().unwrap().model_code = model_code;
    let mut supervisor = ChargeSupervisor::new(PACK_PROFILE);

    // Tracks the last-seen PROTECT cause so a latch is logged to the event
    // log once per episode, not on every poll while it stays latched.
    let mut last_protection = ProtectionStatus::Normal;
    loop {
        wdt.reset();
        let p = poll(&mut xy, &sensor_data, &recorder, &mut last_protection);
        let action = supervisor.tick(p, POLL_INTERVAL);
        apply_action(&mut xy, &mut supervisor, action, &recorder);
        // Publish supervisor state for /api + LCD. `active_phase()` is
        // only Some while regulating; Pending/Tripped surface as None
        // so dashboards distinguish "not yet charging" from a real phase.
        {
            let mut sd = sensor_data.lock().unwrap();
            sd.charge_phase = supervisor.active_phase();
            sd.charge_fault = supervisor.fault();
        }
        thread::sleep(POLL_INTERVAL);
    }
}

/// One read cycle: poll the buck, push readings into shared sensor data,
/// snapshot the latest battery sample, and return the supervisor's
/// per-tick view of the world.
///
/// The Modbus reads run *without* the `SensorData` lock held — UART
/// transactions can take up to `response_timeout` (500 ms) and we don't
/// want HTTP / LCD / INA blocked on that. The mutex is only acquired in
/// short scopes around the actual writes/reads.
fn poll<D: XyDevice>(
    xy: &mut D,
    sensor_data: &Mutex<SensorData>,
    recorder: &EventRecorder,
    last_protection: &mut ProtectionStatus,
) -> PollResult {
    // Single bulk read covers V_SET..V_IN, PROTECT, CVCC, OUTPUT_EN —
    // one Modbus round-trip instead of three. PROTECT is necessarily
    // Normal while OUTPUT_EN is on; non-Normal here means the buck
    // self-disabled this session (boot_sequence wiped 0x0010).
    let (setpoints, output) = match xy.read_status() {
        Ok(s) => {
            // Input UVLO = the DC supply was disconnected / sagged. Treat it
            // as a benign, self-clearing "PS offline" status, not a fault.
            let ps_offline = !s.output_on && s.protection == ProtectionStatus::Lvp;
            {
                let mut sd = sensor_data.lock().unwrap();
                sd.update_ps(PsReading {
                    voltage: s.v_out,
                    current: s.i_out,
                    power: s.p_out,
                    v_set: s.v_set,
                    i_set: s.i_set,
                });
                sd.ps_offline = ps_offline;
            }
            let setpoints = Some(Setpoints {
                v_set: s.v_set,
                i_set: s.i_set,
            });
            let output = Some(if s.output_on {
                *last_protection = ProtectionStatus::Normal;
                BuckOutput::On
            } else {
                // LVP is surfaced as "PS offline" above and recovers on its
                // own — don't pollute the event log with it. Other causes
                // record once per latch episode (rising edge / cause change);
                // the warn! stays per-poll for log visibility.
                if s.protection != ProtectionStatus::Lvp
                    && let Some(ev) = XyError::from_protection(s.protection)
                {
                    warn!("XY PROTECT latched: {}", s.protection);
                    if *last_protection != s.protection {
                        recorder.record(Event::Xy(ev));
                    }
                }
                *last_protection = s.protection;
                BuckOutput::Off {
                    cause: Some(s.protection),
                }
            });
            (setpoints, output)
        }
        Err(e) => {
            warn!("XY read_status: {e}");
            recorder.record(Event::Xy(XyError::ReadStatus));
            (None, None)
        }
    };
    let battery = sensor_data
        .lock()
        .unwrap()
        .battery_reading()
        .map(|b| BatterySample {
            voltage: b.voltage,
            current: b.current,
        });
    PollResult {
        setpoints,
        output,
        battery,
    }
}

/// Write V_SET, optionally cycling output around the write for safety on
/// step-downs. Stepping V_SET down with output enabled drives reverse
/// current through the XY7025's synchronous low-side FET and can blow
/// it (and propagate upstream — neither port has anti-backup
/// protection). For step-ups the cycle is unnecessary; we write live.
///
/// Partial failure leaves the buck OFF and lets the supervisor latch
/// naturally on the next tick (SettingsDrift if V_SET committed but the
/// re-enable failed, OutputUnexpectedlyOff otherwise). A reboot is the
/// recovery path either way — the buck is safe, just not regulating.
fn apply_update_voltage<D: XyDevice>(
    xy: &mut D,
    supervisor: &mut ChargeSupervisor,
    target_v: f32,
    cycle_output: bool,
    recorder: &EventRecorder,
) {
    if !cycle_output {
        match xy.set_voltage(target_v) {
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
        }
        return;
    }

    info!("charge phase step-down → V_set = {target_v:.2} V (cycling output)");
    if let Err(e) = xy.set_output(false) {
        warn!("XY safe-step-down set_output(false): {e} — will retry next tick");
        recorder.record(Event::Xy(XyError::SetOutput));
        return;
    }
    // Let inductor current decay and the buck's internal state quiesce
    // before reprogramming V_SET. Modbus latency would usually cover
    // this, but an explicit sleep removes the dependence on timing.
    thread::sleep(STEP_DOWN_SETTLE);
    if let Err(e) = xy.set_voltage(target_v) {
        warn!("XY safe-step-down set_voltage({target_v}): {e} — buck stays OFF, will retry");
        recorder.record(Event::Xy(XyError::SetVoltage));
        return;
    }
    // V_SET is now on the buck — commit the phase so the next tick's
    // drift check sees matching setpoints. If the re-enable below fails,
    // the supervisor will latch on output-off, which is the safe outcome.
    supervisor.ack_voltage_update();
    if let Err(e) = xy.set_output(true) {
        error!("XY safe-step-down set_output(true): {e} — buck OFF after voltage commit");
        recorder.record(Event::Xy(XyError::SetOutput));
        return;
    }
    info!(
        "charge phase → {}: V_set = {target_v:.2} V (step-down complete)",
        supervisor.phase().label()
    );
}

fn apply_action<D: XyDevice>(
    xy: &mut D,
    supervisor: &mut ChargeSupervisor,
    action: Action,
    recorder: &EventRecorder,
) {
    match action {
        Action::None => {}
        Action::EnableOutput { resume_absorb } => {
            info!("supervisor enabling output (resume_absorb={resume_absorb})");
            match xy.set_output(true) {
                Ok(()) => supervisor.ack_enable(resume_absorb),
                Err(e) => {
                    warn!("XY set_output(true): {e} — supervisor stays Pending, will retry");
                    recorder.record(Event::Xy(XyError::SetOutput));
                }
            }
        }
        Action::UpdateVoltage {
            target_v,
            cycle_output,
        } => apply_update_voltage(xy, supervisor, target_v, cycle_output, recorder),
        Action::DisableOutput(reason) => match xy.set_output(false) {
            Ok(()) => {
                error!("CHARGE FAULT ({reason}): PS output DISABLED");
                // Record once per latch episode: ack_disable transitions
                // out of `Tripped { acked: false }`, so subsequent ticks
                // stop re-emitting DisableOutput for the same fault.
                recorder.record(Event::Xy(XyError::ChargeFaultLatched));
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
