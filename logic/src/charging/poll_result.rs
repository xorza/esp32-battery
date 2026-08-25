//! One poll cycle's view of the world, as handed to the supervisor.

use xy_modbus::{ProtectionStatus, Setpoints};

/// One poll cycle's view of the world for the supervisor.
/// `setpoints` is from the V_SET/I_SET readback; `setpoints.is_some()`
/// doubles as the modbus-healthy signal. `battery` is independent —
/// it's the latest fresh INA228 reading.
#[derive(Copy, Clone, Debug, Default)]
pub struct PollResult {
    pub battery: Option<BatterySample>,
    pub setpoints: Option<Setpoints>,
    /// `None` means the OUTPUT_EN read itself failed.
    pub output: Option<BuckOutput>,
}

/// What the buck's OUTPUT_EN register reported this poll, plus the
/// PROTECT (0x0010) cause when output is off. The two were separate
/// fields once but they covary: PROTECT is necessarily Normal while
/// output is on, and is read in the same bulk transaction as OUTPUT_EN,
/// so the relation belongs in the type. `cause: Normal` covers the
/// "output is off and the buck reports no protection cause" case
/// (e.g. fresh-off after boot, post-disable, panel toggle).
#[derive(Copy, Clone, Debug)]
pub enum BuckOutput {
    /// OUTPUT_EN reads 1.
    On,
    /// OUTPUT_EN reads 0; PROTECT register value carried inline.
    Off { cause: ProtectionStatus },
}

/// Latest fresh battery reading fed to the supervisor. Voltage is used for
/// OV detection, current drives the phase machine. Power isn't needed.
#[derive(Copy, Clone, Debug)]
pub struct BatterySample {
    pub voltage: f32,
    pub current: f32,
}

impl BatterySample {
    /// Charging current as a positive number.
    ///
    /// The INA228 wiring on this board reads battery current **negative
    /// while charging**, so every threshold in `charging` is stated
    /// positive and compared against this. Named rather than left as a
    /// unary minus at each site: a dropped sign here reads a discharging
    /// pack as one taking charge.
    pub(super) fn charging_a(self) -> f32 {
        -self.current
    }
}
