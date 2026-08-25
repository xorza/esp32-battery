//! Which constant-voltage setpoint the supervisor is holding.

use strum::IntoStaticStr;

#[derive(Copy, Clone, Debug, PartialEq, Eq, IntoStaticStr)]
#[strum(serialize_all = "lowercase")]
pub enum Phase {
    Float,
    Absorb,
}

impl Phase {
    pub fn label(self) -> &'static str {
        self.into()
    }
}
