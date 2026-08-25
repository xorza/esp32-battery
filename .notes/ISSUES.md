# Open issues

- A `ChargeState::Parked` supervisor latches when the buck self-disables on a
  self-clearing cause, rather than holding and resuming the park when the
  cause lifts. The load stays on the pack once the rail returns.

- No pack temperature sensor is fitted. The supervisor's charge-window check
  is written and tested, but `PACK_TEMP` is `Absent`, so nothing stops this
  board charging a frozen pack. Needs an I2C part alongside the INA228 and a
  one-line flip to `Fitted`.

- A pack that leaves the charge window while the buck is sourcing latches, so
  a pack cooling below 0 °C overnight needs a manual reboot in the morning.
  The condition is self-clearing and wants a hold, but no hold state exists
  for a supervisor-detected condition — only for buck-reported protections.
