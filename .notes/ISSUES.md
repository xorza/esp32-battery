# Open issues

- A `ChargeState::Parked` supervisor latches when the buck self-disables on a
  self-clearing cause, rather than holding and resuming the park when the
  cause lifts. The load stays on the pack once the rail returns.
