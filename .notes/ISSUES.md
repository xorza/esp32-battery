# Open issues

- Protection holds are unbounded: `ChargeEvent::SelfDisabled` and
  `ChargeEvent::SelfEnabled` can alternate indefinitely. An input rail that
  sags under charge current gives a repeating LVP hold/resume loop with no
  flap limit and no back-off on `regulation_a`.

- No pack temperature reaches the supervisor. Charging below 0 °C is not
  inhibited for any chemistry. The buck's OTP covers its own die only.

- Nothing bounds pack discharge. The buck's LVP register is input-side UVLO,
  so after a latch or during a protection hold the pack supplies the load with
  no firmware-side cutoff.
