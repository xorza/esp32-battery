# Open issues

- No pack temperature reaches the supervisor. Charging below 0 °C is not
  inhibited for any chemistry. The buck's OTP covers its own die only.

- Nothing bounds pack discharge. The buck's LVP register is input-side UVLO,
  so after a latch or during a protection hold the pack supplies the load with
  no firmware-side cutoff.
