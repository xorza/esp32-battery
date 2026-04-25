board boots:
if no creds:
    start AP+STA mixed
    wait for user enters captive portal creds
    show progress spinner, try to connect sta, dont drop ap
    if sta connected:
        confirm closing captive portal
        restart wifi STA only, save creds
    else if could not connect for 20 secs:
        stop spinner, notify user about wrong creds on captive


if creds:
    start STA only, connect to saved creds
    if could not connect for 2 hours:
        start AP+STA mixed
        continue to try to connect sta
        if sta connected restart wifi STA only


## States

- `Captive`     — radio AP+STA mixed, captive HTTP+DNS bundle up.
- `Sta`         — radio STA-only, dashboard server up.

## Transitions

- boot, no creds          → `Captive`
- boot, creds              → `Sta`
- `Captive`, sta associated → save creds, `Sta`   (radio restart STA-only)
- `Sta`, ≥ 2h not associated → `Captive`           (radio restart Mixed)

## Submission sub-state (only meaningful in `Captive`)

`Idle` → `Pending` (`/save` parked creds) → `Trying` (supervisor applied via
`set_sta_creds_live`, no radio restart) → `Failed` (20s timeout) or bundle
dropped on association success.
