# xy-modbus

Modbus-RTU driver for the XY-series programmable buck converters
(XY7025, XY6020L, XY6015, XY-SK60, XY-SK120, XY-SK120X). These modules
share a common register layout — the differences between models are
mechanical (max V/A/W), not protocol.

`no_std`, no dependencies. Bring your own UART transport.

## Usage

```rust,ignore
use xy_modbus::{Xy, SafetyLimits};

let mut xy = Xy::new(my_transport);

xy.set_protection(SafetyLimits {
    lvp_v: 22.0,
    ovp_v: 15.0,
    ocp_a: 15.0,
})?;
xy.set_voltage(13.5)?;
xy.set_current_limit(10.0)?;
xy.set_output(true)?;

let s = xy.read_status()?;
println!("{:.2} V @ {:.2} A", s.v_out, s.i_out);
```

## Bringing your own transport

Implement [`ModbusTransport`] over your platform's UART. The
[`framing`] module exposes the on-wire codec (`build_*` /
`parse_*` / `crc16_modbus`), so an implementation is typically
under 100 lines:

```rust,ignore
use xy_modbus::{ModbusTransport, RtuError, framing};

struct MyTransport { /* uart handle, timing config */ }

impl ModbusTransport for MyTransport {
    fn read_holding(&mut self, slave: u8, addr: u16, dst: &mut [u16])
        -> Result<(), RtuError>
    {
        let req = framing::build_read_request(slave, addr, dst.len() as u16);
        let mut buf = [0u8; framing::MAX_ADU];
        let n = self.transact(&req, &mut buf)?;
        framing::parse_read_response(&buf[..n], slave, dst)?;
        Ok(())
    }
    // write_single_holding, write_multiple_holdings analogous
    # fn write_single_holding(&mut self, _: u8, _: u16, _: u16) -> Result<(), RtuError> { unimplemented!() }
    # fn write_multiple_holdings(&mut self, _: u8, _: u16, _: &[u16]) -> Result<(), RtuError> { unimplemented!() }
}
```

The transport implementer is responsible for UART timing — the
inter-frame gap, response timeout, and post-write quiet gap. The
XY-series wants ~50 ms between frames and ~500 ms response window;
see [`DATASHEET.md`](DATASHEET.md) §2 for empirical values.

## What's in the API

- **Live readings** — `read_status`, `read_setpoints`,
  `read_voltage_out` / `_in`, `read_current_out`, `read_power_out`,
  `read_temperatures`, `read_totals`
- **Setpoints** — `set_voltage`, `set_current_limit`,
  `set_protection` / `read_protection`, `set_power_on_output`
- **Output control** — `set_output` / `read_output`,
  `read_protection_status` / `clear_protection_status`,
  `read_reg_mode`
- **Front panel & misc** — `read/set_lock`, `read/set_backlight`,
  `read/set_sleep_minutes`, `read/set_buzzer`,
  `read/set_temp_unit`, `read/set_temp_offset_*`
- **Identity & comms** — `read_model`, `read_version`,
  `read/set_slave_address`, `read/set_baud_rate`
- **Memory groups (M0–M9)** — `read_group(n)`, `write_group(n, &p)`,
  `recall_group(n)`

WiFi-pairing block (registers 0x0030–0x0034) is documented in the
datasheet but not yet exposed at the high-level API.

## Boot / safety policy

This crate exposes the device protocol; it intentionally does **not**
prescribe a power-on / fault-recovery policy. See [`DATASHEET.md`](DATASHEET.md)
§7 for the recommended bring-up checklist (program protection
*before* raising V-SET, force OUTPUT_EN off until verification
passes, etc.) — translate that into the routine that fits your
application.

## Protocol reference

See [`DATASHEET.md`](DATASHEET.md) for the full register map, CRC
algorithm, wire-level examples, and known firmware quirks.

## License

MIT OR Apache-2.0.
