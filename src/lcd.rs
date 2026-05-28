use core::fmt::Write;
use std::net::Ipv4Addr;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use esp32_battery_logic::data::{Ina228Reading, PsReading, SensorData};
use esp32_battery_logic::error_log::EventLog;

use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::geometry::{OriginDimensions, Point, Size};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::ascii::{FONT_5X8, FONT_6X10, FONT_10X20};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::text::Text;
use esp_idf_hal::gpio::{AnyIOPin, PinDriver};
use esp_idf_hal::ledc::{LedcDriver, LedcTimerDriver, config::TimerConfig};
use esp_idf_hal::spi::config::{Config as SpiConfig, DriverConfig};
use esp_idf_hal::spi::{Dma, SpiDeviceDriver, SpiDriver};
use esp_idf_hal::units::FromValueType;
use mipidsi::Builder;
use mipidsi::interface::SpiInterface;
use mipidsi::models::ST7789;
use mipidsi::options::{Orientation, Rotation};

use crate::board::LcdPins;
use crate::net::{NetStatus, NetStatusHandle};

// === Hardware ======================================================

const SPI_BUF_SIZE: usize = 32768;
const DMA_BUF_SIZE: usize = 32768;
const REFRESH_INTERVAL: Duration = Duration::from_millis(500);
const BACKLIGHT_PERCENT: u32 = 50;

// === Layout ========================================================
//
// 320×172 panel, landscape via Rotation::Deg270.
//
// Upper region (y < LOWER_Y): 2×2 grid of 150×22 value cells with
// FONT_5X8 labels above each, plus an uptime corner.
//
// Lower region (y ≥ LOWER_Y): up to four non-overlapping 22-tall bands.
// Each "kv row" packs label + value into the same band (FONT_10X20
// monospace) so values line up across rows. Stacking a separate
// label band above a value band would clip the value's ascenders or
// push the layout past 172 px.

const DISPLAY_W: u32 = 320;
const DISPLAY_H: u32 = 172;
const LOWER_Y: i32 = 68;

// Upper grid.
const COL_LEFT: i32 = 5;
const COL_RIGHT: i32 = 165;
const ROW1_TOP: i32 = 14;
const ROW2_TOP: i32 = 46;
const ROW1_LABEL_Y: i32 = 10;
const ROW2_LABEL_Y: i32 = 44;
const LABEL_OFFSET_X: i32 = 8;
const VALUE_W: u32 = 150;
const VALUE_H: u32 = 22;

// Uptime corner.
const UPTIME_X: i32 = 240;
const UPTIME_W: u32 = 80;
const UPTIME_H: u32 = 12;

// Lower bands (non-overlapping, each 22 tall).
const LOWER_LEFT_X: i32 = 20;
const LOWER_TITLE_TOP: i32 = 70;
const LOWER_ROW1_TOP: i32 = 96;
const LOWER_ROW2_TOP: i32 = 120;
const LOWER_ROW3_TOP: i32 = 144;
// Label up to 4 chars (FONT_10X20 → 40 px) + 10 px gap; values align here.
const LOWER_VALUE_X: i32 = LOWER_LEFT_X + 4 * 10 + 10;

// === Colors ========================================================

const COLOR_BG: Rgb565 = Rgb565::BLACK;
const COLOR_LABEL: Rgb565 = Rgb565::new(18, 36, 18);
const COLOR_VOLTAGE: Rgb565 = Rgb565::new(0, 63, 0);
const COLOR_PSU_CURRENT: Rgb565 = Rgb565::new(31, 38, 0); // orange, matches #ff9800
const COLOR_POWER: Rgb565 = Rgb565::new(31, 20, 0);
const COLOR_CHARGING: Rgb565 = Rgb565::new(6, 55, 10);
const COLOR_DISCHARGING: Rgb565 = Rgb565::new(31, 28, 0);
const COLOR_IDLE: Rgb565 = Rgb565::new(18, 36, 18);
const COLOR_IP: Rgb565 = Rgb565::new(0, 57, 31);
const COLOR_WARNING: Rgb565 = Rgb565::RED;

/// Sign convention: negative current = charging, positive = discharging.
const BATTERY_IDLE_THRESHOLD_A: f32 = 0.05;

fn battery_current_color(current: f32) -> Rgb565 {
    if current.abs() < BATTERY_IDLE_THRESHOLD_A {
        COLOR_IDLE
    } else if current < 0.0 {
        COLOR_CHARGING
    } else {
        COLOR_DISCHARGING
    }
}

// === Scratch framebuffer ===========================================
//
// Single 320×22 buffer (~14 KB BSS) used for flicker-free composition
// of one row at a time. Baseline at y=16 leaves a FONT_10X20 line's
// ascenders/descenders inside the band.

const SCRATCH_W: u32 = DISPLAY_W;
const SCRATCH_H: u32 = 22;
const SCRATCH_PX: usize = (SCRATCH_W * SCRATCH_H) as usize;
const BAND_BASELINE: i32 = 16;

struct Scratch {
    pixels: &'static mut [Rgb565],
}

impl Scratch {
    fn from_buf(pixels: &'static mut [Rgb565]) -> Self {
        assert_eq!(pixels.len(), SCRATCH_PX);
        pixels.fill(COLOR_BG);
        Self { pixels }
    }

    fn clear(&mut self) {
        self.pixels.fill(COLOR_BG);
    }

    fn blit<D>(&self, display: &mut D, at: Point, w: u32, h: u32)
    where
        D: DrawTarget<Color = Rgb565>,
        D::Error: core::fmt::Debug,
    {
        let area = Rectangle::new(at, Size::new(w, h));
        let row_w = SCRATCH_W as usize;
        let take_w = w as usize;
        let pixels = self
            .pixels
            .chunks(row_w)
            .take(h as usize)
            .flat_map(|row| row[..take_w].iter().copied());
        display.fill_contiguous(&area, pixels).unwrap();
    }
}

impl OriginDimensions for Scratch {
    fn size(&self) -> Size {
        Size::new(SCRATCH_W, SCRATCH_H)
    }
}

impl DrawTarget for Scratch {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Rgb565>>,
    {
        for Pixel(p, c) in pixels {
            if p.x >= 0 && p.x < SCRATCH_W as i32 && p.y >= 0 && p.y < SCRATCH_H as i32 {
                self.pixels[p.y as usize * SCRATCH_W as usize + p.x as usize] = c;
            }
        }
        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Rgb565) -> Result<(), Self::Error> {
        let area = area.intersection(&Rectangle::new(Point::zero(), self.size()));
        if let Some(br) = area.bottom_right() {
            let x0 = area.top_left.x as usize;
            let w = (br.x - area.top_left.x + 1) as usize;
            for y in area.top_left.y..=br.y {
                let start = y as usize * SCRATCH_W as usize + x0;
                self.pixels[start..start + w].fill(color);
            }
        }
        Ok(())
    }
}

/// Compose into `scratch` via `f`, then blit a full-width band at `top`.
/// Free function (not a method on `Ui`) so callers can borrow the closure
/// arg disjointly from other `Ui` fields.
fn render_band<D>(scratch: &mut Scratch, display: &mut D, top: i32, f: impl FnOnce(&mut Scratch))
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    scratch.clear();
    f(scratch);
    scratch.blit(display, Point::new(0, top), SCRATCH_W, SCRATCH_H);
}

// === Lower-region state ============================================
//
// One value per visually distinct lower-region layout. The UI redraws
// the lower region only when this changes, so `Eq` collapses the
// previous three-field comparison (status / ip / has_errors) into one.

#[derive(Clone, PartialEq, Eq)]
enum LowerKey {
    Captive {
        trying: bool,
    },
    Connecting,
    Host {
        ip: Option<Ipv4Addr>,
        has_errors: bool,
        ps_offline: bool,
    },
}

impl LowerKey {
    fn from_inputs(
        net: NetStatus,
        ip: Option<Ipv4Addr>,
        has_errors: bool,
        ps_offline: bool,
    ) -> Self {
        match net {
            NetStatus::Captive => Self::Captive { trying: false },
            NetStatus::CaptiveTrying => Self::Captive { trying: true },
            NetStatus::Connecting => Self::Connecting,
            NetStatus::Host => Self::Host {
                ip,
                has_errors,
                ps_offline,
            },
        }
    }
}

// === UI ============================================================

struct Ui<D> {
    display: D,
    scratch: Scratch,
    last_lower: Option<LowerKey>,
}

impl<D> Ui<D>
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    fn new(display: D, scratch: Scratch) -> Self {
        Self {
            display,
            scratch,
            last_lower: None,
        }
    }

    /// Paint upper-region static labels once; they never move and the
    /// per-tick cell repaint preserves the band above each value.
    fn draw_static_labels(&mut self) {
        let style = MonoTextStyle::new(&FONT_5X8, COLOR_LABEL);
        for &(text, pos) in &[
            (
                "VOLTAGE",
                Point::new(COL_LEFT + LABEL_OFFSET_X, ROW1_LABEL_Y),
            ),
            (
                "BATTERY",
                Point::new(COL_LEFT + LABEL_OFFSET_X, ROW2_LABEL_Y),
            ),
            (
                "POWER",
                Point::new(COL_RIGHT + LABEL_OFFSET_X, ROW1_LABEL_Y),
            ),
            ("PSU", Point::new(COL_RIGHT + LABEL_OFFSET_X, ROW2_LABEL_Y)),
        ] {
            Text::new(text, pos, style).draw(&mut self.display).unwrap();
        }
    }

    /// 150×22 upper-region cell; text composed from `args`.
    fn cell(&mut self, top: Point, color: Rgb565, args: core::fmt::Arguments<'_>) {
        let mut buf = heapless::String::<32>::new();
        let _ = buf.write_fmt(args);
        self.scratch.clear();
        Text::new(
            &buf,
            Point::new(0, BAND_BASELINE),
            MonoTextStyle::new(&FONT_10X20, color),
        )
        .draw(&mut self.scratch)
        .unwrap();
        self.scratch.blit(&mut self.display, top, VALUE_W, VALUE_H);
    }

    /// Lower-region title band: title at the left, optional small badge
    /// (e.g. "Connecting...", "ERR!") flush to the right.
    fn title_row(&mut self, top: i32, title: &str, color: Rgb565, badge: Option<(&str, Rgb565)>) {
        render_band(&mut self.scratch, &mut self.display, top, |s| {
            Text::new(
                title,
                Point::new(LOWER_LEFT_X, BAND_BASELINE),
                MonoTextStyle::new(&FONT_10X20, color),
            )
            .draw(s)
            .unwrap();
            if let Some((b, c)) = badge {
                // FONT_10X20 advances 10 px/char; pad the right margin generously so
                // the panel's column-offset quirk doesn't clip the last glyph.
                let w = (b.len() as i32) * 10;
                Text::new(
                    b,
                    Point::new(SCRATCH_W as i32 - w - 14, BAND_BASELINE),
                    MonoTextStyle::new(&FONT_10X20, c),
                )
                .draw(s)
                .unwrap();
            }
        });
    }

    /// Lower-region label+value band. Both texts are FONT_10X20 so
    /// ascenders/descenders share a baseline; label is dim, value uses
    /// `value_color`. Values across rows align at `LOWER_VALUE_X`.
    fn kv_row(&mut self, top: i32, label: &str, value: &str, value_color: Rgb565) {
        render_band(&mut self.scratch, &mut self.display, top, |s| {
            Text::new(
                label,
                Point::new(LOWER_LEFT_X, BAND_BASELINE),
                MonoTextStyle::new(&FONT_10X20, COLOR_LABEL),
            )
            .draw(s)
            .unwrap();
            Text::new(
                value,
                Point::new(LOWER_VALUE_X, BAND_BASELINE),
                MonoTextStyle::new(&FONT_10X20, value_color),
            )
            .draw(s)
            .unwrap();
        });
    }

    fn clear_lower(&mut self) {
        let area = Rectangle::new(
            Point::new(0, LOWER_Y),
            Size::new(DISPLAY_W, DISPLAY_H - LOWER_Y as u32),
        );
        self.display.fill_solid(&area, COLOR_BG).unwrap();
    }

    /// Upper region — four values + uptime. Repainted every tick.
    fn draw_upper(&mut self, bat: Ina228Reading, ps: PsReading, uptime: Duration) {
        self.cell(
            Point::new(COL_LEFT, ROW1_TOP),
            COLOR_VOLTAGE,
            format_args!("{:.2} V", bat.voltage),
        );
        self.cell(
            Point::new(COL_RIGHT, ROW1_TOP),
            COLOR_POWER,
            format_args!("{:.2} W", bat.power),
        );
        // Battery: sign conveyed by color, magnitude only in text.
        self.cell(
            Point::new(COL_LEFT, ROW2_TOP),
            battery_current_color(bat.current),
            format_args!("{:.3} A", bat.current.abs()),
        );
        self.cell(
            Point::new(COL_RIGHT, ROW2_TOP),
            COLOR_PSU_CURRENT,
            format_args!("{:.3} A", ps.current),
        );

        let mut buf = heapless::String::<16>::new();
        let secs = uptime.as_secs();
        let _ = write!(
            buf,
            "{}h {:02}m {:02}s",
            secs / 3600,
            (secs % 3600) / 60,
            secs % 60
        );
        self.scratch.clear();
        Text::new(
            &buf,
            Point::new(0, 8),
            MonoTextStyle::new(&FONT_6X10, COLOR_LABEL),
        )
        .draw(&mut self.scratch)
        .unwrap();
        self.scratch.blit(
            &mut self.display,
            Point::new(UPTIME_X, 0),
            UPTIME_W,
            UPTIME_H,
        );
    }

    /// Lower region — repainted only when the visible state changes.
    fn draw_lower(&mut self, key: LowerKey) {
        if self.last_lower.as_ref() == Some(&key) {
            return;
        }
        self.clear_lower();
        match &key {
            LowerKey::Captive { trying } => self.draw_captive(*trying),
            LowerKey::Connecting => self.draw_connecting(),
            LowerKey::Host {
                ip,
                has_errors,
                ps_offline,
            } => self.draw_host(*ip, *has_errors, *ps_offline),
        }
        self.last_lower = Some(key);
    }

    fn draw_captive(&mut self, trying: bool) {
        let badge = trying.then_some(("Connecting...", COLOR_DISCHARGING));
        self.title_row(LOWER_TITLE_TOP, "WiFi Setup", Rgb565::WHITE, badge);
        self.kv_row(LOWER_ROW1_TOP, "SSID", crate::wifi::AP_SSID, COLOR_VOLTAGE);
        self.kv_row(LOWER_ROW2_TOP, "PASS", crate::wifi::AP_PASS, COLOR_VOLTAGE);

        let [a, b, c, d] = crate::wifi::AP_GATEWAY;
        let mut buf = heapless::String::<32>::new();
        let _ = write!(buf, "http://{a}.{b}.{c}.{d}/");
        self.kv_row(LOWER_ROW3_TOP, "URL", &buf, COLOR_VOLTAGE);
    }

    fn draw_connecting(&mut self) {
        self.title_row(LOWER_ROW2_TOP, "Connecting...", Rgb565::WHITE, None);
    }

    fn draw_host(&mut self, ip: Option<Ipv4Addr>, has_errors: bool, ps_offline: bool) {
        // PS-offline (benign, self-clearing) and errors are independent —
        // show whichever are active, both if need be. Error color wins when
        // combined since it's the more severe of the two.
        let badge = match (ps_offline, has_errors) {
            (true, true) => Some(("PS OFF ERR!", COLOR_WARNING)),
            (true, false) => Some(("PS OFFLINE", COLOR_DISCHARGING)),
            (false, true) => Some(("ERR!", COLOR_WARNING)),
            (false, false) => None,
        };
        self.title_row(LOWER_TITLE_TOP, "Connected", Rgb565::WHITE, badge);

        let mut buf = heapless::String::<32>::new();
        match ip {
            Some(addr) => {
                let _ = write!(buf, "{addr}");
            }
            None => {
                let _ = write!(buf, "--");
            }
        }
        self.kv_row(LOWER_ROW1_TOP, "IP", &buf, COLOR_IP);

        buf.clear();
        let _ = write!(buf, "https://{}.local", crate::wifi::HOSTNAME);
        self.kv_row(LOWER_ROW2_TOP, "URL", &buf, COLOR_IP);

        buf.clear();
        let _ = write!(buf, "{}", crate::PACK_PROFILE);
        self.kv_row(LOWER_ROW3_TOP, "PACK", &buf, COLOR_VOLTAGE);
    }
}

// === Thread entry point ============================================

pub fn start(
    pins: LcdPins,
    sensor_data: Arc<Mutex<SensorData>>,
    event_log: Arc<Mutex<EventLog>>,
    status: NetStatusHandle,
) {
    thread::Builder::new()
        .stack_size(16384)
        .spawn(move || run(pins, sensor_data, event_log, status))
        .unwrap();
}

fn run(
    pins: LcdPins,
    sensor_data: Arc<Mutex<SensorData>>,
    event_log: Arc<Mutex<EventLog>>,
    status: NetStatusHandle,
) {
    // BSS-resident backing storage; this thread is spawned exactly once
    // and these statics are not referenced anywhere else.
    static mut SPI_BUF: [u8; SPI_BUF_SIZE] = [0; SPI_BUF_SIZE];
    static mut SCRATCH_PIXELS: [Rgb565; SCRATCH_PX] = [COLOR_BG; SCRATCH_PX];
    // `&raw mut` is a raw pointer; clippy's `deref_addrof` misfires here.
    #[allow(clippy::deref_addrof)]
    let spi_buf: &'static mut [u8] = unsafe { &mut *&raw mut SPI_BUF };
    #[allow(clippy::deref_addrof)]
    let scratch_pixels: &'static mut [Rgb565] = unsafe { &mut *&raw mut SCRATCH_PIXELS };

    let timer = LedcTimerDriver::new(
        pins.ledc_timer,
        &TimerConfig::default().frequency(1.kHz().into()),
    )
    .unwrap();
    let mut blk = LedcDriver::new(pins.ledc_channel, timer, pins.blk).unwrap();
    let max_duty = blk.get_max_duty();
    blk.set_duty(max_duty * BACKLIGHT_PERCENT / 100).unwrap();

    let spi_driver = SpiDriver::new(
        pins.spi,
        pins.sclk,
        pins.mosi,
        None::<AnyIOPin>,
        &DriverConfig::new().dma(Dma::Auto(DMA_BUF_SIZE)),
    )
    .unwrap();
    let spi_device = SpiDeviceDriver::new(
        &spi_driver,
        Some(pins.cs),
        &SpiConfig::default().baudrate(40.MHz().into()),
    )
    .unwrap();
    let dc = PinDriver::output(pins.dc).unwrap();
    let rst = PinDriver::output(pins.rst).unwrap();
    let spi_iface = SpiInterface::new(spi_device, dc, spi_buf);

    let mut display = Builder::new(ST7789, spi_iface)
        .reset_pin(rst)
        .orientation(Orientation::new().rotate(Rotation::Deg270))
        .invert_colors(mipidsi::options::ColorInversion::Inverted)
        .display_size(172, 320)
        .display_offset(34, 0)
        .init(&mut esp_idf_hal::delay::Ets)
        .unwrap();
    display.clear(COLOR_BG).unwrap();

    let mut ui = Ui::new(display, Scratch::from_buf(scratch_pixels));
    ui.draw_static_labels();

    loop {
        thread::sleep(REFRESH_INTERVAL);

        let net_status = status.load();
        let ip = (net_status == NetStatus::Host)
            .then(crate::wifi::sta_ip)
            .flatten();
        let has_errors = !event_log.lock().unwrap().is_empty();

        let (bat, ps, ps_offline) = {
            let sd = sensor_data.lock().unwrap();
            (
                sd.battery_reading().unwrap_or_default(),
                sd.ps_reading().unwrap_or_default(),
                sd.ps_offline,
            )
        };

        ui.draw_upper(bat, ps, crate::clock::uptime());
        ui.draw_lower(LowerKey::from_inputs(
            net_status, ip, has_errors, ps_offline,
        ));
    }
}
