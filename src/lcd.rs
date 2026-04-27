use core::fmt::Write;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use esp32_battery_logic::data::SensorData;
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

// --- SPI ---
const SPI_BUF_SIZE: usize = 32768;
const DMA_BUF_SIZE: usize = 32768;

// --- Display ---
const DISPLAY_W: u32 = 320;
const DISPLAY_H: u32 = 172;
const LOWER_Y: i32 = 68;
const REFRESH_INTERVAL: Duration = Duration::from_millis(500);
const BACKLIGHT_PERCENT: u32 = 50;

// --- Colors ---
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

// --- Layout (display-absolute pixels) ---
//
// Upper region (y < LOWER_Y): four 150×22 value cells in a 2×2 grid plus a
// small uptime corner. Static labels above each cell are painted once at boot
// directly to the display.
const COL_LEFT: i32 = 5;
const COL_RIGHT: i32 = 165;
const LABEL_OFFSET_X: i32 = 8;
const ROW1_LABEL_Y: i32 = 10;
const ROW2_LABEL_Y: i32 = 44;
const VALUE_W: u32 = 150;
const VALUE_H: u32 = 22;
const ROW1_TOP: i32 = 14;
const ROW2_TOP: i32 = 46;
const UPTIME_X: i32 = 240;
const UPTIME_W: u32 = 80;
const UPTIME_H: u32 = 12;

// --- Scratch buffer ---
//
// Single 320×22 framebuffer (≈14 KB BSS) used for everything that needs
// flicker-free composition: the upper value cells, the uptime corner, and
// each lower-region text row. Lower-region overlays are painted only on
// net-state change, so the per-row-blit cost is irrelevant.
const SCRATCH_W: u32 = DISPLAY_W;
const SCRATCH_H: u32 = 22;
const SCRATCH_PX: usize = (SCRATCH_W * SCRATCH_H) as usize;

// FONT_10X20 baseline within a 22-tall band — text drawn at this y lands
// flush against the band's bottom with 2 px of headroom above.
const BAND_BASELINE_LARGE: i32 = 16;
// FONT_6X10 baseline within the same band — keeps labels near the top so a
// label row blitted above its value row leaves visible spacing.
const BAND_BASELINE_SMALL: i32 = 10;

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

    /// Blit a `w × h` top-left subregion to the display.
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

// --- Drawing helpers ---

fn format_uptime(uptime: Duration, out: &mut heapless::String<16>) {
    out.clear();
    let secs = uptime.as_secs();
    let h = secs / 3600;
    let m = (secs % 3600) / 60;
    let s = secs % 60;
    let _ = write!(out, "{}h {:02}m {:02}s", h, m, s);
}

fn draw_value<D>(scratch: &mut Scratch, display: &mut D, text: &str, cell_top: Point, color: Rgb565)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    scratch.clear();
    Text::new(
        text,
        Point::new(0, BAND_BASELINE_LARGE),
        MonoTextStyle::new(&FONT_10X20, color),
    )
    .draw(scratch)
    .unwrap();
    scratch.blit(display, cell_top, VALUE_W, VALUE_H);
}

/// Render one text-row band: clear scratch, run `f` to paint, then blit
/// `(SCRATCH_W × h)` at `screen_top`. Bands may overlap vertically without
/// flicker because the overlapping rows are background-on-background.
fn render_band<D>(
    scratch: &mut Scratch,
    display: &mut D,
    screen_top: i32,
    h: u32,
    f: impl FnOnce(&mut Scratch),
) where
    D: DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    scratch.clear();
    f(scratch);
    scratch.blit(display, Point::new(0, screen_top), SCRATCH_W, h);
}

fn clear_lower<D>(display: &mut D)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    let area = Rectangle::new(
        Point::new(0, LOWER_Y),
        Size::new(DISPLAY_W, DISPLAY_H - LOWER_Y as u32),
    );
    display.fill_solid(&area, COLOR_BG).unwrap();
}

// --- Lower-region states ---
//
// Each line is a 22-tall band whose screen top is `baseline_y - baseline_in_band`.
// FONT_6X10 baselines use BAND_BASELINE_SMALL, FONT_10X20 uses BAND_BASELINE_LARGE.

fn draw_host<D>(
    scratch: &mut Scratch,
    display: &mut D,
    ip: Option<std::net::Ipv4Addr>,
    has_errors: bool,
    buf: &mut heapless::String<32>,
) where
    D: DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    let label = MonoTextStyle::new(&FONT_6X10, COLOR_LABEL);
    let value = MonoTextStyle::new(&FONT_10X20, COLOR_IP);
    let warn = MonoTextStyle::new(&FONT_10X20, COLOR_WARNING);

    // "IP" label (FONT_6X10, baseline at screen y = LOWER_Y + 24 = 92)
    render_band(scratch, display, 92 - BAND_BASELINE_SMALL, SCRATCH_H, |s| {
        Text::new("IP", Point::new(20, BAND_BASELINE_SMALL), label)
            .draw(s)
            .unwrap();
    });

    // IP value (FONT_10X20, baseline at screen y = LOWER_Y + 44 = 112) plus
    // an inline "ERR!" indicator at the right when the event log has entries.
    buf.clear();
    match ip {
        Some(addr) => {
            let _ = write!(buf, "{addr}");
        }
        None => {
            let _ = write!(buf, "--");
        }
    }
    render_band(
        scratch,
        display,
        112 - BAND_BASELINE_LARGE,
        SCRATCH_H,
        |s| {
            Text::new(buf, Point::new(20, BAND_BASELINE_LARGE), value)
                .draw(s)
                .unwrap();
            if has_errors {
                Text::new(
                    "ERR!",
                    Point::new(SCRATCH_W as i32 - 50, BAND_BASELINE_LARGE),
                    warn,
                )
                .draw(s)
                .unwrap();
            }
        },
    );

    // "URL" label (baseline at screen y = LOWER_Y + 60 = 128)
    render_band(
        scratch,
        display,
        128 - BAND_BASELINE_SMALL,
        SCRATCH_H,
        |s| {
            Text::new("URL", Point::new(20, BAND_BASELINE_SMALL), label)
                .draw(s)
                .unwrap();
        },
    );

    // URL value (baseline at screen y = LOWER_Y + 80 = 148)
    buf.clear();
    let _ = write!(buf, "https://{}.local", crate::wifi::HOSTNAME);
    render_band(
        scratch,
        display,
        148 - BAND_BASELINE_LARGE,
        SCRATCH_H,
        |s| {
            Text::new(buf, Point::new(20, BAND_BASELINE_LARGE), value)
                .draw(s)
                .unwrap();
        },
    );
}

fn draw_captive_portal<D>(
    scratch: &mut Scratch,
    display: &mut D,
    trying: bool,
    buf: &mut heapless::String<32>,
) where
    D: DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    let title = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
    let value = MonoTextStyle::new(&FONT_10X20, COLOR_VOLTAGE);
    let label = MonoTextStyle::new(&FONT_6X10, COLOR_LABEL);
    let trying_style = MonoTextStyle::new(&FONT_6X10, COLOR_DISCHARGING);

    // Title row + optional "Connecting..." badge at the right.
    // Baseline at screen y = LOWER_Y + 18 = 86 (was 14 in legacy; nudged down
    // 4 px so the glyph sits cleanly inside the band rather than clipping).
    render_band(scratch, display, 86 - BAND_BASELINE_LARGE, SCRATCH_H, |s| {
        Text::new("WiFi Setup", Point::new(20, BAND_BASELINE_LARGE), title)
            .draw(s)
            .unwrap();
        if trying {
            Text::new(
                "Connecting...",
                Point::new(190, BAND_BASELINE_LARGE),
                trying_style,
            )
            .draw(s)
            .unwrap();
        }
    });

    for (label_text, label_baseline, value_text, value_baseline) in [
        ("SSID", 94, crate::wifi::AP_SSID, 110),
        ("PASSWORD", 122, crate::wifi::AP_PASS, 138),
    ] {
        render_band(
            scratch,
            display,
            label_baseline - BAND_BASELINE_SMALL,
            SCRATCH_H,
            |s| {
                Text::new(label_text, Point::new(20, BAND_BASELINE_SMALL), label)
                    .draw(s)
                    .unwrap();
            },
        );
        render_band(
            scratch,
            display,
            value_baseline - BAND_BASELINE_LARGE,
            SCRATCH_H,
            |s| {
                Text::new(value_text, Point::new(20, BAND_BASELINE_LARGE), value)
                    .draw(s)
                    .unwrap();
            },
        );
    }

    // "OPEN" label + gateway URL.
    render_band(
        scratch,
        display,
        150 - BAND_BASELINE_SMALL,
        SCRATCH_H,
        |s| {
            Text::new("OPEN", Point::new(20, BAND_BASELINE_SMALL), label)
                .draw(s)
                .unwrap();
        },
    );
    let [a, b, c, d] = crate::wifi::AP_GATEWAY;
    buf.clear();
    let _ = write!(buf, "http://{a}.{b}.{c}.{d}/");
    render_band(
        scratch,
        display,
        166 - BAND_BASELINE_LARGE,
        SCRATCH_H,
        |s| {
            Text::new(buf, Point::new(20, BAND_BASELINE_LARGE), value)
                .draw(s)
                .unwrap();
        },
    );
}

fn draw_connecting<D>(scratch: &mut Scratch, display: &mut D)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    let title = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
    render_band(
        scratch,
        display,
        128 - BAND_BASELINE_LARGE,
        SCRATCH_H,
        |s| {
            Text::new("Connecting...", Point::new(20, BAND_BASELINE_LARGE), title)
                .draw(s)
                .unwrap();
        },
    );
}

// --- Main thread ---

pub fn start(
    pins: LcdPins,
    sensor_data: Arc<Mutex<SensorData>>,
    event_log: Arc<Mutex<EventLog>>,
    status: NetStatusHandle,
) {
    thread::Builder::new()
        .stack_size(16384)
        .spawn(move || {
            // Backing storage in BSS (single thread, taken once).
            static mut SPI_BUF: [u8; SPI_BUF_SIZE] = [0; SPI_BUF_SIZE];
            static mut SCRATCH_PIXELS: [Rgb565; SCRATCH_PX] = [COLOR_BG; SCRATCH_PX];

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

            // SAFETY: this thread is spawned exactly once and these statics
            // are not referenced anywhere else.
            let spi_ptr = &raw mut SPI_BUF;
            let scratch_ptr = &raw mut SCRATCH_PIXELS;
            let spi_buf: &'static mut [u8] = unsafe { &mut *spi_ptr };
            let scratch_pixels: &'static mut [Rgb565] = unsafe { &mut *scratch_ptr };
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

            // Static labels — drawn once directly to display.
            let label_style = MonoTextStyle::new(&FONT_5X8, COLOR_LABEL);
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
                Text::new(text, pos, label_style)
                    .draw(&mut display)
                    .unwrap();
            }

            let mut scratch = Scratch::from_buf(scratch_pixels);
            let mut prev_status = NetStatus::Connecting;
            let mut prev_ip: Option<std::net::Ipv4Addr> = None;
            let mut prev_has_errors = false;
            let mut text_buf = heapless::String::<32>::new();
            let mut up_buf = heapless::String::<16>::new();

            loop {
                thread::sleep(REFRESH_INTERVAL);

                let net_status = status.load();
                let ip = if net_status == NetStatus::Host {
                    crate::wifi::sta_ip()
                } else {
                    None
                };
                let has_errors = !event_log.lock().unwrap().is_empty();
                let need_redraw = net_status != prev_status
                    || (net_status == NetStatus::Host
                        && (ip != prev_ip || has_errors != prev_has_errors));

                let (r1, r2) = {
                    let sd = sensor_data.lock().unwrap();
                    (
                        sd.battery_reading().unwrap_or_default(),
                        sd.ps_reading().unwrap_or_default(),
                    )
                };

                // Upper region — four value cells + uptime, every tick.
                text_buf.clear();
                let _ = write!(text_buf, "{:.2} V", r1.voltage);
                draw_value(
                    &mut scratch,
                    &mut display,
                    &text_buf,
                    Point::new(COL_LEFT, ROW1_TOP),
                    COLOR_VOLTAGE,
                );

                text_buf.clear();
                let _ = write!(text_buf, "{:.2} W", r1.power);
                draw_value(
                    &mut scratch,
                    &mut display,
                    &text_buf,
                    Point::new(COL_RIGHT, ROW1_TOP),
                    COLOR_POWER,
                );

                text_buf.clear();
                // Sign is conveyed by color — show magnitude only.
                let _ = write!(text_buf, "{:.3} A", r1.current.abs());
                draw_value(
                    &mut scratch,
                    &mut display,
                    &text_buf,
                    Point::new(COL_LEFT, ROW2_TOP),
                    battery_current_color(r1.current),
                );

                text_buf.clear();
                let _ = write!(text_buf, "{:.3} A", r2.current);
                draw_value(
                    &mut scratch,
                    &mut display,
                    &text_buf,
                    Point::new(COL_RIGHT, ROW2_TOP),
                    COLOR_PSU_CURRENT,
                );

                format_uptime(crate::clock::uptime(), &mut up_buf);
                scratch.clear();
                Text::new(
                    &up_buf,
                    Point::new(0, 8),
                    MonoTextStyle::new(&FONT_6X10, COLOR_LABEL),
                )
                .draw(&mut scratch)
                .unwrap();
                scratch.blit(&mut display, Point::new(UPTIME_X, 0), UPTIME_W, UPTIME_H);

                // Lower region — repainted only on visible state change.
                if need_redraw {
                    prev_status = net_status;
                    prev_ip = ip;
                    prev_has_errors = has_errors;
                    clear_lower(&mut display);
                    match net_status {
                        NetStatus::Captive => {
                            draw_captive_portal(&mut scratch, &mut display, false, &mut text_buf)
                        }
                        NetStatus::CaptiveTrying => {
                            draw_captive_portal(&mut scratch, &mut display, true, &mut text_buf)
                        }
                        NetStatus::Connecting => draw_connecting(&mut scratch, &mut display),
                        NetStatus::Host => {
                            draw_host(&mut scratch, &mut display, ip, has_errors, &mut text_buf)
                        }
                    }
                }
            }
        })
        .unwrap();
}
