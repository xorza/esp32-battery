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
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle, Triangle};
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

// --- SPI / DMA ---

const SPI_BUF_SIZE: usize = 32768;
const DMA_BUF_SIZE: usize = 32768;

// --- Timing ---

const REFRESH_INTERVAL: Duration = Duration::from_millis(500);

// --- Colors ---

const COLOR_BG: Rgb565 = Rgb565::BLACK;
const COLOR_LABEL: Rgb565 = Rgb565::new(18, 36, 18);
const COLOR_VOLTAGE: Rgb565 = Rgb565::new(0, 63, 0);
const COLOR_PSU_CURRENT: Rgb565 = Rgb565::new(31, 38, 0); // orange, matches #ff9800
const COLOR_POWER: Rgb565 = Rgb565::new(31, 20, 0);
const COLOR_CHARGING: Rgb565 = Rgb565::new(6, 55, 10); // green
const COLOR_DISCHARGING: Rgb565 = Rgb565::new(31, 28, 0); // orange
const COLOR_IDLE: Rgb565 = Rgb565::new(18, 36, 18); // light gray-green
const COLOR_IP: Rgb565 = Rgb565::new(0, 57, 31); // cyan
const COLOR_WARNING: Rgb565 = Rgb565::RED;
/// Dead-band below which the battery is considered idle (|I| < 50 mA).
const BATTERY_IDLE_THRESHOLD_A: f32 = 0.05;

/// Sign convention: negative current = charging, positive = discharging.
fn battery_current_color(current: f32) -> Rgb565 {
    if current.abs() < BATTERY_IDLE_THRESHOLD_A {
        COLOR_IDLE
    } else if current < 0.0 {
        COLOR_CHARGING
    } else {
        COLOR_DISCHARGING
    }
}

// --- Layout ---

const COL_LEFT: i32 = 5;
const COL_RIGHT: i32 = 165;
const ROW1_LABEL_Y: i32 = 10;
const ROW1_VALUE_Y: i32 = 30;
const ROW2_LABEL_Y: i32 = 44;
const ROW2_VALUE_Y: i32 = 62;
const UPTIME_X: i32 = 240;

const VALUE_W: u32 = 150;
const VALUE_H: u32 = 22;

const LOWER_Y: i32 = 68;
const LOWER_W: u32 = 320;
// The lower region is 104 px tall but rendered through a half-height
// scratch buffer in two passes (top half then bottom half) to halve BSS
// cost; pixels outside the buffer are clipped by `Framebuf::set_pixel`
// and `fill_solid`.
const LOWER_H: u32 = 52;

/// Backlight brightness 0–100%.
const BACKLIGHT_PERCENT: u32 = 50;

// --- Framebuffer ---

struct Framebuf<const W: u32, const H: u32> {
    pixels: &'static mut [Rgb565],
}

impl<const W: u32, const H: u32> Framebuf<W, H> {
    fn from_buf(pixels: &'static mut [Rgb565]) -> Self {
        assert_eq!(pixels.len(), (W * H) as usize);
        pixels.fill(COLOR_BG);
        Self { pixels }
    }

    fn clear(&mut self) {
        self.pixels.fill(COLOR_BG);
    }

    fn set_pixel(&mut self, x: i32, y: i32, color: Rgb565) {
        if x >= 0 && x < W as i32 && y >= 0 && y < H as i32 {
            self.pixels[y as usize * W as usize + x as usize] = color;
        }
    }

    fn blit<D: DrawTarget<Color = Rgb565>>(&self, display: &mut D, top_left: Point)
    where
        D::Error: core::fmt::Debug,
    {
        let area = Rectangle::new(top_left, Size::new(W, H));
        display
            .fill_contiguous(&area, self.pixels.iter().copied())
            .unwrap();
    }

    fn blit_rows<D: DrawTarget<Color = Rgb565>>(&self, display: &mut D, top_left: Point, rows: u32)
    where
        D::Error: core::fmt::Debug,
    {
        let area = Rectangle::new(top_left, Size::new(W, rows));
        let pixel_count = W as usize * rows as usize;
        display
            .fill_contiguous(&area, self.pixels[..pixel_count].iter().copied())
            .unwrap();
    }
}

impl<const W: u32, const H: u32> OriginDimensions for Framebuf<W, H> {
    fn size(&self) -> Size {
        Size::new(W, H)
    }
}

impl<const W: u32, const H: u32> DrawTarget for Framebuf<W, H> {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Rgb565>>,
    {
        for Pixel(point, color) in pixels {
            self.set_pixel(point.x, point.y, color);
        }
        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Rgb565) -> Result<(), Self::Error> {
        let area = area.intersection(&Rectangle::new(Point::zero(), self.size()));
        if let Some(bottom_right) = area.bottom_right() {
            let x0 = area.top_left.x as usize;
            let w = (bottom_right.x - area.top_left.x + 1) as usize;
            for y in area.top_left.y..=bottom_right.y {
                let start = y as usize * W as usize + x0;
                self.pixels[start..start + w].fill(color);
            }
        }
        Ok(())
    }
}

type FieldBuf = Framebuf<VALUE_W, VALUE_H>;
type LowerBuf = Framebuf<LOWER_W, LOWER_H>;

// --- Drawing helpers ---

fn format_uptime(uptime: core::time::Duration) -> heapless::String<16> {
    let secs = uptime.as_secs();
    let h = secs / 3600;
    let m = (secs % 3600) / 60;
    let s = secs % 60;
    let mut out = heapless::String::new();
    let _ = write!(out, "{}h {:02}m {:02}s", h, m, s);
    out
}

fn draw_value<D: DrawTarget<Color = Rgb565>>(
    display: &mut D,
    fb: &mut FieldBuf,
    text: &str,
    screen_pos: Point,
    color: Rgb565,
) where
    D::Error: core::fmt::Debug,
{
    fb.clear();
    Text::new(
        text,
        Point::new(0, 16),
        MonoTextStyle::new(&FONT_10X20, color),
    )
    .draw(fb)
    .unwrap();
    fb.blit(display, Point::new(screen_pos.x, screen_pos.y - 16));
}

// --- Host-mode rendering ---

// Lower-region draw helpers below take a `y_off` and render in
// lower-region-absolute coordinates minus that offset, so the same
// function renders either the top half (y_off=0) or the bottom half
// (y_off=LOWER_H). Out-of-buffer pixels are clipped by Framebuf.

fn draw_host(
    lb: &mut LowerBuf,
    y_off: i32,
    ip: Option<std::net::Ipv4Addr>,
    has_errors: bool,
    buf: &mut heapless::String<32>,
) {
    let label = MonoTextStyle::new(&FONT_6X10, COLOR_LABEL);
    let value = MonoTextStyle::new(&FONT_10X20, COLOR_IP);

    Text::new("IP", Point::new(20, 24 - y_off), label)
        .draw(lb)
        .unwrap();
    buf.clear();
    match ip {
        Some(addr) => {
            let _ = write!(buf, "{addr}");
        }
        None => {
            let _ = write!(buf, "--");
        }
    }
    Text::new(buf, Point::new(20, 44 - y_off), value)
        .draw(lb)
        .unwrap();

    Text::new("URL", Point::new(20, 60 - y_off), label)
        .draw(lb)
        .unwrap();
    buf.clear();
    let _ = write!(buf, "https://{}.local", crate::wifi::HOSTNAME);
    Text::new(buf, Point::new(20, 80 - y_off), value)
        .draw(lb)
        .unwrap();

    if has_errors {
        draw_warning_triangle(lb, y_off);
    }
}

/// Filled red warning triangle with a black "!" inside, anchored to the
/// right side of the lower region. Drawn only when the event log holds
/// at least one entry — a hint that the user should check `/api/errors`.
fn draw_warning_triangle(lb: &mut LowerBuf, y_off: i32) {
    let cx = LOWER_W as i32 - 50;
    let cy_top = 20 - y_off;
    let cy_bot = 84 - y_off;
    let half = 32;
    Triangle::new(
        Point::new(cx, cy_top),
        Point::new(cx - half, cy_bot),
        Point::new(cx + half, cy_bot),
    )
    .into_styled(PrimitiveStyle::with_fill(COLOR_WARNING))
    .draw(lb)
    .unwrap();
    Text::new(
        "!",
        Point::new(cx - 5, cy_bot - 14),
        MonoTextStyle::new(&FONT_10X20, Rgb565::BLACK),
    )
    .draw(lb)
    .unwrap();
}

// --- Captive portal overlay ---

fn draw_captive_portal(
    lb: &mut LowerBuf,
    y_off: i32,
    trying: bool,
    buf: &mut heapless::String<32>,
) {
    let title = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
    let value = MonoTextStyle::new(&FONT_10X20, COLOR_VOLTAGE);
    let label = MonoTextStyle::new(&FONT_6X10, COLOR_LABEL);

    Text::new("WiFi Setup", Point::new(20, 14 - y_off), title)
        .draw(lb)
        .unwrap();
    Text::new("SSID", Point::new(20, 26 - y_off), label)
        .draw(lb)
        .unwrap();
    Text::new(crate::wifi::AP_SSID, Point::new(20, 42 - y_off), value)
        .draw(lb)
        .unwrap();
    Text::new("PASSWORD", Point::new(20, 54 - y_off), label)
        .draw(lb)
        .unwrap();
    Text::new(crate::wifi::AP_PASS, Point::new(20, 70 - y_off), value)
        .draw(lb)
        .unwrap();
    Text::new("OPEN", Point::new(20, 82 - y_off), label)
        .draw(lb)
        .unwrap();
    let [a, b, c, d] = crate::wifi::AP_GATEWAY;
    buf.clear();
    let _ = write!(buf, "http://{a}.{b}.{c}.{d}/");
    Text::new(buf, Point::new(20, 98 - y_off), value)
        .draw(lb)
        .unwrap();

    if trying {
        // Top-right indicator so the AP creds remain readable while STA
        // is mid-association on the user's submitted creds.
        Text::new(
            "Connecting...",
            Point::new(190, 14 - y_off),
            MonoTextStyle::new(&FONT_6X10, COLOR_DISCHARGING),
        )
        .draw(lb)
        .unwrap();
    }
}

fn draw_connecting(lb: &mut LowerBuf, y_off: i32) {
    let title = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
    Text::new("Connecting...", Point::new(20, 60 - y_off), title)
        .draw(lb)
        .unwrap();
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
            // PWM backlight at reduced brightness
            let timer = LedcTimerDriver::new(
                pins.ledc_timer,
                &TimerConfig::default().frequency(1.kHz().into()),
            )
            .unwrap();
            let mut blk = LedcDriver::new(pins.ledc_channel, timer, pins.blk).unwrap();
            let max_duty = blk.get_max_duty();
            blk.set_duty(max_duty * BACKLIGHT_PERCENT / 100).unwrap();

            // Backing storage for SPI interface scratch buffer and the two
            // framebuffers lives in BSS — sums to ~105 KB and was previously
            // heap-allocated (and leaked) at boot.
            static mut SPI_BUF: [u8; SPI_BUF_SIZE] = [0; SPI_BUF_SIZE];
            static mut FIELD_PIXELS: [Rgb565; (VALUE_W * VALUE_H) as usize] =
                [COLOR_BG; (VALUE_W * VALUE_H) as usize];
            static mut LOWER_PIXELS: [Rgb565; (LOWER_W * LOWER_H) as usize] =
                [COLOR_BG; (LOWER_W * LOWER_H) as usize];

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
            let field_ptr = &raw mut FIELD_PIXELS;
            let lower_ptr = &raw mut LOWER_PIXELS;
            let spi_buf: &'static mut [u8] = unsafe { &mut *spi_ptr };
            let field_pixels: &'static mut [Rgb565] = unsafe { &mut *field_ptr };
            let lower_pixels: &'static mut [Rgb565] = unsafe { &mut *lower_ptr };
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

            // Static labels (drawn once)
            let label_style = MonoTextStyle::new(&FONT_5X8, COLOR_LABEL);
            for &(text, pos) in &[
                ("VOLTAGE", Point::new(COL_LEFT + 8, ROW1_LABEL_Y)),
                ("BATTERY", Point::new(COL_LEFT + 8, ROW2_LABEL_Y)),
                ("POWER", Point::new(COL_RIGHT + 8, ROW1_LABEL_Y)),
                ("PSU", Point::new(COL_RIGHT + 8, ROW2_LABEL_Y)),
            ] {
                Text::new(text, pos, label_style)
                    .draw(&mut display)
                    .unwrap();
            }

            let mut fb = FieldBuf::from_buf(field_pixels);
            let mut lb = LowerBuf::from_buf(lower_pixels);
            let mut prev_status = NetStatus::Connecting;
            let mut prev_ip: Option<std::net::Ipv4Addr> = None;
            let mut prev_has_errors = false;

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
                let mut buf = heapless::String::<32>::new();

                let (r1, r2) = {
                    let sd = sensor_data.lock().unwrap();
                    (
                        sd.battery_reading().unwrap_or_default(),
                        sd.ps_reading().unwrap_or_default(),
                    )
                };
                let uptime = crate::clock::uptime();
                buf.clear();

                // Values
                let _ = write!(buf, "{:.2} V", r1.voltage);
                draw_value(
                    &mut display,
                    &mut fb,
                    &buf,
                    Point::new(COL_LEFT, ROW1_VALUE_Y),
                    COLOR_VOLTAGE,
                );

                buf.clear();
                let _ = write!(buf, "{:.2} W", r1.power);
                draw_value(
                    &mut display,
                    &mut fb,
                    &buf,
                    Point::new(COL_RIGHT, ROW1_VALUE_Y),
                    COLOR_POWER,
                );

                buf.clear();
                // Sign is conveyed by color (green=charging, orange=discharging);
                // show magnitude only.
                let _ = write!(buf, "{:.3} A", r1.current.abs());
                draw_value(
                    &mut display,
                    &mut fb,
                    &buf,
                    Point::new(COL_LEFT, ROW2_VALUE_Y),
                    battery_current_color(r1.current),
                );

                buf.clear();
                let _ = write!(buf, "{:.3} A", r2.current);
                draw_value(
                    &mut display,
                    &mut fb,
                    &buf,
                    Point::new(COL_RIGHT, ROW2_VALUE_Y),
                    COLOR_PSU_CURRENT,
                );

                // Uptime
                let up = format_uptime(uptime);
                fb.clear();
                Text::new(
                    &up,
                    Point::new(0, 8),
                    MonoTextStyle::new(&FONT_6X10, COLOR_LABEL),
                )
                .draw(&mut fb)
                .unwrap();
                fb.blit_rows(&mut display, Point::new(UPTIME_X, 0), 12);

                // Lower region: IP+warning (Host) or captive/connecting overlays.
                // Repainted only when something visible has changed.
                if need_redraw {
                    prev_status = net_status;
                    prev_ip = ip;
                    prev_has_errors = has_errors;
                    let half_h = LOWER_H as i32;
                    for y_off in [0, half_h] {
                        lb.clear();
                        match net_status {
                            NetStatus::Captive => {
                                draw_captive_portal(&mut lb, y_off, false, &mut buf)
                            }
                            NetStatus::CaptiveTrying => {
                                draw_captive_portal(&mut lb, y_off, true, &mut buf)
                            }
                            NetStatus::Connecting => draw_connecting(&mut lb, y_off),
                            NetStatus::Host => draw_host(&mut lb, y_off, ip, has_errors, &mut buf),
                        }
                        lb.blit(&mut display, Point::new(0, LOWER_Y + y_off));
                    }
                }
            }
        })
        .unwrap();
}
