use core::fmt::Write;
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::geometry::{OriginDimensions, Point, Size};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::ascii::{FONT_5X8, FONT_6X10, FONT_10X20};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::text::Text;
use esp_idf_hal::gpio::{AnyIOPin, AnyOutputPin, PinDriver};
use esp_idf_hal::ledc::{LedcDriver, LedcTimerDriver, config::TimerConfig};
use esp_idf_hal::spi::config::{Config as SpiConfig, DriverConfig};
use esp_idf_hal::spi::{Dma, SPI2, SpiDeviceDriver, SpiDriver};
use esp_idf_hal::units::FromValueType;
use mipidsi::Builder;
use mipidsi::interface::SpiInterface;
use mipidsi::models::ST7789;
use mipidsi::options::{Orientation, Rotation};

use esp32_battery_logic::data::{Platform, SensorData};

// --- SPI / DMA ---

const SPI_BUF_SIZE: usize = 32768;
const DMA_BUF_SIZE: usize = 32768;

// --- Timing ---

const REFRESH_INTERVAL: Duration = Duration::from_millis(500);

// --- Colors ---

const COLOR_BG: Rgb565 = Rgb565::BLACK;
const COLOR_LABEL: Rgb565 = Rgb565::new(18, 36, 18);
const COLOR_VOLTAGE: Rgb565 = Rgb565::new(0, 63, 0);
const COLOR_BAT_CURRENT: Rgb565 = Rgb565::new(31, 50, 0);
const COLOR_PSU_CURRENT: Rgb565 = Rgb565::new(0, 40, 31);
const COLOR_POWER: Rgb565 = Rgb565::new(31, 20, 0);
const COLOR_GRID: Rgb565 = Rgb565::new(4, 8, 4);

// --- Layout ---

const COL_LEFT: i32 = 5;
const COL_RIGHT: i32 = 165;
const ROW1_LABEL_Y: i32 = 8;
const ROW1_VALUE_Y: i32 = 28;
const ROW2_LABEL_Y: i32 = 38;
const ROW2_VALUE_Y: i32 = 58;
const UPTIME_X: i32 = 240;

const VALUE_W: u32 = 150;
const VALUE_H: u32 = 22;
const VALUE_PIXELS: usize = (VALUE_W * VALUE_H) as usize;

const GRAPH_Y: i32 = 62;
const GRAPH_W: u32 = 320;
const GRAPH_H: u32 = 110;
const GRAPH_PIXELS: usize = (GRAPH_W * GRAPH_H) as usize;

// --- Pins ---

pub struct LcdPins {
    pub sclk: AnyIOPin<'static>,
    pub mosi: AnyIOPin<'static>,
    pub cs: AnyOutputPin<'static>,
    pub dc: AnyOutputPin<'static>,
    pub rst: AnyOutputPin<'static>,
    pub blk: AnyOutputPin<'static>,
    pub spi: SPI2<'static>,
    pub ledc_timer: esp_idf_hal::ledc::TIMER0<'static>,
    pub ledc_channel: esp_idf_hal::ledc::CHANNEL0<'static>,
}

/// Backlight brightness 0–100%.
const BACKLIGHT_PERCENT: u32 = 50;

// --- Framebuffer ---

struct Framebuf<const W: u32, const H: u32, const N: usize> {
    pixels: Box<[Rgb565; N]>,
}

impl<const W: u32, const H: u32, const N: usize> Framebuf<W, H, N> {
    fn new() -> Self {
        Self {
            pixels: Box::new([COLOR_BG; N]),
        }
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

impl<const W: u32, const H: u32, const N: usize> OriginDimensions for Framebuf<W, H, N> {
    fn size(&self) -> Size {
        Size::new(W, H)
    }
}

impl<const W: u32, const H: u32, const N: usize> DrawTarget for Framebuf<W, H, N> {
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

type FieldBuf = Framebuf<VALUE_W, VALUE_H, VALUE_PIXELS>;
type GraphBuf = Framebuf<GRAPH_W, GRAPH_H, GRAPH_PIXELS>;

// --- Drawing helpers ---

fn format_uptime(secs: u32) -> heapless::String<16> {
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

fn map_to_y(val: f32, min: f32, max: f32, h: u32) -> i32 {
    if (max - min).abs() < 0.001 {
        return h as i32 / 2;
    }
    let normalized = (val - min) / (max - min);
    let y = (1.0 - normalized) * (h as f32 - 1.0);
    y.clamp(0.0, h as f32 - 1.0) as i32
}

fn draw_line(gb: &mut GraphBuf, x0: i32, y0: i32, x1: i32, y1: i32, color: Rgb565) {
    let dx = (x1 - x0).abs();
    let dy = -(y1 - y0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx + dy;
    let mut x = x0;
    let mut y = y0;
    loop {
        gb.set_pixel(x, y, color);
        gb.set_pixel(x, y + 1, color);
        if x == x1 && y == y1 {
            break;
        }
        let e2 = 2 * err;
        if e2 >= dy {
            err += dy;
            x += sx;
        }
        if e2 <= dx {
            err += dx;
            y += sy;
        }
    }
}

// --- Graph rendering ---

fn draw_graph(gb: &mut GraphBuf, history: &[(f32, f32, f32)], buf: &mut heapless::String<16>) {
    let n = history.len();
    if n < 2 {
        return;
    }

    // Compute ranges
    let mut v_min = f32::MAX;
    let mut v_max = f32::MIN;
    let mut c_min = f32::MAX;
    let mut c_max = f32::MIN;

    for &(v, c1, c2) in history {
        v_min = v_min.min(v);
        v_max = v_max.max(v);
        c_min = c_min.min(c1).min(c2);
        c_max = c_max.max(c1).max(c2);
    }

    let v_range = (v_max - v_min).max(0.1);
    v_min -= v_range * 0.05;
    v_max += v_range * 0.05;
    let c_range = (c_max - c_min).max(0.01);
    c_min -= c_range * 0.05;
    c_max += c_range * 0.05;

    // Grid lines
    for i in 1..4 {
        let gy = (GRAPH_H as i32 * i) / 4;
        for gx in (0..GRAPH_W as i32).step_by(4) {
            gb.set_pixel(gx, gy, COLOR_GRID);
        }
    }

    // Scale labels
    let scale_style = MonoTextStyle::new(&FONT_5X8, COLOR_LABEL);
    buf.clear();
    let _ = write!(buf, "{:.1}V", v_max);
    Text::new(buf, Point::new(12, 8), scale_style)
        .draw(gb)
        .unwrap();
    buf.clear();
    let _ = write!(buf, "{:.1}V", v_min);
    Text::new(buf, Point::new(12, GRAPH_H as i32 - 6), scale_style)
        .draw(gb)
        .unwrap();

    // Traces: voltage on its own scale, both currents share a scale
    let margin = 2i32;
    let plot_h = GRAPH_H - margin as u32 * 2;
    let x_scale = (GRAPH_W as f32 - 1.0) / (n - 1) as f32;

    let traces: [(f32, f32, Rgb565); 3] = [
        (v_min, v_max, COLOR_VOLTAGE),
        (c_min, c_max, COLOR_BAT_CURRENT),
        (c_min, c_max, COLOR_PSU_CURRENT),
    ];

    for i in 1..n {
        let x0 = ((i - 1) as f32 * x_scale) as i32;
        let x1 = (i as f32 * x_scale) as i32;
        let prev = history[i - 1];
        let curr = history[i];
        let vals_prev = [prev.0, prev.1, prev.2];
        let vals_curr = [curr.0, curr.1, curr.2];

        for (j, &(lo, hi, color)) in traces.iter().enumerate() {
            draw_line(
                gb,
                x0,
                margin + map_to_y(vals_prev[j], lo, hi, plot_h),
                x1,
                margin + map_to_y(vals_curr[j], lo, hi, plot_h),
                color,
            );
        }
    }
}

// --- Captive portal overlay ---

fn draw_captive_portal(gb: &mut GraphBuf) {
    let title = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
    let value = MonoTextStyle::new(&FONT_10X20, COLOR_VOLTAGE);
    let label = MonoTextStyle::new(&FONT_6X10, COLOR_LABEL);

    Text::new("WiFi Setup", Point::new(20, 24), title)
        .draw(gb)
        .unwrap();
    Text::new("SSID", Point::new(20, 44), label)
        .draw(gb)
        .unwrap();
    Text::new(crate::wifi::AP_SSID, Point::new(20, 64), value)
        .draw(gb)
        .unwrap();
    Text::new("PASSWORD", Point::new(20, 82), label)
        .draw(gb)
        .unwrap();
    Text::new(crate::wifi::AP_PASS, Point::new(20, 102), value)
        .draw(gb)
        .unwrap();
}

// --- Main thread ---

pub fn start_lcd_thread<P: Platform + Send + 'static>(
    pins: LcdPins,
    sensor_data: Arc<Mutex<SensorData<P>>>,
) {
    thread::Builder::new()
        .stack_size(8192)
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

            // Leaked for 'static lifetime required by SpiInterface (thread runs forever)
            let spi_buf = Box::leak(Box::new([0u8; SPI_BUF_SIZE]));
            let spi_iface = SpiInterface::new(spi_device, dc, &mut *spi_buf);

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
                ("VOLTAGE", Point::new(COL_LEFT, ROW1_LABEL_Y)),
                ("BATTERY", Point::new(COL_LEFT, ROW2_LABEL_Y)),
                ("POWER", Point::new(COL_RIGHT, ROW1_LABEL_Y)),
                ("PSU", Point::new(COL_RIGHT, ROW2_LABEL_Y)),
            ] {
                Text::new(text, pos, label_style)
                    .draw(&mut display)
                    .unwrap();
            }

            let mut fb = FieldBuf::new();
            let mut gb = GraphBuf::new();
            let mut prev_captive = false;

            loop {
                thread::sleep(REFRESH_INTERVAL);

                let (r1, r2, uptime_s, history) = {
                    let sd = sensor_data.lock().unwrap();
                    let hist: heapless::Vec<(f32, f32, f32), 144> = sd
                        .history()
                        .iter()
                        .map(|s| (s.voltage, s.battery_current, s.ps_current))
                        .collect();
                    (
                        sd.battery_reading,
                        sd.ps_reading,
                        crate::uptime_s(),
                        hist,
                    )
                };

                let mut buf = heapless::String::<16>::new();

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
                let _ = write!(buf, "{:.3} A", r1.current);
                draw_value(
                    &mut display,
                    &mut fb,
                    &buf,
                    Point::new(COL_LEFT, ROW2_VALUE_Y),
                    COLOR_BAT_CURRENT,
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
                let up = format_uptime(uptime_s);
                fb.clear();
                Text::new(
                    &up,
                    Point::new(0, 8),
                    MonoTextStyle::new(&FONT_6X10, COLOR_LABEL),
                )
                .draw(&mut fb)
                .unwrap();
                fb.blit_rows(&mut display, Point::new(UPTIME_X, 0), 12);

                // Graph / Captive portal
                let is_captive = crate::CAPTIVE_PORTAL_ACTIVE.load(Ordering::Relaxed);
                if is_captive && prev_captive {
                    continue;
                }
                prev_captive = is_captive;

                gb.clear();
                if is_captive {
                    draw_captive_portal(&mut gb);
                } else {
                    draw_graph(&mut gb, &history, &mut buf);
                }
                gb.blit(&mut display, Point::new(0, GRAPH_Y));
            }
        })
        .unwrap();
}
