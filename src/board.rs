//! Board-specific peripheral wiring. All GPIO / peripheral assignments live here.

#[cfg(not(any(feature = "esp32c3", feature = "esp32c6")))]
compile_error!("enable exactly one MCU feature: `esp32c3` or `esp32c6`");

#[cfg(all(feature = "esp32c3", feature = "esp32c6"))]
compile_error!("enable exactly one MCU feature, not both `esp32c3` and `esp32c6`");

use esp_idf_hal::gpio::AnyIOPin;
use esp_idf_hal::modem::Modem;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::uart::UART1;

#[cfg(feature = "lcd")]
use esp_idf_hal::gpio::AnyOutputPin;
#[cfg(feature = "lcd")]
use esp_idf_hal::ledc::{CHANNEL0, TIMER0};
#[cfg(feature = "lcd")]
use esp_idf_hal::spi::SPI2;

pub struct I2cPins {
    pub i2c: esp_idf_hal::i2c::I2C0<'static>,
    pub sda: AnyIOPin<'static>,
    pub scl: AnyIOPin<'static>,
}

#[cfg(feature = "lcd")]
pub struct LcdPins {
    pub spi: SPI2<'static>,
    pub sclk: AnyIOPin<'static>,
    pub mosi: AnyIOPin<'static>,
    pub cs: AnyOutputPin<'static>,
    pub dc: AnyOutputPin<'static>,
    pub rst: AnyOutputPin<'static>,
    pub blk: AnyOutputPin<'static>,
    pub ledc_timer: TIMER0<'static>,
    pub ledc_channel: CHANNEL0<'static>,
}

pub struct XyPins {
    pub uart: UART1<'static>,
    pub tx: AnyIOPin<'static>,
    pub rx: AnyIOPin<'static>,
}

pub struct Board {
    pub modem: Modem<'static>,
    pub i2c: I2cPins,
    pub xy: XyPins,
    #[cfg(feature = "lcd")]
    pub lcd: LcdPins,
}

impl Board {
    pub fn take() -> Self {
        let p = Peripherals::take().unwrap();

        #[cfg(feature = "esp32c3")]
        let i2c = I2cPins {
            i2c: p.i2c0,
            sda: p.pins.gpio8.into(),
            scl: p.pins.gpio9.into(),
        };
        #[cfg(feature = "esp32c6")]
        let i2c = I2cPins {
            i2c: p.i2c0,
            sda: p.pins.gpio23.into(),
            scl: p.pins.gpio20.into(),
        };

        #[cfg(feature = "esp32c3")]
        let xy = XyPins {
            uart: p.uart1,
            tx: p.pins.gpio4.into(),
            rx: p.pins.gpio5.into(),
        };
        #[cfg(feature = "esp32c6")]
        let xy = XyPins {
            uart: p.uart1,
            tx: p.pins.gpio16.into(),
            rx: p.pins.gpio17.into(),
        };

        #[cfg(feature = "lcd")]
        let lcd = LcdPins {
            spi: p.spi2,
            sclk: p.pins.gpio7.into(),
            mosi: p.pins.gpio6.into(),
            cs: p.pins.gpio14.into(),
            dc: p.pins.gpio15.into(),
            rst: p.pins.gpio21.into(),
            blk: p.pins.gpio22.into(),
            ledc_timer: p.ledc.timer0,
            ledc_channel: p.ledc.channel0,
        };

        Self {
            modem: p.modem,
            i2c,
            xy,
            #[cfg(feature = "lcd")]
            lcd,
        }
    }
}
