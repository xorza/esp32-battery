//! Board-specific peripheral wiring. All GPIO / peripheral assignments live here.

use esp_idf_hal::gpio::AnyIOPin;
use esp_idf_hal::i2c::{I2cBusDriver, config::BusConfig};
use esp_idf_hal::modem::Modem;
use esp_idf_hal::peripherals::Peripherals;

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

impl I2cPins {
    /// Construct an I2C bus driver and leak it for a `'static` lifetime.
    /// The bus lives for the entire process so the leak is intentional.
    pub fn init_bus(self) -> &'static I2cBusDriver<'static> {
        Box::leak(Box::new(
            I2cBusDriver::new(self.i2c, self.sda, self.scl, &BusConfig::new()).unwrap(),
        ))
    }
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

pub struct Board {
    pub modem: Modem<'static>,
    pub i2c: I2cPins,
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
            sda: p.pins.gpio20.into(),
            scl: p.pins.gpio23.into(),
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
            #[cfg(feature = "lcd")]
            lcd,
        }
    }
}
