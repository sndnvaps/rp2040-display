//! # Raspberry Pi Pico (monochome) 128x64 OLED Display with SSD1306 Driver Example
//!
//! This example assumes you got an 128x64 OLED Display with an SSD1306 driver
//! connected to your Raspberry Pi Pico. The +3.3V voltage source of the
//! Raspberry Pi Pico will be used, and the output pins 21 and 22 of the board
//! (on the lower right).
//!
//! It will demonstrate how to get an I2C device and use it with the ssd1306 crate.
//! Additionally you can also see how to format a number into a string using
//! [core::fmt].
//!
//! The following diagram will show how things should be connected.
//! These displays usually can take 3.3V up to 5V.
//!
//! ```text
//!                              VCC   SCL
//!                   /------------\    /----------\
//!                   |        GND  \  /  SDA      |
//!   _|USB|_         |    /-----\  |  |  /--------+--\
//!  |1  R 40|        |   /    __|__|__|__|___     |  |
//!  |2  P 39|        |  /    | ____________  |    |  |
//!  |3    38|- GND --+-/     | |Hello worl|  |    |  |
//!  |4  P 37|        |       | |Hello Rust|  |    |  |
//!  |5  I 36|-+5V -/       | |counter: 1|  |    |  |
//!  |6  C   |                | |          |  |    |  |
//!  |7  O   |                | """"""""""""  |    |  |
//!  |       |                 """""""""""""""     |  |
//!  |       |       (SSD1306 128x64 OLED Display) |  |
//!  .........                                     /  /
//!  |       |                                    /  /
//!  |     22|-GP15 I2C1 SCL---------------------/  /
//!  |20   21|-GP14 I2C1 SDA-----------------------/
//!   """""""
//! Symbols:
//!     - (+) crossing lines, not connected
//!     - (o) connected lines
//! ```
//! DHT11/DHT22 use GP22 for one-line data transport
//! See the `Cargo.toml` file for Copyright and license details.
//! pickup from https://github.com/rp-rs/rp-hal-boards/blob/7431e73c51a89b2fe81afb51896b2530afd40b8c/boards/rp-pico/examples/pico_i2c_oled_display_ssd1306.rs
#![no_std]
#![no_main]

use core::fmt::Write;

// For string formatting.
// The macro for our start-up function
// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::{entry, hal, hal::pac};

// Time handling traits:
use fugit::RateExtU32;

// Timer for the delay on the display:
use embedded_hal::{delay::DelayNs, digital::OutputPin};

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp2040_hal::Clock;

// For in the graphics drawing utilities like the font
// and the drawing routines:
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Line, PrimitiveStyle},
    text::{Baseline, Text},
};
use u8g2_fonts::U8g2TextStyle;

// For dht11 sensor
#[cfg(feature = "dht11")]
use dht_sensor::{dht11::Reading, DhtReading};
//use dht11::Dht11;

#[cfg(feature = "dht22")]
use dht_sensor::{dht22::Reading, DhtReading};

// The display driver:
use ssd1306::{prelude::*, Ssd1306};

pub mod fmtbuf;
use fmtbuf::FmtBuf;
pub mod dhtxx;
use dhtxx::get;
/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals,
/// gets a handle on the I2C peripheral,
/// initializes the SSD1306 driver, initializes the text builder
/// and then draws some text on the display.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    let core = cortex_m::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure two pins as being I²C, not GPIO
    let sda_pin: hal::gpio::Pin<_, hal::gpio::FunctionI2C, _> = pins.gpio14.reconfigure();
    let scl_pin: hal::gpio::Pin<_, hal::gpio::FunctionI2C, _> = pins.gpio15.reconfigure();

    // Create the I²C driver, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let i2c = hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );

    // Create the I²C display interface:
    let interface = ssd1306::I2CDisplayInterface::new(i2c);

    // Create a driver instance and initialize:
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    // Create a text style for drawing the font:
    let character_style =
        U8g2TextStyle::new(u8g2_fonts::fonts::u8g2_font_wqy12_t_gb2312, BinaryColor::On);

    let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    //for dhtxx sensor
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    // Use GPIO 22 as an InOutPin
    let mut dht_pin = hal::gpio::InOutPin::new(pins.gpio22);
    let _ = dht_pin.set_high();

    let mut line0_p2: FmtBuf = FmtBuf::new();
    #[cfg(feature = "dht11")]
    write!(&mut line0_p2, "{}", "dht11").unwrap();

    #[cfg(feature = "dht22")]
    write!(&mut line0_p2, "{}", "dht22").unwrap();

    // Perform a sensor reading
    let mut line1 = FmtBuf::new();
    let mut line2 = FmtBuf::new();

    loop {
        // Empty the display:
        // Draw 3 lines of text:
        //reset before loop
        let _ = display.clear(BinaryColor::Off);
        line1.reset();
        line2.reset();

        // Perform a sensor reading
        let measurement = Reading::read(&mut delay, &mut dht_pin).unwrap();
        let (temp, humi) = get(measurement).value();
        //(temp, humi) = (measurement.temperature, measurement.relative_humidity);

        Text::with_baseline("SensorType", Point::new(3, 2),   character_style.clone(), Baseline::Top)
            .draw(&mut display)
            .unwrap();

        Text::with_baseline(
            line0_p2.as_str(),
            Point::new(74, 2),
            character_style.clone(),
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();

        write!(&mut line1, "温度：{}°C", temp).unwrap(); // ℃ ,°C
        Text::with_baseline(
            line1.as_str(),
            Point::new(32, 22),
            character_style.clone(),
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();

        write!(&mut line2, "湿度：{}%", humi).unwrap();
        Text::with_baseline(
            line2.as_str(),
            Point::new(32, 38),
            character_style.clone(),
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();

        Line::new(Point::new(0, 0), Point::new(127, 0))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(&mut display)
            .unwrap();

        Line::new(Point::new(0, 0), Point::new(0, 63))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(&mut display)
            .unwrap();

        Line::new(Point::new(0, 63), Point::new(127, 63))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(&mut display)
            .unwrap();

        Line::new(Point::new(127, 0), Point::new(127, 63))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(&mut display)
            .unwrap();

        Line::new(Point::new(70, 0), Point::new(70, 16))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(&mut display)
            .unwrap();

        Line::new(Point::new(0, 16), Point::new(127, 16))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(&mut display)
            .unwrap();

        Line::new(Point::new(0, 15), Point::new(127, 15))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(&mut display)
            .unwrap();

        display.flush().unwrap();
        // delay for 1 sec
        //per loop is 1 sec
        timer.delay_ms(1000);
    }
}
