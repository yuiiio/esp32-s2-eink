#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    clock::ClockControl,
    delay::Delay,
    gpio::{Io, Level, Output},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);

    let clocks = ClockControl::max(system.clock_control).freeze();
    let delay = Delay::new(&clocks);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = Output::new(io.pins.gpio15, Level::High);

    /* sd card */
    /*
    SDCARD_CS = 34
    SDCARD_MOSI 35
    SDCARD_SCLK = 36
    SDCARD_MISO = 37
    */

    /* eink display */
    const HEIGHT: u16 = 1072;
    const WIDTH: u16 = 1448;

    let mut mode1 = Output::new(io.pins.gpio11, Level::High);
    let mut ckv = Output::new(io.pins.gpio14, Level::High);
    let mut spv = Output::new(io.pins.gpio12, Level::High);

    let mut xcl = Output::new(io.pins.gpio39, Level::High);
    let mut xle = Output::new(io.pins.gpio40, Level::High);
    let mut xoe = Output::new(io.pins.gpio38, Level::High);
    let mut xstl = Output::new(io.pins.gpio33, Level::High);

    let mut d0 = Output::new(io.pins.gpio18, Level::High);
    let mut d1 = Output::new(io.pins.gpio21, Level::High);
    let mut d2 = Output::new(io.pins.gpio16, Level::High);
    let mut d3 = Output::new(io.pins.gpio17, Level::High);
    let mut d4 = Output::new(io.pins.gpio7, Level::High);
    let mut d5 = Output::new(io.pins.gpio9, Level::High);
    let mut d6 = Output::new(io.pins.gpio10, Level::High);
    let mut d7 = Output::new(io.pins.gpio13, Level::High);

    for cycle in 0..32 {
    /* start frame */
    xoe.set_high();
    mode1.set_high();
    spv.set_low();
    ckv.set_low();
    delay.delay(1.micros());
    ckv.set_high();
    spv.set_high();

    for _line in 0..HEIGHT {
        /* write row */
        xstl.set_low();
        for _i in 0..(WIDTH/4) {
            /* black b01010101 */
            /* white b10101010 */
            /* can write 4 pixel for onece */
            if cycle < 16 {
            d0.set_high();
            d1.set_low();
            d2.set_high();
            d3.set_low();
            d4.set_high();
            d5.set_low();
            d6.set_high();
            d7.set_low();
            } else {
            d0.set_low();
            d1.set_high();
            d2.set_low();
            d3.set_high();
            d4.set_low();
            d5.set_high();
            d6.set_low();
            d7.set_high();
            }

            xcl.set_high();
            xcl.set_low();
        }

        xstl.set_high();
        xcl.set_high();
        xcl.set_low();

        xle.set_high();
        xle.set_low();

        ckv.set_low();
        delay.delay(1.micros());
        ckv.set_high();
    }

    /* end frame */
    mode1.set_low();
    xoe.set_low();
    }

    loop {
        led.set_high();
        delay.delay(1.secs());
        led.set_low();
        delay.delay(2.secs());
    }
}
