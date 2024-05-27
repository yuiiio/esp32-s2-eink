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
    let mut touch_out = io.pins.gpio3;

    let mut adc1_config = AdcConfig::new();
    let mut adc1_pin = adc1_config.enable_pin(io.pins.gpio5, Attenuation::Attenuation11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);

    loop {
        touch_out.set_high();
        delay.delay(1.micros());
        touch_out.set_low();
        delay.delay(1.micros());
        let pin_value: u16 = nb::block!(adc1.read_oneshot(&mut adc1_pin)).unwrap();
        if pin_value > (1 << 12) {
            led.set_high();
        } else {
            led.set_low();
        }
    }
}
