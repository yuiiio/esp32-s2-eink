#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    clock::ClockControl,
    delay::Delay,
    gpio::{Io, Level, Output, AnyOutput, NO_PIN},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    spi::{SpiMode, master::Spi},
};
use embedded_sdmmc::{
    Mode, VolumeIdx,
    sdcard::{SdCard, DummyCsPin},
    VolumeManager,
};
use embedded_hal_bus::spi::ExclusiveDevice;

struct FakeTimesource {}

impl embedded_sdmmc::TimeSource for FakeTimesource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

/* eink display */
const HEIGHT: usize = 1072;
const WIDTH: usize = 1448;

struct EinkDisplay {
    pub mode1: AnyOutput<'static>,
    pub ckv: AnyOutput<'static>,
    pub spv: AnyOutput<'static>,

    pub xcl: AnyOutput<'static>,
    pub xle: AnyOutput<'static>,
    pub xoe: AnyOutput<'static>,
    pub xstl: AnyOutput<'static>,

    pub d0: AnyOutput<'static>,
    pub d1: AnyOutput<'static>,
    pub d2: AnyOutput<'static>,
    pub d3: AnyOutput<'static>,
    pub d4: AnyOutput<'static>,
    pub d5: AnyOutput<'static>,
    pub d6: AnyOutput<'static>,
    pub d7: AnyOutput<'static>,

    pub delay: Delay,
}

impl EinkDisplay
{
    #[inline(always)]
    fn start_frame(&mut self) {
        self.xoe.set_high();
        self.mode1.set_high();
        self.spv.set_low();
        self.ckv.set_low();
        self.delay.delay(1.micros());
        self.ckv.set_high();
        self.spv.set_high();
    }

    #[inline(always)]
    fn end_frame(&mut self) {
        self.mode1.set_low();
        self.xoe.set_low();
    }

    // 1pixel 2 bit, so *2 data
    // [0, 0] No action
    // [0, 1] Draw black
    // [1, 0] Draw white
    // [1, 1] No action
    #[inline(always)]
    fn write_row(&mut self, row_data: &[u8; WIDTH/4]) {
        self.xstl.set_low();
        /* can write 4 pixel for onece */
        for pos in 0..(WIDTH/4) {
            let four_pixels = row_data[pos];
            if four_pixels & 0b00000001 == 0b00000001 {
                self.d0.set_high();
            } else {
                self.d0.set_low();
            }
            if four_pixels & 0b00000010 == 0b00000010 {
                self.d1.set_high();
            } else {
                self.d1.set_low();
            }
            if four_pixels & 0b00000100 == 0b00000100 {
                self.d2.set_high();
            } else {
                self.d2.set_low();
            }
            if four_pixels & 0b00001000 ==  0b0001000 {
                self.d3.set_high();
            } else {
                self.d3.set_low();
            }
            if four_pixels & 0b00010000 == 0b00010000 {
                self.d4.set_high();
            } else {
                self.d4.set_low();
            }
            if four_pixels & 0b00100000 == 0b00100000 {
                self.d5.set_high();
            } else {
                self.d5.set_low();
            }
            if four_pixels & 0b01000000 == 0b01000000 {
                self.d6.set_high();
            } else {
                self.d6.set_low();
            }
            if four_pixels & 0b10000000 == 0b10000000 {
                self.d7.set_high();
            } else {
                self.d7.set_low();
            }
            self.xcl.set_high();
            self.xcl.set_low();
        }
        self.xstl.set_high();
        self.xcl.set_high();
        self.xcl.set_low();

        self.xle.set_high();
        self.xle.set_low();

        self.ckv.set_low();
        self.delay.delay(1.micros());
        self.ckv.set_high();
    }
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);

    let clocks = ClockControl::max(system.clock_control).freeze();
    let delay = Delay::new(&clocks);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = AnyOutput::new(io.pins.gpio15, Level::High);

    /* sd card */
    let sclk = io.pins.gpio36;
    let miso = io.pins.gpio37;
    let mosi = io.pins.gpio35;
    let cs = Output::new(io.pins.gpio34, Level::Low);

    let spi = Spi::new(
        peripherals.SPI2,
        400u32.kHz(),
        SpiMode::Mode0,
        &clocks,
    ).with_pins(Some(sclk), Some(mosi), Some(miso), NO_PIN);

    let spi_device = ExclusiveDevice::new_no_delay(spi, DummyCsPin).unwrap();

    let sdcard = SdCard::new(spi_device, cs, delay);

    let mut volume_manager = VolumeManager::new(sdcard, FakeTimesource{});

    /*
    let mut volume0 = volume_manager.open_volume(VolumeIdx(0)).unwrap();
    let mut root_dir = volume0.open_root_dir().unwrap();
    if let Ok(file) = root_dir.open_file_in_dir("MY_FILE.TXT", Mode::ReadOnly) {
    }
    */

    let mode1 = AnyOutput::new(io.pins.gpio11, Level::High);
    let ckv = AnyOutput::new(io.pins.gpio14, Level::High);
    let spv = AnyOutput::new(io.pins.gpio12, Level::High);

    let xcl = AnyOutput::new(io.pins.gpio39, Level::High);
    let xle = AnyOutput::new(io.pins.gpio40, Level::High);
    let xoe = AnyOutput::new(io.pins.gpio38, Level::High);
    let xstl = AnyOutput::new(io.pins.gpio33, Level::High);

    let d0 = AnyOutput::new(io.pins.gpio18, Level::High);
    let d1 = AnyOutput::new(io.pins.gpio21, Level::High);
    let d2 = AnyOutput::new(io.pins.gpio16, Level::High);
    let d3 = AnyOutput::new(io.pins.gpio17, Level::High);
    let d4 = AnyOutput::new(io.pins.gpio7, Level::High);
    let d5 = AnyOutput::new(io.pins.gpio9, Level::High);
    let d6 = AnyOutput::new(io.pins.gpio10, Level::High);
    let d7 = AnyOutput::new(io.pins.gpio13, Level::High);

    let mut eink_display = EinkDisplay { mode1, ckv, spv, xcl, xle, xoe, xstl, d0, d1, d2, d3, d4, d5, d6, d7, delay };

    loop {
        led.set_high();
        for _cycle in 0..4 {
            eink_display.start_frame();
            for _line in 0..HEIGHT {
                let raw_data: [u8; WIDTH/4] = [0b01010101; WIDTH/4];
                eink_display.write_row(&raw_data);
            }
            eink_display.end_frame();
        }

        led.set_low();
        for _cycle in 0..4 {
            eink_display.start_frame();
            for _line in 0..HEIGHT {
                let raw_data: [u8; WIDTH/4] = [0b10101010; WIDTH/4];
                eink_display.write_row(&raw_data);
            }
            eink_display.end_frame();
        }
    }
}
