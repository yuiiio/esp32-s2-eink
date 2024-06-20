#![no_std]
#![no_main]
#![feature(allocator_api)]


extern crate alloc;
use alloc::vec::Vec;

use esp_alloc;

use core::ptr::addr_of_mut;
use core::fmt::Write;

use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    clock::ClockControl,
    delay::Delay,
    gpio::{Io, Level, Output, AnyOutput, NO_PIN},
    otg_fs::{Usb, UsbBus},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    spi::{SpiMode, master::Spi},
    psram,
};
use usb_device::prelude::{UsbDeviceBuilder, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use embedded_sdmmc::{
    Mode, VolumeIdx,
    sdcard::{SdCard, DummyCsPin},
    VolumeManager,
};
use embedded_hal_bus::spi::ExclusiveDevice;

#[global_allocator]
static PSRAM_ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_psram_heap() {
    unsafe {
        PSRAM_ALLOCATOR.init(psram::psram_vaddr_start() as *mut u8, psram::PSRAM_BYTES);
    }
}

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

struct SerialWrapper<'b, B: usb_device::class_prelude::UsbBus>(SerialPort<'b, B>);
impl<B: usb_device::class_prelude::UsbBus> core::fmt::Write for SerialWrapper<'_, B> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let _ = self.0.write(s.as_bytes());
        Ok(())
    }
}

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

const FOUR_BPP_BUF_SIZE: usize = WIDTH*HEIGHT*4/8;

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

    #[inline(always)]
    fn write_4bpp_image<U: core::alloc::Allocator>(&mut self, img_buf: &Vec<u8, U>) {
        for grayscale in 5..9 {
            let mut pos: usize = 0;
            self.start_frame();
            for _line in 0..HEIGHT {
                let mut buf: [u8; WIDTH/4] = [0u8; WIDTH/4];
                for i in 0..(WIDTH/4) {
                    let mut b: u8 = 0b10101010; // white
                    if (img_buf[pos] >> 4) <= grayscale { b ^= 0b11000000 };//reverse => black 
                    if (img_buf[pos] & 0x0f) <= grayscale { b ^= 0b00110000 };
                    pos += 1;
                    if (img_buf[pos] >> 4) <= grayscale { b ^= 0b00001100 };
                    if (img_buf[pos] & 0x0f) <= grayscale { b ^= 0b00000011 };
                    pos += 1;
                    buf[i] = b;
                }
                self.write_row(&buf);
            }
            self.end_frame();
        }
    }
    #[inline(always)]
    fn write_4bpp_reverse_image<U: core::alloc::Allocator>(&mut self, img_buf: &Vec<u8, U>) {
        for grayscale in 6..8 {
            let mut pos: usize = 0;
            self.start_frame();
            for _line in 0..HEIGHT {
                let mut buf: [u8; WIDTH/4] = [0u8; WIDTH/4];
                for i in 0..(WIDTH/4) {
                    let mut b: u8 = 0b10101010; // white
                    if (img_buf[pos] >> 4) > grayscale { b ^= 0b11000000 };//reverse => black 
                    if (img_buf[pos] & 0x0f) > grayscale { b ^= 0b00110000 };
                    pos += 1;
                    if (img_buf[pos] >> 4) > grayscale { b ^= 0b00001100 };
                    if (img_buf[pos] & 0x0f) > grayscale { b ^= 0b00000011 };
                    pos += 1;
                    buf[i] = b;
                }
                self.write_row(&buf);
            }
            self.end_frame();
        }
    }

    #[inline(always)]
    fn write_all_black(&mut self) {
        for _cycle in 0..4 {
            self.start_frame();
            for _line in 0..HEIGHT {
                let raw_data: [u8; WIDTH/4] = [0b01010101; WIDTH/4];//black
                self.write_row(&raw_data);
            }
            self.end_frame();
        }
    }
    #[inline(always)]
    fn write_all_white(&mut self) {
        for _cycle in 0..4 {
            self.start_frame();
            for _line in 0..HEIGHT {
                let raw_data: [u8; WIDTH/4] = [0b10101010; WIDTH/4];//white
                self.write_row(&raw_data);
            }
            self.end_frame();
        }
    }
}

fn open_4bpp_image<D: embedded_sdmmc::BlockDevice, T: embedded_sdmmc::TimeSource, U: core::alloc::Allocator>(volume_manager: &mut VolumeManager<D, T>, img_buf: &mut Vec<u8, U>, file_name: &str) {
    match volume_manager.open_volume(VolumeIdx(0)) {
        Ok(mut volume0) => {  
            let mut root_dir = volume0.open_root_dir().unwrap();
            /*
            let mut example_file = root_dir.open_file_in_dir("MY_FILE.TXT", Mode::ReadOnly).unwrap();
            let mut txt1 = [0u8; 2];
            example_file.read(&mut txt1).unwrap();
            let mut txt2 = [0u8; 3];
            example_file.read(&mut txt2).unwrap();
            loop {
                if usb_dev.poll(&mut [&mut serial.0]) {
                    break;
                }
            }
            writeln!(serial, "MY_FILE.TXT: txt1: {:?}\n", txt1).unwrap();
            writeln!(serial, "MY_FILE.TXT: txt2: {:?}\n", txt2).unwrap();
            */
            let mut file = root_dir.open_file_in_dir(file_name, Mode::ReadOnly).unwrap();
            let mut tiff_header = [0u8; 8];
            file.read(&mut tiff_header).unwrap();// first 8 bytes is annotation header
            /*
            loop {
                if usb_dev.poll(&mut [&mut serial.0]) {
                    break;
                }
            }
            writeln!(serial, "output.tif header: {:?}\n", tiff_header).unwrap();
            */
            file.read(img_buf).unwrap();
        },
        Err(error) => {
            /*
            loop {
                if usb_dev.poll(&mut [&mut serial.0]) {
                    break;
                }
            }
            writeln!(serial, "open_volume Err {:?}\n", error).unwrap();
            */
        },
    };
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    psram::init_psram(peripherals.PSRAM);
    init_psram_heap();

    let system = SystemControl::new(peripherals.SYSTEM);

    let clocks = ClockControl::max(system.clock_control).freeze();
    let delay = Delay::new(&clocks);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = AnyOutput::new(io.pins.gpio15, Level::High);

    /*usb serial debug*/
    let usb = Usb::new(peripherals.USB0, io.pins.gpio19, io.pins.gpio20);
    let usb_bus = UsbBus::new(usb, unsafe { &mut *addr_of_mut!(EP_MEMORY) });

    let serial = SerialPort::new(&usb_bus);
    let mut serial = SerialWrapper(serial);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x303A, 0x3001))
        .device_class(USB_CLASS_CDC)
        .build();
    
    /* debug */
    /*
    'outer: loop {
        if !usb_dev.poll(&mut [&mut serial.0]) {
            continue;
        }

        let mut buf = [0u8; 64];

        match serial.0.read(&mut buf) {
            Ok(count) if count > 0 => {
                // Echo back in upper case
                for c in buf[0..count].iter_mut() {
                    if *c == 0x63 { // c
                        break 'outer;
                    }
                    if 0x61 <= *c && *c <= 0x7a {
                        *c &= !0x20;
                    }
                }

                let mut write_offset = 0;
                while write_offset < count {
                    match serial.0.write(&buf[write_offset..count]) {
                        Ok(len) => { 
                            if len > 0 {
                                write_offset += len;
                            }
                        },
                        _ => {}
                    }
                }
            }
            _ => {}
        }
    }

    writeln!(serial, "\nSuccess serialWrapper test\n").unwrap();
    */

    /* sd card */
    let sclk = io.pins.gpio36;
    let miso = io.pins.gpio37;
    let mosi = io.pins.gpio35;
    let cs = Output::new(io.pins.gpio34, Level::High);

    let spi = Spi::new(
        peripherals.SPI2,
        25u32.MHz(),
        SpiMode::Mode0,
        &clocks,
    ).with_pins(Some(sclk), Some(mosi), Some(miso), NO_PIN);

    let spi_device = ExclusiveDevice::new_no_delay(spi, DummyCsPin).unwrap();

    let sdcard = SdCard::new(spi_device, cs, delay);

    /*
    loop {
        if usb_dev.poll(&mut [&mut serial.0]) {
            break;
        }
    }
    writeln!(serial, "Card size is {} bytes\n", sdcard.num_bytes().unwrap()).unwrap();
    */

    let mut volume_manager = VolumeManager::new(sdcard, FakeTimesource{});

    let mut img_buf: Vec<u8, _> = Vec::with_capacity_in(FOUR_BPP_BUF_SIZE, &PSRAM_ALLOCATOR);
    for _i in 0..FOUR_BPP_BUF_SIZE {
        img_buf.push(0u8);
    }
    // too big for dram? so use psram(2M)
    
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

    eink_display.write_all_white();
    eink_display.write_all_black();
    led.set_low();
    loop {
        open_4bpp_image(&mut volume_manager, &mut img_buf, "02.tif");
        //eink_display.write_all_black();
        //reverse image can effective clear than all black flush ?
        eink_display.write_4bpp_reverse_image(&img_buf);
        eink_display.write_4bpp_image(&img_buf);
        led.set_high();
        delay.delay(2.secs());
        led.set_low();

        open_4bpp_image(&mut volume_manager, &mut img_buf, "01.tif");
        //eink_display.write_all_black();
        eink_display.write_4bpp_reverse_image(&img_buf);
        eink_display.write_4bpp_image(&img_buf);
        led.set_high();
        delay.delay(2.secs());
        led.set_low();
    }
}
