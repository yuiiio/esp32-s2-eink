#![no_std]
#![no_main]
#![feature(allocator_api)]
#![feature(asm_experimental_arch)]


extern crate alloc;
use alloc::vec::Vec;
use alloc::string::String;

use esp_alloc;

use core::ptr::addr_of_mut;
use core::fmt::Write;
use core::arch::asm;

use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    clock::ClockControl,
    delay::Delay,
    gpio::{Io, Level, Output, AnyOutput, NO_PIN},
    otg_fs::{Usb, UsbBus},
    peripherals::{Peripherals, GPIO, IO_MUX, DEDICATED_GPIO},
    prelude::*,
    system::SystemControl,
    spi::{SpiMode, master::Spi},
    psram,
};

use usb_device::prelude::{UsbDeviceBuilder, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use embedded_storage::{ReadStorage, Storage};
use esp_storage::FlashStorage;

use embedded_sdmmc::{
    Error,
    BlockDevice,
    Mode, VolumeIdx,
    sdcard::{SdCard, DummyCsPin},
    SdCardError,
    VolumeManager,
    Directory,
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

/*
const DR_REG_DEDICATED_GPIO_BASE: usize = 0x3f4cf000;
const DEDICATED_GPIO_OUT_CPU_EN_REG: usize = DR_REG_DEDICATED_GPIO_BASE + 0x10;

const DR_REG_SYSTEM_BASE: usize = 0x3f4c0000;
const DPORT_CPU_PERI_CLK_EN_REG: usize = DR_REG_SYSTEM_BASE + 0x10;
const DPORT_CPU_PERI_RST_EN_REG: usize = DR_REG_SYSTEM_BASE + 0x014;
const DPORT_CLK_EN_DEDICATED_GPIO: usize = 1 << 7;
const DPORT_RST_EN_DEDICATED_GPIO: usize = 1 << 7;
*/

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

}

impl EinkDisplay
{
    #[inline(always)]
    fn start_frame(&mut self) {
        self.xoe.set_high();
        self.mode1.set_high();
        self.spv.set_low();
        self.ckv.set_low();
        unsafe {
            asm!("nop");
            asm!("nop");
            asm!("nop");
            asm!("nop");

            asm!("nop");
            asm!("nop");
            asm!("nop");
            asm!("nop");
        }
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
            // write 8bit
            unsafe { 
                asm!("wur.gpio_out {0}", in(reg) four_pixels);
            };
            self.xcl.set_high();
            self.xcl.set_low();
        }
        self.xstl.set_high();
        self.xcl.set_high();
        self.xcl.set_low();

        self.xle.set_high();
        self.xle.set_low();

        self.ckv.set_low();
        unsafe {
            asm!("nop");
            asm!("nop");
            asm!("nop");
            asm!("nop");

            asm!("nop");
            asm!("nop");
            asm!("nop");
            asm!("nop");
        }
        self.ckv.set_high();
    }

    #[inline(always)]
    fn write_4bpp_image<U: core::alloc::Allocator>(&mut self, img_buf: &Vec<u8, U>) {
        for grayscale in [4, 7, 10] {
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
            // black
            let four_pixels: u8 = 0b01010101;
            unsafe {
                asm!("wur.gpio_out {0}", in(reg) four_pixels);
            }
            self.start_frame();
            for _line in 0..HEIGHT {
                self.xstl.set_low();
                /* can write 4 pixel for onece */
                for _pos in 0..(WIDTH/4) {
                    self.xcl.set_high();
                    self.xcl.set_low();
                }
                self.xstl.set_high();
                self.xcl.set_high();
                self.xcl.set_low();

                self.xle.set_high();
                self.xle.set_low();

                self.ckv.set_low();
                //self.delay.delay(1.micros());
                unsafe {
                    asm!("nop");
                    asm!("nop");
                    asm!("nop");
                    asm!("nop");

                    asm!("nop");
                    asm!("nop");
                    asm!("nop");
                    asm!("nop");
                }
                self.ckv.set_high();
            }
            self.end_frame();
        }
    }
    #[inline(always)]
    fn write_all_white(&mut self) {
        for _cycle in 0..4 {
            // white
            let four_pixels: u8 = 0b10101010;
            unsafe {
                asm!("wur.gpio_out {0}", in(reg) four_pixels);
            }
            self.start_frame();
            for _line in 0..HEIGHT {
                self.xstl.set_low();
                /* can write 4 pixel for onece */
                for _pos in 0..(WIDTH/4) {
                    self.xcl.set_high();
                    self.xcl.set_low();
                }
                self.xstl.set_high();
                self.xcl.set_high();
                self.xcl.set_low();

                self.xle.set_high();
                self.xle.set_low();

                self.ckv.set_low();
                //self.delay.delay(1.micros());
                unsafe {
                    asm!("nop");
                    asm!("nop");
                    asm!("nop");
                    asm!("nop");

                    asm!("nop");
                    asm!("nop");
                    asm!("nop");
                    asm!("nop");
                }
                self.ckv.set_high();
            }
            self.end_frame();
        }
    }
}

fn open_4bpp_image<D: embedded_sdmmc::BlockDevice, T: embedded_sdmmc::TimeSource, const MAX_DIRS: usize, const MAX_FILES: usize, const MAX_VOLUMES: usize, U: core::alloc::Allocator>
(root_dir: &mut Directory<D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>, img_buf: &mut Vec<u8, U>, file_name: &str) -> Result<(), Error<SdCardError>>
where embedded_sdmmc::Error<SdCardError>: From<embedded_sdmmc::Error<<D as BlockDevice>::Error>> {
    let mut file = root_dir.open_file_in_dir(file_name, Mode::ReadOnly)?;

    let mut tiff_header = [0u8; 8];
    file.read(&mut tiff_header)?;// first 8 bytes is annotation header
    
    file.read(img_buf)?;
    Ok(())
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    psram::init_psram(peripherals.PSRAM);
    init_psram_heap();

    /* enable dedicated gpio peripheral */
    peripherals.SYSTEM.cpu_peri_clk_en().modify(|_, w| {
        w.clk_en_dedicated_gpio().bit(true)
    });
    peripherals.SYSTEM.cpu_peri_rst_en().modify(|_, w| {
        w.rst_en_dedicated_gpio().bit(true)
    });
    peripherals.SYSTEM.cpu_peri_rst_en().modify(|_, w| {
        w.rst_en_dedicated_gpio().bit(false)
    });

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
    
    /*
    /* debug */
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
    
    /* flash rom */
    let mut bytes = [0u8; 4];
    let mut flash = FlashStorage::new();
    let flash_addr = 0x9000;// default NVS size: 0x6000
    //write!(serial, "Flash size = {}\n", flash.capacity()).unwrap();
    flash.read(flash_addr, &mut bytes).unwrap();
    //write!(serial, "read = {:02x?}\n", &bytes[..4]).unwrap();
    
    let last_opend_num: u32 = u32::from_be_bytes(bytes);

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
    

    
    /* get the values of dedicated GPIO from the CPU, not peripheral registers */
    for i in 0..8{
        unsafe { &*DEDICATED_GPIO::PTR }.out_cpu().modify(|_, w| {
            w.sel(i).bit(true)
        });
    }

    // # esp32s2 technical reference page 171
    // pro_alonegpio_out0: (235)
    // ..
    // pro_alonegpio_out7: (242)
    // GPIO_FUNCx_OUT_SEL_CFG
    let eink_data_bus_ios: [usize; 8] = [18, 21, 16, 17, 7, 9, 10, 13];  
    for i in 0..8 {
        unsafe { &*GPIO::PTR }.func_out_sel_cfg(eink_data_bus_ios[i]).modify(|_, w| unsafe {
            w.out_sel()
                .bits(235 + (i as u16))
                .inv_sel()
                .bit(false)
                .oen_sel()
                .bit(false)
                .oen_inv_sel()
                .bit(false)
        });
    }
 
    // GPIO_ENABLE_REG(0~31)
    let mut enable_pins: u32 = 0x00000000;
    for i in 0..8 {
        enable_pins |= 1 << (eink_data_bus_ios[i] % 32);
    }

    unsafe { &*GPIO::PTR }.enable_w1ts().write(|w| unsafe {
        w.bits(enable_pins)
    });
    
    // IO_MUX_MCU_SEL
    // hm RegisterBlock in esp32s2 pac doesnot impl gpio(num)
    unsafe { &*IO_MUX::PTR}.gpio18().modify(|_, w| unsafe { w.mcu_sel().bits(1) }); // set to Function 1
    unsafe { &*IO_MUX::PTR}.gpio21().modify(|_, w| unsafe { w.mcu_sel().bits(1) });
    unsafe { &*IO_MUX::PTR}.gpio16().modify(|_, w| unsafe { w.mcu_sel().bits(1) });
    unsafe { &*IO_MUX::PTR}.gpio17().modify(|_, w| unsafe { w.mcu_sel().bits(1) });
    unsafe { &*IO_MUX::PTR}.gpio7().modify(|_, w| unsafe { w.mcu_sel().bits(1) });
    unsafe { &*IO_MUX::PTR}.gpio9().modify(|_, w| unsafe { w.mcu_sel().bits(1) });
    unsafe { &*IO_MUX::PTR}.gpio10().modify(|_, w| unsafe { w.mcu_sel().bits(1) });
    unsafe { &*IO_MUX::PTR}.gpio13().modify(|_, w| unsafe { w.mcu_sel().bits(1) });

    let mode1 = AnyOutput::new(io.pins.gpio11, Level::High);
    let ckv = AnyOutput::new(io.pins.gpio14, Level::High);
    let spv = AnyOutput::new(io.pins.gpio12, Level::High);

    let xcl = AnyOutput::new(io.pins.gpio39, Level::High);
    let xle = AnyOutput::new(io.pins.gpio40, Level::High);
    let xoe = AnyOutput::new(io.pins.gpio38, Level::High);
    let xstl = AnyOutput::new(io.pins.gpio33, Level::High);

    let mut eink_display = EinkDisplay { mode1, ckv, spv, xcl, xle, xoe, xstl };
    eink_display.write_all_white();
    eink_display.write_all_black();

    let mut volume0 = volume_manager.open_volume(VolumeIdx(0)).expect("failed to open volume");
    let mut root_dir = volume0.open_root_dir().expect("failed to open volume");

    let mut file_name = String::with_capacity(7); //xx.tif

    let mut i: u32 = if last_opend_num > 99 {
        0
    } else {
        last_opend_num
    };

    /*self cpu impl touch pad*/
    let mut touch_out = AnyOutput::new(io.pins.gpio1, Level::Low);

    let mut adc1_config = AdcConfig::new();
    let mut touch_left = adc1_config.enable_pin(io.pins.gpio2, Attenuation::Attenuation11dB);
    let mut touch_right = adc1_config.enable_pin(io.pins.gpio3, Attenuation::Attenuation11dB);
    let mut touch_center = adc1_config.enable_pin(io.pins.gpio4, Attenuation::Attenuation11dB);
    let mut touch_top = adc1_config.enable_pin(io.pins.gpio5, Attenuation::Attenuation11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);

    /*
    let mut f = 0;
    #[allow(unreachable_code)]
    loop {
        touch_out.set_high();
        delay.delay(40.micros());
        touch_out.set_low();
        delay.delay(10.micros());
        let left_pin_value = adc1.read_blocking(&mut touch_left);
        touch_out.set_high();
        delay.delay(40.micros());
        touch_out.set_low();
        delay.delay(10.micros());
        let right_pin_value = adc1.read_blocking(&mut touch_right);
        if !usb_dev.poll(&mut [&mut serial.0]) {
            continue;
        }
        f = f+1;
        if f > 4000 {
            write!(serial, "left: {}, right: {}\n", left_pin_value, right_pin_value).unwrap();
            f = 0;
        }
    }

    #[allow(unreachable_code)]
    */
    loop {
        'inner: loop {
            touch_out.set_high();
            delay.delay(40.micros());
            touch_out.set_low();
            delay.delay(10.micros());
            let left_pin_value = adc1.read_blocking(&mut touch_left);
            touch_out.set_high();
            delay.delay(40.micros());
            touch_out.set_low();
            delay.delay(10.micros());
            let right_pin_value = adc1.read_blocking(&mut touch_right);

            if left_pin_value > 5110 {
                if i == 0 {
                    //i = 99;
                } else {
                    i = i - 1;
                }
                break 'inner;
            }
            if right_pin_value > 5190 {
                if i == 99 {
                    i = 0;
                } else {
                    i = i + 1;
                }
                break 'inner;
            }
        }
        file_name.clear();
        write!(&mut file_name, "{0: >02}.tif", i).unwrap();
        match open_4bpp_image(&mut root_dir, &mut img_buf, &file_name) {
            Ok(_) => {
                //eink_display.write_4bpp_reverse_image(&img_buf);
                eink_display.write_all_black();
                eink_display.write_4bpp_image(&img_buf);
                flash.write(flash_addr, &i.to_be_bytes()).unwrap();
            },
            Err(_error) => {
                /* not found file */
            },
        };
    }
}
