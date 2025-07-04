#![no_std]
#![no_main]
#![feature(allocator_api)]
#![feature(asm_experimental_arch)]

extern crate alloc;
use alloc::string::String;
use alloc::vec::Vec;

use esp_alloc;

use core::arch::asm;
use core::fmt::Write;
use core::ptr::addr_of_mut;

use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    delay::Delay,
    gpio::{Io, Level, Output, OutputConfig},
    main,
    otg_fs::{Usb, UsbBus},
    peripherals::{DEDICATED_GPIO, GPIO, IO_MUX, SYSTEM},
    psram,
    spi::master::Spi,
    time::Rate,
};

use usb_device::prelude::{UsbDeviceBuilder, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use embedded_storage::{ReadStorage, Storage};
use esp_storage::FlashStorage;

use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{
    sdcard::SdCard, BlockDevice, Directory, Error, Mode, SdCardError, ShortFileName, VolumeIdx,
    VolumeManager,
};

//static PSRAM_ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();
// seems conflicts with global allocator maybe current esp-hal issue.

fn init_psram_heap(start: *mut u8, size: usize) {
    unsafe {
        //PSRAM_ALLOCATOR.add_region(esp_alloc::HeapRegion::new(
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            start,
            size,
            esp_alloc::MemoryCapability::External.into(), // Internal ?
        ));
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

const FOUR_BPP_BUF_SIZE: usize = WIDTH * HEIGHT * 4 / 8;

const BLACK_FOUR_PIXEL: u8 = 0b01010101;
const WHITE_FOUR_PIXEL: u8 = 0b10101010;
const NONE_FOUR_PIXEL: u8 = 0b00000000;

struct EinkDisplay {
    pub mode1: Output<'static>,
    pub ckv: Output<'static>,
    pub spv: Output<'static>,

    pub xcl: Output<'static>,
    pub xle: Output<'static>,
    pub xoe: Output<'static>,
    pub xstl: Output<'static>,
    pub delay: Delay,
}

impl EinkDisplay {
    #[inline(always)]
    fn start_frame(&mut self) {
        self.xoe.set_high();
        self.mode1.set_high();
        self.spv.set_low();
        self.ckv.set_low();
        self.delay.delay_micros(1);
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
    fn write_row(&mut self, row_data: &[u8; WIDTH / 4]) {
        self.xstl.set_low();
        /* can write 4 pixel for onece */
        for pos in 0..(WIDTH / 4) {
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
        self.delay.delay_micros(1);
        self.ckv.set_high();
    }

    fn write_4bpp_image<U: core::alloc::Allocator>(&mut self, img_buf: &Vec<u8, U>) {
        for grayscale in [12, 8] {
            let mut pos: usize = 0;
            let mut buf: [u8; WIDTH / 4] = [0u8; WIDTH / 4];
            self.start_frame();

            for i in 0..(WIDTH / 4) {
                let mut b: u8 = WHITE_FOUR_PIXEL; // white
                if (img_buf[pos] >> 4) <= grayscale {
                    b ^= 0b11000000
                }; //reverse => black
                if (img_buf[pos] & 0x0f) <= grayscale {
                    b ^= 0b00110000
                };
                pos += 1;
                if (img_buf[pos] >> 4) <= grayscale {
                    b ^= 0b00001100
                };
                if (img_buf[pos] & 0x0f) <= grayscale {
                    b ^= 0b00000011
                };
                pos += 1;
                buf[i] = b;
            }
            for _line in 0..(HEIGHT - 1) {
                self.xstl.set_low();
                /* can write 4 pixel for onece */
                for pos in 0..(WIDTH / 4) {
                    let four_pixels = buf[pos];
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
                //self.delay.delay_micros(1);
                for i in 0..(WIDTH / 4) {
                    let mut b: u8 = WHITE_FOUR_PIXEL; // white
                    if (img_buf[pos] >> 4) <= grayscale {
                        b ^= 0b11000000
                    }; //reverse => black
                    if (img_buf[pos] & 0x0f) <= grayscale {
                        b ^= 0b00110000
                    };
                    pos += 1;
                    if (img_buf[pos] >> 4) <= grayscale {
                        b ^= 0b00001100
                    };
                    if (img_buf[pos] & 0x0f) <= grayscale {
                        b ^= 0b00000011
                    };
                    pos += 1;
                    buf[i] = b;
                }
                self.ckv.set_high();
            }
            self.xstl.set_low();
            /* can write 4 pixel for onece */
            for pos in 0..(WIDTH / 4) {
                let four_pixels = buf[pos];
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
            self.delay.delay_micros(1);
            self.ckv.set_high();
            self.end_frame();
        }
    }

    fn write_4bpp_reverse_image<U: core::alloc::Allocator>(&mut self, img_buf: &Vec<u8, U>) {
        for grayscale in [8] {
            let mut pos: usize = 0;
            let mut buf: [u8; WIDTH / 4] = [0u8; WIDTH / 4];
            self.start_frame();

            for i in 0..(WIDTH / 4) {
                let mut b: u8 = WHITE_FOUR_PIXEL; // white
                if (img_buf[pos] >> 4) > grayscale {
                    b ^= 0b11000000
                }; //reverse => black
                if (img_buf[pos] & 0x0f) > grayscale {
                    b ^= 0b00110000
                };
                pos += 1;
                if (img_buf[pos] >> 4) > grayscale {
                    b ^= 0b00001100
                };
                if (img_buf[pos] & 0x0f) > grayscale {
                    b ^= 0b00000011
                };
                pos += 1;
                buf[i] = b;
            }
            for _line in 0..(HEIGHT - 1) {
                self.xstl.set_low();
                /* can write 4 pixel for onece */
                for pos in 0..(WIDTH / 4) {
                    let four_pixels = buf[pos];
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
                //self.delay.delay_micros(1);
                for i in 0..(WIDTH / 4) {
                    let mut b: u8 = WHITE_FOUR_PIXEL; // white
                    if (img_buf[pos] >> 4) > grayscale {
                        b ^= 0b11000000
                    }; //reverse => black
                    if (img_buf[pos] & 0x0f) > grayscale {
                        b ^= 0b00110000
                    };
                    pos += 1;
                    if (img_buf[pos] >> 4) > grayscale {
                        b ^= 0b00001100
                    };
                    if (img_buf[pos] & 0x0f) > grayscale {
                        b ^= 0b00000011
                    };
                    pos += 1;
                    buf[i] = b;
                }
                self.ckv.set_high();
            }
            self.xstl.set_low();
            /* can write 4 pixel for onece */
            for pos in 0..(WIDTH / 4) {
                let four_pixels = buf[pos];
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
            self.delay.delay_micros(1);
            self.ckv.set_high();
            self.end_frame();
        }
    }

    fn write_4bpp_partialy_reverse_image<U: core::alloc::Allocator>(
        &mut self,
        img_buf: &Vec<u8, U>,
    ) {
        for grayscale in [8] {
            let mut pos: usize = 0;
            let mut buf: [u8; WIDTH / 4] = [0u8; WIDTH / 4];
            self.start_frame();

            for i in 0..(WIDTH / 4) {
                let mut b: u8 = NONE_FOUR_PIXEL;
                if (img_buf[pos] >> 4) > grayscale {
                    b ^= 0b01000000
                }; //NONE => black
                if (img_buf[pos] & 0x0f) > grayscale {
                    b ^= 0b00010000
                };
                pos += 1;
                if (img_buf[pos] >> 4) > grayscale {
                    b ^= 0b00000100
                };
                if (img_buf[pos] & 0x0f) > grayscale {
                    b ^= 0b00000001
                };
                pos += 1;
                buf[i] = b;
            }
            for _line in 0..(HEIGHT - 1) {
                self.xstl.set_low();
                /* can write 4 pixel for onece */
                for pos in 0..(WIDTH / 4) {
                    let four_pixels = buf[pos];
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
                //self.delay.delay_micros(1);
                for i in 0..(WIDTH / 4) {
                    let mut b: u8 = NONE_FOUR_PIXEL;
                    if (img_buf[pos] >> 4) > grayscale {
                        b ^= 0b01000000
                    }; //NONE => black
                    if (img_buf[pos] & 0x0f) > grayscale {
                        b ^= 0b00010000
                    };
                    pos += 1;
                    if (img_buf[pos] >> 4) > grayscale {
                        b ^= 0b00000100
                    };
                    if (img_buf[pos] & 0x0f) > grayscale {
                        b ^= 0b00000001
                    };
                    pos += 1;
                    buf[i] = b;
                }
                self.ckv.set_high();
            }
            self.xstl.set_low();
            /* can write 4 pixel for onece */
            for pos in 0..(WIDTH / 4) {
                let four_pixels = buf[pos];
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
            self.delay.delay_micros(1);
            self.ckv.set_high();
            self.end_frame();
        }
    }

    fn write_all_black(&mut self) {
        for _cycle in 0..1 {
            // black
            let four_pixels: u8 = BLACK_FOUR_PIXEL;
            unsafe {
                asm!("wur.gpio_out {0}", in(reg) four_pixels);
            }
            self.start_frame();
            for _line in 0..HEIGHT {
                self.xstl.set_low();
                /* can write 4 pixel for onece */
                for _pos in 0..(WIDTH / 4) {
                    self.xcl.set_high();
                    self.xcl.set_low();
                }
                self.xstl.set_high();
                self.xcl.set_high();
                self.xcl.set_low();

                self.xle.set_high();
                self.xle.set_low();

                self.ckv.set_low();
                self.delay.delay_micros(1);
                self.ckv.set_high();
            }
            self.end_frame();
        }
    }

    fn write_all_white(&mut self) {
        for _cycle in 0..1 {
            // white
            let four_pixels: u8 = WHITE_FOUR_PIXEL;
            unsafe {
                asm!("wur.gpio_out {0}", in(reg) four_pixels);
            }
            self.start_frame();
            for _line in 0..HEIGHT {
                self.xstl.set_low();
                /* can write 4 pixel for onece */
                for _pos in 0..(WIDTH / 4) {
                    self.xcl.set_high();
                    self.xcl.set_low();
                }
                self.xstl.set_high();
                self.xcl.set_high();
                self.xcl.set_low();

                self.xle.set_high();
                self.xle.set_low();

                self.ckv.set_low();
                self.delay.delay_micros(1);
                self.ckv.set_high();
            }
            self.end_frame();
        }
    }

    fn write_all_black_white(&mut self) {
        // black
        let four_pixels: u8 = BLACK_FOUR_PIXEL;
        unsafe {
            asm!("wur.gpio_out {0}", in(reg) four_pixels);
        }
        self.start_frame();
        for _line in 0..HEIGHT {
            self.xstl.set_low();
            /* can write 4 pixel for onece */
            for _pos in 0..(WIDTH / 4) {
                self.xcl.set_high();
                self.xcl.set_low();
            }
            self.xstl.set_high();
            self.xcl.set_high();
            self.xcl.set_low();

            self.xle.set_high();
            self.xle.set_low();

            self.ckv.set_low();
            self.delay.delay_micros(1);
            self.ckv.set_high();
        }
        self.end_frame();
        // white
        let four_pixels: u8 = WHITE_FOUR_PIXEL;
        unsafe {
            asm!("wur.gpio_out {0}", in(reg) four_pixels);
        }
        self.start_frame();
        for _line in 0..HEIGHT {
            self.xstl.set_low();
            /* can write 4 pixel for onece */
            for _pos in 0..(WIDTH / 4) {
                self.xcl.set_high();
                self.xcl.set_low();
            }
            self.xstl.set_high();
            self.xcl.set_high();
            self.xcl.set_low();

            self.xle.set_high();
            self.xle.set_low();

            self.ckv.set_low();
            self.delay.delay_micros(1);
            self.ckv.set_high();
        }
        self.end_frame();
    }

    fn write_top_indicator(&mut self, first_commit: bool, status_var: u32, pre_status_var: u32) {
        // partial update
        const TOP_INDICATOR_WIDTH_DIV_4: usize = 100 / 4;
        const SPLIT_WIDTH: usize = 4 / 4;
        const BAR_WIDTH: u32 = 20;
        for _cycle in 0..1 {
            self.start_frame();
            for line in 0..HEIGHT {
                self.xstl.set_low();
                if first_commit {
                    if status_var.abs_diff(line as u32) <= BAR_WIDTH {
                        //black button
                        let four_pixels: u8 = BLACK_FOUR_PIXEL;
                        unsafe {
                            asm!("wur.gpio_out {0}", in(reg) four_pixels);
                        }
                        for _i in 0..(TOP_INDICATOR_WIDTH_DIV_4 - SPLIT_WIDTH) {
                            // draw bar
                            self.xcl.set_high();
                            self.xcl.set_low();
                        }
                    } else {
                        //white backgruond
                        let four_pixels: u8 = WHITE_FOUR_PIXEL;
                        unsafe {
                            asm!("wur.gpio_out {0}", in(reg) four_pixels);
                        }
                        for _i in 0..(TOP_INDICATOR_WIDTH_DIV_4 - SPLIT_WIDTH) {
                            // draw bar
                            self.xcl.set_high();
                            self.xcl.set_low();
                        }
                    }
                    //black
                    let four_pixels: u8 = BLACK_FOUR_PIXEL;
                    unsafe {
                        asm!("wur.gpio_out {0}", in(reg) four_pixels);
                    }
                    for _i in 0..SPLIT_WIDTH {
                        // draw split bar
                        self.xcl.set_high();
                        self.xcl.set_low();
                    }
                } else {
                    if status_var.abs_diff(line as u32) <= BAR_WIDTH {
                        if pre_status_var.abs_diff(line as u32) > BAR_WIDTH {
                            //black button
                            let four_pixels: u8 = BLACK_FOUR_PIXEL;
                            unsafe {
                                asm!("wur.gpio_out {0}", in(reg) four_pixels);
                            }
                            for _i in 0..(TOP_INDICATOR_WIDTH_DIV_4 - SPLIT_WIDTH) {
                                // draw bar
                                self.xcl.set_high();
                                self.xcl.set_low();
                            }
                        } else {
                            //none
                            let four_pixels: u8 = 0b00000000;
                            unsafe {
                                asm!("wur.gpio_out {0}", in(reg) four_pixels);
                            }
                            for _i in 0..(TOP_INDICATOR_WIDTH_DIV_4 - SPLIT_WIDTH) {
                                // draw bar
                                self.xcl.set_high();
                                self.xcl.set_low();
                            }
                        }
                    } else {
                        if pre_status_var.abs_diff(line as u32) <= BAR_WIDTH {
                            //white background
                            let four_pixels: u8 = WHITE_FOUR_PIXEL;
                            unsafe {
                                asm!("wur.gpio_out {0}", in(reg) four_pixels);
                            }
                            for _i in 0..(TOP_INDICATOR_WIDTH_DIV_4 - SPLIT_WIDTH) {
                                // clear pre bar
                                self.xcl.set_high();
                                self.xcl.set_low();
                            }
                        } else {
                            //none
                            let four_pixels: u8 = 0b00000000;
                            unsafe {
                                asm!("wur.gpio_out {0}", in(reg) four_pixels);
                            }
                            for _i in 0..(TOP_INDICATOR_WIDTH_DIV_4 - SPLIT_WIDTH) {
                                // draw bar
                                self.xcl.set_high();
                                self.xcl.set_low();
                            }
                        }
                    }
                }
                // none
                let four_pixels: u8 = 0b00000000;
                unsafe {
                    asm!("wur.gpio_out {0}", in(reg) four_pixels);
                }
                /* can write 4 pixel for onece */
                for _i in 0..((WIDTH / 4) - TOP_INDICATOR_WIDTH_DIV_4) {
                    //skip not changed area
                    self.xcl.set_high();
                    self.xcl.set_low();
                }

                self.xstl.set_high();
                self.xcl.set_high();
                self.xcl.set_low();

                self.xle.set_high();
                self.xle.set_low();

                self.ckv.set_low();
                self.delay.delay_micros(1);
                self.ckv.set_high();
            }
            self.end_frame();
        }
    }

    fn write_bottom_indicator(&mut self, first_commit: bool, status_var: u32, pre_status_var: u32) {
        // partial update
        const BOTTOM_INDICATOR_WIDTH_DIV_4: usize = 100 / 4;
        const SPLIT_WIDTH: usize = 4 / 4;
        const BAR_WIDTH: u32 = 20;
        for _cycle in 0..1 {
            self.start_frame();
            for line in 0..HEIGHT {
                // none
                let four_pixels: u8 = 0b00000000;
                unsafe {
                    asm!("wur.gpio_out {0}", in(reg) four_pixels);
                }
                self.xstl.set_low();
                /* can write 4 pixel for onece */
                for _i in 0..((WIDTH / 4) - BOTTOM_INDICATOR_WIDTH_DIV_4) {
                    //skip not changed area
                    self.xcl.set_high();
                    self.xcl.set_low();
                }
                if first_commit {
                    //black
                    let four_pixels: u8 = BLACK_FOUR_PIXEL;
                    unsafe {
                        asm!("wur.gpio_out {0}", in(reg) four_pixels);
                    }
                    self.delay.delay_micros(1);
                    for _i in 0..SPLIT_WIDTH {
                        // draw split bar
                        self.xcl.set_high();
                        self.xcl.set_low();
                    }
                    if status_var.abs_diff(line as u32) <= BAR_WIDTH {
                        //black button
                        let four_pixels: u8 = BLACK_FOUR_PIXEL;
                        unsafe {
                            asm!("wur.gpio_out {0}", in(reg) four_pixels);
                        }
                        for _i in SPLIT_WIDTH..BOTTOM_INDICATOR_WIDTH_DIV_4 {
                            // draw bar
                            self.xcl.set_high();
                            self.xcl.set_low();
                        }
                    } else {
                        //white backgruond
                        let four_pixels: u8 = WHITE_FOUR_PIXEL;
                        unsafe {
                            asm!("wur.gpio_out {0}", in(reg) four_pixels);
                        }
                        for _i in SPLIT_WIDTH..BOTTOM_INDICATOR_WIDTH_DIV_4 {
                            // draw bar
                            self.xcl.set_high();
                            self.xcl.set_low();
                        }
                    }
                } else {
                    // none
                    let four_pixels: u8 = 0b00000000;
                    unsafe {
                        asm!("wur.gpio_out {0}", in(reg) four_pixels);
                    }
                    for _i in 0..SPLIT_WIDTH {
                        // skip split bar
                        self.xcl.set_high();
                        self.xcl.set_low();
                    }
                    if status_var.abs_diff(line as u32) <= BAR_WIDTH {
                        if pre_status_var.abs_diff(line as u32) > BAR_WIDTH {
                            //black button
                            let four_pixels: u8 = BLACK_FOUR_PIXEL;
                            unsafe {
                                asm!("wur.gpio_out {0}", in(reg) four_pixels);
                            }
                            for _i in SPLIT_WIDTH..BOTTOM_INDICATOR_WIDTH_DIV_4 {
                                // draw bar
                                self.xcl.set_high();
                                self.xcl.set_low();
                            }
                        } else {
                            //none
                            let four_pixels: u8 = 0b00000000;
                            unsafe {
                                asm!("wur.gpio_out {0}", in(reg) four_pixels);
                            }
                            for _i in SPLIT_WIDTH..BOTTOM_INDICATOR_WIDTH_DIV_4 {
                                // draw bar
                                self.xcl.set_high();
                                self.xcl.set_low();
                            }
                        }
                    } else {
                        if pre_status_var.abs_diff(line as u32) <= BAR_WIDTH {
                            //white background
                            let four_pixels: u8 = WHITE_FOUR_PIXEL;
                            unsafe {
                                asm!("wur.gpio_out {0}", in(reg) four_pixels);
                            }
                            for _i in SPLIT_WIDTH..BOTTOM_INDICATOR_WIDTH_DIV_4 {
                                // clear pre bar
                                self.xcl.set_high();
                                self.xcl.set_low();
                            }
                        } else {
                            //none
                            let four_pixels: u8 = 0b00000000;
                            unsafe {
                                asm!("wur.gpio_out {0}", in(reg) four_pixels);
                            }
                            for _i in SPLIT_WIDTH..BOTTOM_INDICATOR_WIDTH_DIV_4 {
                                // draw bar
                                self.xcl.set_high();
                                self.xcl.set_low();
                            }
                        }
                    }
                }
                self.xstl.set_high();
                self.xcl.set_high();
                self.xcl.set_low();

                self.xle.set_high();
                self.xle.set_low();

                self.ckv.set_low();
                self.delay.delay_micros(1);
                self.ckv.set_high();
            }
            self.end_frame();
        }
    }
}

fn open_4bpp_image<
    D: embedded_sdmmc::BlockDevice,
    T: embedded_sdmmc::TimeSource,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
    U: core::alloc::Allocator,
>(
    cur_dir: &mut Directory<D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
    img_buf: &mut Vec<u8, U>,
    file_name: &str,
) -> Result<(), Error<SdCardError>>
where
    embedded_sdmmc::Error<SdCardError>: From<embedded_sdmmc::Error<<D as BlockDevice>::Error>>,
{
    let file = cur_dir.open_file_in_dir(file_name, Mode::ReadOnly)?;

    let mut tiff_header = [0u8; 8];
    file.read(&mut tiff_header)?; // first 8 bytes is annotation header

    file.read(img_buf)?;
    Ok(())
}

#[main]
fn main() -> ! {
    let peripherals =
        esp_hal::init(esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max()));
    let (start, size) = psram::psram_raw_parts(&peripherals.PSRAM);
    init_psram_heap(start, size);

    /* enable dedicated gpio peripheral */
    SYSTEM::regs()
        .cpu_peri_clk_en()
        .modify(|_, w| w.clk_en_dedicated_gpio().bit(true));
    SYSTEM::regs()
        .cpu_peri_rst_en()
        .modify(|_, w| w.rst_en_dedicated_gpio().bit(true));
    SYSTEM::regs()
        .cpu_peri_rst_en()
        .modify(|_, w| w.rst_en_dedicated_gpio().bit(false));

    let delay = Delay::new();

    let mut led = Output::new(peripherals.GPIO15, Level::High, OutputConfig::default());

    /*usb serial debug*/
    let usb = Usb::new(peripherals.USB0, peripherals.GPIO20, peripherals.GPIO19);
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
                    if *c == 0x63 {
                        // c
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
                        }
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
    let flash_addr = 0x9000; // default NVS size: 0x6000
                             //write!(serial, "Flash size = {}\n", flash.capacity()).unwrap();
    flash.read(flash_addr, &mut bytes).unwrap();
    //write!(serial, "read = {:02x?}\n", &bytes[..4]).unwrap();

    let last_opend_page_num: u16 = u16::from_be_bytes([bytes[2], bytes[3]]);
    let last_opend_dir_num: u16 = u16::from_be_bytes([bytes[0], bytes[1]]);

    /* sd card */
    let sclk = peripherals.GPIO36;
    let miso = peripherals.GPIO37;
    let mosi = peripherals.GPIO35;
    let cs = Output::new(peripherals.GPIO34, Level::High, OutputConfig::default());

    let spi = Spi::new(
        peripherals.SPI2,
        esp_hal::spi::master::Config::default()
            .with_frequency(Rate::from_mhz(50))
            .with_mode(esp_hal::spi::Mode::_0),
    )
    .unwrap()
    .with_sck(sclk)
    .with_mosi(mosi)
    .with_miso(miso);

    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    // should DummyCsPin?

    let sdcard = SdCard::new(spi_device, delay);

    /*
    loop {
        if usb_dev.poll(&mut [&mut serial.0]) {
            break;
        }
    }
    writeln!(serial, "Card size is {} bytes\n", sdcard.num_bytes().unwrap()).unwrap();
    */

    let volume_manager = VolumeManager::new(sdcard, FakeTimesource {});

    /*
    let mut img_buf_1: Vec<u8, _> = Vec::with_capacity_in(FOUR_BPP_BUF_SIZE, &esp_alloc::HEAP);
    for _i in 0..FOUR_BPP_BUF_SIZE {
        img_buf_1.push(0u8);
    }
    */

    // too big for dram? so use psram(2M)
    let mut img_buf_2: Vec<u8, _> = Vec::with_capacity_in(FOUR_BPP_BUF_SIZE, &esp_alloc::HEAP);
    for _i in 0..FOUR_BPP_BUF_SIZE {
        img_buf_2.push(0u8);
    }

    /* get the values of dedicated GPIO from the CPU, not peripheral registers */
    for i in 0..8 {
        unsafe { &*DEDICATED_GPIO::PTR }
            .out_cpu()
            .modify(|_, w| w.sel(i).bit(true));
    }

    // # esp32s2 technical reference page 171
    // pro_alonegpio_out0: (235)
    // ..
    // pro_alonegpio_out7: (242)
    // GPIO_FUNCx_OUT_SEL_CFG
    let eink_data_bus_ios: [usize; 8] = [18, 21, 16, 17, 7, 9, 10, 13];
    for i in 0..8 {
        unsafe { &*GPIO::PTR }
            .func_out_sel_cfg(eink_data_bus_ios[i])
            .modify(|_, w| unsafe {
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

    unsafe { &*GPIO::PTR }
        .enable_w1ts()
        .write(|w| unsafe { w.bits(enable_pins) });

    // IO_MUX_MCU_SEL
    // hm RegisterBlock in esp32s2 pac doesnot impl gpio(num)
    unsafe { &*IO_MUX::PTR }
        .gpio18()
        .modify(|_, w| unsafe { w.mcu_sel().bits(1) }); // set to Function 1
    unsafe { &*IO_MUX::PTR }
        .gpio21()
        .modify(|_, w| unsafe { w.mcu_sel().bits(1) });
    unsafe { &*IO_MUX::PTR }
        .gpio16()
        .modify(|_, w| unsafe { w.mcu_sel().bits(1) });
    unsafe { &*IO_MUX::PTR }
        .gpio17()
        .modify(|_, w| unsafe { w.mcu_sel().bits(1) });
    unsafe { &*IO_MUX::PTR }
        .gpio7()
        .modify(|_, w| unsafe { w.mcu_sel().bits(1) });
    unsafe { &*IO_MUX::PTR }
        .gpio9()
        .modify(|_, w| unsafe { w.mcu_sel().bits(1) });
    unsafe { &*IO_MUX::PTR }
        .gpio10()
        .modify(|_, w| unsafe { w.mcu_sel().bits(1) });
    unsafe { &*IO_MUX::PTR }
        .gpio13()
        .modify(|_, w| unsafe { w.mcu_sel().bits(1) });

    let mode1 = Output::new(peripherals.GPIO11, Level::High, OutputConfig::default());
    let ckv = Output::new(peripherals.GPIO14, Level::High, OutputConfig::default());
    let spv = Output::new(peripherals.GPIO12, Level::High, OutputConfig::default());

    let xcl = Output::new(peripherals.GPIO39, Level::High, OutputConfig::default());
    let xle = Output::new(peripherals.GPIO40, Level::High, OutputConfig::default());
    let xoe = Output::new(peripherals.GPIO38, Level::High, OutputConfig::default());
    let xstl = Output::new(peripherals.GPIO33, Level::High, OutputConfig::default());

    let mut eink_display = EinkDisplay {
        mode1,
        ckv,
        spv,
        xcl,
        xle,
        xoe,
        xstl,
        delay,
    };
    eink_display.write_all_white();
    eink_display.write_all_black();

    let volume0 = volume_manager
        .open_volume(VolumeIdx(0))
        .expect("failed to open volume");
    let root_dir = volume0.open_root_dir().expect("failed to open volume");

    //let mut root_dir_child_dirs = Vec::with_capacity_in(9999, &PSRAM_ALLOCATOR); //DirEntry is 32 bytes // ShortFileName is [u8; 11]
    let mut root_dir_directories_len: u16 = 0;
    root_dir
        .iterate_dir(|entry| {
            if entry.attributes.is_directory()
                && entry.name != ShortFileName::parent_dir()
                && entry.name != ShortFileName::this_dir()
            {
                root_dir_directories_len += 1;
            }
        })
        .unwrap();

    let mut dir_name = String::with_capacity(5); //xxxx
    let mut file_name = String::with_capacity(8); //xxx.tif

    root_dir_directories_len = if root_dir_directories_len > 9999 {
        9999
    } else {
        root_dir_directories_len
    };

    let mut cur_dir: u16 = if last_opend_dir_num > root_dir_directories_len {
        1
    } else {
        if last_opend_dir_num == 0 {
            1
        } else {
            last_opend_dir_num
        }
    };

    /*
    loop {
        if usb_dev.poll(&mut [&mut serial.0]) {
            break;
        }
    }
    writeln!(serial, "cur_dir is {}\n", cur_dir).unwrap();
    */

    write!(&mut dir_name, "{0: >04}", cur_dir).unwrap();
    let mut cur_dir_files_len: u16 = 0;
    let mut cur_child_dir = root_dir.open_dir(dir_name.as_str()).unwrap();

    cur_child_dir
        .iterate_dir(|_entry| {
            cur_dir_files_len += 1;
        })
        .unwrap();
    cur_dir_files_len -= 2; // child_dir contains . and .. in DIrEntries
    cur_dir_files_len = if cur_dir_files_len > 999 {
        999
    } else {
        cur_dir_files_len
    };

    //let pre_buf = &mut img_buf_1;
    let mut next_buf = &mut img_buf_2;

    let mut cur_page: u16 = if last_opend_page_num > cur_dir_files_len {
        0
    } else {
        last_opend_page_num
    };

    /*self cpu impl touch pad*/
    let mut touch_out = Output::new(peripherals.GPIO1, Level::Low, OutputConfig::default());

    let mut adc1_config = AdcConfig::new();
    let mut touch_left = adc1_config.enable_pin(peripherals.GPIO2, Attenuation::_11dB);
    let mut touch_right = adc1_config.enable_pin(peripherals.GPIO3, Attenuation::_11dB);
    let mut touch_center = adc1_config.enable_pin(peripherals.GPIO4, Attenuation::_11dB);
    let mut touch_top = adc1_config.enable_pin(peripherals.GPIO5, Attenuation::_11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);

    const TOUCH_LEFT_THRESHOLD: u16 = 2950;
    const TOUCH_RIGHT_THRESHOLD: u16 = 2950;
    const TOUCH_CENTER_THRESHOLD: u16 = 2950;
    const TOUCH_TOP_THRESHOLD: u16 = 3100;
    const TOUCH_PULSE_HIGH_DELAY_NS: u32 = 400000;

    /*
    const RECORD_LEN: usize = 20;
    let mut pulse_record: [u16; RECORD_LEN] = [0; RECORD_LEN];
    // ADC_ATTEN_DB_11: 0~2500mv
    // 12bit adc(but seems max 8192(1<<13) ?)
    loop {
        touch_out.set_high();
        delay.delay_nanos(TOUCH_PULSE_HIGH_DELAY_NS);
        touch_out.set_low();
        //delay.delay_nanos(1);
        // timer
        for i in 0..RECORD_LEN {
            pulse_record[i] = adc1.read_blocking(&mut touch_top);
            //delay.delay_nanos(1);
        }
        // timer / RECORD_LEN = freq
        usb_dev.poll(&mut [&mut serial.0]);
        for i in 0..RECORD_LEN {
            serial
                .0
                .write(&[(pulse_record[i] >> 8) as u8, (pulse_record[i] & 0xff) as u8])
                .ok();
        }
    }
    */

    /*
    // benchmark
    write!(&mut file_name, "{0: >03}.tif", cur_page).unwrap();
    match open_4bpp_image(&mut cur_child_dir, &mut img_buf, &file_name) {
        Ok(_) => {
            let t1 = esp_hal::time::Instant::now();
            eink_display.write_4bpp_reverse_image(&img_buf);
            //eink_display.write_all_black_white();
            eink_display.write_4bpp_image(&img_buf);
            let t2 = esp_hal::time::Instant::now();

            let elapsed = t2 - t1;
            loop {
                if usb_dev.poll(&mut [&mut serial.0]) {
                    break;
                }
            }
            writeln!(serial, "benchmark elapsed: {}\n", elapsed).unwrap();
        }
        Err(_) => {}
    }
    */

    led.set_low();
    loop {
        'inner: loop {
            touch_out.set_high();
            delay.delay_nanos(TOUCH_PULSE_HIGH_DELAY_NS);
            touch_out.set_low();
            let left_pin_value = adc1.read_blocking(&mut touch_left);
            touch_out.set_high();
            delay.delay_nanos(TOUCH_PULSE_HIGH_DELAY_NS);
            touch_out.set_low();
            let right_pin_value = adc1.read_blocking(&mut touch_right);
            touch_out.set_high();
            delay.delay_nanos(TOUCH_PULSE_HIGH_DELAY_NS);
            touch_out.set_low();
            let center_pin_value = adc1.read_blocking(&mut touch_center);
            touch_out.set_high();
            delay.delay_nanos(TOUCH_PULSE_HIGH_DELAY_NS);
            touch_out.set_low();
            let top_left_pin_value = adc1.read_blocking(&mut touch_top);

            if top_left_pin_value > TOUCH_TOP_THRESHOLD {
                eink_display.write_all_black_white();
            }

            if left_pin_value > TOUCH_LEFT_THRESHOLD {
                if cur_page == 0 {
                    if cur_dir == 1 {
                        cur_dir = root_dir_directories_len; // circling
                    } else {
                        cur_dir -= 1;
                    }
                    dir_name.clear();
                    write!(&mut dir_name, "{0: >04}", cur_dir).unwrap();
                    cur_child_dir.close().unwrap();
                    cur_child_dir = root_dir.open_dir(dir_name.as_str()).unwrap();

                    cur_dir_files_len = 0;
                    cur_child_dir
                        .iterate_dir(|_entry| {
                            cur_dir_files_len += 1;
                        })
                        .unwrap();
                    cur_dir_files_len -= 2; // child_dir contains . and .. in DIrEntries
                    cur_dir_files_len = if cur_dir_files_len > 999 {
                        999
                    } else {
                        cur_dir_files_len
                    };

                    cur_page = cur_dir_files_len - 1;
                } else {
                    cur_page = cur_page - 1;
                }
                break 'inner;
            }
            if right_pin_value > TOUCH_RIGHT_THRESHOLD {
                if cur_page == (cur_dir_files_len - 1) {
                    if cur_dir == root_dir_directories_len {
                        cur_dir = 1; // circling
                    } else {
                        cur_dir += 1;
                    }
                    dir_name.clear();
                    write!(&mut dir_name, "{0: >04}", cur_dir).unwrap();
                    cur_child_dir.close().unwrap();
                    cur_child_dir = root_dir.open_dir(dir_name.as_str()).unwrap();

                    cur_dir_files_len = 0;
                    cur_child_dir
                        .iterate_dir(|_entry| {
                            cur_dir_files_len += 1;
                        })
                        .unwrap();
                    cur_dir_files_len -= 2; // child_dir contains . and .. in DIrEntries
                    cur_dir_files_len = if cur_dir_files_len > 999 {
                        999
                    } else {
                        cur_dir_files_len
                    };

                    cur_page = 0;
                } else {
                    cur_page = cur_page + 1;
                }
                break 'inner;
            }
            if center_pin_value > TOUCH_CENTER_THRESHOLD {
                let bottom_indicator_pos_current: u32 =
                    HEIGHT as u32 - ((cur_page as u32 * HEIGHT as u32) / cur_dir_files_len as u32);
                eink_display.write_bottom_indicator(true, bottom_indicator_pos_current, 0);
                let mut bottom_pre_status_var = bottom_indicator_pos_current;

                delay.delay_millis(500u32);

                '_page_indicator: loop {
                    touch_out.set_high();
                    delay.delay_nanos(TOUCH_PULSE_HIGH_DELAY_NS);
                    touch_out.set_low();
                    let left_pin_value = adc1.read_blocking(&mut touch_left);
                    touch_out.set_high();
                    delay.delay_nanos(TOUCH_PULSE_HIGH_DELAY_NS);
                    touch_out.set_low();
                    let right_pin_value = adc1.read_blocking(&mut touch_right);
                    touch_out.set_high();
                    delay.delay_nanos(TOUCH_PULSE_HIGH_DELAY_NS);
                    touch_out.set_low();
                    let center_pin_value = adc1.read_blocking(&mut touch_center);
                    touch_out.set_high();
                    delay.delay_nanos(TOUCH_PULSE_HIGH_DELAY_NS);
                    touch_out.set_low();
                    let top_left_pin_value = adc1.read_blocking(&mut touch_top);

                    const SKIP_PAGE: u16 = 1;
                    if left_pin_value > TOUCH_LEFT_THRESHOLD {
                        if cur_page < SKIP_PAGE {
                            cur_page = cur_dir_files_len - 1; //circling
                        } else {
                            cur_page = cur_page - SKIP_PAGE;
                        }
                        let bottom_indicator_pos_current: u32 = HEIGHT as u32
                            - ((cur_page as u32 * HEIGHT as u32) / cur_dir_files_len as u32);
                        eink_display.write_bottom_indicator(
                            false,
                            bottom_indicator_pos_current,
                            bottom_pre_status_var,
                        );
                        bottom_pre_status_var = bottom_indicator_pos_current;
                    }
                    if right_pin_value > TOUCH_RIGHT_THRESHOLD {
                        if cur_page > (cur_dir_files_len - 1) - SKIP_PAGE {
                            cur_page = 0; //circling
                        } else {
                            cur_page = cur_page + SKIP_PAGE;
                        }
                        let bottom_indicator_pos_current: u32 = HEIGHT as u32
                            - ((cur_page as u32 * HEIGHT as u32) / cur_dir_files_len as u32);
                        eink_display.write_bottom_indicator(
                            false,
                            bottom_indicator_pos_current,
                            bottom_pre_status_var,
                        );
                        bottom_pre_status_var = bottom_indicator_pos_current;
                    }
                    if center_pin_value > TOUCH_CENTER_THRESHOLD {
                        break 'inner;
                    }

                    if top_left_pin_value > TOUCH_TOP_THRESHOLD {
                        let top_indicator_pos_current: u32 = HEIGHT as u32
                            - ((cur_dir as u32 * HEIGHT as u32) / root_dir_directories_len as u32);
                        eink_display.write_top_indicator(true, top_indicator_pos_current, 0);
                        let mut top_pre_status_var = top_indicator_pos_current;

                        delay.delay_millis(500_u32);
                        'chaptor_indicator: loop {
                            touch_out.set_high();
                            delay.delay_nanos(TOUCH_PULSE_HIGH_DELAY_NS);
                            touch_out.set_low();
                            let left_pin_value = adc1.read_blocking(&mut touch_left);
                            touch_out.set_high();
                            delay.delay_nanos(TOUCH_PULSE_HIGH_DELAY_NS);
                            touch_out.set_low();
                            let right_pin_value = adc1.read_blocking(&mut touch_right);
                            touch_out.set_high();
                            delay.delay_nanos(TOUCH_PULSE_HIGH_DELAY_NS);
                            touch_out.set_low();
                            let center_pin_value = adc1.read_blocking(&mut touch_center);
                            touch_out.set_high();
                            delay.delay_nanos(TOUCH_PULSE_HIGH_DELAY_NS);
                            touch_out.set_low();
                            let top_left_pin_value = adc1.read_blocking(&mut touch_top);

                            if left_pin_value > TOUCH_LEFT_THRESHOLD {
                                if cur_dir == 1 {
                                    cur_dir = root_dir_directories_len; //circling
                                } else {
                                    cur_dir = cur_dir - 1;
                                }
                                let top_indicator_pos_current: u32 = HEIGHT as u32
                                    - ((cur_dir as u32 * HEIGHT as u32)
                                        / root_dir_directories_len as u32);
                                eink_display.write_top_indicator(
                                    false,
                                    top_indicator_pos_current,
                                    top_pre_status_var,
                                );
                                top_pre_status_var = top_indicator_pos_current;
                            }
                            if right_pin_value > TOUCH_RIGHT_THRESHOLD {
                                if cur_dir == root_dir_directories_len {
                                    cur_dir = 1; //circling
                                } else {
                                    cur_dir = cur_dir + 1;
                                }
                                let top_indicator_pos_current: u32 = HEIGHT as u32
                                    - ((cur_dir as u32 * HEIGHT as u32)
                                        / root_dir_directories_len as u32);
                                eink_display.write_top_indicator(
                                    false,
                                    top_indicator_pos_current,
                                    top_pre_status_var,
                                );
                                top_pre_status_var = top_indicator_pos_current;
                            }
                            if center_pin_value > TOUCH_CENTER_THRESHOLD {
                                dir_name.clear();
                                write!(&mut dir_name, "{0: >04}", cur_dir).unwrap();
                                cur_child_dir.close().unwrap();
                                cur_child_dir = root_dir.open_dir(dir_name.as_str()).unwrap();

                                cur_dir_files_len = 0;
                                cur_child_dir
                                    .iterate_dir(|_entry| {
                                        cur_dir_files_len += 1;
                                    })
                                    .unwrap();
                                cur_dir_files_len -= 2; // child_dir contains . and .. in DIrEntries
                                cur_dir_files_len = if cur_dir_files_len > 999 {
                                    999
                                } else {
                                    cur_dir_files_len
                                };

                                cur_page = 0;
                                break 'inner;
                            }
                            /* need pending cur_dir maybe
                            if top_left_pin_value > TOUCH_TOP_THRESHOLD {
                                break 'chaptor_indicator;
                            }
                            */
                        }
                    }
                }
            }
        }
        file_name.clear();
        write!(&mut file_name, "{0: >03}.tif", cur_page).unwrap();
        match open_4bpp_image(&mut cur_child_dir, &mut next_buf, &file_name) {
            Ok(_) => {
                //eink_display.write_4bpp_reverse_image(&pre_buf);
                //eink_display.write_4bpp_partialy_reverse_image(&pre_buf);
                //eink_display.write_all_black_white();
                eink_display.write_all_white();
                eink_display.write_all_white();
                eink_display.write_all_black();
                //eink_display.write_4bpp_reverse_image(&next_buf);
                eink_display.write_4bpp_image(&next_buf);
                flash
                    .write(
                        flash_addr,
                        &(((cur_dir as u32) << 16) + (cur_page as u32)).to_be_bytes(),
                    )
                    .unwrap();
            }
            Err(_error) => {
                /* not found file */
                eink_display.write_all_white();
                eink_display.write_all_black();
            }
        };
        //core::mem::swap(pre_buf, next_buf);
    }
}
