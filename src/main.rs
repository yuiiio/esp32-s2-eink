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
    peripherals::{Peripherals, GPIO, IO_MUX, DEDICATED_GPIO, RTC_IO, LPWR, SENS},
    rtc_cntl::Rtc,
    prelude::*,
    system::SystemControl,
    spi::{SpiMode, master::Spi},
    psram,
};

use usb_device::prelude::{UsbDeviceBuilder, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

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
        for grayscale in [0, 3, 6, 9] {
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
    #[allow(dead_code)]
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
    
/* register from esp-idf/components/hal/esp32s2/include/hal/touch_sensor_ll.h */
const TOUCH_LL_READ_RAW: u8 = 0x0;
#[allow(dead_code)]
const TOUCH_LL_READ_BENCHMARK: u8 = 0x2;
#[allow(dead_code)]
const TOUCH_LL_READ_SMOOTH: u8 = 0x3;
const TOUCH_LL_TIMER_FORCE_DONE: u8 = 0x3;
const TOUCH_LL_TIMER_DONE: u8 = 0x0;
/* hal/include/hal/touch_sensor_types.h */

const TOUCH_PAD_INTR_MASK_DONE: u32 = 1 << 0;
const TOUCH_PAD_INTR_MASK_ACTIVE: u32 = 1 << 1;
const TOUCH_PAD_INTR_MASK_INACTIVE: u32 = 1 << 2;
const TOUCH_PAD_INTR_MASK_SCAN_DONE: u32 = 1 << 3;
const TOUCH_PAD_INTR_MASK_TIMEOUT: u32 = 1 << 4;

const TOUCH_PAD_INTR_MASK_ALL: u32 = 
TOUCH_PAD_INTR_MASK_DONE 
| TOUCH_PAD_INTR_MASK_ACTIVE 
| TOUCH_PAD_INTR_MASK_TIMEOUT 
| TOUCH_PAD_INTR_MASK_INACTIVE 
| TOUCH_PAD_INTR_MASK_SCAN_DONE;

const SOC_TOUCH_SENSOR_NUM: u16 = 15; // esp32s2
const TOUCH_PAD_BIT_MASK_ALL: u16 = (1<<SOC_TOUCH_SENSOR_NUM) -1;

/*esp32s2 specific number*/
const TOUCH_PAD_MEASURE_CYCLE_DEFAULT: u16 = 500;
const TOUCH_LL_PAD_MEASURE_WAIT_MAX: u8 = 0xff;
const TOUCH_PAD_SLEEP_CYCLE_DEFAULT: u16 = 0xf;

#[allow(non_camel_case_types)]
#[allow(dead_code)]
enum touch_high_volt_t {
    TOUCH_HVOLT_KEEP = -1,
    TOUCH_HVOLT_2V4 = 0,
    TOUCH_HVOLT_2V5,      
    TOUCH_HVOLT_2V6,      
    TOUCH_HVOLT_2V7,      
    TOUCH_HVOLT_MAX,
}

#[allow(non_camel_case_types)]
#[allow(dead_code)]
enum touch_low_volt_t {
    TOUCH_LVOLT_KEEP = -1,
    TOUCH_LVOLT_0V5 = 0,  
    TOUCH_LVOLT_0V6,      
    TOUCH_LVOLT_0V7,      
    TOUCH_LVOLT_0V8,      
    TOUCH_LVOLT_MAX,
}

#[allow(non_camel_case_types)]
#[allow(dead_code)]
enum touch_volt_atten_t {
    TOUCH_HVOLT_ATTEN_KEEP = -1,
    TOUCH_HVOLT_ATTEN_1V5 = 0,  
    TOUCH_HVOLT_ATTEN_1V,       
    TOUCH_HVOLT_ATTEN_0V5,      
    TOUCH_HVOLT_ATTEN_0V,       
    TOUCH_HVOLT_ATTEN_MAX,
}

#[allow(non_camel_case_types)]
#[allow(dead_code)]
#[derive(Clone)]
enum touch_pad_t {
    TOUCH_PAD_NUM0 = 0,
    TOUCH_PAD_NUM1,
    TOUCH_PAD_NUM2,
    TOUCH_PAD_NUM3,
    TOUCH_PAD_NUM4,
    TOUCH_PAD_NUM5,
    TOUCH_PAD_NUM6,
    TOUCH_PAD_NUM7,
    TOUCH_PAD_NUM8,
    TOUCH_PAD_NUM9,
    TOUCH_PAD_NUM10,
    TOUCH_PAD_NUM11,
    TOUCH_PAD_NUM12,
    TOUCH_PAD_NUM13,
    TOUCH_PAD_NUM14,
    TOUCH_PAD_MAX,
}

#[allow(dead_code)]
const TOUCH_PAD_CONN_HIGHZ: bool = false;
const TOUCH_PAD_CONN_GND: bool = true;

const TOUCH_PAD_HIGH_VOLTAGE_THRESHOLD: touch_high_volt_t = touch_high_volt_t::TOUCH_HVOLT_2V7;
const TOUCH_PAD_LOW_VOLTAGE_THRESHOLD: touch_low_volt_t = touch_low_volt_t::TOUCH_LVOLT_0V5;
const TOUCH_PAD_ATTEN_VOLTAGE_THRESHOLD: touch_volt_atten_t = touch_volt_atten_t::TOUCH_HVOLT_ATTEN_0V5;
const TOUCH_PAD_IDLE_CH_CONNECT_DEFAULT: bool = TOUCH_PAD_CONN_GND;

fn touch_ll_stop_fsm(rtc_cntl: &LPWR) {
    rtc_cntl.touch_ctrl2().modify(|_, w| {
        w.touch_start_en().bit(false) //stop touch fsm
        .touch_slp_timer_en().bit(false)
    });
    rtc_cntl.touch_ctrl2().modify(|_, w| unsafe {
        w.touch_timer_force_done().bits(TOUCH_LL_TIMER_FORCE_DONE)
        .touch_timer_force_done().bits(TOUCH_LL_TIMER_DONE)
    });
}

fn touch_ll_intr_disable(rtc_cntl: &LPWR, mask: &u32) {
    if mask & TOUCH_PAD_INTR_MASK_DONE == TOUCH_PAD_INTR_MASK_DONE {
        rtc_cntl.int_ena().modify(|_, w| { w.touch_done().bit(false) });
    }
    if mask & TOUCH_PAD_INTR_MASK_ACTIVE == TOUCH_PAD_INTR_MASK_ACTIVE {
        rtc_cntl.int_ena().modify(|_, w| { w.touch_active().bit(false) });
    }
    if mask & TOUCH_PAD_INTR_MASK_INACTIVE == TOUCH_PAD_INTR_MASK_INACTIVE {
        rtc_cntl.int_ena().modify(|_, w| { w.touch_inactive().bit(false) });
    }
    if mask & TOUCH_PAD_INTR_MASK_SCAN_DONE == TOUCH_PAD_INTR_MASK_SCAN_DONE {
        rtc_cntl.int_ena().modify(|_, w| { w.touch_scan_done().bit(false) });
    }
    if mask & TOUCH_PAD_INTR_MASK_TIMEOUT == TOUCH_PAD_INTR_MASK_TIMEOUT {
        rtc_cntl.int_ena().modify(|_, w| { w.touch_timeout().bit(false) });
    }
}
fn touch_ll_intr_clear(rtc_cntl: &LPWR, mask: &u32) {
    if mask & TOUCH_PAD_INTR_MASK_DONE == TOUCH_PAD_INTR_MASK_DONE {
        rtc_cntl.int_clr().write(|w| { w.touch_done().bit(true) });
    }
    if mask & TOUCH_PAD_INTR_MASK_ACTIVE == TOUCH_PAD_INTR_MASK_ACTIVE {
        rtc_cntl.int_clr().write(|w| { w.touch_active().bit(true) });
    }
    if mask & TOUCH_PAD_INTR_MASK_INACTIVE == TOUCH_PAD_INTR_MASK_INACTIVE {
        rtc_cntl.int_clr().write(|w| { w.touch_inactive().bit(true) });
    }
    if mask & TOUCH_PAD_INTR_MASK_SCAN_DONE == TOUCH_PAD_INTR_MASK_SCAN_DONE {
        rtc_cntl.int_clr().write(|w| { w.touch_scan_done().bit(true) });
    }
    if mask & TOUCH_PAD_INTR_MASK_TIMEOUT == TOUCH_PAD_INTR_MASK_TIMEOUT {
        rtc_cntl.int_clr().write(|w| { w.touch_timeout().bit(true) });
    }
}

fn touch_ll_clear_channel_mask(sens: &SENS, rtc_cntl: &LPWR, disable_mask: &u16) {
    sens.sar_touch_conf().modify(|r, w| unsafe {
        w.touch_outen().bits(r.touch_outen().bits() & !(disable_mask & TOUCH_PAD_BIT_MASK_ALL))
    });
    rtc_cntl.touch_scan_ctrl().modify(|r, w| unsafe {
        w.touch_scan_pad_map().bits(r.touch_scan_pad_map().bits() & !(disable_mask & TOUCH_PAD_BIT_MASK_ALL))
    });
}

fn touch_ll_clear_trigger_status_mask(sens: &SENS) {
    sens.sar_touch_conf().write(|w| {
        w.touch_status_clr().bit(true)
    });
}

fn touch_ll_set_meas_times(rtc_cntl: &LPWR, meas_time: u16) {
    rtc_cntl.touch_ctrl1().write(|w| unsafe {
        w.touch_meas_num().bits(meas_time)
    });
    rtc_cntl.touch_ctrl2().write(|w| unsafe {
        w.touch_xpd_wait().bits(TOUCH_LL_PAD_MEASURE_WAIT_MAX)
    });
}

fn touch_ll_set_sleep_time(rtc_cntl: &LPWR, sleep_time: u16) {
    rtc_cntl.touch_ctrl1().write(|w| unsafe {
        w.touch_sleep_cycles().bits(sleep_time)
    });
}

fn touch_ll_sleep_low_power(rtc_cntl: &LPWR, is_low_power: bool) {
    rtc_cntl.touch_ctrl2().write(|w| {
        w.touch_dbias().bit(is_low_power)
    });
}

fn touch_ll_set_voltage_high(rtc_cntl: &LPWR, refh: touch_high_volt_t) {
    rtc_cntl.touch_ctrl2().write(|w| unsafe {
        w.touch_drefh().bits(refh as u8)
    });
}
fn touch_ll_set_voltage_low(rtc_cntl: &LPWR, refl: touch_low_volt_t) {
    rtc_cntl.touch_ctrl2().write(|w| unsafe {
        w.touch_drefl().bits(refl as u8)
    });
}
fn touch_ll_set_voltage_attenuation(rtc_cntl: &LPWR, atten: touch_volt_atten_t) {
    rtc_cntl.touch_ctrl2().write(|w| unsafe {
        w.touch_drange().bits(atten as u8)
    });
}
fn touch_ll_set_idle_channel_connect(rtc_cntl: &LPWR, conn_type: bool) {
    rtc_cntl.touch_scan_ctrl().write(|w| {
        w.touch_inactive_connection().bit(conn_type)
    });
}
fn touch_ll_clkgate(rtc_cntl: &LPWR, enable: bool) {
    rtc_cntl.touch_ctrl2().write(|w| {
        w.touch_clkgate_en().bit(enable)
    });
}

fn touch_ll_reset_benchmark(sens: &SENS, touch_num: touch_pad_t) {
    match touch_num {
        touch_pad_t::TOUCH_PAD_MAX => {
            sens.sar_touch_chn_st().write(|w| unsafe {
                w.touch_channel_clr().bits(TOUCH_PAD_BIT_MASK_ALL)
            });
        },
        _ => {
            sens.sar_touch_chn_st().write(|w| unsafe {
                w.touch_channel_clr().bits(1 << (touch_num as u8))
            });
        },
    }
}

fn touch_ll_sleep_reset_benchmark(rtc_cntl: &LPWR) {
    rtc_cntl.touch_approach().write(|w| {
        w.touch_slp_channel_clr().bit(true)
    });
}
/* 
const int rtc_io_num_map[SOC_GPIO_PIN_COUNT] = {
    RTCIO_GPIO0_CHANNEL,    //GPIO0
    RTCIO_GPIO1_CHANNEL,    //GPIO1
    RTCIO_GPIO2_CHANNEL,    //GPIO2
    RTCIO_GPIO3_CHANNEL,    //GPIO3
    RTCIO_GPIO4_CHANNEL,    //GPIO4
    RTCIO_GPIO5_CHANNEL,    //GPIO5
    RTCIO_GPIO6_CHANNEL,    //GPIO6
    RTCIO_GPIO7_CHANNEL,    //GPIO7
    RTCIO_GPIO8_CHANNEL,    //GPIO8
    RTCIO_GPIO9_CHANNEL,    //GPIO9
    RTCIO_GPIO10_CHANNEL,   //GPIO10
    RTCIO_GPIO11_CHANNEL,   //GPIO11
    RTCIO_GPIO12_CHANNEL,   //GPIO12
    RTCIO_GPIO13_CHANNEL,   //GPIO13
    RTCIO_GPIO14_CHANNEL,   //GPIO14
    RTCIO_GPIO15_CHANNEL,   //GPIO15
    RTCIO_GPIO16_CHANNEL,   //GPIO16
    RTCIO_GPIO17_CHANNEL,   //GPIO17
    RTCIO_GPIO18_CHANNEL,   //GPIO18
    RTCIO_GPIO19_CHANNEL,   //GPIO19
    RTCIO_GPIO20_CHANNEL,   //GPIO20
    RTCIO_GPIO21_CHANNEL,   //GPIO21
    -1,//GPIO22
    .
    .
    -1,//GPIO46
    ]
*/

#[allow(non_camel_case_types)]
#[allow(dead_code)]
enum rtcio_ll_func_t {
    RTCIO_LL_FUNC_RTC = 0x0,
    RTCIO_LL_FUNC_DIGITAL = 0x1,
}

const RTCIO_LL_PIN_FUNC: u8 = 0;

fn rtcio_ll_iomux_func_sel(rtcio_num: u8, func: u8) {
    unsafe { &*RTC_IO::PTR}.touch_pad(rtcio_num as usize).write(|w| unsafe {
        w.fun_sel().bits(func) // this 17.18 only 2bit but u8 in
    });
}

fn rtcio_ll_function_select(sens: &SENS, rtcio_num: u8, func: rtcio_ll_func_t) {
    match func {
        rtcio_ll_func_t::RTCIO_LL_FUNC_RTC => {
            sens.sar_io_mux_conf() .write(|w| {
                w.iomux_clk_gate_en().bit(true)
            });
            // 0: GPIO connected to digital GPIO module. 1: GPIO connected to analog RTC module.
            unsafe { &*RTC_IO::PTR}.touch_pad(rtcio_num as usize).write(|w| {
                w.mux_sel().bit(true)
            });
            //0:RTC FUNCTION 1,2,3:Reserved
            rtcio_ll_iomux_func_sel(rtcio_num, RTCIO_LL_PIN_FUNC);
        },
        rtcio_ll_func_t::RTCIO_LL_FUNC_DIGITAL => {
            unsafe { &*RTC_IO::PTR}.touch_pad(rtcio_num as usize).write(|w| {
                w.mux_sel().bit(false)
            });
            sens.sar_io_mux_conf() .write(|w| {
                w.iomux_clk_gate_en().bit(false)
            });
        },
    }
}

fn rtc_gpio_init(sens: &SENS, gpio_num: u8) {
    //rtcio_hal_function_select ==
    rtcio_ll_function_select(sens, gpio_num, rtcio_ll_func_t::RTCIO_LL_FUNC_RTC);
}

#[allow(non_camel_case_types)]
#[allow(dead_code)]
enum rtc_gpio_mode_t {
    RTC_GPIO_MODE_INPUT_ONLY ,
    RTC_GPIO_MODE_OUTPUT_ONLY,
    RTC_GPIO_MODE_INPUT_OUTPUT,
    RTC_GPIO_MODE_DISABLED,
    RTC_GPIO_MODE_OUTPUT_OD,
    RTC_GPIO_MODE_INPUT_OUTPUT_OD,
}
#[allow(non_camel_case_types)]
const RTCIO_LL_OUTPUT_NORMAL: bool = false;
const RTCIO_LL_OUTPUT_OD: bool = true;

fn rtcio_ll_output_mode_set(rtcio_num: u8, mode: bool) {
    unsafe { &*RTC_IO::PTR}.pin(rtcio_num as usize).write(|w| {
        w.pad_driver().bit(mode)
    });
}

fn rtcio_ll_output_disable(rtcio_num: u8) {
    unsafe { &*RTC_IO::PTR}.enable_w1tc().write(|w| unsafe {
        w.bits(1 << (rtcio_num as usize))
    });
}
fn rtcio_ll_output_enable(rtcio_num: u8) {
    unsafe { &*RTC_IO::PTR}.rtc_gpio_enable_w1ts().write(|w| unsafe {
        w.bits(1 << (rtcio_num as usize))
    });
}

fn rtcio_ll_input_disable(rtcio_num: u8) {
    unsafe { &*RTC_IO::PTR}.touch_pad(rtcio_num as usize).write(|w| {
        w.fun_ie().bit(false)
    });
}
fn rtcio_ll_input_enable(rtcio_num: u8) {
    unsafe { &*RTC_IO::PTR}.touch_pad(rtcio_num as usize).write(|w| {
        w.fun_ie().bit(true)
    });
}

fn rtcio_hal_set_direction(rtcio_num: u8, mode: rtc_gpio_mode_t) {
    match mode {
        rtc_gpio_mode_t::RTC_GPIO_MODE_INPUT_ONLY => {
            rtcio_ll_output_mode_set(rtcio_num, RTCIO_LL_OUTPUT_NORMAL);
            rtcio_ll_output_disable(rtcio_num);
            rtcio_ll_input_enable(rtcio_num);
        },
        rtc_gpio_mode_t::RTC_GPIO_MODE_OUTPUT_ONLY => {
            rtcio_ll_output_mode_set(rtcio_num, RTCIO_LL_OUTPUT_NORMAL);
            rtcio_ll_output_enable(rtcio_num);
            rtcio_ll_input_disable(rtcio_num);
        },
        rtc_gpio_mode_t::RTC_GPIO_MODE_INPUT_OUTPUT => {
            rtcio_ll_output_mode_set(rtcio_num, RTCIO_LL_OUTPUT_NORMAL);
            rtcio_ll_output_enable(rtcio_num);
            rtcio_ll_input_enable(rtcio_num);
        },
        rtc_gpio_mode_t::RTC_GPIO_MODE_DISABLED => {
            rtcio_ll_output_mode_set(rtcio_num, RTCIO_LL_OUTPUT_NORMAL);
            rtcio_ll_output_disable(rtcio_num);
            rtcio_ll_input_disable(rtcio_num);
        },
        rtc_gpio_mode_t::RTC_GPIO_MODE_OUTPUT_OD => {
            rtcio_ll_output_mode_set(rtcio_num, RTCIO_LL_OUTPUT_OD);
            rtcio_ll_output_enable(rtcio_num);
            rtcio_ll_input_disable(rtcio_num);
        },
        rtc_gpio_mode_t::RTC_GPIO_MODE_INPUT_OUTPUT_OD => {
            rtcio_ll_output_mode_set(rtcio_num, RTCIO_LL_OUTPUT_OD);
            rtcio_ll_output_enable(rtcio_num);
            rtcio_ll_input_enable(rtcio_num);
        },
    }
}

fn rtcio_ll_pulldown_disable(rtcio_num: u8) {
    unsafe { &*RTC_IO::PTR}.touch_pad(rtcio_num as usize).write(|w| {
        w.rue().bit(false)
    });
}
fn rtcio_ll_pullup_disable(rtcio_num: u8) {
    unsafe { &*RTC_IO::PTR}.touch_pad(rtcio_num as usize).write(|w| {
        w.rde().bit(false)
    });
}

fn touch_pad_io_init(sens: &SENS, touch_num: &touch_pad_t) {
    let gpio_num = (*touch_num).clone() as u8; // 0 is internal denoise channel
    rtc_gpio_init(sens, gpio_num);
    //rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_DISABLED) ==
    rtcio_hal_set_direction(gpio_num, rtc_gpio_mode_t::RTC_GPIO_MODE_DISABLED);
    //rtc_gpio_pulldown_dis(gpio_num); ==
    rtcio_ll_pulldown_disable(gpio_num);
    //rtc_gpio_pullup_dis(gpio_num); == 
    rtcio_ll_pullup_disable(gpio_num);
}

#[allow(non_camel_case_types)]
#[allow(dead_code)]
#[derive(Clone)]
enum touch_cnt_slope_t {
    TOUCH_PAD_SLOPE_0 = 0,
    TOUCH_PAD_SLOPE_1 = 1,
    TOUCH_PAD_SLOPE_2 = 2,
    TOUCH_PAD_SLOPE_3 = 3,
    TOUCH_PAD_SLOPE_4 = 4,
    TOUCH_PAD_SLOPE_5 = 5,
    TOUCH_PAD_SLOPE_6 = 6,
    TOUCH_PAD_SLOPE_7 = 7,
    TOUCH_PAD_SLOPE_MAX,
}

#[allow(non_camel_case_types)]
const TOUCH_PAD_TIE_OPT_LOW: bool = false;
#[allow(dead_code)]
const TOUCH_PAD_TIE_OPT_HIGH: bool = true;

const TOUCH_PAD_THRESHOLD_MAX: u32 = 0x1FFFFF; // esp32s2 (SOC_TOUCH_SENSOR_VERSION == 2)
const TOUCH_PAD_SLOPE_DEFAULT: touch_cnt_slope_t = touch_cnt_slope_t::TOUCH_PAD_SLOPE_7;
const TOUCH_PAD_TIE_OPT_DEFAULT: bool = TOUCH_PAD_TIE_OPT_LOW;

fn touch_ll_set_threshold(sens: &SENS, touch_num: &touch_pad_t, threshold: u32) {
    match touch_num {
        touch_pad_t::TOUCH_PAD_NUM0 => {// 0 is internal denoise channel
        },
        touch_pad_t::TOUCH_PAD_NUM1 => {
            sens.sar_touch_thres1().write(|w| unsafe {w.touch_out_th1().bits(threshold)});
        },
        touch_pad_t::TOUCH_PAD_NUM2 => {
            sens.sar_touch_thres2().write(|w| unsafe {w.touch_out_th2().bits(threshold)});
        },
        touch_pad_t::TOUCH_PAD_NUM3 => {
            sens.sar_touch_thres3().write(|w| unsafe {w.touch_out_th3().bits(threshold)});
        },
        touch_pad_t::TOUCH_PAD_NUM4 => {
            sens.sar_touch_thres4().write(|w| unsafe {w.touch_out_th4().bits(threshold)});
        },
        touch_pad_t::TOUCH_PAD_NUM5 => {
            sens.sar_touch_thres5().write(|w| unsafe {w.touch_out_th5().bits(threshold)});
        },
        touch_pad_t::TOUCH_PAD_NUM6 => {
            sens.sar_touch_thres6().write(|w| unsafe {w.touch_out_th6().bits(threshold)});
        },
        touch_pad_t::TOUCH_PAD_NUM7 => {
            sens.sar_touch_thres7().write(|w| unsafe {w.touch_out_th7().bits(threshold)});
        },
        touch_pad_t::TOUCH_PAD_NUM8 => {
            sens.sar_touch_thres8().write(|w| unsafe {w.touch_out_th8().bits(threshold)});
        },
        touch_pad_t::TOUCH_PAD_NUM9 => {
            sens.sar_touch_thres9().write(|w| unsafe {w.touch_out_th9().bits(threshold)});
        },
        touch_pad_t::TOUCH_PAD_NUM10 => {
            sens.sar_touch_thres10().write(|w| unsafe {w.touch_out_th10().bits(threshold)});
        },
        touch_pad_t::TOUCH_PAD_NUM11 => {
            sens.sar_touch_thres11().write(|w| unsafe {w.touch_out_th11().bits(threshold)});
        },
        touch_pad_t::TOUCH_PAD_NUM12 => {
            sens.sar_touch_thres12().write(|w| unsafe {w.touch_out_th12().bits(threshold)});
        },
        touch_pad_t::TOUCH_PAD_NUM13 => {
            sens.sar_touch_thres13().write(|w| unsafe {w.touch_out_th13().bits(threshold)});
        },
        touch_pad_t::TOUCH_PAD_NUM14 => {
            sens.sar_touch_thres14().write(|w| unsafe {w.touch_out_th14().bits(threshold)});
        },
        touch_pad_t::TOUCH_PAD_MAX => {
        },
    }
}

fn touch_ll_set_slope(touch_num: &touch_pad_t, slope: touch_cnt_slope_t) {
    unsafe { &*RTC_IO::PTR}.touch_pad((*touch_num).clone() as usize).write(|w| unsafe {
        w.dac().bits(slope as u8) // 23:25 ,3bit
    });
}

fn touch_ll_set_tie_option(touch_num: &touch_pad_t, opt: bool) {
    unsafe { &*RTC_IO::PTR}.touch_pad((*touch_num).clone() as usize).write(|w| {
        w.tie_opt().bit(opt)
    });
}

fn touch_hal_config(sens: &SENS, touch_num: &touch_pad_t) {
    touch_ll_set_threshold(sens, touch_num, TOUCH_PAD_THRESHOLD_MAX);
    touch_ll_set_slope(touch_num, TOUCH_PAD_SLOPE_DEFAULT);
    touch_ll_set_tie_option(touch_num, TOUCH_PAD_TIE_OPT_DEFAULT);
}

fn touch_ll_set_channel_mask(sens: &SENS, rtc_cntl: &LPWR, enable_mask: u16) {
    rtc_cntl.touch_scan_ctrl().modify(|r, w| unsafe {
        w.touch_scan_pad_map().bits(r.touch_scan_pad_map().bits() | (enable_mask & TOUCH_PAD_BIT_MASK_ALL))
    });
    sens.sar_touch_conf().modify(|r, w| unsafe {
        w.touch_outen().bits(r.touch_outen().bits() | (enable_mask) & TOUCH_PAD_BIT_MASK_ALL)
    });
}

fn touch_pad_config(sens: &SENS, rtc_cntl: &LPWR, touch_num: &touch_pad_t) {
    touch_pad_io_init(sens, touch_num);
    touch_hal_config(sens, touch_num);
    
    //touch_hal_set_channel_mask(1 << (*touch_num as u8)) ==
    touch_ll_set_channel_mask(sens, rtc_cntl, 1 << ((*touch_num).clone() as u8));
}

#[allow(non_camel_case_types)]
#[allow(dead_code)]
#[derive(Clone)]
enum touch_pad_denoise_grade_t {
    TOUCH_PAD_DENOISE_BIT12 = 0,  /* Denoise range is 12bit */
    TOUCH_PAD_DENOISE_BIT10 = 1,  /* Denoise range is 10bit */
    TOUCH_PAD_DENOISE_BIT8 = 2,   /* Denoise range is 8bit */
    TOUCH_PAD_DENOISE_BIT4 = 3,   /* Denoise range is 4bit */
    TOUCH_PAD_DENOISE_MAX
}
#[allow(non_camel_case_types)]
#[allow(dead_code)]
#[derive(Clone)]
enum touch_pad_denoise_cap_t {
    TOUCH_PAD_DENOISE_CAP_L0 = 0, /* Denoise channel internal reference capacitance is 5pf */
    TOUCH_PAD_DENOISE_CAP_L1 = 1, /* Denoise channel internal reference capacitance is 6.4pf */
    TOUCH_PAD_DENOISE_CAP_L2 = 2, /* Denoise channel internal reference capacitance is 7.8pf */
    TOUCH_PAD_DENOISE_CAP_L3 = 3, /* Denoise channel internal reference capacitance is 9.2pf */
    TOUCH_PAD_DENOISE_CAP_L4 = 4, /* Denoise channel internal reference capacitance is 10.6pf */
    TOUCH_PAD_DENOISE_CAP_L5 = 5, /* Denoise channel internal reference capacitance is 12.0pf */
    TOUCH_PAD_DENOISE_CAP_L6 = 6, /* Denoise channel internal reference capacitance is 13.4pf */
    TOUCH_PAD_DENOISE_CAP_L7 = 7, /* Denoise channel internal reference capacitance is 14.8pf */
    TOUCH_PAD_DENOISE_CAP_MAX = 8
}


#[allow(non_camel_case_types)]
struct touch_pad_denoise_t {
    grade: touch_pad_denoise_grade_t,
    cap_level: touch_pad_denoise_cap_t,
}

#[allow(non_camel_case_types)]
#[derive(Clone)]
struct touch_hal_meas_mode_t {
    slope: touch_cnt_slope_t,
    tie_opt: bool,
}

fn touch_hal_set_meas_mode(touch_num: touch_pad_t, meas: &touch_hal_meas_mode_t) {
    touch_ll_set_slope(&touch_num, meas.slope.clone());
    touch_ll_set_tie_option(&touch_num, meas.tie_opt);
}

const SOC_TOUCH_DENOISE_CHANNEL: touch_pad_t = touch_pad_t::TOUCH_PAD_NUM0; //esp32s2

fn touch_ll_denoise_set_cap_level(rtc_cntl: &LPWR, cap_level: &touch_pad_denoise_cap_t) {
    rtc_cntl.touch_ctrl2().write(|w| unsafe {w.touch_refc().bits((*cap_level).clone() as u8)});
}

fn touch_ll_denoise_set_grade(rtc_cntl: &LPWR, grade: &touch_pad_denoise_grade_t) {
    rtc_cntl.touch_scan_ctrl().write(|w| unsafe {w.touch_denoise_res().bits((*grade).clone() as u8)});
}

fn touch_hal_denoise_set_config(rtc_cntl: &LPWR, denoise: &touch_pad_denoise_t) {
    touch_ll_denoise_set_cap_level(rtc_cntl, &denoise.cap_level);
    touch_ll_denoise_set_grade(rtc_cntl, &denoise.grade);
}

fn touch_pad_denoise_set_config(rtc_cntl: &LPWR, denoise: &touch_pad_denoise_t) {
    const MEAS: touch_hal_meas_mode_t = touch_hal_meas_mode_t {
        slope: TOUCH_PAD_SLOPE_DEFAULT,
        tie_opt: TOUCH_PAD_TIE_OPT_DEFAULT, 
    };
    touch_hal_set_meas_mode(SOC_TOUCH_DENOISE_CHANNEL, &MEAS);
    touch_hal_denoise_set_config(rtc_cntl, denoise);
}

fn touch_ll_denoise_enable(rtc_cntl: &LPWR) {
    rtc_cntl.touch_scan_ctrl().write(|w| {w.touch_denoise_en().bit(true)});
}

fn touch_hal_denoise_enable(sens: &SENS, rtc_cntl: &LPWR) {
    touch_ll_clear_channel_mask(sens, rtc_cntl, &(1 << SOC_TOUCH_DENOISE_CHANNEL as u8)); //duplicate ?
    touch_ll_denoise_enable(rtc_cntl);
}

fn touch_pad_denoise_enable(sens: &SENS, rtc_cntl: &LPWR) {
    //touch_hal_clear_channel_mask ==
    touch_ll_clear_channel_mask(sens, rtc_cntl, &(1 << SOC_TOUCH_DENOISE_CHANNEL as u8));
    touch_hal_denoise_enable(sens, rtc_cntl);
}

const TOUCH_FSM_MODE_TIMER: bool = false;
#[allow(dead_code)]
const TOUCH_FSM_MODE_SW: bool = true;

fn touch_ll_set_fsm_mode(rtc_cntl: &LPWR, mode: bool) {
    rtc_cntl.touch_ctrl2().write(|w| {w.touch_start_force().bit(mode)});
}

fn touch_pad_set_fsm_mode(rtc_cntl: &LPWR, mode: bool) {
    //touch_hal_set_fsm_mode ==
    touch_ll_set_fsm_mode(rtc_cntl, mode);
}

fn touch_ll_start_fsm(rtc_cntl: &LPWR,) {
    rtc_cntl.touch_ctrl2().write(|w| unsafe {w.touch_timer_force_done().bits(TOUCH_LL_TIMER_FORCE_DONE)});
    rtc_cntl.touch_ctrl2().write(|w| unsafe {w.touch_timer_force_done().bits(TOUCH_LL_TIMER_DONE)});
    rtc_cntl.touch_ctrl2().modify(|r, w| {
        w.touch_slp_timer_en().bit(r.touch_start_force().bit() == TOUCH_FSM_MODE_TIMER)
    });
}

fn touch_pad_fsm_start(rtc_cntl: &LPWR,) {
    //touch_hal_start_fsm(); ==
    touch_ll_start_fsm(rtc_cntl);
}

fn touch_ll_read_raw_data(sens: &SENS, touch_num: touch_pad_t) -> u32 {
    sens.sar_touch_conf().write(|w| unsafe {
        w.touch_data_sel().bits(TOUCH_LL_READ_RAW)
    });
    match touch_num {
        touch_pad_t::TOUCH_PAD_NUM0 => {// 0 is internal denoise channel
            0
        },
        touch_pad_t::TOUCH_PAD_NUM1 => {
            sens.sar_touch_status1().read().touch_pad1_data().bits()
        },
        touch_pad_t::TOUCH_PAD_NUM2 => {
            sens.sar_touch_status2().read().touch_pad2_data().bits()
        },
        touch_pad_t::TOUCH_PAD_NUM3 => {
            sens.sar_touch_status3().read().touch_pad3_data().bits()
        },
        touch_pad_t::TOUCH_PAD_NUM4 => {
            sens.sar_touch_status4().read().touch_pad4_data().bits()
        },
        touch_pad_t::TOUCH_PAD_NUM5 => {
            sens.sar_touch_status5().read().touch_pad5_data().bits()
        },
        touch_pad_t::TOUCH_PAD_NUM6 => {
            sens.sar_touch_status6().read().touch_pad6_data().bits()
        },
        touch_pad_t::TOUCH_PAD_NUM7 => {
            sens.sar_touch_status7().read().touch_pad7_data().bits()
        },
        touch_pad_t::TOUCH_PAD_NUM8 => {
            sens.sar_touch_status8().read().touch_pad8_data().bits()
        },
        touch_pad_t::TOUCH_PAD_NUM9 => {
            sens.sar_touch_status9().read().touch_pad9_data().bits()
        },
        touch_pad_t::TOUCH_PAD_NUM10 => {
            sens.sar_touch_status10().read().touch_pad10_data().bits()
        },
        touch_pad_t::TOUCH_PAD_NUM11 => {
            sens.sar_touch_status11().read().touch_pad11_data().bits()
        },
        touch_pad_t::TOUCH_PAD_NUM12 => {
            sens.sar_touch_status12().read().touch_pad12_data().bits()
        },
        touch_pad_t::TOUCH_PAD_NUM13 => {
            sens.sar_touch_status13().read().touch_pad13_data().bits()
        },
        touch_pad_t::TOUCH_PAD_NUM14 => {
            sens.sar_touch_status14().read().touch_pad14_data().bits()
        },
        touch_pad_t::TOUCH_PAD_MAX => {
            0
        },
    }
}

fn touch_pad_read_raw_data(sens: &SENS, touch_num: touch_pad_t, raw_data: &mut u32) {
    //touch_hal_read_raw_data ==
    *raw_data = touch_ll_read_raw_data(sens, touch_num);
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

    /* touch_pad */
    //  touch_pad_init(void)
    //      touch_hal_init(void)
    touch_ll_stop_fsm(&peripherals.LPWR);
    touch_ll_intr_disable(&peripherals.LPWR, &TOUCH_PAD_INTR_MASK_ALL);
    touch_ll_intr_clear(&peripherals.LPWR, &TOUCH_PAD_INTR_MASK_ALL);
    touch_ll_clear_channel_mask(&peripherals.SENS, &peripherals.LPWR, &TOUCH_PAD_BIT_MASK_ALL);
    touch_ll_clear_trigger_status_mask(&peripherals.SENS);
    touch_ll_set_meas_times(&peripherals.LPWR, TOUCH_PAD_MEASURE_CYCLE_DEFAULT);
    touch_ll_set_sleep_time(&peripherals.LPWR, TOUCH_PAD_SLEEP_CYCLE_DEFAULT);
    /* Configure the touch-sensor power domain into self-bias since bandgap-bias
     * level is different under sleep-mode compared to running-mode. self-bias is
     * always on after chip startup. */
    touch_ll_sleep_low_power(&peripherals.LPWR, true);
    touch_ll_set_voltage_high(&peripherals.LPWR, TOUCH_PAD_HIGH_VOLTAGE_THRESHOLD);
    touch_ll_set_voltage_low(&peripherals.LPWR, TOUCH_PAD_LOW_VOLTAGE_THRESHOLD);
    touch_ll_set_voltage_attenuation(&peripherals.LPWR, TOUCH_PAD_ATTEN_VOLTAGE_THRESHOLD);
    touch_ll_set_idle_channel_connect(&peripherals.LPWR, TOUCH_PAD_IDLE_CH_CONNECT_DEFAULT);
    /* Clear touch channels to initialize the channel value (benchmark, raw_data).
     * Note: Should call it after enable clock gate. */
    touch_ll_clkgate(&peripherals.LPWR, true);  // Enable clock gate for touch sensor.
    touch_ll_reset_benchmark(&peripherals.SENS, touch_pad_t::TOUCH_PAD_MAX);
    touch_ll_sleep_reset_benchmark(&peripherals.LPWR);
    //      end touch_halt_init
    //  end touch_pad_init

    const TOUCH_BUTTON_NUM: usize = 7;
    const TOUCH_BUTTONS: [touch_pad_t; TOUCH_BUTTON_NUM] = [ //touch0 is internal denoise channel
        touch_pad_t::TOUCH_PAD_NUM1, //gpio1
        touch_pad_t::TOUCH_PAD_NUM2, //gpio2
        touch_pad_t::TOUCH_PAD_NUM3, //gpio3
        touch_pad_t::TOUCH_PAD_NUM4, //gpio4
        touch_pad_t::TOUCH_PAD_NUM5, //gpio5
        touch_pad_t::TOUCH_PAD_NUM6, //gpio6
        touch_pad_t::TOUCH_PAD_NUM8, //gpio8
    ];

    for i in 0..TOUCH_BUTTON_NUM {
        touch_pad_config(&peripherals.SENS, &peripherals.LPWR, &TOUCH_BUTTONS[i]);
    }
    
    /* If you want change the touch sensor default setting, please write here(after initialize). There are examples: */
    /*
    touch_pad_set_measurement_interval(TOUCH_PAD_SLEEP_CYCLE_DEFAULT);
    touch_pad_set_charge_discharge_times(TOUCH_PAD_MEASURE_CYCLE_DEFAULT);
    touch_pad_set_voltage(TOUCH_PAD_HIGH_VOLTAGE_THRESHOLD, TOUCH_PAD_LOW_VOLTAGE_THRESHOLD, TOUCH_PAD_ATTEN_VOLTAGE_THRESHOLD);
    touch_pad_set_idle_channel_connect(TOUCH_PAD_IDLE_CH_CONNECT_DEFAULT);
    for (int i = 0; i < TOUCH_BUTTON_NUM; i++) {
        touch_pad_set_cnt_mode(button[i], TOUCH_PAD_SLOPE_DEFAULT, TOUCH_PAD_TIE_OPT_DEFAULT);
    }
    */
    
    /* Denoise setting at TouchSensor 0. */
    let denoise = touch_pad_denoise_t {
        grade: touch_pad_denoise_grade_t::TOUCH_PAD_DENOISE_BIT4,
        cap_level: touch_pad_denoise_cap_t::TOUCH_PAD_DENOISE_CAP_L4,
    };
    touch_pad_denoise_set_config(&peripherals.LPWR, &denoise);
    touch_pad_denoise_enable(&peripherals.SENS, &peripherals.LPWR);
    
    /* Enable touch sensor clock. Work mode is "timer trigger". */
    touch_pad_set_fsm_mode(&peripherals.LPWR, TOUCH_FSM_MODE_TIMER);
    touch_pad_fsm_start(&peripherals.LPWR);
    

    /*usb serial debug*/
    let usb = Usb::new(peripherals.USB0, io.pins.gpio19, io.pins.gpio20);
    let usb_bus = UsbBus::new(usb, unsafe { &mut *addr_of_mut!(EP_MEMORY) });

    let serial = SerialPort::new(&usb_bus);
    let mut serial = SerialWrapper(serial);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x303A, 0x3001))
        .device_class(USB_CLASS_CDC)
        .build();
    
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

    let mut touch_value: [u32; TOUCH_BUTTON_NUM] = [0; TOUCH_BUTTON_NUM];
    loop {
        if !usb_dev.poll(&mut [&mut serial.0]) {
            continue;
        }
        write!(serial, "touch_value: {:?}\n", touch_value).unwrap();
        delay.delay(2.secs());
        for i in 0..TOUCH_BUTTON_NUM {
            touch_pad_read_raw_data(&peripherals.SENS, TOUCH_BUTTONS[i].clone(), &mut touch_value[i]);
        }
    }


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
    led.set_low();

    let mut volume0 = volume_manager.open_volume(VolumeIdx(0)).expect("failed to open volume");
    let mut root_dir = volume0.open_root_dir().expect("failed to open volume");

    let mut file_name = String::with_capacity(7);
    loop {
        for i in 0..99 {
            file_name.clear();
            write!(&mut file_name, "{0: >02}.tif", i).unwrap();
            match open_4bpp_image(&mut root_dir, &mut img_buf, &file_name) {
                Ok(_) => {
                    //eink_display.write_4bpp_reverse_image(&img_buf);
                    eink_display.write_all_black();
                    eink_display.write_4bpp_image(&img_buf);
                    led.set_high();
                    delay.delay(2.secs());
                    led.set_low();
                },
                Err(_error) => {
                    /* not found file */
                },
            };
        }
    }
}
