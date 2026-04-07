extern crate alloc;

use core::arch::asm;

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Output, AnyPin},
};

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
pub const HEIGHT: usize = 1072;
pub const WIDTH: usize = 1448;

pub const TWO_BPP_BUF_SIZE: usize = WIDTH * HEIGHT * 2 / 8;

pub const BLACK_FOUR_PIXEL: u8 = 0b01010101;
pub const WHITE_FOUR_PIXEL: u8 = 0b10101010;
pub const NONE_FOUR_PIXEL: u8 = 0b00000000;

// waveform for grayscale
const WAVEFORM: [[u8; 4]; 2] = [ 
    [0b01, 0b10, 0b01, 0b10], 
    [0b01, 0b01, 0b10, 0b10], 
];

// put LUT on SRAM
#[link_section = ".dram0.data"]
static LUT: [[u8; 256]; 2] = {
    let mut table = [[0u8; 256]; 2];
    let mut state = 0;
    while state < 2 {
        let mut i = 0;
        while i < 256 {
            let src = i as u8;
            let mut j = 0;
            while j < 4 {
                let shift = (3 - j) * 2;
                let mask = 0b11 << shift;
                let form = WAVEFORM[state as usize][((src & mask) >> shift) as usize];
                table[state as usize][i] |=  form << shift;
                j += 1;
            }
            i += 1;
        }
        state += 1;
    }
    table
};

const REVERSE_WAVEFORM: [[u8; 4]; 2] = [ 
    [0b10, 0b10, 0b01, 0b01], 
    [0b10, 0b10, 0b01, 0b01], 
];

// put LUT on SRAM
#[link_section = ".dram0.data"]
static REVERSE_LUT: [[u8; 256]; 2] = {
    let mut table = [[0u8; 256]; 2];
    let mut state = 0;
    while state < 2 {
        let mut i = 0;
        while i < 256 {
            let src = i as u8;
            let mut j = 0;
            while j < 4 {
                let shift = (3 - j) * 2;
                let mask = 0b11 << shift;
                let form = REVERSE_WAVEFORM[state as usize][((src & mask) >> shift) as usize];
                table[state as usize][i] |=  form << shift;
                j += 1;
            }
            i += 1;
        }
        state += 1;
    }
    table
};

pub struct EinkDisplay {
    pub mode1: Output<'static>,
    pub ckv: Output<'static>,
    pub spv: Output<'static>,

    pub xcl: Output<'static>,
    pub xle: Output<'static>,
    pub xoe: Output<'static>,
    pub xstl: Output<'static>,
    pub delay: Delay,
    pub _pin_guard: [AnyPin<'static>; 8],
}

impl EinkDisplay {
    fn start_frame(&mut self) {
        self.xoe.set_high();
        self.mode1.set_high();
        self.spv.set_low();
        self.ckv.set_low();
        self.delay.delay_micros(1);
        self.ckv.set_high();
        self.spv.set_high();
    }

    fn end_frame(&mut self) {
        self.mode1.set_low();
        self.xoe.set_low();
    }

    // 1pixel 2 bit, so *2 data
    // [0, 0] No action
    // [0, 1] Draw black
    // [1, 0] Draw white
    // [1, 1] No action
    #[allow(dead_code)]
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

    pub fn write_2bpp_image(&mut self, img_buf: &[u8; TWO_BPP_BUF_SIZE]) {
        for state in 0..2 {
            let mut buf_pos: usize = 0;
            self.start_frame();

            for _line in 0..HEIGHT {
                self.xstl.set_low();
                /* can write 4 pixel for onece */
                for _i in 0..(WIDTH / 4) {
                    let b = LUT[state as usize][img_buf[buf_pos] as usize];
                    buf_pos += 1;
                    // write 8bit
                    unsafe {
                        asm!("wur.gpio_out {0}", in(reg) b);
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
            self.end_frame();
        }
    }

    pub fn write_2bpp_image_rev(&mut self, img_buf: &[u8; TWO_BPP_BUF_SIZE]) {
        for state in 0..1 {
            let mut buf_pos: usize = 0;
            self.start_frame();

            for _line in 0..HEIGHT {
                self.xstl.set_low();
                /* can write 4 pixel for onece */
                for _i in 0..(WIDTH / 4) {
                    let b = REVERSE_LUT[state as usize][img_buf[buf_pos] as usize];
                    buf_pos += 1;
                    // write 8bit
                    unsafe {
                        asm!("wur.gpio_out {0}", in(reg) b);
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
            self.end_frame();
        }
    }

    pub fn write_all(&mut self, solid_pixel: u8) {
        unsafe {
            asm!("wur.gpio_out {0}", in(reg) solid_pixel);
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

    /// Generalized slider drawing at any horizontal position
    /// - x_pos: horizontal position in pixels (must be multiple of 4)
    /// - slider_width: width in pixels (must be multiple of 4)
    /// - bar_half_height: half height of the slider knob
    /// - draw_split_left: draw separator line on left edge
    /// - draw_split_right: draw separator line on right edge
    pub fn write_slider(
        &mut self,
        x_pos: usize,
        slider_width: usize,
        bar_half_height: u32,
        draw_split_left: bool,
        draw_split_right: bool,
        first_commit: bool,
        status_var: u32,
        pre_status_var: u32,
    ) {
        const SPLIT_WIDTH: usize = 4; // pixels
        let x_pos_div4 = x_pos / 4;
        let slider_width_div4 = slider_width / 4;
        let split_width_div4 = SPLIT_WIDTH / 4;

        let left_split = if draw_split_left { split_width_div4 } else { 0 };
        let right_split = if draw_split_right { split_width_div4 } else { 0 };
        let bar_width_div4 = slider_width_div4 - left_split - right_split;

        self.start_frame();
        for line in 0..HEIGHT {
            self.xstl.set_low();

            // skip pixels before slider
            if x_pos_div4 > 0 {
                unsafe { asm!("wur.gpio_out {0}", in(reg) NONE_FOUR_PIXEL); }
                for _i in 0..x_pos_div4 {
                    self.xcl.set_high();
                    self.xcl.set_low();
                }
            }

            let in_current = status_var.abs_diff(line as u32) <= bar_half_height;
            let in_previous = pre_status_var.abs_diff(line as u32) <= bar_half_height;

            // determine what to draw for slider body
            let pixel = if first_commit {
                if in_current { BLACK_FOUR_PIXEL } else { WHITE_FOUR_PIXEL }
            } else {
                if in_current && !in_previous {
                    BLACK_FOUR_PIXEL // new position
                } else if !in_current && in_previous {
                    WHITE_FOUR_PIXEL // clear old position
                } else {
                    NONE_FOUR_PIXEL // no change
                }
            };

            // left split bar
            if draw_split_left {
                if first_commit {
                    unsafe { asm!("wur.gpio_out {0}", in(reg) BLACK_FOUR_PIXEL); }
                } else {
                    unsafe { asm!("wur.gpio_out {0}", in(reg) NONE_FOUR_PIXEL); }
                }
                for _i in 0..split_width_div4 {
                    self.xcl.set_high();
                    self.delay.delay_micros(1);
                    self.xcl.set_low();
                }
            }

            // slider body
            unsafe { asm!("wur.gpio_out {0}", in(reg) pixel); }
            for _i in 0..bar_width_div4 {
                self.xcl.set_high();
                self.xcl.set_low();
            }

            // right split bar
            if draw_split_right {
                if first_commit {
                    unsafe { asm!("wur.gpio_out {0}", in(reg) BLACK_FOUR_PIXEL); }
                } else {
                    unsafe { asm!("wur.gpio_out {0}", in(reg) NONE_FOUR_PIXEL); }
                }
                for _i in 0..split_width_div4 {
                    self.xcl.set_high();
                    self.delay.delay_micros(1);
                    self.xcl.set_low();
                }
            }

            // skip pixels after slider
            let remaining = (WIDTH / 4) - x_pos_div4 - slider_width_div4;
            if remaining > 0 {
                unsafe { asm!("wur.gpio_out {0}", in(reg) NONE_FOUR_PIXEL); }
                for _i in 0..remaining {
                    self.xcl.set_high();
                    self.xcl.set_low();
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

    // Convenience wrappers for backward compatibility
    pub fn write_top_indicator(&mut self, first_commit: bool, status_var: u32, pre_status_var: u32) {
        self.write_slider(
            0,      // x_pos: left edge
            100,    // width
            20,     // bar_half_height
            false,  // no left split
            true,   // right split
            first_commit,
            status_var,
            pre_status_var,
        );
    }

    pub fn write_bottom_indicator(&mut self, first_commit: bool, status_var: u32, pre_status_var: u32) {
        self.write_slider(
            WIDTH - 100,  // x_pos: right edge
            100,          // width
            20,           // bar_half_height
            true,         // left split
            false,        // no right split
            first_commit,
            status_var,
            pre_status_var,
        );
    }
}

