extern crate alloc;

use core::arch::asm;

use esp_backtrace as _;
use esp_hal::{
    gpio::AnyPin,
};

use crate::fontdata::{FONT_GLYPHS, FONT_WIDTH, FONT_BYTE_HEIGHT};

//GPIO0〜31 → OUT
//GPIO32〜46 → OUT1
const GPIO_OUT_W1TS_REG: *mut u32 = 0x3F404008 as *mut u32;
const GPIO_OUT_W1TC_REG: *mut u32 = 0x3F40400C as *mut u32;
const GPIO_OUT1_W1TS_REG: *mut u32 = 0x3F404014 as *mut u32;
const GPIO_OUT1_W1TC_REG: *mut u32 = 0x3F404018 as *mut u32;
// avoid hal runtime check for gpio set_high/low.
pub struct MyGpio<const PIN: u32>;

impl<const PIN: u32> MyGpio<PIN> {
    const REG_SET: *mut u32 = if PIN < 32 { GPIO_OUT_W1TS_REG as *mut u32 } else { GPIO_OUT1_W1TS_REG as *mut u32 };
    const REG_CLR: *mut u32 = if PIN < 32 { GPIO_OUT_W1TC_REG as *mut u32 } else { GPIO_OUT1_W1TC_REG as *mut u32 };
    const MASK: u32 = 1u32 << (if PIN < 32 { PIN } else { PIN - 32 });

    #[inline(always)]
    pub fn set_high(&self) {
        unsafe { core::ptr::write_volatile(Self::REG_SET, Self::MASK); }
    }

    #[inline(always)]
    pub fn set_low(&self) {
        unsafe { core::ptr::write_volatile(Self::REG_CLR, Self::MASK); }
    }
}

fn nop_delay() {
    let mut n = 15;
    while n > 0 {
        unsafe {
            asm!(
                "nop",
                "nop",
                "nop",
                "nop",
                options(nomem, nostack)
            );
        }
        n -= 1;
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
pub const WIDTH: usize = 1072;
pub const HEIGHT: usize = 1448;

pub const TWO_BPP_BUF_SIZE: usize = HEIGHT * WIDTH * 2 / 8;

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

pub struct EinkDisplay<
const MODE1: u32,
const CKV: u32,
const SPV: u32,
const XCL: u32,
const XLE: u32,
const XOE: u32,
const XSTL: u32,
> {
    pub mode1: MyGpio<MODE1>,
    pub ckv: MyGpio<CKV>,
    pub spv: MyGpio<SPV>,

    pub xcl: MyGpio<XCL>,
    pub xle: MyGpio<XLE>,
    pub xoe: MyGpio<XOE>,
    pub xstl: MyGpio<XSTL>,
    pub _pin_guard: [AnyPin<'static>; 8],
}

impl<
const MODE1: u32,
const CKV: u32,
const SPV: u32,
const XCL: u32,
const XLE: u32,
const XOE: u32,
const XSTL: u32,
> EinkDisplay<
MODE1,
CKV,
SPV,
XCL,
XLE,
XOE,
XSTL, 
> {
    fn start_frame(&mut self) {
        self.xoe.set_high();
        self.mode1.set_high();
        self.spv.set_low();
        self.ckv.set_low();
        nop_delay();
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

    pub fn write_2bpp_image(&mut self, img_buf: &[u8; TWO_BPP_BUF_SIZE]) {
        for state in 0..2 {
            self.start_frame();
            let lut = unsafe {
                LUT.get_unchecked(state)
            };
            for row in img_buf.chunks_exact(HEIGHT/4) {
                self.xstl.set_low();
                /* can write 4 pixel for onece */
                for &src in row {
                    let b = unsafe {
                        *lut.get_unchecked(src as usize)
                    };
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
                nop_delay();
                self.ckv.set_high();
            }
            self.end_frame();
        }
    }

    pub fn write_2bpp_image_rev(&mut self, img_buf: &[u8; TWO_BPP_BUF_SIZE]) {
        for state in 0..1 {
            self.start_frame();
            let rev_lut = unsafe {
                REVERSE_LUT.get_unchecked(state)
            };
            for row in img_buf.chunks_exact(HEIGHT/4) {
                self.xstl.set_low();
                /* can write 4 pixel for onece */
                for &src in row {
                    let b = unsafe {
                        *rev_lut.get_unchecked(src as usize)
                    };
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
                nop_delay();
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
        for _line in 0..WIDTH {
            self.xstl.set_low();
            /* can write 4 pixel for onece */
            for _pos in 0..(HEIGHT / 4) {
                self.xcl.set_high();
                self.xcl.set_low();
            }
            self.xstl.set_high();
            self.xcl.set_high();
            self.xcl.set_low();

            self.xle.set_high();
            self.xle.set_low();

            self.ckv.set_low();
            nop_delay();
            self.ckv.set_high();
        }
        self.end_frame();
    }

    /// Generalized slider drawing at any horizontal position
    /// - x_pos: horizontal position in pixels (must be multiple of 4)
    /// - slider_height: height in pixels (must be multiple of 4)
    /// - bar_half_height: half height of the slider knob
    /// - draw_split_left: draw separator line on left edge
    /// - draw_split_right: draw separator line on right edge
    pub fn write_slider(
        &mut self,
        y_pos: usize,
        slider_height: usize,
        bar_half_height: u32,
        draw_split_left: bool,
        draw_split_right: bool,
        first_commit: bool,
        status_var: u32,
        pre_status_var: u32,
    ) {
        const SPLIT_HEIGHT: usize = 4; // pixels
        let x_pos_div4 = y_pos / 4;
        let slider_height_div4 = slider_height / 4;
        let split_height_div4 = SPLIT_HEIGHT / 4;

        let left_split = if draw_split_left { split_height_div4 } else { 0 };
        let right_split = if draw_split_right { split_height_div4 } else { 0 };
        let bar_height_div4 = slider_height_div4 - left_split - right_split;

        self.start_frame();
        for line in 0..WIDTH {
            self.xstl.set_low();

            // skip pixels before slider
            if x_pos_div4 > 0 {
                unsafe { asm!("wur.gpio_out {0}", in(reg) NONE_FOUR_PIXEL); }
                for _i in 0..x_pos_div4 {
                    self.xcl.set_high();
                    self.xcl.set_low();
                }
                nop_delay();
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
                for _i in 0..split_height_div4 {
                    nop_delay();
                    self.xcl.set_high();
                    self.xcl.set_low();
                }
            }

            // slider body
            unsafe { asm!("wur.gpio_out {0}", in(reg) pixel); }
            for _i in 0..bar_height_div4 {
                nop_delay();
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
                for _i in 0..split_height_div4 {
                    nop_delay();
                    self.xcl.set_high();
                    self.xcl.set_low();
                }
            }

            // skip pixels after slider
            let remaining = (HEIGHT / 4) - x_pos_div4 - slider_height_div4;
            if remaining > 0 {
                unsafe { asm!("wur.gpio_out {0}", in(reg) NONE_FOUR_PIXEL); }
                nop_delay();
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
            nop_delay();
            self.ckv.set_high();
        }
        self.end_frame();
    }

    // Convenience wrappers for backward compatibility
    pub fn write_top_indicator(&mut self, first_commit: bool, status_var: u32, pre_status_var: u32) {
        self.write_slider(
            0,      // y_pos: top edge
            100,    // height
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
            HEIGHT - 100,  // y_pos: bottom edge
            100,          // height
            20,           // bar_half_height
            true,         // left split
            false,        // no right split
            first_commit,
            status_var,
            pre_status_var,
        );
    }

    /// Center slider for title selection (y_pos = HEIGHT/2 - height/2, rounded to multiple of 4)
    pub fn write_center_indicator(&mut self, first_commit: bool, status_var: u32, pre_status_var: u32) {
        const SLIDER_HEIGHT: usize = 100;
        const X_POS: usize = (HEIGHT / 2 - SLIDER_HEIGHT / 2) / 4 * 4; // 672
        self.write_slider(
            X_POS,
            SLIDER_HEIGHT,
            20,           // bar_half_height
            true,         // left split
            true,         // right split (both sides for center)
            first_commit,
            status_var,
            pre_status_var,
        );
    }
}

impl<
const MODE1: u32,
const CKV: u32,
const SPV: u32,
const XCL: u32,
const XLE: u32,
const XOE: u32,
const XSTL: u32,
> EinkDisplay<MODE1, CKV, SPV, XCL, XLE, XOE, XSTL>
{
    /// 文字列スライス(一行)を指定位置にフォントで表示（バッファ展開せず、直接画面に pixel 書き込み）
    /// - str: 表示する文字列
    /// - x: 文字列の左端位置（4ピクセル align)
    /// - y: 文字列の左端位置（4ピクセル align)
    /// - 画面端を超える文字は無視 
    /// - とりあえず画面4x4ピクセルをフォントの1ピクセルとする
    pub fn write_fontbuf_at_pos(
        &mut self,
        str: &str,
        x: usize,
        y: usize,
    ) {
        let display_chars = str.as_bytes();
        let y_div4 = y / 4;
        let y = y_div4 * 4; // 4 align
        let x_div4 = x / 4;
        let x = x_div4 * 4; // 4 align

        let text_len = display_chars.len();
        let max_chars = (WIDTH - x / 4) / FONT_WIDTH;
        let text_len = text_len.min(max_chars);

        self.start_frame();
         // skip w_row after end text, first
        unsafe { asm!("wur.gpio_out {0}", in(reg) NONE_FOUR_PIXEL); }
        for _row in 0..(WIDTH - x - text_len * FONT_WIDTH) {
            self.xstl.set_low();
            for _pos in 0..(HEIGHT / 4) {
                self.xcl.set_high();
                self.xcl.set_low();
            }
            self.xstl.set_high();
            self.xcl.set_high();
            self.xcl.set_low();

            self.xle.set_high();
            self.xle.set_low();

            self.ckv.set_low();
            nop_delay();
            self.ckv.set_high();
        }
        for texi in (0..text_len).rev() {
            let glyph_index = (display_chars[texi] - 0x20) as usize;
            let glyph = &FONT_GLYPHS[glyph_index];
            for row in 0..FONT_WIDTH {
                self.xstl.set_low();
                // skip pixels before text height
                if y_div4 > 0 {
                    unsafe { asm!("wur.gpio_out {0}", in(reg) NONE_FOUR_PIXEL); }
                    for _i in 0..y_div4 {
                        self.xcl.set_high();
                        self.xcl.set_low();
                    }
                    nop_delay();
                }

                let h_stride = row * FONT_BYTE_HEIGHT;
                for hb in 0..FONT_BYTE_HEIGHT {
                    let byte_idx = h_stride + hb;
                    for iy in 0..8{
                        let mut b = WHITE_FOUR_PIXEL;
                        if glyph[byte_idx] & (1 << iy) != 0 {
                            b = BLACK_FOUR_PIXEL;
                        }
                        unsafe {
                            asm!("wur.gpio_out {0}", in(reg) b);
                        };
                        self.xcl.set_high();
                        self.xcl.set_low();
                    }
                }
                unsafe { asm!("wur.gpio_out {0}", in(reg) NONE_FOUR_PIXEL); }
                // skip pixels after text height
                for _i in 0..((HEIGHT - y - FONT_BYTE_HEIGHT * 8 * 4) / 4) {
                    self.xcl.set_high();
                    self.xcl.set_low();
                }
                self.xstl.set_high();
                self.xcl.set_high();
                self.xcl.set_low();

                self.xle.set_high();
                self.xle.set_low();

                self.ckv.set_low();
                nop_delay();
                self.ckv.set_high();
            }
        }
        // skip w_row before start text, end
        unsafe { asm!("wur.gpio_out {0}", in(reg) NONE_FOUR_PIXEL); }
        for _row in 0..x {
            self.xstl.set_low();
            for _pos in 0..(HEIGHT / 4) {
                self.xcl.set_high();
                self.xcl.set_low();
            }
            self.xstl.set_high();
            self.xcl.set_high();
            self.xcl.set_low();

            self.xle.set_high();
            self.xle.set_low();

            self.ckv.set_low();
            nop_delay();
            self.ckv.set_high();
        }

        self.end_frame();
    }
}

