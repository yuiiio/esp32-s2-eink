#![no_std]
#![no_main]
#![feature(allocator_api)]
#![feature(asm_experimental_arch)]

extern crate alloc;
use alloc::string::String;
use alloc::boxed::Box;

use esp_alloc;

use core::fmt::Write;
use core::ops::ControlFlow;
use core::ptr::addr_of_mut;

use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    delay::Delay,
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{Level, Output, OutputConfig, AnyPin, Pin},
    main,
    otg_fs::{Usb, UsbBus},
    peripherals::{DEDICATED_GPIO, GPIO, IO_MUX, SYSTEM},
    psram,
    spi::master::Spi,
    time::Rate,
};

esp_bootloader_esp_idf::esp_app_desc!();

use usb_device::prelude::{UsbDeviceBuilder, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use embedded_storage::{ReadStorage, Storage};
use esp_storage::FlashStorage;

use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{
    SdCard, BlockDevice, Directory, Error, Mode, SdCardError, ShortFileName, VolumeIdx,
    VolumeManager,
};

mod eink;
use eink::{EinkDisplay, TWO_BPP_BUF_SIZE, HEIGHT, BLACK_FOUR_PIXEL, WHITE_FOUR_PIXEL};

mod touch;
use touch::{TouchInput, TouchThresholds};

mod page_cache;
use page_cache::{PageCache, PageId};

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

/*
static mut EP_MEMORY: [u32; 1024] = [0; 1024];

struct SerialWrapper<'b, B: usb_device::class_prelude::UsbBus>(SerialPort<'b, B>);
impl<B: usb_device::class_prelude::UsbBus> core::fmt::Write for SerialWrapper<'_, B> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let _ = self.0.write(s.as_bytes());
        Ok(())
    }
}
*/

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

/// Count subdirectories in a directory (excluding . and ..)
/// Returns early when max_count is reached
fn count_subdirs<
    D: embedded_sdmmc::BlockDevice,
    T: embedded_sdmmc::TimeSource,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
>(
    dir: &Directory<D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
    max_count: u16,
) -> u16 {
    let mut count: u16 = 0;
    let _ = dir.iterate_dir(|entry| {
        if entry.attributes.is_directory()
            && entry.name != ShortFileName::parent_dir()
            && entry.name != ShortFileName::this_dir()
        {
            count += 1;
            if count >= max_count {
                return ControlFlow::Break(());
            }
        }
        ControlFlow::Continue(())
    });
    count
}

/// Count files in a directory (excluding . and ..)
/// Returns early when max_count is reached
fn count_entries<
    D: embedded_sdmmc::BlockDevice,
    T: embedded_sdmmc::TimeSource,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
>(
    dir: &Directory<D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
    max_count: u16,
) -> u16 {
    let mut count: u16 = 0;
    let _ = dir.iterate_dir(|entry| {
        if entry.name != ShortFileName::parent_dir()
            && entry.name != ShortFileName::this_dir()
        {
            count += 1;
            if count >= max_count {
                return ControlFlow::Break(());
            }
        }
        ControlFlow::Continue(())
    });
    count
}

fn open_2bpp_image<
    D: embedded_sdmmc::BlockDevice,
    T: embedded_sdmmc::TimeSource,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
>(
    cur_dir: &mut Directory<D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
    img_buf: &mut [u8; TWO_BPP_BUF_SIZE],
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
        .modify(|_, w| w.dedicated_gpio_clk_en().bit(true));
    SYSTEM::regs()
        .cpu_peri_rst_en()
        .modify(|_, w| w.dedicated_gpio_rst().bit(true));
    SYSTEM::regs()
        .cpu_peri_rst_en()
        .modify(|_, w| w.dedicated_gpio_rst().bit(false));

    let delay = Delay::new();

    let mut led = Output::new(peripherals.GPIO15, Level::High, OutputConfig::default());

    /*usb serial debug*/
    /*
    let usb = Usb::new(peripherals.USB0, peripherals.GPIO20, peripherals.GPIO19);
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
    let mut flash = FlashStorage::new(peripherals.FLASH);
    let flash_addr = 0x9000; // default NVS size: 0x6000
                             //write!(serial, "Flash size = {}\n", flash.capacity()).unwrap();
    flash.read(flash_addr, &mut bytes).unwrap();
    //write!(serial, "read = {:02x?}\n", &bytes[..4]).unwrap();

    // Flash format: title (12 bits) | chapter (10 bits) | page (10 bits) = 32 bits

    /* sd card with DMA */
    let sclk = peripherals.GPIO36;
    let miso = peripherals.GPIO37;
    let mosi = peripherals.GPIO35;
    let cs = Output::new(peripherals.GPIO34, Level::High, OutputConfig::default());

    // DMA buffers - 512 bytes for SD card block size + some margin
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(1024);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let spi = Spi::new(
        peripherals.SPI2,
        esp_hal::spi::master::Config::default()
            .with_frequency(Rate::from_mhz(80))
            .with_mode(esp_hal::spi::Mode::_0),
    )
    .unwrap()
    .with_sck(sclk)
    .with_mosi(mosi)
    .with_miso(miso)
    .with_dma(peripherals.DMA_SPI2)
    .with_buffers(dma_rx_buf, dma_tx_buf);

    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

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

    // Page cache in PSRAM (5 buffers × 379KB ≈ 1.9MB)
    // prev_display is managed by index, no separate buffer needed
    let mut page_cache = PageCache::new();


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
    let _pin_guard: [AnyPin; 8] = [ // cannot be use a macro?
        peripherals.GPIO18.degrade(),
        peripherals.GPIO21.degrade(),
        peripherals.GPIO16.degrade(),
        peripherals.GPIO17.degrade(),
        peripherals.GPIO7.degrade(),
        peripherals.GPIO9.degrade(),
        peripherals.GPIO10.degrade(),
        peripherals.GPIO13.degrade(),
    ];
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
    for i in 0..8 {
    unsafe { &*IO_MUX::PTR }
        .gpio(eink_data_bus_ios[i] % 32)
        .modify(|_, w| unsafe { w.mcu_sel().bits(1) }); // set to Function 1
    }

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
        _pin_guard,
    };
    eink_display.write_all(WHITE_FOUR_PIXEL);
    eink_display.write_all(BLACK_FOUR_PIXEL);

    let volume0 = volume_manager
        .open_volume(VolumeIdx(0))
        .expect("failed to open volume");
    let root_dir = volume0.open_root_dir().expect("failed to open volume");

    const MAX_ROOT_DIRS: u16 = 9999;
    const MAX_CHILD_FILES: u16 = 999;

    let mut chapter_name = String::with_capacity(5); //xxxx (chapter number)
    let mut file_name = String::with_capacity(8); //xxx.tif

    // Open /books directory as new root for book navigation
    let books_dir = root_dir.open_dir("BOOKS").expect("failed to open /BOOKS directory");

    const MAX_TITLES: u16 = 999;
    const MAX_CHAPTERS: u16 = 999;

    // Count books (title directories) in /books
    let books_count = count_subdirs(&books_dir, MAX_TITLES);

    // Get n-th directory name (1-indexed)
    fn get_nth_dir_name<
        D: embedded_sdmmc::BlockDevice,
        T: embedded_sdmmc::TimeSource,
        const MAX_DIRS: usize,
        const MAX_FILES: usize,
        const MAX_VOLUMES: usize,
    >(
        dir: &Directory<D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
        n: u16,
        name_buf: &mut String,
    ) {
        name_buf.clear();
        let mut count: u16 = 0;
        let _ = dir.iterate_dir(|entry| {
            if entry.attributes.is_directory()
                && entry.name != ShortFileName::parent_dir()
                && entry.name != ShortFileName::this_dir()
            {
                count += 1;
                if count == n {
                    // Convert ShortFileName to string
                    let base = entry.name.base_name();
                    for &b in base.iter() {
                        if b == b' ' || b == 0 {
                            break;
                        }
                        let _ = name_buf.push(b as char);
                    }
                    return ControlFlow::Break(());
                }
            }
            ControlFlow::Continue(())
        });
    }

    // Persistent state: title (12 bits), chapter (10 bits), page (10 bits) = 32 bits
    let last_title_num: u16 = ((bytes[0] as u16) << 4) | ((bytes[1] as u16) >> 4);
    let last_chapter_num: u16 = (((bytes[1] as u16) & 0x0F) << 6) | ((bytes[2] as u16) >> 2);
    let last_page_num: u16 = (((bytes[2] as u16) & 0x03) << 8) | (bytes[3] as u16);

    let mut cur_title: u16 = if last_title_num == 0 || last_title_num > books_count {
        1
    } else {
        last_title_num
    };

    // Get current title directory name and open it
    let mut title_name = String::with_capacity(12); // 8.3 format max
    get_nth_dir_name(&books_dir, cur_title, &mut title_name);
    let mut title_dir = books_dir.open_dir(title_name.as_str()).unwrap();

    // Count chapters in current title
    let mut chapters_count = count_subdirs(&title_dir, MAX_CHAPTERS);

    let mut cur_chapter: u16 = if last_chapter_num == 0 || last_chapter_num > chapters_count {
        1
    } else {
        last_chapter_num
    };

    // Open chapter directory
    write!(&mut chapter_name, "{0: >04}", cur_chapter).unwrap();
    let mut cur_child_dir = title_dir.open_dir(chapter_name.as_str()).unwrap();
    let mut cur_dir_files_len = count_entries(&cur_child_dir, MAX_CHILD_FILES);

    let mut cur_page: u16 = if last_page_num > cur_dir_files_len {
        0
    } else {
        last_page_num
    };

    /*self cpu impl touch pad*/
    let touch_out = Output::new(peripherals.GPIO1, Level::Low, OutputConfig::default());

    let mut adc1_config = AdcConfig::new();
    let mut touch_left = adc1_config.enable_pin(peripherals.GPIO2, Attenuation::_11dB);
    let mut touch_right = adc1_config.enable_pin(peripherals.GPIO3, Attenuation::_11dB);
    let mut touch_center = adc1_config.enable_pin(peripherals.GPIO4, Attenuation::_11dB);
    let mut touch_top = adc1_config.enable_pin(peripherals.GPIO5, Attenuation::_11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);

    let mut touch_input = TouchInput::new(
        touch_out,
        delay,
        TouchInput::DEFAULT_PULSE_DELAY_NS,
        TouchThresholds::default_thresholds(),
    );

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
    match open_2bpp_image(&mut cur_child_dir, next_buf, &file_name) {
        Ok(_) => {
            let t1 = esp_hal::time::Instant::now();
            /*
            eink_display.write_all(WHITE_FOUR_PIXEL);
            eink_display.write_all(WHITE_FOUR_PIXEL);
            eink_display.write_all(BLACK_FOUR_PIXEL);
            */
            eink_display.write_2bpp_image_rev(pre_buf);

            core::mem::swap(&mut pre_buf, &mut next_buf);

            eink_display.write_2bpp_image(pre_buf);
            let t2 = esp_hal::time::Instant::now();

            let elapsed = t2 - t1;
            loop {
                if usb_dev.poll(&mut [&mut serial.0]) {
                    break;
                }
            }
            writeln!(serial, "benchmark elapsed: {}\n", elapsed).unwrap();
            loop {
                if usb_dev.poll(&mut [&mut serial.0]) {
                    break;
                }
            }
        }
        Err(_) => {}
    }
    */

    led.set_low();
    loop {
        'inner: loop {
            let touch = touch_input.read_all(
                &mut adc1,
                &mut touch_left,
                &mut touch_right,
                &mut touch_center,
                &mut touch_top,
            );

            if touch.top {
                eink_display.write_all(BLACK_FOUR_PIXEL);
                eink_display.write_all(WHITE_FOUR_PIXEL);
            }

            if touch.left {
                if cur_page == 0 {
                    if cur_chapter == 1 {
                        // Go to previous title's last chapter
                        if cur_title == 1 {
                            cur_title = books_count; // circling
                        } else {
                            cur_title -= 1;
                        }
                        title_dir.close().unwrap();
                        get_nth_dir_name(&books_dir, cur_title, &mut title_name);
                        title_dir = books_dir.open_dir(title_name.as_str()).unwrap();
                        chapters_count = count_subdirs(&title_dir, MAX_CHAPTERS);
                        cur_chapter = chapters_count;
                    } else {
                        cur_chapter -= 1;
                    }
                    chapter_name.clear();
                    write!(&mut chapter_name, "{0: >04}", cur_chapter).unwrap();
                    cur_child_dir.close().unwrap();
                    cur_child_dir = title_dir.open_dir(chapter_name.as_str()).unwrap();
                    cur_dir_files_len = count_entries(&cur_child_dir, MAX_CHILD_FILES);

                    cur_page = cur_dir_files_len - 1;
                } else {
                    cur_page = cur_page - 1;
                }
                break 'inner;
            }
            if touch.right {
                if cur_page == (cur_dir_files_len - 1) {
                    if cur_chapter == chapters_count {
                        // Go to next title's first chapter
                        if cur_title == books_count {
                            cur_title = 1; // circling
                        } else {
                            cur_title += 1;
                        }
                        title_dir.close().unwrap();
                        get_nth_dir_name(&books_dir, cur_title, &mut title_name);
                        title_dir = books_dir.open_dir(title_name.as_str()).unwrap();
                        chapters_count = count_subdirs(&title_dir, MAX_CHAPTERS);
                        cur_chapter = 1;
                    } else {
                        cur_chapter += 1;
                    }
                    chapter_name.clear();
                    write!(&mut chapter_name, "{0: >04}", cur_chapter).unwrap();
                    cur_child_dir.close().unwrap();
                    cur_child_dir = title_dir.open_dir(chapter_name.as_str()).unwrap();
                    cur_dir_files_len = count_entries(&cur_child_dir, MAX_CHILD_FILES);

                    cur_page = 0;
                } else {
                    cur_page = cur_page + 1;
                }
                break 'inner;
            }
            if touch.center {
                let bottom_indicator_pos_current: u32 =
                    HEIGHT as u32 - ((cur_page as u32 * HEIGHT as u32) / cur_dir_files_len as u32);
                eink_display.write_bottom_indicator(true, bottom_indicator_pos_current, 0);
                let mut bottom_pre_status_var = bottom_indicator_pos_current;

                touch_input.delay().delay_millis(500u32);

                '_page_indicator: loop {
                    let touch = touch_input.read_all(
                        &mut adc1,
                        &mut touch_left,
                        &mut touch_right,
                        &mut touch_center,
                        &mut touch_top,
                    );

                    let mut indicator_refresh = false;
                    const SKIP_PAGE: u16 = 1;
                    if touch.left {
                        if cur_page < SKIP_PAGE {
                            // open pre chapter dir
                            if cur_chapter == 1 {
                                // Go to previous title's last chapter
                                if cur_title == 1 {
                                    cur_title = books_count; //circling
                                } else {
                                    cur_title = cur_title - 1;
                                }
                                title_dir.close().unwrap();
                                get_nth_dir_name(&books_dir, cur_title, &mut title_name);
                                title_dir = books_dir.open_dir(title_name.as_str()).unwrap();
                                chapters_count = count_subdirs(&title_dir, MAX_CHAPTERS);
                                cur_chapter = chapters_count;
                            } else {
                                cur_chapter = cur_chapter - 1;
                            }
                            chapter_name.clear();
                            write!(&mut chapter_name, "{0: >04}", cur_chapter).unwrap();
                            cur_child_dir.close().unwrap();
                            cur_child_dir = title_dir.open_dir(chapter_name.as_str()).unwrap();
                            cur_dir_files_len = count_entries(&cur_child_dir, MAX_CHILD_FILES);
                            cur_page = cur_dir_files_len - 1; // pre chapter dir's last
                            indicator_refresh = true;

                        } else {
                            cur_page = cur_page - SKIP_PAGE;
                        }
                        let bottom_indicator_pos_current: u32 = HEIGHT as u32
                            - ((cur_page as u32 * HEIGHT as u32) / cur_dir_files_len as u32);
                        eink_display.write_bottom_indicator(
                            indicator_refresh,
                            bottom_indicator_pos_current,
                            bottom_pre_status_var,
                        );
                        bottom_pre_status_var = bottom_indicator_pos_current;
                    }
                    if touch.right {
                        if cur_page > (cur_dir_files_len - 1) - SKIP_PAGE {
                            // open next chapter dir
                            if cur_chapter == chapters_count {
                                // Go to next title's first chapter
                                if cur_title == books_count {
                                    cur_title = 1; //circling
                                } else {
                                    cur_title = cur_title + 1;
                                }
                                title_dir.close().unwrap();
                                get_nth_dir_name(&books_dir, cur_title, &mut title_name);
                                title_dir = books_dir.open_dir(title_name.as_str()).unwrap();
                                chapters_count = count_subdirs(&title_dir, MAX_CHAPTERS);
                                cur_chapter = 1;
                            } else {
                                cur_chapter = cur_chapter + 1;
                            }
                            chapter_name.clear();
                            write!(&mut chapter_name, "{0: >04}", cur_chapter).unwrap();
                            cur_child_dir.close().unwrap();
                            cur_child_dir = title_dir.open_dir(chapter_name.as_str()).unwrap();
                            cur_dir_files_len = count_entries(&cur_child_dir, MAX_CHILD_FILES);
                            cur_page = 0; // next chapter dir's first
                            indicator_refresh = true;
                        } else {
                            cur_page = cur_page + SKIP_PAGE;
                        }
                        let bottom_indicator_pos_current: u32 = HEIGHT as u32
                            - ((cur_page as u32 * HEIGHT as u32) / cur_dir_files_len as u32);
                        eink_display.write_bottom_indicator(
                            indicator_refresh,
                            bottom_indicator_pos_current,
                            bottom_pre_status_var,
                        );
                        bottom_pre_status_var = bottom_indicator_pos_current;
                    }
                    if touch.center {
                        break 'inner;
                    }

                    if touch.top {
                        // Chapter indicator mode - navigate chapters within current title
                        let top_indicator_pos_current: u32 = HEIGHT as u32
                            - ((cur_chapter as u32 * HEIGHT as u32) / chapters_count as u32);
                        eink_display.write_top_indicator(true, top_indicator_pos_current, 0);
                        let mut top_pre_status_var = top_indicator_pos_current;

                        touch_input.delay().delay_millis(500_u32);
                        '_chaptor_indicator: loop {
                            let touch = touch_input.read_all(
                                &mut adc1,
                                &mut touch_left,
                                &mut touch_right,
                                &mut touch_center,
                                &mut touch_top,
                            );

                            if touch.left {
                                if cur_chapter == 1 {
                                    cur_chapter = chapters_count; //circling within title
                                } else {
                                    cur_chapter = cur_chapter - 1;
                                }
                                let top_indicator_pos_current: u32 = HEIGHT as u32
                                    - ((cur_chapter as u32 * HEIGHT as u32)
                                        / chapters_count as u32);
                                eink_display.write_top_indicator(
                                    false,
                                    top_indicator_pos_current,
                                    top_pre_status_var,
                                );
                                top_pre_status_var = top_indicator_pos_current;
                            }
                            if touch.right {
                                if cur_chapter == chapters_count {
                                    cur_chapter = 1; //circling within title
                                } else {
                                    cur_chapter = cur_chapter + 1;
                                }
                                let top_indicator_pos_current: u32 = HEIGHT as u32
                                    - ((cur_chapter as u32 * HEIGHT as u32)
                                        / chapters_count as u32);
                                eink_display.write_top_indicator(
                                    false,
                                    top_indicator_pos_current,
                                    top_pre_status_var,
                                );
                                top_pre_status_var = top_indicator_pos_current;
                            }
                            if touch.center {
                                chapter_name.clear();
                                write!(&mut chapter_name, "{0: >04}", cur_chapter).unwrap();
                                cur_child_dir.close().unwrap();
                                cur_child_dir = title_dir.open_dir(chapter_name.as_str()).unwrap();
                                cur_dir_files_len = count_entries(&cur_child_dir, MAX_CHILD_FILES);

                                cur_page = 0;
                                break 'inner;
                            }
                            if touch.top {
                                // Title indicator mode - navigate between titles
                                let center_indicator_pos_current: u32 = HEIGHT as u32
                                    - ((cur_title as u32 * HEIGHT as u32) / books_count as u32);
                                eink_display.write_center_indicator(true, center_indicator_pos_current, 0);
                                let mut center_pre_status_var = center_indicator_pos_current;

                                touch_input.delay().delay_millis(500_u32);
                                '_title_indicator: loop {
                                    let touch = touch_input.read_all(
                                        &mut adc1,
                                        &mut touch_left,
                                        &mut touch_right,
                                        &mut touch_center,
                                        &mut touch_top,
                                    );

                                    if touch.left {
                                        if cur_title == 1 {
                                            cur_title = books_count; //circling
                                        } else {
                                            cur_title = cur_title - 1;
                                        }
                                        let center_indicator_pos_current: u32 = HEIGHT as u32
                                            - ((cur_title as u32 * HEIGHT as u32)
                                                / books_count as u32);
                                        eink_display.write_center_indicator(
                                            false,
                                            center_indicator_pos_current,
                                            center_pre_status_var,
                                        );
                                        center_pre_status_var = center_indicator_pos_current;
                                    }
                                    if touch.right {
                                        if cur_title == books_count {
                                            cur_title = 1; //circling
                                        } else {
                                            cur_title = cur_title + 1;
                                        }
                                        let center_indicator_pos_current: u32 = HEIGHT as u32
                                            - ((cur_title as u32 * HEIGHT as u32)
                                                / books_count as u32);
                                        eink_display.write_center_indicator(
                                            false,
                                            center_indicator_pos_current,
                                            center_pre_status_var,
                                        );
                                        center_pre_status_var = center_indicator_pos_current;
                                    }
                                    if touch.center {
                                        // Confirm title selection, open first chapter
                                        title_dir.close().unwrap();
                                        get_nth_dir_name(&books_dir, cur_title, &mut title_name);
                                        title_dir = books_dir.open_dir(title_name.as_str()).unwrap();
                                        chapters_count = count_subdirs(&title_dir, MAX_CHAPTERS);

                                        cur_chapter = 1;
                                        chapter_name.clear();
                                        write!(&mut chapter_name, "{0: >04}", cur_chapter).unwrap();
                                        cur_child_dir.close().unwrap();
                                        cur_child_dir = title_dir.open_dir(chapter_name.as_str()).unwrap();
                                        cur_dir_files_len = count_entries(&cur_child_dir, MAX_CHILD_FILES);

                                        cur_page = 0;
                                        break 'inner;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        let current_page_id = PageId::new(cur_title, cur_chapter, cur_page);

        // Try to get from cache, or load from SD card
        let (buf, was_cached) = page_cache.get_or_alloc(current_page_id);

        if !was_cached {
            // Load from SD card
            file_name.clear();
            write!(&mut file_name, "{0: >03}.tif", cur_page).unwrap();
            if open_2bpp_image(&mut cur_child_dir, buf, &file_name).is_err() {
                eink_display.write_all(WHITE_FOUR_PIXEL);
                eink_display.write_all(BLACK_FOUR_PIXEL);
                continue;
            }
        }

        // Display: reverse previous, then show current
        eink_display.write_2bpp_image_rev(page_cache.prev_buffer());
        eink_display.write_2bpp_image(page_cache.current_buffer());

        // Mark current as displayed (index swap, no memory copy)
        page_cache.mark_displayed();

        // Save state to flash: title (12 bits) | chapter (10 bits) | page (10 bits) = 32 bits
        let state: u32 = ((cur_title as u32) << 20)
            | ((cur_chapter as u32 & 0x3FF) << 10)
            | (cur_page as u32 & 0x3FF);
        flash.write(flash_addr, &state.to_be_bytes()).unwrap();
    }
}
