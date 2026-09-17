//! Standalone reproduction for the ESP32-S2 stale-DMA-link bug.
//!
//! `SpiDma` falls back to a CPU-driven FIFO transfer whenever a transfer is
//! shorter than `min_async_transfer_size`. PDMA chips (ESP32, ESP32-S2) have no
//! DMA enable bit in `dma_conf`: the peripheral reads from the descriptors
//! instead of the FIFO for as long as a link is armed, so `disable_dma()` has to
//! clear `dma_in_link`/`dma_out_link`. Without that, the CPU transfer following
//! a DMA transfer is silently routed into the previous transfer's descriptors.
//!
//! When that happens the CPU path reads back the bytes it just wrote into the
//! FIFO, because the received data went into the DMA descriptor instead. So a
//! broken run echoes the TX pattern, which is what this program looks for.
//!
//! Unlike the hil-test suite this needs no debug probe: it reports through the
//! on-board LED, and over UART0 if you have a serial console attached.
//!
//! Wiring: none. The MISO pin is driven by us as a plain GPIO output and read back
//! by the SPI peripheral through the GPIO matrix, so MISO carries a level we
//! control exactly. No loopback, no external connection.
//!
//!   cargo build --release --bin spi_dma_stale_link
//!   espflash flash --before no-reset --no-stub \
//!       ./target/xtensa-esp32s2-none-elf/release/spi_dma_stale_link
//!
//! LED (GPIO15):
//!   slow 1 Hz blink  -> PASS, the CPU transfer survived the DMA transfer
//!   fast 10 Hz blink -> FAIL, the stale link swallowed the CPU transfer
//!   2 blinks + pause -> the MISO pin does not read back what we drive; pick another
//!   3 blinks + pause -> SPI did not sample the MISO pin; pick another one

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    dma_rx_buffer,
    dma_tx_buffer,
    gpio::{Flex, InputConfig, Level, Output, OutputConfig},
    main,
    spi::{
        Mode,
        master::{Config, Spi},
    },
    time::Rate,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

/// Transfers shorter than this take the CPU/FIFO path.
const THRESHOLD: usize = 128;
/// Long enough to go through DMA, which arms the link.
const DMA_LEN: usize = 512;
/// Short enough to take the CPU path.
const CPU_LEN: usize = 4;
/// The TX pattern a broken CPU transfer echoes back.
const CPU_TX: [u8; CPU_LEN] = [0xde, 0xad, 0xbe, 0xef];

enum Outcome {
    Pass,
    StaleLink,
    PinNotReadable,
    MisoNotSampled,
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();

    // ---- pin selection -------------------------------------------------
    // GPIO6 and GPIO8 are the only two pins in this bank that `src/main.rs`
    // leaves alone; it claims 1-5, 7, 9-18, 21, 33-40 for the touch pads, the
    // e-ink bus and the SD card. GPIO41/GPIO42 are equally free if these two
    // turn out to be wired to something on the board. Avoid GPIO19/20 (USB),
    // GPIO26-32 (flash/PSRAM) and GPIO43/44 (UART0).
    //
    // MISO_PIN is not wired to any SPI bus: we drive it ourselves as a plain
    // GPIO output and let the SPI peripheral read it back as MISO. MOSI is left
    // unassigned, the TX data does not need to reach a pad.
    let sclk = peripherals.GPIO6;
    let miso_pin = peripherals.GPIO8;

    // The board's LED. Shared with `src/main.rs` on purpose: it is the only
    // output this program has when no serial console is attached.
    let mut led = Output::new(peripherals.GPIO15, Level::Low, OutputConfig::default());
    // ---------------------------------------------------------------------
    // `Flex`, not `Output`: we need the pad driven *and* its input buffer on, so
    // that the SPI peripheral can read back the level we are driving. `Output`
    // leaves the input buffer off, and `Input::new` would turn the driver off.
    let mut miso = Flex::new(miso_pin);
    miso.apply_output_config(&OutputConfig::default());
    miso.apply_input_config(&InputConfig::default());
    miso.set_output_enable(true);
    miso.set_input_enable(true);
    miso.set_low();

    // The signal handed to `with_miso` is frozen, so it keeps the enables we
    // just set instead of letting the SPI driver reconfigure the pin.
    let miso_signal = miso.peripheral_input();

    // Does the pad read back what we drive? If not, the pin is tied to
    // something on the board and the rest of the run would be meaningless.
    miso.set_high();
    let reads_high = miso.is_high();
    miso.set_low();
    let reads_low = miso.is_low();
    if !(reads_high && reads_low) {
        println!("the MISO pin does not read back what we drive; pick a different one");
        report(&mut led, delay, Outcome::PinNotReadable);
    }

    let spi = Spi::new(
        peripherals.SPI2,
        Config::default()
            // MISO carries a static level here, so there is no reason to push the
            // clock. Slow enough that GPIO matrix latency cannot matter.
            .with_frequency(Rate::from_mhz(1))
            .with_mode(Mode::_0)
            .with_min_async_transfer_size(THRESHOLD),
    )
    .unwrap()
    .with_sck(sclk)
    .with_miso(miso_signal)
    .with_dma(peripherals.DMA_SPI2);

    let mut spi = spi.with_buffers(
        dma_rx_buffer!(DMA_LEN).unwrap(),
        dma_tx_buffer!(DMA_LEN).unwrap(),
    );

    let dma_tx = [0x5au8; DMA_LEN];
    let mut dma_rx = [0u8; DMA_LEN];
    let mut cpu_rx = [0u8; CPU_LEN];

    let mut outcome = Outcome::Pass;

    // Each round runs the sequence at both MISO polarities, and always flips the
    // level between the DMA transfer and the CPU transfer. A CPU transfer that
    // is really reading the line has to follow the flip; one that is reading a
    // stale FIFO cannot.
    'outer: for round in 0..8 {
        for (dma_level, cpu_level) in [(Level::Low, Level::High), (Level::High, Level::Low)] {
            let dma_expected = if dma_level == Level::High { 0xff } else { 0x00 };
            let cpu_expected = if cpu_level == Level::High { 0xff } else { 0x00 };

            miso.set_level(dma_level);
            dma_rx.fill(0xaa);
            spi.transfer(&mut dma_rx, &dma_tx).unwrap();
            if !dma_rx.iter().all(|&b| b == dma_expected) {
                println!(
                    "round {}: DMA transfer read {:02x?}, expected {:02x} throughout",
                    round,
                    &dma_rx[..8],
                    dma_expected
                );
                outcome = Outcome::MisoNotSampled;
                break 'outer;
            }

            miso.set_level(cpu_level);
            cpu_rx.fill(0xaa);
            spi.transfer(&mut cpu_rx, &CPU_TX).unwrap();
            if !cpu_rx.iter().all(|&b| b == cpu_expected) {
                if cpu_rx == CPU_TX {
                    println!(
                        "round {}: CPU transfer echoed the TX pattern -- the RX data went into \
                         the stale DMA descriptor",
                        round
                    );
                } else {
                    println!(
                        "round {}: CPU transfer read {:02x?}, expected {:02x} throughout",
                        round, cpu_rx, cpu_expected
                    );
                }
                outcome = Outcome::StaleLink;
                break 'outer;
            }
        }
    }

    report(&mut led, delay, outcome);
}

fn report(led: &mut Output<'_>, delay: Delay, outcome: Outcome) -> ! {
    let (on, off, burst, gap) = match outcome {
        Outcome::Pass => {
            println!("PASS: CPU transfer works after a DMA transfer");
            (500u32, 500u32, 1, 0u32)
        }
        Outcome::StaleLink => {
            println!("FAIL: stale DMA link swallowed the CPU transfer");
            (50, 50, 1, 0)
        }
        Outcome::PinNotReadable => (100, 100, 2, 1000),
        Outcome::MisoNotSampled => (100, 100, 3, 1000),
    };

    loop {
        for _ in 0..burst {
            led.set_high();
            delay.delay_millis(on);
            led.set_low();
            delay.delay_millis(off);
        }
        if gap > 0 {
            delay.delay_millis(gap);
        }
    }
}
