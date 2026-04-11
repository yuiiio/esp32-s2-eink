# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Embedded Rust firmware for an e-ink reader device using the ESP32-S2 microcontroller. Controls a 1448x1072 2BPP e-ink display with capacitive touch input and SD card storage.

## Build Commands

```bash
# Build and flash (requires espflash 2.1.0+ or 3.3.0+ with flags)
cargo run --release

# Build only
cargo build --release
```

The project uses the ESP Rust toolchain (`channel = "esp"`) with target `xtensa-esp32s2-none-elf`. Flash command uses `espflash flash --before no-reset --no-stub`.

## Hardware Requirements

- ESP32-S2 with PSRAM
- SD card: MBR FAT32 with partition type 0x0C (W95 FAT32)

## Image Conversion

Convert images to 2BPP TIFF format for the e-ink display:

```bash
# Single image
./next-convert.sh input.jpg 0    # 0 = rotation degrees

# Batch convert directory of images organized by chapter
./convert-files-and-mkdir.sh /path/to/jpgs rotation offset
```

Images must be TIFF, 1448x1072, 2-bit depth, uncompressed. Output goes to numbered directories (0001/, 0002/...) with numbered pages (000.tif, 001.tif...).

## Architecture

**src/main.rs** - Application entry point and peripheral initialization:
- PSRAM heap setup for ~800KB image buffers
- SPI-based SD card file system (embedded-sdmmc)
- ADC-based capacitive touch detection (4 zones: left, right, center, top)
- Navigation state machine for chapter/page browsing
- Flash storage for persistent state

**src/eink.rs** - E-ink display driver:
- `EinkDisplay` struct manages display control via dedicated GPIO
- LUT (Look-Up Table) waveform definitions for grayscale rendering
- Uses inline assembly (`wur.gpio_out`) for fast 8-bit parallel writes
- Key methods: `write_2bpp_image()`, `write_all()`, `write_top_indicator()`, `write_bottom_indicator()`

**src/touch.rs** - Capacitive touch input:
- `TouchInput` struct handles ADC-based touch sensing
- Pulse charging with configurable thresholds for 4 zones

**src/page_cache.rs** - PSRAM page cache:
- `PageCache` manages 5 page buffers (~1.9MB total) with LRU eviction
- Supports prefetching and chapter/title invalidation

## SD Card File Structure

```
/sdcard/BOOKS/
  TITLE1/           # Title directories (8.3 format)
    0001/           # Chapter directories (0001-9999)
      000.tif       # Page files (000-999)
      001.tif
    0002/
      ...
  TITLE2/
    ...
```

## Key Dependencies

- `esp-hal` - ESP32 HAL with PSRAM support (git dependency)
- `embedded-sdmmc` - FAT32 SD card driver
- `esp-storage` - Internal flash for state persistence
