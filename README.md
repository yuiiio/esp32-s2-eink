# Xtensa using esp patched llvm 

wr_mask_gpio_out, set_bit_gpio_out, clr_bit_gpio_out, get_gpio_in 

like esp32s2 specific instructions supports

xtensa-esp32-elf-objdump install palace is like

~/.rustup/toolchains/esp/xtensa-esp-elf/esp-13.2.0_20230928/xtensa-esp-elf/bin/xtensa-esp32-elf-objdump

# PSRAM
PSRAM fails to init when using espflash ver 1.7.0 :(

but espflash latest 3.0.1 fails connect to esp32s2 other reason... :(

espflash @2.0.1 works success connect and works psram correct :)

# SdCard (embedded-sdmmc-rs)
supports MBR FAT32 and **need [W95 FAT32] type(0x0c)**

can use convert-to-eink-image.sh to convert image

can check binary tif by
```
fn main() {
    let bytes = include_bytes!("output.tif");
    /*
    println!("{:?}", bytes);
    println!("{}", bytes.len());
    */
    let image_size = 1448*1072*4/8;
    //println!("image(w*h*4bit) is {} bytes", image_size);
    let idf_pointer: u32 =
        ((bytes[7] as u32) << 24) +
        ((bytes[6] as u32) << 16) +
        ((bytes[5] as u32) << 8)  +
        ((bytes[4] as u32) << 0);
    //println!("idf pointer is {}", idf_pointer);
    // 8 bytes to idf_pointer is image data
    //println!("image size is {}", idf_pointer - 8);

    let image_array = &bytes[8..idf_pointer as usize];
    assert_eq!(image_array.len(), image_size);
    println!("image array\n {:?}", image_array);
    println!("image_array size: {}", image_array.len());
}
```
