CYW43xx WiFi SoC firmware
=========================

This directory contains firmware patch blobs that need to be downloaded on to the
CYW43xx SoC in order for it to function correctly.

The binary blobs are converted to a header file, for example:

    xxd -i cyfmac43439-sdio.bin

The macros `cyw43_chipset_firmware_blob` and `cyw43_chipset_clm_blob` are then
defined to the firmware and CLM blob arrays respectively, and then used externally.

The Bluetooth firmware binary for the 43439 (eg found on the Raspberry Pi Pico W)
is available as a static array in `cyw43_btfw_43439.h` and has the following format:

    1 byte: number of characters in version string including null terminator
    n bytes: zero terminated version string
    1 byte: number of records following

    Each record then has the following format:
        1 byte: data count
        2 bytes: address
        1 byte: address type
        n bytes: data

The Bluetooth firmware binary for the 4343A1 (eg found in the Murata 1DX) is
available as a static array in `cyw43_btfw_4343A1.h`.
