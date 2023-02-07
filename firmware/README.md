CYW43xx WiFi SoC firmware
=========================

This directory contains firmware patch blobs that need to be downloaded on to the
CYW43xx SoC in order for it to function correctly.

The WiFi firmware is padded to 512 bytes and then the CLM appended to that to create the combined binary file.

For example:

    $ cp 43439A0.bin 43439A0_padded.bin
    $ dd if=/dev/zero of=43439A0_padded.bin bs=1 count=1 seek=$(( ($(stat -c %s 43439A0.bin) / 512) * 512 + 512 - 1))
    $ cat 43439A0_padded.bin 43439A0.clm_blob > 43439A0-7.95.49.00.combined

This binary is then converted to a header file, e.g. xxd -i 43439A0-7.95.49.00.combined
The macros `CYW43_WIFI_FW_LEN`, `CYW43_CLM_LEN` specify the unpadded size of the original firmware binaries in bytes.

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
