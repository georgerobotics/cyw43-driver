CYW43xx WiFi/BT SoC driver
==========================

This is a driver for the CYW43xx WiFi/BT SoC.

There are four layers to the driver:

1. SDIO or SPI bus interface, provided by the host device/system.  The header
   files `cyw43_sdio.h` and `cyw43_spi.h` define this interface and what the
   host system needs to implement.

2. Low-level CYW43xx interface, managing the bus, control messages, Ethernet
   frames and asynchronous events.  Includes download of SoC firmware.  The
   header file `cyw43_ll.h` defines the interface to this layer.

3. Mid-level CYW43xx control, to control and set WiFi parameters and manage
   events.  See `cyw43_ctrl.c` and the header `cyw43.h`.

4. TCP/IP bindings to lwIP.  See `cyw43_lwip.c` and the header `cyw43.h`.

Configuration of the driver is managed through `cyw43_config.h`.
