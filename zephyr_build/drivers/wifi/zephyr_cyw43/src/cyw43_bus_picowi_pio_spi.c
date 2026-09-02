/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2023 Beechwoods Software, Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define CYW43_USE_DMA 0

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "cyw43.h"
#include "cyw43_internal.h"
#include "cyw43_spi.h"
#include "cyw43_debug_pins.h"
#include "cyw43_config.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include "hardware/gpio.h"
#include "picowi_pio/picowi_pio.h"
#include "picowi_pio/picowi_pico.h"

#define WHD_BUS_SPI_BACKPLANE_READ_PADD_SIZE        (4)

#if CYW43_SPI_PIO
#define WL_REG_ON CYW43_PIN_WL_REG_ON
#define DATA_OUT_PIN CYW43_PIN_WL_CMD
#define DATA_IN_PIN CYW43_PIN_WL_CMD
#define IRQ_PIN CYW43_PIN_WL_CMD

#define CLOCK_PIN CYW43_PIN_WL_CLOCK
#define CS_PIN CYW43_PIN_WL_CS

#define IRQ_SAMPLE_DELAY_NS 100

#define CLOCK_DIV 2
#define CLOCK_DIV_MINOR 0
#define PADS_DRIVE_STRENGTH PADS_BANK0_GPIO0_DRIVE_VALUE_12MA

#if !CYW43_USE_SPI
#error CYW43_USE_SPI should be true
#endif

/* Set to ENABLE_SPI_DUMPING 1 in cyw43_configport.h to enable */
#if ENABLE_SPI_DUMPING //NDEBUG
bool enable_spi_packet_dumping=false; /* set to true to dump */
#define DUMP_SPI_TRANSACTIONS(A) if (enable_spi_packet_dumping) {A}
#else
#define DUMP_SPI_TRANSACTIONS(A)
#endif

bool do_ridiculous_byte_reordering=true;

#undef SWAP32
__force_inline static uint32_t __swap16x2(uint32_t a) {
        __asm ("rev16 %0, %0" : "+l" (a) : : );
        return a;
}
#define SWAP32(a) __swap16x2(a)

#if ENABLE_SPI_DUMPING
static void dump_bytes(const uint8_t *bptr, uint32_t len) {
        unsigned int i = 0;

        for (i = 0; i < len;) {
                if ((i & 0x07) == 0) {
                        printf(" ");
                }
                printf("%02x ", bptr[i++]);
        }
}
#endif

void picowi_gpio_setup() {
        printf("picowi_gpio_setup()\n");
        io_set(SD_ON_PIN, IO_OUT, IO_NOPULL);
        io_out(SD_ON_PIN, 0);
        io_set(SD_CS_PIN, IO_OUT, IO_NOPULL);
        io_out(SD_CS_PIN, 1);
        io_set(SD_CLK_PIN, IO_OUT, IO_NOPULL);
        io_out(SD_CLK_PIN, 0);
        io_set(SD_CMD_PIN, IO_OUT, IO_NOPULL);
        io_out(SD_CMD_PIN, 0);
        k_busy_wait(100000);
        io_out(SD_ON_PIN, 1);
        k_busy_wait(50000);
        io_set(SD_CMD_PIN, IO_IN, IO_PULLUP);
        io_set(SD_IRQ_PIN, IO_IN, IO_NOPULL);
}

int cyw43_spi_init(cyw43_int_t *self) {

        printf("\n\nRunning with the picowi PIO-SPI low level interface\n\n\n");

        /* Only does something if CYW43_LOGIC_DEBUG=1 */
        logic_debug_init();

        gpio_set_input_hysteresis_enabled(SD_DIN_PIN, true);

        k_busy_wait(2000);

        picowi_gpio_setup();

        pio_init();

        assert(!self->bus_data);

        return 0;
}

void cyw43_spi_deinit(cyw43_int_t *self) {
        return;
#if 0
        if (self->bus_data) {
                bus_data_t *bus_data = (bus_data_t *)self->bus_data;
                if (bus_data->pio_sm >= 0) {
                        if (bus_data->pio_offset != -1)
                                pio_remove_program(bus_data->pio, &SPI_PROGRAM_FUNC, bus_data->pio_offset);
                        pio_sm_unclaim(bus_data->pio, bus_data->pio_sm);
                }
                if (bus_data->dma_out >= 0) {
                        dma_channel_cleanup(bus_data->dma_out);
                        dma_channel_unclaim(bus_data->dma_out);
                        bus_data->dma_out = -1;
                }
                if (bus_data->dma_in >= 0) {
                        dma_channel_cleanup(bus_data->dma_in);
                        dma_channel_unclaim(bus_data->dma_in);
                        bus_data->dma_in = -1;
                }
                self->bus_data = NULL;
        }
#endif
}

int cyw43_spi_transfer(cyw43_int_t *self, const uint8_t *tx, size_t tx_length, uint8_t *rx,
                       size_t rx_length) {

        bool tx_only = false;

        if ((tx == NULL) && (rx == NULL)) {
                return CYW43_FAIL_FAST_CHECK(-CYW43_EINVAL);
        }

        if (rx != NULL) {
                if (tx == NULL) {
                        tx = rx;
                        assert(tx_length && tx_length < rx_length);
                }
                else {
                        tx_only=true;
                }
        }

        if (rx == NULL) {
                tx_only=true;
        }

        io_out(CS_PIN, 0);

        pio_spi_write((unsigned char *)tx, (int)(tx_length * 8));

        DUMP_SPI_TRANSACTIONS(
                printf("TXed:");
                dump_bytes(tx, tx_length);
                printf("\n");
        )

        if (!tx_only) {
                memset(rx, 0, rx_length);

                if (do_ridiculous_byte_reordering) {
                        int i;
                        for (i=0; i<rx_length; i+=8) {
                                pio_spi_read((unsigned char *)(rx + i + 4), 32);
                                pio_spi_read((unsigned char *)(rx + i), 32);
                        }
                        if (i > rx_length) {
                                int bits_remaining = i - rx_length;
                                pio_spi_read((unsigned char *)(rx + i - 8), bits_remaining *8 );
                        }
                }
                else  {
                        /* pad with 4 bytes of zeros, because why not? ;) */
                        pio_spi_read((unsigned char *)(rx + WHD_BUS_SPI_BACKPLANE_READ_PADD_SIZE), (int)(rx_length *8));
                }

                DUMP_SPI_TRANSACTIONS(
                        printf("RXed:");
                        dump_bytes(rx, rx_length);
                        printf("\n");
                )
        }

        io_out(SD_CS_PIN, 1);

        return 0;
}

// Initialise our gpios
void cyw43_spi_gpio_setup(void) {
        return;
#if 0
        printf("cyw43_spi_gpio_setup()\n");

        /* Setup WL_REG_ON (23) */
        gpio_init(WL_REG_ON);
        gpio_set_dir(WL_REG_ON, GPIO_OUT);
        gpio_pull_up(WL_REG_ON);

        /* Setup CS (25) */
        gpio_init(CS_PIN);
        gpio_set_dir(CS_PIN, GPIO_OUT);
        gpio_put(CS_PIN, true);

        /* Setup DO, DI and IRQ (24) */
        gpio_init(DATA_OUT_PIN);
        gpio_set_dir(DATA_OUT_PIN, GPIO_OUT);
        gpio_put(DATA_OUT_PIN, false);
#endif
}

/* Reset wifi chip */
void cyw43_spi_reset(void) {
        return;
#if 0
        printf("cyw43_spi_reset()\n");
        gpio_put(WL_REG_ON, false); // off
        k_busy_wait(20000);
        gpio_put(WL_REG_ON, true); // on
        k_busy_wait(250000);

        // Setup IRQ (24) - also used for DO, DI
        gpio_init(IRQ_PIN);
        gpio_set_dir(IRQ_PIN, GPIO_IN);
#endif
}

static inline uint32_t make_cmd(bool write, bool inc, uint32_t fn, uint32_t addr, uint32_t sz) {
    return write << 31 | inc << 30 | fn << 28 | (addr & 0x1ffff) << 11 | sz;
}

#if CYW43_VERBOSE_DEBUG
static const char *func_name(int fn) {
        switch (fn)
        {
        case BUS_FUNCTION:
                return "BUS_FUNCTION";
        case BACKPLANE_FUNCTION:
                return "BACKPLANE_FUNCTION";
        case WLAN_FUNCTION:
                return "WLAN_FUNCTION";
        default:
                return "UNKNOWN";
        }
}
#endif

uint32_t read_reg_u32_swap(cyw43_int_t *self, uint32_t fn, uint32_t reg) {
        uint32_t buf[2] = {0};
        assert(fn != BACKPLANE_FUNCTION);
        buf[0] = SWAP32(make_cmd(false, true, fn, reg, 4));
        int ret = cyw43_spi_transfer(self, NULL, 4, (uint8_t *)buf, 8);
        if (ret != 0) {
                return ret;
        }
        return SWAP32(buf[1]);
}

static inline uint32_t _cyw43_read_reg(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint size) {
        /* Padding plus max read size of 32 bits + another 4? */
        static_assert(CYW43_BACKPLANE_READ_PAD_LEN_BYTES % 4 == 0, "");
        int index = (CYW43_BACKPLANE_READ_PAD_LEN_BYTES / 4) + 1 + 1;
        uint32_t buf32[index];
        uint8_t *buf = (uint8_t *)buf32;
        const uint32_t padding = (fn == BACKPLANE_FUNCTION) ? CYW43_BACKPLANE_READ_PAD_LEN_BYTES : 0; // Add response delay
        buf32[0] = make_cmd(false, true, fn, reg, size);

        if (fn == BACKPLANE_FUNCTION) {
                logic_debug_set(pin_BACKPLANE_READ, 1);
        }
        int ret = cyw43_spi_transfer(self, NULL, 4, buf, 8 + padding);
        if (fn == BACKPLANE_FUNCTION) {
                logic_debug_set(pin_BACKPLANE_READ, 0);
        }

        if (ret != 0) {
                return ret;
        }
        uint32_t result = buf32[padding > 0 ? index - 1 : 1];
        CYW43_VDEBUG("cyw43_read_reg_u%d %s 0x%lx=0x%lx\n", size * 8,
                     func_name(fn), (unsigned long)reg, (unsigned long)result);
        return result;
}

uint32_t cyw43_read_reg_u32(cyw43_int_t *self, uint32_t fn, uint32_t reg) {
        return _cyw43_read_reg(self, fn, reg, 4);
}

int cyw43_read_reg_u16(cyw43_int_t *self, uint32_t fn, uint32_t reg) {
        return _cyw43_read_reg(self, fn, reg, 2);
}

int cyw43_read_reg_u8(cyw43_int_t *self, uint32_t fn, uint32_t reg) {
        return _cyw43_read_reg(self, fn, reg, 1);
}

/* This is only used to switch the word order on boot */
int write_reg_u32_swap(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint32_t val) {
        uint32_t buf[2];
        /* Boots up in little endian so command needs swapping too */
        buf[0] = SWAP32(make_cmd(true, true, fn, reg, 4));
        buf[1] = SWAP32(val);
        int ret = cyw43_spi_transfer(self, (uint8_t *)buf, 8, NULL, 0);
        CYW43_VDEBUG("write_reg_u32_swap %s 0x%lx=0x%lx\n",
                     func_name(fn), (unsigned long)reg, (unsigned long)val);
        return ret;
}

static inline int _cyw43_write_reg(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint32_t val, uint size) {
        uint32_t buf[2];
        buf[0] = make_cmd(true, true, fn, reg, size);
        buf[1] = val;
        if (fn == BACKPLANE_FUNCTION) {
                /* In case of f1 overflow */
                self->last_size = 8;
                self->last_header[0] = buf[0];
                self->last_header[1] = buf[1];
                self->last_backplane_window = self->cur_backplane_window;
        }

        if (fn == BACKPLANE_FUNCTION) {
                logic_debug_set(pin_BACKPLANE_WRITE, 1);
        }

        int ret = cyw43_spi_transfer(self, (uint8_t *)buf, 8, NULL, 0);

        if (fn == BACKPLANE_FUNCTION) {
                logic_debug_set(pin_BACKPLANE_WRITE, 0);
        }

        CYW43_VDEBUG("cyw43_write_reg_u%d %s 0x%lx=0x%lx\n",size * 8,
                     func_name(fn), (unsigned long)reg, (unsigned long)val);
        return ret;
}

int cyw43_write_reg_u32(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint32_t val) {
        return _cyw43_write_reg(self, fn, reg, val, 4);
}

int cyw43_write_reg_u16(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint16_t val) {
        return _cyw43_write_reg(self, fn, reg, val, 2);
}

int cyw43_write_reg_u8(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint32_t val) {
        return _cyw43_write_reg(self, fn, reg, val, 1);
}

#if CYW43_BUS_MAX_BLOCK_SIZE > 0x7f8
#error Block size is wrong for SPI
#endif

int cyw43_read_bytes(cyw43_int_t *self, uint32_t fn, uint32_t addr, size_t len, uint8_t *buf) {

        /* this flag is initialized to true, but should remain false starting with the first call 
           to this function. */
        do_ridiculous_byte_reordering=false;

        assert(fn != BACKPLANE_FUNCTION || (len <= CYW43_BUS_MAX_BLOCK_SIZE));
        const uint32_t padding = (fn == BACKPLANE_FUNCTION) ? CYW43_BACKPLANE_READ_PAD_LEN_BYTES : 0; // Add response delay
        size_t aligned_len = (len + 3) & ~3;
        assert(aligned_len > 0 && aligned_len <= 0x7f8);
        assert(buf == self->spid_buf || buf < self->spid_buf || buf >= (self->spid_buf + sizeof(self->spid_buf)));
        self->spi_header[padding > 0 ? 0 : (CYW43_BACKPLANE_READ_PAD_LEN_BYTES / 4)] = make_cmd(false, true, fn, addr, len);
        if (fn == WLAN_FUNCTION) {
                logic_debug_set(pin_WIFI_RX, 1);
        }
        int ret = cyw43_spi_transfer(self, NULL, 4, (uint8_t *)&self->spi_header[padding > 0 ? 0 : (CYW43_BACKPLANE_READ_PAD_LEN_BYTES / 4)], aligned_len + 4 + padding);
        if (fn == WLAN_FUNCTION) {
                logic_debug_set(pin_WIFI_RX, 0);
        }
        if (ret != 0) {
                printf("cyw43_read_bytes error %d", ret);
                return ret;
        }
        if (buf != self->spid_buf) { // avoid a copy in the usual case just to add the header
                memcpy(buf, self->spid_buf, len);
        }
        return 0;
}

/* See whd_bus_spi_transfer_bytes
   Note, uses spid_buf if src isn't using it already
   Apart from firmware download this appears to only be used for wlan functions?
*/
int cyw43_write_bytes(cyw43_int_t *self, uint32_t fn, uint32_t addr, size_t len, const uint8_t *src) {
        assert(fn != BACKPLANE_FUNCTION || (len <= CYW43_BUS_MAX_BLOCK_SIZE));
        const size_t aligned_len = (len + 3) & ~3u;
        assert(aligned_len > 0 && aligned_len <= 0x7f8);
        if (fn == WLAN_FUNCTION) {
                /* Wait for FIFO to be ready to accept data */
                int f2_ready_attempts = 1000;
                while (f2_ready_attempts-- > 0) {
                        uint32_t bus_status = cyw43_read_reg_u32(self, BUS_FUNCTION, SPI_STATUS_REGISTER);
                        if (bus_status & STATUS_F2_RX_READY) {
                                logic_debug_set(pin_F2_RX_READY_WAIT, 0);
                                break;
                        } else {
                                logic_debug_set(pin_F2_RX_READY_WAIT, 1);
                        }
                }
                if (f2_ready_attempts <= 0) {
                        printf("F2 not ready\n");
                        return CYW43_FAIL_FAST_CHECK(-CYW43_EIO);
                }
        }
        if (src == self->spid_buf) { // avoid a copy in the usual case just to add the header
                self->spi_header[(CYW43_BACKPLANE_READ_PAD_LEN_BYTES / 4)] = make_cmd(true, true, fn, addr, len);
                logic_debug_set(pin_WIFI_TX, 1);
                int res = cyw43_spi_transfer(self, (uint8_t *)&self->spi_header[(CYW43_BACKPLANE_READ_PAD_LEN_BYTES / 4)], aligned_len + 4, NULL, 0);	
                logic_debug_set(pin_WIFI_TX, 0);
                return res;
        } else {
                /* todo: would be nice to get rid of this. Only used for firmware download? */
                assert(src < self->spid_buf || src >= (self->spid_buf + sizeof(self->spid_buf)));
                self->spi_header[(CYW43_BACKPLANE_READ_PAD_LEN_BYTES / 4)] = make_cmd(true, true, fn, addr, len);	
                memcpy(self->spid_buf, src, len);
                return cyw43_spi_transfer(self, (uint8_t *)&self->spi_header[(CYW43_BACKPLANE_READ_PAD_LEN_BYTES / 4)], aligned_len + 4, NULL, 0);
        }
}
#endif
