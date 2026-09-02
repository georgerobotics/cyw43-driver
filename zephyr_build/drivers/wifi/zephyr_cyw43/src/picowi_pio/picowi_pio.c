// PicoWi PIO interface driver, see https://iosoft.blog/picowi
//
// Copyright (c) 2022, Jeremy P Bentham
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <stdio.h>

#include <stdlib.h>
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "picowi_pico.h"
#include "picowi_pio.h"

#include "picowi_pio.pio.h"

static PIO wifi_pio = pio0;
static uint wifi_sm;
static io_rw_8 *wifi_txfifo;

void pio_init(void) 
{
  printf("pio_init()\n");
    wifi_sm = pio_claim_unused_sm(wifi_pio, true);
    wifi_txfifo = (io_rw_8 *) &wifi_pio->txf[0];
    uint offset = pio_add_program(wifi_pio, &picowi_pio_program);
    pio_sm_config c = picowi_pio_program_get_default_config(offset);
    // Set I/O pins to be controlled
    pio_gpio_init(wifi_pio, SD_CLK_PIN);
    pio_gpio_init(wifi_pio, SD_CMD_PIN);
    pio_sm_set_consecutive_pindirs(wifi_pio, wifi_sm, SD_CLK_PIN, 1, true);
    pio_sm_set_consecutive_pindirs(wifi_pio, wifi_sm, SD_CMD_PIN, 1, true);
    // Configure data pin as I/O, clock pin as O/P (sideset)
    sm_config_set_out_pins(&c, SD_CMD_PIN, 1);
    sm_config_set_in_pins(&c, SD_CMD_PIN);
    sm_config_set_sideset_pins(&c, SD_CLK_PIN);
    // Get 8 bits from FIFOs, disable auto-pull & auto-push
    sm_config_set_out_shift(&c, false, false, 8);
    sm_config_set_in_shift(&c, false, false, 8);
    // Set data rate
#if PIO_SPI_FREQ >= 40000000
    sm_config_set_clkdiv_int_frac(&c, 1, 0);
#else   
    float div = (float)clock_get_hz(clk_sys) / (PIO_SPI_FREQ * 3.1);
    sm_config_set_clkdiv(&c, div);
#endif    
    pio_sm_init(wifi_pio, wifi_sm, offset, &c);
    pio_sm_clear_fifos(wifi_pio, wifi_sm);
    pio_sm_set_enabled(wifi_pio, wifi_sm, true);
}

// Read data block from SPI interface
void pio_spi_read(uint8_t *dp, int nbits)
{
    int rxlen = nbits / 8;
    int reader = PIO_SPI_FREQ >= 30000000 ? picowi_pio_offset_reader : picowi_pio_offset_slow_reader;
    pio_sm_exec(wifi_pio, wifi_sm, pio_encode_jmp(reader));
    pio_sm_put(wifi_pio, wifi_sm, rxlen - 1);
    while (rxlen > 0)
    {
        if (!pio_sm_is_rx_fifo_empty(wifi_pio, wifi_sm))
        {
            *dp++ = pio_sm_get(wifi_pio, wifi_sm);
            rxlen--;
        }
    }
}

// Write data block to SPI interface
void pio_spi_write(uint8_t *dp, int nbits)
{
    pio_sm_clear_fifos(wifi_pio, wifi_sm);
    pio_sm_exec(wifi_pio, wifi_sm, pio_encode_jmp(picowi_pio_offset_writer));
    pio_sm_set_consecutive_pindirs(wifi_pio, wifi_sm, SD_CMD_PIN, 1, true);
    int n = 0;
    while (n < nbits)
    {
        if (!pio_sm_is_tx_fifo_full(wifi_pio, wifi_sm))
        {
            *wifi_txfifo = *dp++;
            n += 8;
        }
    }
    while (!pio_sm_is_tx_fifo_empty(wifi_pio, wifi_sm)) ;
    while (wifi_pio->sm[wifi_sm].addr != picowi_pio_offset_writer) ;
    pio_sm_set_consecutive_pindirs(wifi_pio, wifi_sm, SD_CMD_PIN, 1, false);
    pio_sm_exec(wifi_pio, wifi_sm, pio_encode_jmp(picowi_pio_offset_stall));
}

// EOf
