/*
 * This file is part of the cyw43-driver
 *
 * Copyright (C) 2019-2026 George Robotics Pty Ltd
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * \internal
 * \file
*/

#ifndef CYW43_INCLUDED_CYW43_INTERNAL_H
#define CYW43_INCLUDED_CYW43_INTERNAL_H

#define BUS_FUNCTION (0)
#define BACKPLANE_FUNCTION (1)
#define WLAN_FUNCTION (2)

typedef struct _cyw43_int_t {
    void *cb_data;

    uint32_t startup_t0;
    uint32_t cur_backplane_window;
    uint8_t wwd_sdpcm_packet_transmit_sequence_number;
    uint8_t wwd_sdpcm_last_bus_data_credit;
    uint8_t wlan_flow_control;
    uint16_t wwd_sdpcm_requested_ioctl_id;
    bool bus_is_up;
    bool had_successful_packet;
    #if CYW43_BACKPLANE_READ_PAD_LEN_BYTES > 0
    uint32_t spi_header[(CYW43_BACKPLANE_READ_PAD_LEN_BYTES / 4) + 1] __attribute__((aligned(4))); // Must be before spid_buf
    #endif
    uint8_t spid_buf[2048];

    uint8_t last_ssid_joined[36];
    // private info for the bus implementation
    void *bus_data;

    #if CYW43_INCLUDE_LEGACY_F1_OVERFLOW_WORKAROUND_VARIABLES
    // These variables are no longer needed (they were never used in any production
    // code), but a port can enable them if it still expects them to be there.
    uint32_t last_header[2];
    size_t last_size;
    uint32_t last_backplane_window;
    #endif
} cyw43_int_t;

static_assert(sizeof(cyw43_int_t) == sizeof(cyw43_ll_t), "");

// Read/write a number of bytes.
// These return 0 on success, <0 errno code on error.
int cyw43_read_bytes(cyw43_int_t *self, uint32_t fn, uint32_t addr, size_t len, uint8_t *buf);
int cyw43_write_bytes(cyw43_int_t *self, uint32_t fn, uint32_t addr, size_t len, const uint8_t *buf);

// Read a single register.
// These return 0 on success, <0 errno code on error.
// TODO: cyw43_read_reg_u32 cannot return <0 on error with 32-bit return type.
int cyw43_read_reg_u8(cyw43_int_t *self, uint32_t fn, uint32_t reg);
int cyw43_read_reg_u16(cyw43_int_t *self, uint32_t fn, uint32_t reg);
uint32_t cyw43_read_reg_u32(cyw43_int_t *self, uint32_t fn, uint32_t reg);

// Write a single register.
// These return 0 on success, <0 errno code on error.
int cyw43_write_reg_u8(cyw43_int_t *self, uint32_t function, uint32_t reg, uint32_t val);
int cyw43_write_reg_u16(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint16_t val);
int cyw43_write_reg_u32(cyw43_int_t *self, uint32_t function, uint32_t reg, uint32_t val);

#endif // CYW43_INCLUDED_CYW43_INTERNAL_H
