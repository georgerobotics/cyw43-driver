/*
 * This file is part of the cyw43-driver
 *
 * Copyright (C) 2019-2026 George Robotics Pty Ltd
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * \file
 * \brief CYW43 SDIO API
*/

#ifndef CYW43_INCLUDED_CYW43_SDIO_H
#define CYW43_INCLUDED_CYW43_SDIO_H

// These must be provided by a port, if the SDIO bus interface is used.
void cyw43_sdio_init(void);
void cyw43_sdio_reinit(void);
void cyw43_sdio_deinit(void);
void cyw43_sdio_set_irq(bool enable);
void cyw43_sdio_enable_high_speed_4bit(void);
int cyw43_sdio_transfer(uint32_t cmd, uint32_t arg, uint32_t *resp);
int cyw43_sdio_transfer_cmd53(bool write, uint32_t block_size, uint32_t arg, size_t len, uint8_t *buf);

#endif // CYW43_INCLUDED_CYW43_SDIO_H
