/*
 * This file is part of the cyw43-driver
 *
 * Copyright (C) 2019-2026 George Robotics Pty Ltd
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * \file
 * \brief CYW43 Bluetooth BUS API
*/

#ifndef CYW43_INCLUDED_CYW43_BTBUS_H
#define CYW43_INCLUDED_CYW43_BTBUS_H

#include "cyw43_ll.h"

int cyw43_btbus_init(cyw43_ll_t *self);
int cyw43_btbus_read(uint8_t *buf, uint32_t max_buf_size, uint32_t *size);
int cyw43_btbus_write(uint8_t *buf, uint32_t size);

#endif
