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

#ifndef CYW43_INCLUDED_CYW43_DEBUG_PINS_H
#define CYW43_INCLUDED_CYW43_DEBUG_PINS_H

#ifdef PICO_BUILD
#include "hardware/gpio.h"
#else
#undef CYW43_LOGIC_DEBUG
#endif

static inline void logic_debug_init(void) {
    #if CYW43_LOGIC_DEBUG
    const int dbg_start = 2;
    const int dbg_end = 14;
    for (int pin = dbg_start; pin <= dbg_end; pin++) {
        gpio_init(pin);
        gpio_set_dir(pin, 1);
        gpio_put(pin, 0);
    }
    #endif
}

static inline void logic_debug_set(uint8_t pin, uint8_t value) {
    #if CYW43_LOGIC_DEBUG
    gpio_put(pin, value);
    #else
    (void)pin;
    (void)value;
    #endif
}

#define pin_BACKPLANE_WRITE 2
#define pin_BACKPLANE_READ 3
#define pin_TX_PKT 4
#define pin_RX_PKT 5
#define pin_F2_RX_READY_WAIT 6
#define pin_WIFI_TX 7
#define pin_WIFI_RX 8
#define pin_F1_NOT_READY 9
#define pin_F1_OVERFLOW 10

#endif // CYW43_INCLUDED_CYW43_DEBUG_PINS_H
