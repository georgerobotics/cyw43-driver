/**
 * Copyright (c) 2023 Beechwoods Software, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* TODO: Need to put in appropriate build conditionals
   to make the cyw43_led features exclusive to the Pico W */

#include <zephyr/device.h>
#include <zephyr/drivers/led.h>

#include "georgerobotics/cyw43.h"

#define DT_DRV_COMPAT infineon_cyw43_led

static int rpi_cyw43_led_on(const struct device *dev, uint32_t led)
{
        if (led == 0) return cyw43_gpio_set(&cyw43_state, 0, true);
        return -ENOTSUP;
}

static int rpi_cyw43_led_off(const struct device *dev, uint32_t led)
{
        if (led == 0) return cyw43_gpio_set(&cyw43_state, 0, false);
        return -ENOTSUP;
}

static int rpi_cyw43_led_init(const struct device *dev)
{
        return 0;
}

static const struct led_driver_api rpi_cyw43_led_api = {
        .on = rpi_cyw43_led_on,
        .off = rpi_cyw43_led_off,
};

DEVICE_DT_INST_DEFINE(0, rpi_cyw43_led_init, NULL, NULL,
                      NULL, POST_KERNEL, CONFIG_LED_INIT_PRIORITY,
                      &rpi_cyw43_led_api);

