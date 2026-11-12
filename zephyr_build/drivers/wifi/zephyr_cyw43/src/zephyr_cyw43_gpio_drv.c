/*
 * Copyright (c) 2023, Beechwoods Software, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>

#include <hardware/gpio.h>
#include <zephyr/logging/log.h>
#include "georgerobotics/cyw43.h"
#include "georgerobotics/cyw43_country.h"

#define DT_DRV_COMPAT infineon_cyw43_gpio

#define ALL_EVENTS (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE \
                    | GPIO_IRQ_LEVEL_LOW | GPIO_IRQ_LEVEL_HIGH)

struct gpio_rpi_cyw43_config {
        struct gpio_driver_config common;
};

static const struct gpio_rpi_cyw43_config _config = {
        .common = { .port_pin_mask = 0, }
};

struct gpio_rpi_cyw43_data {
        struct gpio_driver_config common;
};

static struct gpio_rpi_cyw43_data rpi_data = {
        .common = { .port_pin_mask = 0, }
};
static int gpio_rpi_cyw43_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
        printf("gpio_rpi_cyw43_configure pin=%d flags=%x\n", pin, flags);
        return 0;
}

static int gpio_rpi_cyw43_port_get_raw(const struct device *dev, uint32_t *value)
{
        /* printf("gpio_rpi_cyw43_port_get_raw *value=%d\n", *value); */
        int ret = 0;
        *value = 0;
        for (int pin = 0; pin < CYW43_WL_GPIO_COUNT; pin++) {
                bool val;
                ret = cyw43_gpio_get(&cyw43_state, pin, &val);
                if (ret != 0) {
                        printf("Error from cyw43_gpio_get on pin %d: %d\n", pin, ret);
                }
                if (val) {
                        *value |= (1<<pin);
                        // printf("*%d:%x value is %d\n", pin, (1<<pin), *value);
                }
        }
        return ret;
}

static int gpio_rpi_cyw43_port_set_masked_raw(const struct device *port, uint32_t mask, uint32_t value)
{
        int ret = 0;
        /* printf("gpio_rpi_cyw43_port_set_masked_raw mask=%x value=%d\n", mask, value); */
        for (int pin = 0; pin < NUM_BANK0_GPIOS; pin++) {
                if (pin < CYW43_WL_GPIO_COUNT) {
                        if (mask & (1<<pin)) {
                                ret = cyw43_gpio_set(&cyw43_state, pin, value);
                        }
                } else {
                        printf("pin %d is not supported in cyw43\n", pin);
                        return -ENOTSUP;
                }
        }
        return 0;
}

static int gpio_rpi_cyw43_port_set_bits_raw(const struct device *port, uint32_t pins)
{
        printf("gpio_rpi_cyw43_port_set_bits_raw pins=%x\n", pins);
        int ret = 0;
        for (int i = 0; i < NUM_BANK0_GPIOS; i++) {
                if (pins & (1<<i)) {
                        if (i < CYW43_WL_GPIO_COUNT) {
                                // printf("Setting pin %d to 1\n", i);
                                ret = cyw43_gpio_set(&cyw43_state, i, 1);
                        } else {
                                printf("pin %d is not supported in cyw43\n", i);
                                return -ENOTSUP;
                        }
                }
        }
        return ret;
}

static int gpio_rpi_cyw43_port_clear_bits_raw(const struct device *port, uint32_t pins)
{
        printf("gpio_rpi_cyw43_port_clear_bits_raw pins=%x\n", pins);
        int ret = 0;
        for (int i = 0; i < NUM_BANK0_GPIOS; i++) {
                if (pins & (1<<i)) {
                        if (i < CYW43_WL_GPIO_COUNT) {
                                printf("Clearing pin %d\n", i);
                                ret = cyw43_gpio_set(&cyw43_state, i, 0);
                        } else {
                                printf("pin %d is not supported in cyw43\n", i);
                                return -ENOTSUP;
                        }
                }
        }
        return ret;
}

static int gpio_rpi_cyw43_port_toggle_bits(const struct device *port, uint32_t pins)
{
        printf("gpio_rpi_cyw43_port_toggle_bits pins=%x\n", pins);
        return 0;
}

static int gpio_rpi_cyw43_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin, enum gpio_int_mode mode, enum gpio_int_trig trig)
{
        printf("gpio_rpi_cyw43_pin_interrupt_configure pin=%d mode=%d trig=%d\n", pin, mode, trig);
        return 0;
}

static int gpio_rpi_cyw43_manage_callback(const struct device *port, struct gpio_callback *cb, bool set)
{
        printf("gpio_rpi_cyw43_manage_callback\n");
        return 0;
}

static const struct gpio_driver_api cyw43_gpio_api = {
        .pin_configure = gpio_rpi_cyw43_configure,
        .port_get_raw = gpio_rpi_cyw43_port_get_raw,
        .port_set_masked_raw = gpio_rpi_cyw43_port_set_masked_raw,
        .port_set_bits_raw = gpio_rpi_cyw43_port_set_bits_raw,
        .port_clear_bits_raw = gpio_rpi_cyw43_port_clear_bits_raw,
        .port_toggle_bits = gpio_rpi_cyw43_port_toggle_bits,
        .pin_interrupt_configure = gpio_rpi_cyw43_pin_interrupt_configure,
        .manage_callback = gpio_rpi_cyw43_manage_callback,
};

static int cyw43_gpio_init(const struct device *dev)
{
        return 0;
}

DEVICE_DT_INST_DEFINE(0, cyw43_gpio_init, NULL, &rpi_data,
                      &_config, POST_KERNEL, CONFIG_LED_INIT_PRIORITY,
                      &cyw43_gpio_api);
