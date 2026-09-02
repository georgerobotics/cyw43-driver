/*
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2023 Beechwoods Software, Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* This header is included by cyw43_driver to setup its environment */
#include <stdbool.h>

#ifndef _CYW43_CONFIGPORT_H
#define _CYW43_CONFIGPORT_H

#undef NDEBUG
#define CYW43_VERBOSE_DEBUG 0

#if CYW43_VERBOSE_DEBUG
  #define CYW43_VDEBUG(...)  CYW43_PRINTF(__VA_ARGS__)
#else
  #define CYW43_VDEBUG(...)
#endif

#define CYW43_DEBUG(...)  CYW43_PRINTF(__VA_ARGS__)

/* To enable SPI trace, set ENABLE_SPI_DUMPING to 1
   _and_ then set it to true at points in the code where you
   want to enable it and false where you want to disable it. */
#define ENABLE_SPI_DUMPING 0
#if ENABLE_SPI_DUMPING
extern bool enable_spi_packet_dumping;
#endif

/* This seems to be necessary to avoid some compliation issues */
#define static_assert(expr, msg...) BUILD_ASSERT((expr), "" msg)
#define count_of(a) (sizeof(a)/sizeof((a)[0]))

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <time.h>
#include <assert.h>

static const struct device *rp2040_gpio = DEVICE_DT_GET(DT_NODELABEL(gpio0));

#ifdef __cplusplus
extern "C" {
#endif

#define CYW43_HOST_NAME "PicoW"

#define CYW43_LWIP (0)

#define CYW43_GPIO 1

#define CYW43_LOGIC_DEBUG 0

#define CYW43_USE_OTP_MAC 1

#define CYW43_NO_NETUTILS 1

#define CYW43_IOCTL_TIMEOUT_US 10000000

#define CYW43_USE_STATS 0

/* todo should this be user settable? */
#define CYW43_HAL_MAC_WLAN0 0

#define STATIC static

#define CYW43_USE_SPI 1

#define CYW43_SPI_PIO 1

#ifndef CYW43_CHIPSET_FIRMWARE_INCLUDE_FILE
#if CYW43_ENABLE_BLUETOOTH
#define CYW43_CHIPSET_FIRMWARE_INCLUDE_FILE "wb43439A0_7_95_49_00_combined.h"
#else
#ifdef CONFIG_WIFI_CYW43_USE_CYW4343W
#define CYW43_CHIPSET_FIRMWARE_INCLUDE_FILE "w4343WA1_7_45_98_50_combined.h"
#else
#define CYW43_CHIPSET_FIRMWARE_INCLUDE_FILE "w43439A0_7_95_49_00_combined.h"
#endif
#endif
#endif

#ifndef CYW43_WIFI_NVRAM_INCLUDE_FILE
#ifdef CONFIG_WIFI_CYW43_USE_CYW4343W
#define CYW43_WIFI_NVRAM_INCLUDE_FILE "wifi_nvram_1dx.h"
#else
#define CYW43_WIFI_NVRAM_INCLUDE_FILE "wifi_nvram_43439.h"
#endif
#endif

/* Note, these are negated, because cyw43_driver negates them before returning! */
#define CYW43_EPERM            (-EPERM) /* Operation not permitted */
#define CYW43_EIO              (-EIO) /* I/O error */
#define CYW43_EINVAL           (-EINVAL) /* Invalid argument */
#define CYW43_ETIMEDOUT        (-ETIMEDOUT) /* Connection timed out */

#define CYW43_NUM_GPIOS        CYW43_WL_GPIO_COUNT

#define cyw43_hal_pin_obj_t unsigned int

/* get the number of elements in a fixed-size array */
#define CYW43_ARRAY_SIZE(a) count_of(a)

static inline uint32_t cyw43_hal_ticks_us(void) {
  return (uint32_t) (1000 * k_uptime_get_32());
}

static inline uint32_t cyw43_hal_ticks_ms(void) {
    return k_uptime_get_32();
}

static inline int cyw43_hal_pin_read(cyw43_hal_pin_obj_t pin) {
  return gpio_pin_get(rp2040_gpio, pin);
}

static inline void cyw43_hal_pin_low(cyw43_hal_pin_obj_t pin) {
  gpio_port_clear_bits(rp2040_gpio, (1 << pin));
}

static inline void cyw43_hal_pin_high(cyw43_hal_pin_obj_t pin) {
  gpio_port_set_bits(rp2040_gpio, (1 << pin));
}

#define CYW43_HAL_PIN_MODE_INPUT           (GPIO_INPUT)
#define CYW43_HAL_PIN_MODE_OUTPUT          (GPIO_OUTPUT)

#define CYW43_HAL_PIN_PULL_NONE            (0)
#define CYW43_HAL_PIN_PULL_UP              (1)
#define CYW43_HAL_PIN_PULL_DOWN            (2)

#define CYW43_PIN_WL_REG_ON 23u
#define CYW43_PIN_WL_CMD 24u
#define CYW43_PIN_WL_CLOCK 29u
#define CYW43_PIN_WL_CS 25u
#define CYW43_PIN_WL_HOST_WAKE CYW43_PIN_WL_CMD

#define CYW43_WL_GPIO_COUNT 3

#define __uint16_t uint16_t

static inline void cyw43_hal_pin_config(cyw43_hal_pin_obj_t pin, uint32_t mode, uint32_t pull, __unused uint32_t alt) {
    assert((mode == CYW43_HAL_PIN_MODE_INPUT || mode == CYW43_HAL_PIN_MODE_OUTPUT) && alt == 0);
    assert(pull == CYW43_HAL_PIN_PULL_UP || pull == CYW43_HAL_PIN_PULL_DOWN || pull == CYW43_HAL_PIN_PULL_NONE);
    gpio_pin_configure(rp2040_gpio, pin, mode | (pull==CYW43_HAL_PIN_PULL_UP ? GPIO_PULL_UP : GPIO_PULL_DOWN));
}

void cyw43_hal_get_mac(int idx, uint8_t buf[6]);

void cyw43_hal_generate_laa_mac(int idx, uint8_t buf[6]);


void cyw43_thread_enter(void);

void cyw43_thread_exit(void);

#define CYW43_THREAD_ENTER cyw43_thread_enter();
#define CYW43_THREAD_EXIT cyw43_thread_exit();

#define CYW43_THREAD_LOCK_CHECK

#define CYW43_EVENT_POLL_HOOK k_yield();

void cyw43_await_background_or_timeout_us(uint32_t timeout_us);
/* todo not 100% sure about the timeouts here; MP uses __WFI which will always wakeup periodically */
#define CYW43_SDPCM_SEND_COMMON_WAIT k_sleep(K_MSEC(1));
#define CYW43_DO_IOCTL_WAIT k_sleep(K_MSEC(1));

void cyw43_delay_ms(uint32_t ms);

void cyw43_delay_us(uint32_t us);

void cyw43_schedule_internal_poll_dispatch(void (*func)(void));

void cyw43_post_poll_hook(void);

#define CYW43_POST_POLL_HOOK cyw43_post_poll_hook();

#ifdef __cplusplus
}
#endif

#endif
