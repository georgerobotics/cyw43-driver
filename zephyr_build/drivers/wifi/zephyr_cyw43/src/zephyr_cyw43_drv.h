/**
 * Copyright (c) 2023 Beechwoods Software, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/kernel_structs.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/net/wifi_mgmt.h>

enum zephyr_cyw43_request {
        ZEPHYR_CYW43_REQ_ACTIVE_SCAN,
        ZEPHYR_CYW43_REQ_PASSIVE_SCAN,
        ZEPHYR_CYW43_REQ_CONNECT,
        ZEPHYR_CYW43_REQ_DISCONNECT,
        ZEPHYR_CYW43_REQ_ENABLE_AP,
        ZEPHYR_CYW43_REQ_DISABLE_AP,
        ZEPHYR_CYW43_REQ_IFACE_STATUS,
        ZEPHYR_CYW43_REQ_SET_PM,
        ZEPHYR_CYW43_REQ_NONE
};

enum zephyr_cyw43_role {
        ZEPHYR_CYW43_ROLE_CLIENT,
        ZEPHYR_CYW43_ROLE_AP,
};

struct zephyr_cyw43_cfg {
        struct gpio_dt_spec irq_gpio;
        struct gpio_dt_spec wl_on_gpio;
};

struct zephyr_cyw43_connect_params {
        char ssid[WIFI_SSID_MAX_LEN + 1];
        char psk[65];
        uint32_t security;
        uint32_t channel;
};

struct zephyr_cyw43_power_save_params {
        uint32_t pm_out;
        uint32_t pm_in;
};

typedef struct zephyr_cyw43_device_params {
        struct net_if *iface;
        struct zephyr_cyw43_bus_ops *bus;
        scan_result_cb_t scan_cb;
        struct k_work_q work_q;
        struct k_work request_work;
        struct k_work_delayable status_work;
        struct zephyr_cyw43_connect_params connect_params;
        struct zephyr_cyw43_connect_params ap_params;
        struct zephyr_cyw43_power_save_params pm_params;
        enum zephyr_cyw43_request req;
        enum zephyr_cyw43_role role;
        struct net_stats_wifi stats;
        uint8_t frame_buf[NET_ETH_MAX_FRAME_SIZE];
        struct k_mutex mutex;
        atomic_val_t mutex_owner;
        unsigned int mutex_depth;
        struct k_sem event_sem;
} zephyr_cyw43_dev_t;

static inline void zephyr_cyw43_lock(zephyr_cyw43_dev_t *zephyr_cyw43_dev)
{
        /* Nested locking */
        if (atomic_get(&zephyr_cyw43_dev->mutex_owner) != (atomic_t)(uintptr_t)_current) {
                k_mutex_lock(&zephyr_cyw43_dev->mutex, K_FOREVER);
                atomic_set(&zephyr_cyw43_dev->mutex_owner, (atomic_t)(uintptr_t)_current);
                zephyr_cyw43_dev->mutex_depth = 1;
        } else {
                zephyr_cyw43_dev->mutex_depth++;
        }
}

static inline void zephyr_cyw43_unlock(zephyr_cyw43_dev_t *zephyr_cyw43_dev)
{
        if (!--zephyr_cyw43_dev->mutex_depth) {
                atomic_set(&zephyr_cyw43_dev->mutex_owner, -1);
                k_mutex_unlock(&zephyr_cyw43_dev->mutex);
        }
}

#define CYW43_SECURITY_TO_ZEPHYR_SECURITY(x) \
  ((x) == CYW43_AUTH_OPEN ? WIFI_SECURITY_TYPE_NONE :			\
   ((x) == CYW43_AUTH_WPA_TKIP_PSK ? WIFI_SECURITY_TYPE_WPA_PSK :	\
    ((x) == CYW43_AUTH_WPA2_AES_PSK ? WIFI_SECURITY_TYPE_PSK :		\
     ((x) == CYW43_AUTH_WPA2_MIXED_PSK ? WIFI_SECURITY_TYPE_PSK :	\
      WIFI_SECURITY_TYPE_UNKNOWN))))

#if defined(CONFIG_WIFI_ZEPHYR_CYW43_SHELL)
void zephyr_cyw43_shell_register(zephyr_cyw43_dev_t *dev);
#else
#define zephyr_cyw43_shell_register(dev)
#endif

