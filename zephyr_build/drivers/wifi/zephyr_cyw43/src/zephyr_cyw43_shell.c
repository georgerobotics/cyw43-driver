/**
 * Copyright (c) 2023 Beechwoods Software, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/led.h>

#include "zephyr_cyw43_drv.h"


const struct device *const led_device =  DEVICE_DT_GET_ANY(infineon_cyw43_led);

#if defined(CONFIG_WIFI_ZEPHYR_CYW43_SHELL)
static zephyr_cyw43_dev_t *zephyr_cyw43_dev;

void zephyr_cyw43_shell_register(zephyr_cyw43_dev_t *dev)
{
        /* only one instance supported */
        if (zephyr_cyw43_dev) {
                return;
        }

        zephyr_cyw43_dev = dev;
}

static int zephyr_cyw43_shell_led(const struct shell *sh, size_t argc,
                                  char **argv)
{
        if (zephyr_cyw43_dev == NULL) {
                shell_print(sh, "no zephyr_cyw43_dev device registered");
                return -ENOEXEC;
        }

        if (argc != 2) {
                shell_help(sh);
                return -ENOEXEC;
        }

        if (!strcmp(argv[1], "on")) {
                shell_print(sh, "Enabling LED");
                led_on(led_device, 0);
        }
        else if (!strcmp(argv[1], "off")) {
                shell_print(sh, "Disabling LED");
                led_off(led_device, 0);
        }
        else {
                shell_help(sh);
                return -ENOEXEC;
        }

        return 0;
}

static int zephyr_cyw43_shell_reset(const struct shell *sh, size_t argc,
                                    char **argv)
{
        if (zephyr_cyw43_dev == NULL) {
                shell_print(sh, "no zephyr_cyw43_dev device registered");
                return -ENOEXEC;
        }

        if (argc != 1) {
                shell_help(sh);
                return -ENOEXEC;
        }

        zephyr_cyw43_lock(zephyr_cyw43_dev);

        shell_print(sh, "Resetting CYW43 module");

        zephyr_cyw43_unlock(zephyr_cyw43_dev);

        return 0;
}
#endif

#if defined(CONFIG_WIFI_ZEPHYR_CYW43_SHELL)
SHELL_STATIC_SUBCMD_SET_CREATE(
        zephyr_cyw43_shell,
        SHELL_CMD(led, NULL, "cyw43 led <on|off>", zephyr_cyw43_shell_led),
        SHELL_CMD(reset, NULL, "cyw43 reset", zephyr_cyw43_shell_reset),
        SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(cyw43, &zephyr_cyw43_shell, "CYW43 debug shell", NULL);
#endif
