/**
 * Copyright (c) 2023 Beechwoods Software, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME zephyr_cyw43
#define LOG_LEVEL CONFIG_WIFI_LOG_LEVEL

#if 1

#include <zephyr/logging/log.h>

#else

#undef LOG_MODULE_REGISTER
#define LOG_MODULE_REGISTER(x, y)
#undef LOG_DBG
#define LOG_DBG(x, ...)
#undef LOG_WRN
#define LOG_WRN(x, ...)
#undef LOG_INF
#define LOG_INF(x, ...)
#undef LOG_ERR
#define LOG_ERR(x, ...)

#endif
