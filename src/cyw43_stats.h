/*
 * This file is part of the cyw43-driver
 *
 * Copyright (C) 2019-2026 George Robotics Pty Ltd
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * \file
 * \brief CYW43 Stats API
*/

#ifndef CYW43_STATS_H
#define CYW43_STATS_H

#include <stdint.h>

#if CYW43_USE_STATS

typedef enum cyw43_stat_t_ {
    CYW43_STAT_F1_RESYNC,
    CYW43_STAT_BUS_ERROR,
    CYW43_STAT_SDIO_INT_CLEAR,
    CYW43_STAT_SPI_INT_CLEAR,
    CYW43_STAT_SPI_PACKET_AVAILABLE,
    CYW43_STAT_SLEEP_COUNT,
    CYW43_STAT_WAKE_COUNT,
    CYW43_STAT_LWIP_RUN_COUNT,
    CYW43_STAT_CYW43_RUN_COUNT,
    CYW43_STAT_PENDSV_RUN_COUNT,
    CYW43_STAT_PENDSV_DISABLED_COUNT,
    CYW43_STAT_IRQ_COUNT,
    CYW43_STAT_PACKET_IN_COUNT,
    CYW43_STAT_PACKET_OUT_COUNT,
    CYW43_STAT_LONGEST_IOCTL_TIME,
    CYW43_STAT_LAST // Add new ones before this
} cyw43_stat_t;

extern uint32_t cyw43_stats[CYW43_STAT_LAST];
#define CYW43_STAT_INC(A) cyw43_stats[CYW43_STAT_##A]++
#define CYW43_STAT_GET(A) cyw43_stats[CYW43_STAT_##A]
#define CYW43_STAT_SET(A, B) cyw43_stats[CYW43_STAT_##A] = B
#define CYW43_STAT_CLR(A) cyw43_stats[CYW43_STAT_##A] = 0

#else

#define CYW43_STAT_INC(A)
#define CYW43_STAT_GET(A)
#define CYW43_STAT_SET(A, B)
#define CYW43_STAT_CLR(A)

#endif

void cyw43_dump_stats(void);

#endif // CYW43_STATS_H
