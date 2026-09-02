/*
 * This file is part of the cyw43-driver
 *
 * Copyright (C) 2019-2026 George Robotics Pty Ltd
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdbool.h>
#include <inttypes.h>

#include "cyw43_config.h"
#include "cyw43_stats.h"

// Display and reset some stats
void cyw43_dump_stats(void) {
    #if CYW43_USE_STATS
    CYW43_PRINTF("resync: %" PRIu32 " bus error %" PRIu32 "\n", CYW43_STAT_GET(F1_RESYNC), CYW43_STAT_GET(BUS_ERROR));
    CYW43_PRINTF("sdio cleared: %" PRIu32 " spi cleared: %" PRIu32 ", spi packet available: %" PRIu32 "\n",
        CYW43_STAT_GET(SDIO_INT_CLEAR), CYW43_STAT_GET(SPI_INT_CLEAR), CYW43_STAT_GET(SPI_PACKET_AVAILABLE));
    CYW43_PRINTF("irq count %" PRIu32 " lwip run %" PRIu32 " cyw43 run %" PRIu32 " low_prio_irq run %" PRIu32 " disabled %" PRIu32 "\n", CYW43_STAT_GET(IRQ_COUNT),
        CYW43_STAT_GET(LWIP_RUN_COUNT), CYW43_STAT_GET(CYW43_RUN_COUNT),
        CYW43_STAT_GET(PENDSV_RUN_COUNT), CYW43_STAT_GET(PENDSV_DISABLED_COUNT));
    CYW43_PRINTF("longest ioctl %" PRIu32 "ms\n", CYW43_STAT_GET(LONGEST_IOCTL_TIME) / 1000);
    CYW43_PRINTF("sleep count %" PRIu32 " wake %" PRIu32 "\n", CYW43_STAT_GET(SLEEP_COUNT), CYW43_STAT_GET(WAKE_COUNT));

    // clear some stats
    CYW43_STAT_CLR(SDIO_INT_CLEAR);
    CYW43_STAT_CLR(SPI_INT_CLEAR);
    CYW43_STAT_CLR(SPI_PACKET_AVAILABLE);
    CYW43_STAT_CLR(IRQ_COUNT);
    CYW43_STAT_CLR(LWIP_RUN_COUNT);
    CYW43_STAT_CLR(CYW43_RUN_COUNT);
    CYW43_STAT_CLR(SLEEP_COUNT);
    CYW43_STAT_CLR(WAKE_COUNT);
    CYW43_STAT_CLR(PENDSV_DISABLED_COUNT);
    CYW43_STAT_CLR(PENDSV_RUN_COUNT);
    #else
    static bool reported;
    if (!reported) {
        CYW43_PRINTF("NOTE stats printing is disabled!\n");
        reported = true;
    }
    #endif
}
