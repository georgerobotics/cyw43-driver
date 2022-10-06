/*
 * This file is part of the cyw43-driver
 *
 * Copyright (C) 2019-2022 George Robotics Pty Ltd
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Any redistribution, use, or modification in source or binary form is done
 *    solely for personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY THE LICENSOR AND COPYRIGHT OWNER "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE LICENSOR OR COPYRIGHT OWNER BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This software is also available for use with certain devices under different
 * terms, as set out in the top level LICENSE file.  For commercial licensing
 * options please email contact@georgerobotics.com.au.
 */

#ifndef WIFI_FIRMWARE_43439_H
#define WIFI_FIRMWARE_43439_H

#include CYW43_WIFI_NVRAM_INCLUDE_FILE
#include "wifi_firmware.h"

#if CYW43_USE_SPI
#define CYW43_FW_LEN (224190) // 43439A0.bin
#define CYW43_CLM_LEN (984) // 43439_raspberrypi_picow_v5_220624.clm_blob

extern const uint8_t fw_43439A0_7_95_49_00_combined_start[];
extern const uint8_t fw_43439A0_7_95_49_00_combined_end[];
#define CYW43_FIRMWARE_START fw_43439A0_7_95_49_00_combined_start
#define CYW43_FIRMWARE_END fw_43439A0_7_95_49_00_combined_end

#elif !CYW43_USE_SPI
#define CYW43_FW_LEN (383110) // 7.45.98.50
extern const uint8_t fw_4343WA1_7_45_98_50_start[];
extern const uint8_t fw_4343WA1_7_45_98_50_end[];
#define CYW43_CLM_LEN (7222)
#define CYW43_FIRMWARE_START fw_4343WA1_7_45_98_50_start
#define CYW43_FIRMWARE_END fw_4343WA1_7_45_98_50_end
#endif

/*!
 * \brief Get the firmware binary details
 *
 * This method returns the details of the firmware binary
 *
 * \param fw_details Structure to be filled with firmware details
 * \see cyw43_wifi_firmware_details_t
 */
static inline void cyw43_wifi_firmware_details(cyw43_wifi_firmware_details_t *fw_details) {
    fw_details->raw_size = CYW43_FIRMWARE_END - CYW43_FIRMWARE_START;
    fw_details->raw_data = CYW43_FIRMWARE_START;
    fw_details->fw_size = CYW43_FW_LEN;
    fw_details->clm_size = CYW43_CLM_LEN,
    fw_details->fw_addr = CYW43_FIRMWARE_START;
    fw_details->clm_addr = CYW43_FIRMWARE_START + ((CYW43_FW_LEN + 511) & ~511);
    fw_details->wifi_nvram_len = (sizeof(wifi_nvram_4343) + 63) & ~63;
    fw_details->wifi_nvram_data = wifi_nvram_4343;
}

/*!
 * \brief Get the functions used to load firmware
 *
 * This method returns pointers to functions that load firmware
 *
 * \return structure that contains functions that load firmware
 * \see cyw43_wifi_firmware_funcs_t
 */
static inline const cyw43_wifi_firmware_funcs_t *wifi_fw_funcs(void) {
    static const cyw43_wifi_firmware_funcs_t funcs = {
        #if CYW43_DECOMPRESS_FIRMWARE
        .start = wifi_firmware_start_decompress,
        .get_fw_source = wifi_firmware_get_source_decompress,
        .get_nvram_source = wifi_firmware_get_source_embedded, // not compressed
        .get_clm = wifi_firmware_get_clm_decompress,
        .end = cyw43_decompress_firmware_end,
        #else
        .start = NULL,
        .get_fw_source = wifi_firmware_get_source_embedded,
        .get_nvram_source = wifi_firmware_get_source_embedded,
        .get_clm = wifi_firmware_get_clm_embedded,
        .end = NULL,
        #endif
    };
    return &funcs;
}

#endif