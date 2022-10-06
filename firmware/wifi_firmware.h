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

#ifndef WIFI_FIRMWARE_H
#define WIFI_FIRMWARE_H

#if CYW43_DECOMPRESS_FIRMWARE
#include "cyw43_decompress_firmware.h"
#endif

/*!
 * \brief Structure to store wifi firmware details
 */
//!\{
typedef struct cyw43_wifi_firmware_details {
    size_t raw_size;            ///< size in bytes of the firmware binary
    const uint8_t *raw_data;    ///< pointer to the firmware binary
    size_t fw_size;             ///< Size of the firmware in bytes
    size_t clm_size;            ///< Size of the clm in bytes
    const uint8_t *fw_addr;     ///< Pointer to the firmware in the binary
    const uint8_t *clm_addr;    ///< Pointer to the clm in the binary
    size_t wifi_nvram_len;      ///< Size of nvram data
    const uint8_t *wifi_nvram_data; ///< Pointer to nvram data
} cyw43_wifi_firmware_details_t;
//!\}

/*!
 * \brief Structure to hold function pointers for loading firmware
 */
//!\{
typedef struct cyw43_wifi_firmware_funcs {
    int (*start)(const cyw43_wifi_firmware_details_t *fw_details); ///< start firmware loading
    const uint8_t* (*get_fw_source)(const uint8_t *addr, size_t sz_in, uint8_t *buffer, size_t buffer_len); ///< get fw data
    const uint8_t* (*get_nvram_source)(const uint8_t *addr, size_t sz_in, uint8_t *buffer, size_t buffer_len); ///< get nvram data
    int (*get_clm)(uint8_t *dst, const uint8_t *src, uint32_t len); ///< get clm data
    void (*end)(void); ///< end firmware loading
} cyw43_wifi_firmware_funcs_t;
//!\}

/*!
 * \brief get firmware data from flash
 *
 * Loads firmware data from flash and returns a pointer to it
 *
 * \param addr Address of firmware data required
 * \param sz_in Amount of firmware data required in bytes
 * \param buffer Temporary buffer that can be used to load and return firmware data
 * \param buffer_len Length of temporary buffer in bytes
 * \return Requested firmware data
 */
const uint8_t *wifi_firmware_get_source_storage(const uint8_t *addr, size_t sz_in, uint8_t *buffer, size_t buffer_len);

/*!
 * \brief get firmware data embedded in the elf file binary
 *
 * Loads firmware data from the elf file and returns a pointer to it
 *
 * \param addr Address of firmware data required
 * \param sz_in Amount of firmware data required in bytes
 * \param buffer Temporary buffer that can be used to load and return firmware data
 * \param buffer_len Length of temporary buffer in bytes
 * \return Requested firmware data
 */
const uint8_t *wifi_firmware_get_source_embedded(const uint8_t *addr, size_t sz_in, uint8_t *buffer, size_t buffer_len);

/*!
 * \brief get clm data embedded in the elf file binary
 *
 * Loads clm data from the elf file binary and returns a pointer to it
 *
 * \param dst Destination of clm data
 * \param src Source address of the clm data
 * \param len Amount of data required in bytes
 * \return >=0 on success or <0 on error
 */
int wifi_firmware_get_clm_embedded(uint8_t *dst, const uint8_t *src, uint32_t len);

#if CYW43_DECOMPRESS_FIRMWARE
/*!
 * \brief Start firmware decompression process
 *
 * Prepares and allocates resources needed to decompress firmware
 *
 * \param fw_details Details of the firmware
 * \see cyw43_wifi_firmware_details_t
 * \return >=0 on success or <0 on error
 */
int wifi_firmware_start_decompress(const cyw43_wifi_firmware_details_t* fw_details);

/*!
 * \brief get and decompress firmware data embedded in the elf file binary
 *
 * Loads firmware data from the elf file, decompressed it and returns a pointer to it
 *
 * \param addr Address of firmware data required
 * \param sz_in Amount of firmware data required in bytes
 * \param buffer Temporary buffer that can be used to load and return firmware data
 * \param buffer_len Length of temporary buffer in bytes
 * \return Requested firmware data
 */
const uint8_t *wifi_firmware_get_source_decompress(const uint8_t *addr, size_t sz_in, uint8_t *buffer, size_t buffer_len);

/*!
 * \brief get and decompress clm data embedded in the elf file binary
 *
 * Loads clm data from flash, decompresses it  and returns a pointer to it
 *
 * \param dst Destination of clm data
 * \param src Source address of the clm data
 * \param len Amount of data required in bytes
 * \return >=0 on success or <0 on error
 */
int wifi_firmware_get_clm_decompress(uint8_t *dst, const uint8_t *src, uint32_t len);
#endif

#endif
