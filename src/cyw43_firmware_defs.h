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

#ifndef CYW43_FIRMWARE_DEFS_H
#define CYW43_FIRMWARE_DEFS_H

/*!
 * \brief Structure to store firmware details
 */
//!\{
typedef struct cyw43_firmware_details {
    size_t raw_wifi_fw_len;     ///< Size in bytes of the combined wifi firmware data before extraction
    size_t wifi_fw_len;         ///< Size of the wifi firmware in bytes after extraction
    size_t clm_len;             ///< Size of the clm blob in bytes after extraction
    const uint8_t *wifi_fw_addr;///< Pointer to the raw wifi firmware
    const uint8_t *clm_addr;    ///< Pointer to the raw clm blob in uncompressed firmware
    size_t wifi_nvram_len;      ///< Size of nvram data
    const uint8_t *wifi_nvram_addr; ///< Pointer to nvram data
    size_t raw_bt_fw_len;       ///< size of bluetooth firmware data before extraction
    size_t bt_fw_len;           ///< size of bluetooth firmware data after extraction
    const uint8_t *bt_fw_addr;  ///< Pointer to the bluetooth firmware
} cyw43_firmware_details_t;
//!\}

/*!
 * \brief Structure to hold function pointers for loading firmware
 */
//!\{
typedef struct cyw43_firmware_funcs {
    const cyw43_firmware_details_t* (*firmware_details)(void); ///< get wifi firmware details
    int (*start_wifi_fw)(const cyw43_firmware_details_t *fw_details); ///< start wifi firmware loading
    int (*start_bt_fw)(const cyw43_firmware_details_t *fw_details); ///< start bt firmware loading
    const uint8_t* (*get_wifi_fw)(const uint8_t *addr, size_t sz_in, uint8_t *buffer, size_t buffer_sz); ///< get block of wifi firmware data
    const uint8_t* (*get_bt_fw)(const uint8_t *addr, size_t sz_in, uint8_t *buffer, size_t buffer_sz); ///< get block of bt firmware data
    const uint8_t* (*get_nvram)(const uint8_t *addr, size_t sz_in, uint8_t *buffer, size_t buffer_sz); ///< get block of nvram data
    const uint8_t* (*get_clm)(const uint8_t *addr, size_t sz_in, uint8_t *buffer, size_t buffer_sz); ///< get block of clm data
    void (*end)(void); ///< end firmware loading
} cyw43_firmware_funcs_t;
//!\}

/*!
 * \brief Get the functions used to load firmware
 *
 * This method returns pointers to functions that load firmware
 *
 * \return structure that contains functions that load firmware
 */
const cyw43_firmware_funcs_t *cyw43_get_firmware_funcs(void);

/*!
 * \brief Read an uncompressed firmware data
 *
 * This method reads uncompressed firmware data
 *
 * \param addr Address to start reading
 * \param sz_in Amount data to read in bytes
 * \param buffer Temporary buffer that can be used to read data into
 * \param buffer_sz Size of buffer in bytes
 * \return Pointer to data read
 */
const uint8_t *cyw43_read_uncompressed_firmware(const uint8_t *addr, size_t sz_in, uint8_t *buffer, size_t buffer_sz);

/*!
 * \brief Start reading wifi firmware data
 *
 * This method starts the process of reading compressed wifi firmware
 *
 * \param fw Firmware details
 * \return Zero on success or else an error code
 */
int cyw43_start_compressed_wifi_firmware(const cyw43_firmware_details_t* fw);

/*!
 * \brief Start reading bluetooth firmware data
 *
 * This method starts the process of reading compressed bluetooth firmware
 *
 * \param fw Firmware details
 * \return Zero on success or else an error code
 */
int cyw43_start_compressed_bt_firmware(__unused const cyw43_firmware_details_t* fw);

/*!
 * \brief Read compressed firmware data
 *
 * This method reads compressed firmware data
 *
 * \param addr Address to start reading
 * \param sz_in Amount data to read in bytes
 * \param buffer Temporary buffer that can be used to read data into
 * \param buffer_sz Size of buffer in bytes
 * \return Pointer to data read
 */
const uint8_t *cyw43_read_compressed_firmware(const uint8_t *addr, size_t sz_in, uint8_t *buffer, size_t buffer_sz);

/*!
 * \brief End reading compressed firmware data
 *
 * This method ends the process of reading compressed firmware
 */
void cyw43_end_compressed_firmware(void);

#endif
