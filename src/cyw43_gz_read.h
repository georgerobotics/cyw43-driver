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

#ifndef CYW43_GZ_READ_H
#define CYW43_GZ_READ_H

/*!
 * \brief Start decompressing data
 *
 * This method prepares for decompressing data.
 *
 * \param uzlib_context Pointer used to return the decompression context
 * \param raw_data The compressed data
 * \param raw_size The size in bytes of the data
 * \return >0 on success, the uncompressed data size in bytes or <0 on error
 */
int cyw43_gz_read_start(void **uzlib_context, const uint8_t *raw_data, size_t raw_size);

/*!
 * \brief Get the next block of uncompressed data
 *
 * This method returns the next block of uncompressed data.
 *
 * \param uzlib_context Pointer to the decompression context
 * \param buffer Buffer to fill with uncompressed data
 * \param sz Requested size of uncompressed data in bytes
 * \return The amount of data returned in the buffer or <0 on error
 */
int cyw43_gz_read_next(void *uzlib_context, uint8_t *buffer, size_t sz);

/*!
 * \brief Finish decompressing data
 *
 * \param uzlib_context Pointer to the decompression context
 * This method frees any resources used for decompressing data.
 */
void cyw43_gz_read_end(void *uzlib_context);

#endif