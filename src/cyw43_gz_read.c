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

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "uzlib.h"
#include "cyw43_gz_read.h"
#include "cyw43.h"

#define DICT_SIZE 4096
typedef struct uzlib_data {
    uint8_t dict[DICT_SIZE];
    struct uzlib_uncomp state;
    size_t amount_left;
} uzlib_data_t;

#define CYW43_DECOMPRESS_ERR_NO_MEM -1
#define CYW43_DECOMPRESS_ERR_BAD_HEADER -2
#define CYW43_DECOMPRESS_ERR_NO_MORE -3
#define CYW43_DECOMPRESS_ERR_DECOMPRESS -4

int cyw43_gz_read_start(void **uzlib_context, const uint8_t *raw_data, size_t raw_size)
{
    uzlib_init();

    uzlib_data_t *uzlib = cyw43_malloc(sizeof(uzlib_data_t));
    assert(uzlib);
    if (!uzlib) {
        return CYW43_DECOMPRESS_ERR_NO_MEM;
    }
    uzlib->amount_left = raw_data[raw_size - 1];
    uzlib->amount_left = 256 * uzlib->amount_left + raw_data[raw_size - 2];
    uzlib->amount_left = 256 * uzlib->amount_left + raw_data[raw_size - 3];
    uzlib->amount_left = 256 * uzlib->amount_left + raw_data[raw_size - 4];

    uzlib_uncompress_init(&uzlib->state, uzlib->dict, sizeof(uzlib->dict));

    uzlib->state.source = raw_data;
    uzlib->state.source_limit = raw_data + raw_size - 4;
    uzlib->state.source_read_cb = NULL;

    int res = uzlib_gzip_parse_header(&uzlib->state);
    assert(res == TINF_OK);
    if (res != TINF_OK) {
        cyw43_free(uzlib);
        return CYW43_DECOMPRESS_ERR_BAD_HEADER;
    }

    *uzlib_context = uzlib;
    return (int)uzlib->amount_left;
}

int cyw43_gz_read_next(void *uzlib_context, uint8_t *buffer, size_t sz)
{
    assert(uzlib_context);
    uzlib_data_t *uzlib = (uzlib_data_t *)uzlib_context;

    assert(uzlib->amount_left > 0);
    if (uzlib->amount_left <= 0) {
        return CYW43_DECOMPRESS_ERR_NO_MORE;
    }
    const size_t chunk_sz = uzlib->amount_left < sz ? uzlib->amount_left : sz;
    uzlib->state.dest = buffer;
    uzlib->state.dest_limit = uzlib->state.dest + chunk_sz;
    int res = uzlib_uncompress_chksum(&uzlib->state);
    assert(res == TINF_OK);
    if (res != TINF_OK) {
        return CYW43_DECOMPRESS_ERR_DECOMPRESS;
    }
    uzlib->amount_left -= chunk_sz;
    return chunk_sz;
}

void cyw43_gz_read_end(void *uzlib_context)
{
    assert(uzlib_context);
    uzlib_data_t *uzlib = (uzlib_data_t *)uzlib_context;

    uzlib->amount_left = 0;
    if (uzlib) {
        cyw43_free(uzlib);
        uzlib = NULL;
    }
}
