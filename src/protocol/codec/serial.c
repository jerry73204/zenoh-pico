//
// Copyright (c) 2024 ZettaScale Technology
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   ZettaScale Zenoh Team, <zenoh@zettascale.tech>
//

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "zenoh-pico/utils/checksum.h"
#include "zenoh-pico/utils/encoding.h"
#include "zenoh-pico/utils/logging.h"
#include "zenoh-pico/utils/pointers.h"
#include "zenoh-pico/utils/result.h"

#define KIND_FIELD_LEN 1u
#define LEN_FIELD_LEN 2u
#define CRC32_LEN 4u

/* COBS-encode a frame straight from its parts (nano-ros I12).
 *
 * The frame is header, 2-byte length, payload, 4-byte CRC. The serializer used
 * to copy those into a temporary MFS-sized buffer and COBS-encode that, which
 * cost a second ~1.5 KiB buffer and a copy of every payload byte per send. The
 * encoder below walks the parts in place instead.
 *
 * The output is byte-identical to `_z_cobs_encode` over the concatenated
 * parts, including that encoder's quirk: a block that ends on a 0xFF code
 * always opens the next one, even after the last input byte. (Its
 * `!*byte || len` is tested before `len` is decremented, so it is always true
 * inside the loop.) The peer decodes that, so it is kept. */
typedef struct {
    uint8_t *out;
    size_t cap;
    size_t pos;
    size_t codep;
    uint8_t code;
    bool overflow;
} _z_cobs_stream_t;

static void _z_cobs_stream_init(_z_cobs_stream_t *st, uint8_t *out, size_t cap) {
    st->out = out;
    st->cap = cap;
    st->codep = 0;
    st->pos = 1;
    st->code = 1;
    st->overflow = (cap == 0u);
}

static void _z_cobs_stream_feed(_z_cobs_stream_t *st, const uint8_t *in, size_t len) {
    for (size_t i = 0; (i < len) && !st->overflow; i++) {
        uint8_t byte = in[i];
        if (byte != 0x00) {
            if (st->pos >= st->cap) {
                st->overflow = true;
                return;
            }
            st->out[st->pos++] = byte;
            st->code = (uint8_t)(st->code + 1u);
        }
        if ((byte == 0x00) || (st->code == (uint8_t)0xFF)) {
            st->out[st->codep] = st->code;
            st->code = 1;
            st->codep = st->pos;
            if (st->pos >= st->cap) {
                st->overflow = true;
                return;
            }
            st->pos++;
        }
    }
}

size_t _z_serial_msg_serialize_direct(uint8_t *dest, size_t dest_len, const uint8_t *src, size_t src_len,
                                      uint8_t header) {
    if (src_len > (size_t)UINT16_MAX) {
        _Z_DEBUG("payload too large for the serial length field: %zu", src_len);
        return SIZE_MAX;
    }

    uint32_t crc32 = _z_crc32(src, src_len);
    uint8_t crc_bytes[CRC32_LEN] = {(uint8_t)(crc32 & 0xFF), (uint8_t)((crc32 >> 8) & 0xFF),
                                    (uint8_t)((crc32 >> 16) & 0xFF), (uint8_t)((crc32 >> 24) & 0xFF)};

    uint16_t wire_size = (uint16_t)src_len;
    uint8_t head_bytes[KIND_FIELD_LEN + LEN_FIELD_LEN] = {header, (uint8_t)(wire_size & 0xFF),
                                                          (uint8_t)((wire_size >> 8) & 0xFF)};

    _z_cobs_stream_t st;
    _z_cobs_stream_init(&st, dest, dest_len);
    _z_cobs_stream_feed(&st, head_bytes, sizeof(head_bytes));
    if (src_len != 0u) {
        _z_cobs_stream_feed(&st, src, src_len);
    }
    _z_cobs_stream_feed(&st, crc_bytes, sizeof(crc_bytes));
    /* Close the last block, then the frame delimiter. */
    if (st.overflow || (st.pos + 1u > dest_len)) {
        _Z_DEBUG("destination buffer too small");
        return SIZE_MAX;
    }
    dest[st.codep] = st.code;
    dest[st.pos] = 0x00;

    return st.pos + 1u;
}

size_t _z_serial_msg_serialize(uint8_t *dest, size_t dest_len, const uint8_t *src, size_t src_len, uint8_t header,
                               uint8_t *tmp_buf, size_t tmp_buf_len) {
    /* `tmp_buf` is no longer used (see `_z_serial_msg_serialize_direct`); the
       size check stays so a caller's contract does not change. */
    _ZP_UNUSED(tmp_buf);
    size_t expected_size = src_len + KIND_FIELD_LEN + LEN_FIELD_LEN + CRC32_LEN;
    if (tmp_buf_len < expected_size) {
        _Z_DEBUG("tmp buffer too small: %zu < %zu", tmp_buf_len, expected_size);
        return SIZE_MAX;
    }
    return _z_serial_msg_serialize_direct(dest, dest_len, src, src_len, header);
}

size_t _z_serial_msg_deserialize(const uint8_t *src, size_t src_len, uint8_t *dst, size_t dst_len, uint8_t *header,
                                 uint8_t *tmp_buf, size_t tmp_buf_len) {
    if (tmp_buf_len < src_len) {
        _Z_DEBUG("tmp_buf too small");
        return SIZE_MAX;
    }

    size_t decoded_size = _z_cobs_decode(src, src_len, tmp_buf);

    if (decoded_size < KIND_FIELD_LEN + LEN_FIELD_LEN + CRC32_LEN) {
        _Z_DEBUG("decoded frame too small");
        return SIZE_MAX;
    }

    uint8_t *tmp_buf_ptr = tmp_buf;

    *header = tmp_buf_ptr[0];
    tmp_buf_ptr += sizeof(uint8_t);

    uint16_t wire_size = (uint16_t)(tmp_buf_ptr[0] | (tmp_buf_ptr[1] << 8));
    tmp_buf_ptr += sizeof(uint16_t);

    size_t expected_size = wire_size + KIND_FIELD_LEN + LEN_FIELD_LEN + CRC32_LEN;
    if (expected_size != decoded_size) {
        _Z_DEBUG("wire size mismatch: %zu != %zu", expected_size, decoded_size);
        return SIZE_MAX;
    }

    if (dst_len < wire_size) {
        _Z_DEBUG("destination buffer too small: %zu < %u", dst_len, wire_size);
        return SIZE_MAX;
    }

    if (wire_size != 0) {
        memcpy(dst, tmp_buf_ptr, wire_size);
        tmp_buf_ptr += wire_size;
    }

    uint32_t received_crc =
        (uint32_t)(tmp_buf_ptr[0] | (tmp_buf_ptr[1] << 8) | (tmp_buf_ptr[2] << 16) | (tmp_buf_ptr[3] << 24));

    uint32_t computed_crc = _z_crc32(dst, wire_size);
    if (received_crc != computed_crc) {
        _Z_DEBUG("CRC mismatch. Received: 0x%08" PRIu32 ", Computed: 0x%08" PRIu32, received_crc, computed_crc);
        return SIZE_MAX;
    }

    return wire_size;
}
