/*
 * aes70_ocp1.c - OCP.1 marshalling helpers (variable-length OCA types).
 *
 * The fixed-size scalar accessors are inline in aes70_ocp1.h; this file holds
 * the variable-length string/blob/classID helpers.
 *
 * SPDX-License-Identifier: MIT
 */
#include "aes70_ocp1.h"

/* OcaString length prefix is a CODEPOINT count (not a byte count); the data is
 * the UTF-8 bytes of those codepoints (AES70.js OcaString.js). Walk the lead
 * bytes to recover the byte length. */
size_t ocp1_rd_string(ocp1_rd_t *r, char *dst, size_t cap)
{
    uint16_t cps = ocp1_rd_u16(r);
    if (r->err) { if (cap) dst[0] = '\0'; return 0; }

    const uint8_t *p = r->p + r->off;
    size_t avail = ocp1_rd_remaining(r);
    size_t bytes = 0;
    for (uint16_t c = 0; c < cps; c++) {
        if (bytes >= avail) { r->err = true; break; }
        uint8_t lead = p[bytes];
        size_t seq = lead < 0x80 ? 1 : lead < 0xE0 ? 2 : lead < 0xF0 ? 3 : 4;
        bytes += seq;
    }
    if (r->err || bytes > avail) { r->err = true; if (cap) dst[0] = '\0'; return 0; }

    if (cap) {
        size_t copy = bytes < cap ? bytes : cap - 1;
        memcpy(dst, p, copy);
        dst[copy] = '\0';
    }
    r->off += bytes;                     /* always consume the full field */
    return bytes;
}

const uint8_t *ocp1_rd_blob(ocp1_rd_t *r, uint16_t *out_len)
{
    uint16_t n = ocp1_rd_u16(r);
    if (!ocp1_rd_need(r, n)) { if (out_len) *out_len = 0; return NULL; }
    const uint8_t *ptr = r->p + r->off;
    r->off += n;
    if (out_len) *out_len = n;
    return ptr;
}

void ocp1_wr_string(ocp1_wr_t *w, const char *s)
{
    size_t bytes = s ? strlen(s) : 0;
    /* OcaString prefix is the codepoint count = bytes that are not UTF-8
     * continuation bytes (0b10xxxxxx). Equal to `bytes` for ASCII. */
    size_t cps = 0;
    for (size_t i = 0; i < bytes; i++) {
        if (((uint8_t)s[i] & 0xC0) != 0x80) cps++;
    }
    if (cps > 0xFFFF) cps = 0xFFFF;
    ocp1_wr_u16(w, (uint16_t)cps);
    ocp1_wr_bytes(w, s, bytes);
}

void ocp1_wr_blob(ocp1_wr_t *w, const void *data, uint16_t len)
{
    ocp1_wr_u16(w, len);
    ocp1_wr_bytes(w, data, len);
}

void ocp1_wr_class_id(ocp1_wr_t *w, const uint16_t *fields, uint8_t count)
{
    ocp1_wr_u16(w, count);
    for (uint8_t i = 0; i < count; i++) {
        ocp1_wr_u16(w, fields[i]);
    }
}
