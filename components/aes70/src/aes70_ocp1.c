/*
 * aes70_ocp1.c - OCP.1 marshalling helpers (variable-length OCA types).
 *
 * The fixed-size scalar accessors are inline in aes70_ocp1.h; this file holds
 * the variable-length string/blob/classID helpers.
 *
 * SPDX-License-Identifier: MIT
 */
#include "aes70_ocp1.h"

size_t ocp1_rd_string(ocp1_rd_t *r, char *dst, size_t cap)
{
    uint16_t n = ocp1_rd_u16(r);
    if (!ocp1_rd_need(r, n)) {           /* declared length exceeds the buffer */
        if (cap) dst[0] = '\0';
        return 0;
    }
    size_t copy = n;
    if (cap == 0) {                      /* nothing to copy into */
        r->off += n;
        return n;
    }
    if (copy > cap - 1) copy = cap - 1;
    memcpy(dst, r->p + r->off, copy);
    dst[copy] = '\0';
    r->off += n;                         /* always consume the full field */
    return n;
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
    size_t n = s ? strlen(s) : 0;
    if (n > 0xFFFF) n = 0xFFFF;          /* OcaString length is a u16 */
    ocp1_wr_u16(w, (uint16_t)n);
    ocp1_wr_bytes(w, s, n);
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
