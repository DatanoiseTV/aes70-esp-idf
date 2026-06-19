/*
 * aes70_ocp1.h - OCP.1 (AES70-3) wire constants and big-endian marshalling.
 *
 * Internal header. The OCP.1 binary protocol frames every PDU as:
 *
 *   [SyncVal 0x3B][ProtocolVersion u16=1][PduSize u32][MsgType u8][MsgCount u16]
 *   followed by MsgCount messages, each prefixed with its own inclusive u32 size.
 *
 *   PduSize counts everything after the 3-byte sync+version prefix, i.e.
 *   PduSize = 7 + sum(message bytes); the whole frame on the wire is PduSize + 3.
 *
 * All multi-byte fields are big-endian (network order). Verified against the
 * Wireshark OCP.1 dissector (epan/dissectors/packet-ocp1.c) and docs.deuso.de.
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Frame constants ---------------------------------------------------- */
#define OCP1_SYNC_VAL        0x3B
#define OCP1_PROTO_VERSION   0x0001
#define OCP1_HEADER_LEN      10        /* sync(1)+ver(2)+size(4)+type(1)+count(2) */

/* PduSize covers bytes after the sync+version prefix (3 bytes). */
#define OCP1_PDUSIZE_PREFIX  3

/* Message (PDU) types. */
typedef enum {
    OCP1_CMD       = 0,  /* command, no response */
    OCP1_CMD_RRQ   = 1,  /* command, response required */
    OCP1_NTF       = 2,  /* notification */
    OCP1_RSP       = 3,  /* response */
    OCP1_KEEPALIVE = 4,  /* keep-alive / heartbeat */
} ocp1_msg_type_t;

/* OcaRoot PropertyChanged event id (DefLevel 1, EventIndex 1). */
#define OCP1_EVENT_PROPERTY_CHANGED_LEVEL  1
#define OCP1_EVENT_PROPERTY_CHANGED_INDEX  1

/* ---- Big-endian scalar helpers ------------------------------------------ */
static inline uint16_t ocp1_rd16(const uint8_t *p)
{
    return (uint16_t)((p[0] << 8) | p[1]);
}
static inline uint32_t ocp1_rd32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
}
static inline void ocp1_wr16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v;
}
static inline void ocp1_wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);  p[3] = (uint8_t)v;
}

/* ---- Bounds-checked reader (parses inbound parameters) ------------------ *
 * Every accessor checks remaining length and latches `err` on underrun; once
 * `err` is set, subsequent reads return 0/empty so a handler can check err once
 * at the end instead of after every field. */
typedef struct {
    const uint8_t *p;
    size_t len;
    size_t off;
    bool   err;
} ocp1_rd_t;

static inline void ocp1_rd_init(ocp1_rd_t *r, const uint8_t *p, size_t len)
{
    r->p = p; r->len = len; r->off = 0; r->err = false;
}
static inline size_t ocp1_rd_remaining(const ocp1_rd_t *r)
{
    return r->off <= r->len ? r->len - r->off : 0;
}
static inline bool ocp1_rd_need(ocp1_rd_t *r, size_t n)
{
    if (r->err || ocp1_rd_remaining(r) < n) { r->err = true; return false; }
    return true;
}
static inline uint8_t ocp1_rd_u8(ocp1_rd_t *r)
{
    if (!ocp1_rd_need(r, 1)) return 0;
    return r->p[r->off++];
}
static inline uint16_t ocp1_rd_u16(ocp1_rd_t *r)
{
    if (!ocp1_rd_need(r, 2)) return 0;
    uint16_t v = ocp1_rd16(r->p + r->off); r->off += 2; return v;
}
static inline uint32_t ocp1_rd_u32(ocp1_rd_t *r)
{
    if (!ocp1_rd_need(r, 4)) return 0;
    uint32_t v = ocp1_rd32(r->p + r->off); r->off += 4; return v;
}
static inline uint64_t ocp1_rd_u64(ocp1_rd_t *r)
{
    if (!ocp1_rd_need(r, 8)) return 0;
    uint64_t hi = ocp1_rd32(r->p + r->off);
    uint64_t lo = ocp1_rd32(r->p + r->off + 4);
    r->off += 8; return (hi << 32) | lo;
}
static inline float ocp1_rd_f32(ocp1_rd_t *r)
{
    uint32_t u = ocp1_rd_u32(r); float f; memcpy(&f, &u, 4); return f;
}
static inline double ocp1_rd_f64(ocp1_rd_t *r)
{
    uint64_t u = ocp1_rd_u64(r); double d; memcpy(&d, &u, 8); return d;
}
/* OcaString: u16 byte length + UTF-8 (no NUL). Copies up to cap-1 bytes into
 * dst and NUL-terminates; always consumes the full field. */
size_t ocp1_rd_string(ocp1_rd_t *r, char *dst, size_t cap);
/* OcaBlob / OcaBlobFixedLen helper: returns pointer to `n` bytes and advances. */
const uint8_t *ocp1_rd_blob(ocp1_rd_t *r, uint16_t *out_len);

/* ---- Bounds-checked writer (builds outbound PDUs) ----------------------- *
 * Latches `err` on overflow; the transport checks err before sending. */
typedef struct {
    uint8_t *p;
    size_t   cap;
    size_t   off;
    bool     err;
} ocp1_wr_t;

static inline void ocp1_wr_init(ocp1_wr_t *w, uint8_t *p, size_t cap)
{
    w->p = p; w->cap = cap; w->off = 0; w->err = false;
}
static inline bool ocp1_wr_room(ocp1_wr_t *w, size_t n)
{
    if (w->err || w->off + n > w->cap) { w->err = true; return false; }
    return true;
}
static inline void ocp1_wr_u8(ocp1_wr_t *w, uint8_t v)
{
    if (!ocp1_wr_room(w, 1)) return;
    w->p[w->off++] = v;
}
static inline void ocp1_wr_u16(ocp1_wr_t *w, uint16_t v)
{
    if (!ocp1_wr_room(w, 2)) return;
    ocp1_wr16(w->p + w->off, v); w->off += 2;
}
static inline void ocp1_wr_u32(ocp1_wr_t *w, uint32_t v)
{
    if (!ocp1_wr_room(w, 4)) return;
    ocp1_wr32(w->p + w->off, v); w->off += 4;
}
static inline void ocp1_wr_u64(ocp1_wr_t *w, uint64_t v)
{
    if (!ocp1_wr_room(w, 8)) return;
    ocp1_wr32(w->p + w->off, (uint32_t)(v >> 32));
    ocp1_wr32(w->p + w->off + 4, (uint32_t)v);
    w->off += 8;
}
static inline void ocp1_wr_f32(ocp1_wr_t *w, float f)
{
    uint32_t u; memcpy(&u, &f, 4); ocp1_wr_u32(w, u);
}
static inline void ocp1_wr_f64(ocp1_wr_t *w, double d)
{
    uint64_t u; memcpy(&u, &d, 8); ocp1_wr_u64(w, u);
}
static inline void ocp1_wr_bytes(ocp1_wr_t *w, const void *src, size_t n)
{
    if (!ocp1_wr_room(w, n)) return;
    memcpy(w->p + w->off, src, n); w->off += n;
}
/* OcaString: u16 length + UTF-8 bytes (NULL => empty). */
void ocp1_wr_string(ocp1_wr_t *w, const char *s);
/* OcaBlob: u16 length + bytes. */
void ocp1_wr_blob(ocp1_wr_t *w, const void *data, uint16_t len);
/* OcaClassID: u16 fieldCount + fieldCount * u16. */
void ocp1_wr_class_id(ocp1_wr_t *w, const uint16_t *fields, uint8_t count);

#ifdef __cplusplus
}
#endif
