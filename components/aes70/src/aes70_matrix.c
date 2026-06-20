/*
 * aes70_matrix.c - OcaMatrix (ClassID 1.1.5): a rectangular, coordinate-
 * addressable grid of member objects (e.g. a router/mixer crosspoint array).
 *
 * The matrix is a container/addresser: it holds the member ONos, the current
 * (X,Y) selection, a proxy object and the port geometry. The members do the
 * actual signal work. The batch operations ExecuteMethod (17) and
 * ExecuteCommands (18) -- which re-dispatch an arbitrary method across a set of
 * cells -- are reported NotImplemented; everything needed to define and address
 * a matrix is here.
 *
 * SPDX-License-Identifier: MIT
 */
#include <string.h>
#include <stdlib.h>

#include "aes70_internal.h"

#define MX_MAX 16            /* max rows/columns */

typedef struct {
    uint16_t x_size, y_size;       /* current dimensions (columns x, rows y) */
    uint16_t x_max, y_max;         /* settable bounds */
    uint16_t cur_x, cur_y;         /* current selection; 0 = whole row/column/matrix */
    uint32_t proxy_ono;
    uint8_t  ports_per_row, ports_per_col;
    bool     xy_locked;
    uint32_t members[MX_MAX][MX_MAX];   /* [y][x] member ONos, 0 = empty */
} matrix_priv_t;

void aes70_matrix_init(struct aes70_object *obj, uint16_t x_size, uint16_t y_size)
{
    matrix_priv_t *m = calloc(1, sizeof(*m));
    if (!m) return;
    if (x_size > MX_MAX) x_size = MX_MAX;
    if (y_size > MX_MAX) y_size = MX_MAX;
    m->x_size = x_size; m->y_size = y_size;
    m->x_max = MX_MAX;  m->y_max = MX_MAX;
    m->ports_per_row = 1; m->ports_per_col = 1;
    obj->priv = m;
}

aes70_object_handle_t aes70_matrix_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                          const char *role, uint16_t x_size, uint16_t y_size)
{
    struct aes70_object *o = aes70_object_alloc((aes70_device_t *)dev, AES70_K_MATRIX, parent, role);
    if (!o) return NULL;
    aes70_matrix_init(o, x_size, y_size);
    return (aes70_object_handle_t)o;
}

esp_err_t aes70_matrix_set_member(aes70_object_handle_t matrix, uint16_t x, uint16_t y,
                                  aes70_object_handle_t member)
{
    struct aes70_object *o = (struct aes70_object *)matrix;
    if (!o || o->kind != AES70_K_MATRIX || !o->priv) return ESP_ERR_INVALID_ARG;
    matrix_priv_t *m = o->priv;
    if (x < 1 || x > m->x_size || y < 1 || y > m->y_size) return ESP_ERR_INVALID_ARG;
    m->members[y - 1][x - 1] = member ? aes70_object_ono((aes70_object_handle_t)member) : 0;
    return ESP_OK;
}

aes70_status_t aes70_matrix_dispatch(struct aes70_object *obj, uint16_t idx,
                                     ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc)
{
    matrix_priv_t *m = obj->priv;
    if (!m) return AES70_DEVICE_ERROR;

    switch (idx) {
    case 1:  /* GetCurrentXY -> (X, Y) */
        ocp1_wr_u16(out, m->cur_x); ocp1_wr_u16(out, m->cur_y); *pc = 2; return AES70_OK;
    case 2: { /* SetCurrentXY(X, Y); 0 selects the whole row/column/matrix */
        uint16_t x = ocp1_rd_u16(in), y = ocp1_rd_u16(in);
        if (in->err) return AES70_BAD_FORMAT;
        if (m->xy_locked) return AES70_LOCKED;
        if (x > m->x_size || y > m->y_size) return AES70_PARAMETER_OUT_OF_RANGE;
        m->cur_x = x; m->cur_y = y; return AES70_OK;
    }
    case 3:  /* GetSize -> (xSize, ySize, minX, maxX, minY, maxY) */
        ocp1_wr_u16(out, m->x_size); ocp1_wr_u16(out, m->y_size);
        ocp1_wr_u16(out, 1); ocp1_wr_u16(out, m->x_max);
        ocp1_wr_u16(out, 1); ocp1_wr_u16(out, m->y_max);
        *pc = 6; return AES70_OK;
    case 4: { /* SetSize(xSize, ySize) */
        uint16_t x = ocp1_rd_u16(in), y = ocp1_rd_u16(in);
        if (in->err) return AES70_BAD_FORMAT;
        if (x < 1 || x > m->x_max || y < 1 || y > m->y_max) return AES70_PARAMETER_OUT_OF_RANGE;
        m->x_size = x; m->y_size = y;
        if (m->cur_x > x) m->cur_x = 0;
        if (m->cur_y > y) m->cur_y = 0;
        return AES70_OK;
    }
    case 5:  /* GetMembers -> OcaList2D<OcaUint32> (rows = ySize, cols = xSize) */
        ocp1_wr_u16(out, m->y_size); ocp1_wr_u16(out, m->x_size);
        for (uint16_t y = 0; y < m->y_size; y++)
            for (uint16_t x = 0; x < m->x_size; x++) ocp1_wr_u32(out, m->members[y][x]);
        *pc = 1; return AES70_OK;
    case 6: { /* SetMembers(OcaList2D<OcaUint32>) */
        uint16_t rows = ocp1_rd_u16(in), cols = ocp1_rd_u16(in);
        if (in->err) return AES70_BAD_FORMAT;
        if (rows > m->y_max || cols > m->x_max) return AES70_PARAMETER_OUT_OF_RANGE;
        m->y_size = rows; m->x_size = cols;
        for (uint16_t y = 0; y < rows; y++)
            for (uint16_t x = 0; x < cols; x++) m->members[y][x] = ocp1_rd_u32(in);
        if (in->err) return AES70_BAD_FORMAT;
        return AES70_OK;
    }
    case 7: { /* GetMember(X, Y) -> OcaUint32 */
        uint16_t x = ocp1_rd_u16(in), y = ocp1_rd_u16(in);
        if (in->err) return AES70_BAD_FORMAT;
        if (x < 1 || x > m->x_size || y < 1 || y > m->y_size) return AES70_PARAMETER_OUT_OF_RANGE;
        ocp1_wr_u32(out, m->members[y - 1][x - 1]); *pc = 1; return AES70_OK;
    }
    case 8: { /* SetMember(X, Y, OcaUint32) */
        uint16_t x = ocp1_rd_u16(in), y = ocp1_rd_u16(in); uint32_t ono = ocp1_rd_u32(in);
        if (in->err) return AES70_BAD_FORMAT;
        if (x < 1 || x > m->x_size || y < 1 || y > m->y_size) return AES70_PARAMETER_OUT_OF_RANGE;
        m->members[y - 1][x - 1] = ono; return AES70_OK;
    }
    case 9:  ocp1_wr_u32(out, m->proxy_ono); *pc = 1; return AES70_OK;        /* GetProxy */
    case 10: { uint32_t v = ocp1_rd_u32(in); if (in->err) return AES70_BAD_FORMAT; m->proxy_ono = v; return AES70_OK; }
    case 11: ocp1_wr_u8(out, m->ports_per_row); *pc = 1; return AES70_OK;     /* GetPortsPerRow */
    case 12: { uint8_t v = ocp1_rd_u8(in); if (in->err) return AES70_BAD_FORMAT; m->ports_per_row = v; return AES70_OK; }
    case 13: ocp1_wr_u8(out, m->ports_per_col); *pc = 1; return AES70_OK;     /* GetPortsPerColumn */
    case 14: { uint8_t v = ocp1_rd_u8(in); if (in->err) return AES70_BAD_FORMAT; m->ports_per_col = v; return AES70_OK; }
    case 15: { /* SetCurrentXYLock(X, Y) */
        uint16_t x = ocp1_rd_u16(in), y = ocp1_rd_u16(in);
        if (in->err) return AES70_BAD_FORMAT;
        if (x > m->x_size || y > m->y_size) return AES70_PARAMETER_OUT_OF_RANGE;
        m->cur_x = x; m->cur_y = y; m->xy_locked = true; return AES70_OK;
    }
    case 16: m->xy_locked = false; return AES70_OK;                           /* UnlockCurrent */
    case 17: /* ExecuteMethod  - batch re-dispatch across cells */
    case 18: /* ExecuteCommands */
        return AES70_NOT_IMPLEMENTED;
    default:
        return AES70_BAD_METHOD;
    }
}
