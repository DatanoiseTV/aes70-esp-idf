/*
 * aes70_managers.c - OcaDeviceManager (1.3.1) and OcaBlock (1.1.3) handlers.
 *
 * OcaDeviceManager exposes the device's identity and the manager list that a
 * controller reads on connect. OcaBlock exposes the device's object topology so
 * a controller can enumerate the control objects.
 *
 * SPDX-License-Identifier: MIT
 */
#include <string.h>

#include "aes70_internal.h"

/* ---- OcaDeviceManager (level 3) ----------------------------------------- */
aes70_status_t aes70_devmgr_dispatch(struct aes70_object *obj, uint16_t idx,
                                     ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc)
{
    aes70_device_t *dev = obj->dev;

    switch (idx) {
    case 1: /* GetOcaVersion -> OcaUint16 */
        ocp1_wr_u16(out, 3);                 /* OCA model generation implemented */
        *pc = 1;
        return AES70_OK;
    case 2: /* GetModelGUID -> OcaModelGUID (8 bytes: Reserved[1]+MfrCode[3]+ModelCode[4]) */
        ocp1_wr_bytes(out, dev->model_guid, sizeof(dev->model_guid));
        *pc = 1;
        return AES70_OK;
    case 3: /* GetSerialNumber -> OcaString */
        ocp1_wr_string(out, dev->serial);
        *pc = 1;
        return AES70_OK;
    case 4: /* GetDeviceName -> OcaString */
        ocp1_wr_string(out, dev->device_name);
        *pc = 1;
        return AES70_OK;
    case 5: { /* SetDeviceName <- OcaString */
        char buf[sizeof(dev->device_name)];
        ocp1_rd_string(in, buf, sizeof(buf));
        if (in->err) return AES70_BAD_FORMAT;
        aes70_lock(dev);
        strlcpy(dev->device_name, buf, sizeof(dev->device_name));
        aes70_unlock(dev);
        aes70_notify_property_changed(dev, obj, 3, 4);
        return AES70_OK;
    }
    case 6: /* GetModelDescription -> OcaModelDescription {Manufacturer,Name,Version} */
        ocp1_wr_string(out, dev->manufacturer);
        ocp1_wr_string(out, dev->model);
        ocp1_wr_string(out, dev->version);
        *pc = 1;
        return AES70_OK;
    case 7: /* GetDeviceRole -> OcaString */
        ocp1_wr_string(out, dev->device_role);
        *pc = 1;
        return AES70_OK;
    case 8: { /* SetDeviceRole <- OcaString */
        char buf[sizeof(dev->device_role)];
        ocp1_rd_string(in, buf, sizeof(buf));
        if (in->err) return AES70_BAD_FORMAT;
        aes70_lock(dev);
        strlcpy(dev->device_role, buf, sizeof(dev->device_role));
        aes70_unlock(dev);
        aes70_notify_property_changed(dev, obj, 3, 6);
        return AES70_OK;
    }
    case 11: /* GetEnabled (ControlEnabled) -> OcaBoolean */
        ocp1_wr_u8(out, 1);
        *pc = 1;
        return AES70_OK;
    case 13: /* GetState -> OcaDeviceState (OcaBitSet16; 0 = operational, no flags) */
        ocp1_wr_u16(out, 0);
        *pc = 1;
        return AES70_OK;
    case 15: /* GetResetCause -> OcaResetCause (PowerOn = 0) */
        ocp1_wr_u8(out, 0);
        *pc = 1;
        return AES70_OK;
    case 19: { /* GetManagers -> OcaList<OcaManagerDescriptor> */
        struct aes70_object *mgrs[] = { dev->device_manager, dev->subscription_manager };
        uint16_t n = 0;
        for (size_t i = 0; i < sizeof(mgrs) / sizeof(mgrs[0]); i++) if (mgrs[i]) n++;
        ocp1_wr_u16(out, n);
        for (size_t i = 0; i < sizeof(mgrs) / sizeof(mgrs[0]); i++) {
            struct aes70_object *m = mgrs[i];
            if (!m) continue;
            const aes70_class_desc_t *d = aes70_class_for_kind(m->kind);
            ocp1_wr_u32(out, m->ono);                          /* ObjectNumber */
            ocp1_wr_string(out, d->class_name);                /* Name */
            ocp1_wr_class_id(out, d->class_id, d->class_id_len); /* ClassID */
            ocp1_wr_u16(out, d->class_version);                /* ClassVersion */
        }
        *pc = 1;
        return AES70_OK;
    }
    default:
        return AES70_NOT_IMPLEMENTED;
    }
}

/* ---- OcaBlock (level 3) -------------------------------------------------- */
static void emit_object_identification(struct aes70_object *o, ocp1_wr_t *out)
{
    const aes70_class_desc_t *d = aes70_class_for_kind(o->kind);
    ocp1_wr_u32(out, o->ono);                              /* ONo */
    ocp1_wr_class_id(out, d->class_id, d->class_id_len);   /* ClassIdentification.ClassID */
    ocp1_wr_u16(out, d->class_version);                    /* ClassIdentification.ClassVersion */
}

static uint16_t count_descendants(struct aes70_object *block)
{
    uint16_t n = 0;
    for (uint16_t i = 0; i < block->child_count; i++) {
        n++;
        if (block->children[i]->kind == AES70_K_BLOCK) {
            n = (uint16_t)(n + count_descendants(block->children[i]));
        }
    }
    return n;
}

static void emit_descendants(struct aes70_object *block, ocp1_wr_t *out)
{
    for (uint16_t i = 0; i < block->child_count; i++) {
        struct aes70_object *c = block->children[i];
        emit_object_identification(c, out);  /* OcaBlockMember.MemberObjectIdentification */
        ocp1_wr_u32(out, block->ono);        /* OcaBlockMember.ContainerObjectNumber */
        if (c->kind == AES70_K_BLOCK) emit_descendants(c, out);
    }
}

/* ---- Parameter-data snapshot/recall ------------------------------------- *
 * FetchCurrentParameterData / ApplyParameterData carry an OcaLongBlob whose
 * format is, per AES70, implementation-specific. Ours is: u16 count, then
 * count x { OcaONo(u32), value(f64) } over the block's scalar actuator
 * descendants (gain, mute, switch, ... -- the values held in obj->num). DSP
 * multi-parameter classes' internal parameters are not captured. */
static bool is_param_kind(aes70_kind_t k)
{
    switch (k) {
    case AES70_K_GAIN: case AES70_K_MUTE: case AES70_K_POLARITY:
    case AES70_K_SWITCH: case AES70_K_DELAY: case AES70_K_FREQUENCY:
    case AES70_K_BOOLEAN: case AES70_K_INT32: case AES70_K_UINT16:
    case AES70_K_UINT32: case AES70_K_FLOAT32:
        return true;
    default:
        return false;
    }
}

static uint16_t count_params(struct aes70_object *b)
{
    uint16_t n = 0;
    for (uint16_t i = 0; i < b->child_count; i++) {
        if (is_param_kind(b->children[i]->kind)) n++;
        if (b->children[i]->kind == AES70_K_BLOCK) n = (uint16_t)(n + count_params(b->children[i]));
    }
    return n;
}

static void emit_params(struct aes70_object *b, ocp1_wr_t *out)
{
    for (uint16_t i = 0; i < b->child_count; i++) {
        struct aes70_object *c = b->children[i];
        if (is_param_kind(c->kind)) { ocp1_wr_u32(out, c->ono); ocp1_wr_f64(out, c->num); }
        if (c->kind == AES70_K_BLOCK) emit_params(c, out);
    }
}

aes70_status_t aes70_block_dispatch(struct aes70_object *obj, uint16_t idx,
                                    ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc)
{
    switch (idx) {
    case 1: /* GetType -> OcaONo (deprecated; 0 = none) */
        ocp1_wr_u32(out, 0);
        *pc = 1;
        return AES70_OK;
    case 5: /* GetActionObjects -> OcaList<OcaObjectIdentification> (direct members) */
        aes70_lock(obj->dev);
        ocp1_wr_u16(out, obj->child_count);
        for (uint16_t i = 0; i < obj->child_count; i++) {
            emit_object_identification(obj->children[i], out);
        }
        aes70_unlock(obj->dev);
        *pc = 1;
        return AES70_OK;
    case 6: /* GetActionObjectsRecursive -> OcaList<OcaBlockMember> (all descendants) */
        aes70_lock(obj->dev);
        ocp1_wr_u16(out, count_descendants(obj));
        emit_descendants(obj, out);
        aes70_unlock(obj->dev);
        *pc = 1;
        return AES70_OK;
    case 25: { /* FetchCurrentParameterData -> OcaLongBlob (snapshot of this block) */
        aes70_lock(obj->dev);
        uint16_t n = count_params(obj);
        ocp1_wr_u32(out, (uint32_t)(2 + (uint32_t)n * 12));   /* OcaLongBlob length */
        ocp1_wr_u16(out, n);
        emit_params(obj, out);
        aes70_unlock(obj->dev);
        *pc = 1;
        return AES70_OK;
    }
    case 26: { /* ApplyParameterData(OcaLongBlob) -> restore (AES70.js mislabels the
                * direction; the standard text says it applies the supplied data). */
        uint32_t blen = ocp1_rd_u32(in);
        if (in->err || in->off + blen > in->len) return AES70_BAD_FORMAT;
        ocp1_rd_t b; ocp1_rd_init(&b, in->p + in->off, blen);
        in->off += blen;
        uint16_t n = ocp1_rd_u16(&b);
        for (uint16_t i = 0; i < n; i++) {
            uint32_t ono = ocp1_rd_u32(&b);
            double v = ocp1_rd_f64(&b);
            if (b.err) return AES70_BAD_FORMAT;
            struct aes70_object *t = aes70_device_find(obj->dev, ono);
            if (!t || !is_param_kind(t->kind)) continue;
            uint16_t pl, pi;
            if (!aes70_object_primary_property(t, &pl, &pi)) continue;
            if (t->num_min < t->num_max) {            /* clamp the kinds that carry a range */
                if (v < t->num_min) v = t->num_min;
                if (v > t->num_max) v = t->num_max;
            }
            aes70_object_commit_num(t, v, pl, pi, true);   /* notify + drive the app's DSP */
        }
        return AES70_OK;
    }
    default:
        return AES70_NOT_IMPLEMENTED;
    }
}
