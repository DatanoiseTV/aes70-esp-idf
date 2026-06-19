/*
 * aes70_classes.c - Standard OCA control-class method handlers and the class
 * descriptor table that drives dispatch and GetClassIdentification.
 *
 * Method ids are (DefLevel.MethodIndex). DefLevel is the class's depth in the
 * inheritance tree (OcaRoot=1, OcaWorker=2, OcaActuator/OcaSensor=3, their
 * children=4, OcaBasicActuator children=5). All indices verified against
 * AES70-2 / docs.deuso.de.
 *
 * SPDX-License-Identifier: MIT
 */
#include <string.h>

#include "aes70_internal.h"

/* ---- OcaRoot (level 1) -------------------------------------------------- */
aes70_status_t aes70_root_dispatch(struct aes70_object *obj, uint16_t idx,
                                   ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc)
{
    (void)in;
    const aes70_class_desc_t *d = aes70_class_for_kind(obj->kind);
    switch (idx) {
    case 1: /* GetClassIdentification -> OcaClassIdentification */
        ocp1_wr_class_id(out, d->class_id, d->class_id_len);
        ocp1_wr_u16(out, d->class_version);
        *pc = 1;
        return AES70_OK;
    case 2: /* GetLockable -> OcaBoolean */
        ocp1_wr_u8(out, 1);
        *pc = 1;
        return AES70_OK;
    case 3: /* SetLockNoReadWrite (a.k.a. Lock) */
        obj->lock_state = AES70_LOCK_NO_READ_WRITE;
        return AES70_OK;
    case 4: /* Unlock */
        obj->lock_state = AES70_LOCK_NONE;
        return AES70_OK;
    case 5: /* GetRole -> OcaString */
        ocp1_wr_string(out, obj->role);
        *pc = 1;
        return AES70_OK;
    case 6: /* SetLockNoWrite */
        obj->lock_state = AES70_LOCK_NO_WRITE;
        return AES70_OK;
    case 7: /* GetLockState -> OcaLockState */
        ocp1_wr_u8(out, obj->lock_state);
        *pc = 1;
        return AES70_OK;
    default:
        return AES70_BAD_METHOD;
    }
}

/* ---- OcaWorker (level 2) ------------------------------------------------- */
aes70_status_t aes70_worker_dispatch(struct aes70_object *obj, uint16_t idx,
                                     ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc)
{
    switch (idx) {
    case 1: /* GetEnabled -> OcaBoolean */
        ocp1_wr_u8(out, obj->enabled ? 1 : 0);
        *pc = 1;
        return AES70_OK;
    case 2: { /* SetEnabled <- OcaBoolean */
        uint8_t v = ocp1_rd_u8(in);
        if (in->err) return AES70_BAD_FORMAT;
        obj->enabled = (v != 0);
        aes70_notify_property_changed(obj->dev, obj, 2, 1);
        return AES70_OK;
    }
    case 5: /* GetPorts -> OcaList<OcaPort> (none) */
        ocp1_wr_u16(out, 0);
        *pc = 1;
        return AES70_OK;
    case 8: /* GetLabel -> OcaString */
        ocp1_wr_string(out, obj->label ? obj->label : "");
        *pc = 1;
        return AES70_OK;
    case 9: { /* SetLabel <- OcaString */
        char buf[64];
        ocp1_rd_string(in, buf, sizeof(buf));
        if (in->err) return AES70_BAD_FORMAT;
        char *copy = strdup(buf);
        if (!copy) return AES70_OUT_OF_MEMORY;
        aes70_lock(obj->dev);
        char *old = obj->label; obj->label = copy;
        aes70_unlock(obj->dev);
        free(old);
        aes70_notify_property_changed(obj->dev, obj, 2, 3);
        return AES70_OK;
    }
    case 10: /* GetOwner -> OcaONo */
        ocp1_wr_u32(out, obj->owner_ono);
        *pc = 1;
        return AES70_OK;
    default:
        return AES70_NOT_IMPLEMENTED;
    }
}

/* ---- OcaGain (level 4, ClassID 1.1.1.5) --------------------------------- */
static aes70_status_t gain_dispatch(struct aes70_object *obj, uint16_t idx,
                                    ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc)
{
    switch (idx) {
    case 1: /* GetGain -> Gain, MinGain, MaxGain (OcaDB) */
        ocp1_wr_f32(out, (float)obj->num);
        ocp1_wr_f32(out, (float)obj->num_min);
        ocp1_wr_f32(out, (float)obj->num_max);
        *pc = 3;
        return AES70_OK;
    case 2: { /* SetGain <- Gain */
        float g = ocp1_rd_f32(in);
        if (in->err) return AES70_BAD_FORMAT;
        if (g < obj->num_min || g > obj->num_max) return AES70_PARAMETER_OUT_OF_RANGE;
        aes70_object_commit_num(obj, g, 4, 1, true);
        return AES70_OK;
    }
    default:
        return AES70_BAD_METHOD;
    }
}

/* ---- OcaMute (1.1.1.2) / OcaPolarity (1.1.1.3): GetState/SetState -------- */
static aes70_status_t state2_dispatch(struct aes70_object *obj, uint16_t idx,
                                      ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc)
{
    switch (idx) {
    case 1: /* GetState -> OcaMuteState / OcaPolarityState (1 or 2) */
        ocp1_wr_u8(out, (uint8_t)obj->num);
        *pc = 1;
        return AES70_OK;
    case 2: { /* SetState <- state */
        uint8_t s = ocp1_rd_u8(in);
        if (in->err) return AES70_BAD_FORMAT;
        if (s != 1 && s != 2) return AES70_PARAMETER_OUT_OF_RANGE;
        aes70_object_commit_num(obj, s, 4, 1, true);
        return AES70_OK;
    }
    default:
        return AES70_BAD_METHOD;
    }
}

/* ---- OcaSwitch (1.1.1.4) ------------------------------------------------- */
static aes70_status_t switch_dispatch(struct aes70_object *obj, uint16_t idx,
                                      ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc)
{
    switch (idx) {
    case 1: /* GetPosition -> Position, Min, Max (OcaUint16) */
        ocp1_wr_u16(out, (uint16_t)obj->num);
        ocp1_wr_u16(out, 0);
        ocp1_wr_u16(out, obj->sw_count ? (uint16_t)(obj->sw_count - 1) : 0);
        *pc = 3;
        return AES70_OK;
    case 2: { /* SetPosition <- Position */
        uint16_t p = ocp1_rd_u16(in);
        if (in->err) return AES70_BAD_FORMAT;
        if (obj->sw_count == 0 || p > obj->sw_count - 1) return AES70_PARAMETER_OUT_OF_RANGE;
        aes70_object_commit_num(obj, p, 4, 1, true);
        return AES70_OK;
    }
    case 3: { /* GetPositionName <- Index -> Name */
        uint16_t i = ocp1_rd_u16(in);
        if (in->err) return AES70_BAD_FORMAT;
        if (i >= obj->sw_count) return AES70_PARAMETER_OUT_OF_RANGE;
        ocp1_wr_string(out, obj->sw_names && obj->sw_names[i] ? obj->sw_names[i] : "");
        *pc = 1;
        return AES70_OK;
    }
    case 5: /* GetPositionNames -> OcaList<OcaString> */
        ocp1_wr_u16(out, obj->sw_count);
        for (uint16_t i = 0; i < obj->sw_count; i++) {
            ocp1_wr_string(out, obj->sw_names && obj->sw_names[i] ? obj->sw_names[i] : "");
        }
        *pc = 1;
        return AES70_OK;
    default:
        return AES70_NOT_IMPLEMENTED;
    }
}

/* ---- OcaDelay (1.1.1.7), OcaTimeInterval = float64 seconds --------------- */
static aes70_status_t delay_dispatch(struct aes70_object *obj, uint16_t idx,
                                     ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc)
{
    switch (idx) {
    case 1: /* GetDelayTime -> Time, Min, Max */
        ocp1_wr_f64(out, obj->num);
        ocp1_wr_f64(out, obj->num_min);
        ocp1_wr_f64(out, obj->num_max);
        *pc = 3;
        return AES70_OK;
    case 2: { /* SetDelayTime <- Time */
        double t = ocp1_rd_f64(in);
        if (in->err) return AES70_BAD_FORMAT;
        if (t < obj->num_min || t > obj->num_max) return AES70_PARAMETER_OUT_OF_RANGE;
        aes70_object_commit_num(obj, t, 4, 1, true);
        return AES70_OK;
    }
    default:
        return AES70_BAD_METHOD;
    }
}

/* ---- OcaSensor base (level 3): GetReadingState -------------------------- */
static aes70_status_t sensor_dispatch(struct aes70_object *obj, uint16_t idx,
                                      ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc)
{
    (void)in;
    switch (idx) {
    case 1: /* GetReadingState -> OcaSensorReadingState */
        ocp1_wr_u8(out, obj->reading_state);
        *pc = 1;
        return AES70_OK;
    default:
        return AES70_NOT_IMPLEMENTED;
    }
}

/* ---- OcaLevelSensor (1.1.2.2, level 4): GetReading (read-only) ----------- */
static aes70_status_t level_sensor_dispatch(struct aes70_object *obj, uint16_t idx,
                                            ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc)
{
    (void)in;
    switch (idx) {
    case 1: /* GetReading -> Reading, Min, Max (OcaDB) */
        ocp1_wr_f32(out, (float)obj->num);
        ocp1_wr_f32(out, (float)obj->num_min);
        ocp1_wr_f32(out, (float)obj->num_max);
        *pc = 3;
        return AES70_OK;
    default:
        return AES70_NOT_IMPLEMENTED;
    }
}

/* ---- OcaBasicActuator children (level 5): GetSetting/SetSetting ---------- */
static aes70_status_t basic_actuator_dispatch(struct aes70_object *obj, uint16_t idx,
                                              ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc)
{
    if (idx == 1) {                 /* GetSetting */
        switch (obj->kind) {
        case AES70_K_BOOLEAN:
            ocp1_wr_u8(out, obj->num != 0.0 ? 1 : 0);
            *pc = 1;
            break;
        case AES70_K_STRING:
            ocp1_wr_string(out, obj->str ? obj->str : "");
            *pc = 1;
            break;
        case AES70_K_FLOAT32:
            ocp1_wr_f32(out, (float)obj->num);
            ocp1_wr_f32(out, (float)obj->num_min);
            ocp1_wr_f32(out, (float)obj->num_max);
            *pc = 3;
            break;
        case AES70_K_INT32:
            ocp1_wr_u32(out, (uint32_t)(int32_t)obj->num);
            ocp1_wr_u32(out, (uint32_t)(int32_t)obj->num_min);
            ocp1_wr_u32(out, (uint32_t)(int32_t)obj->num_max);
            *pc = 3;
            break;
        case AES70_K_UINT16:
            ocp1_wr_u16(out, (uint16_t)obj->num);
            ocp1_wr_u16(out, (uint16_t)obj->num_min);
            ocp1_wr_u16(out, (uint16_t)obj->num_max);
            *pc = 3;
            break;
        case AES70_K_UINT32:
            ocp1_wr_u32(out, (uint32_t)obj->num);
            ocp1_wr_u32(out, (uint32_t)obj->num_min);
            ocp1_wr_u32(out, (uint32_t)obj->num_max);
            *pc = 3;
            break;
        default:
            return AES70_NOT_IMPLEMENTED;
        }
        return AES70_OK;
    }
    if (idx == 2) {                 /* SetSetting */
        if (obj->kind == AES70_K_BOOLEAN) {
            uint8_t v = ocp1_rd_u8(in);
            if (in->err) return AES70_BAD_FORMAT;
            aes70_object_commit_num(obj, v ? 1 : 0, 5, 1, true);
            return AES70_OK;
        }
        if (obj->kind == AES70_K_STRING) {
            char buf[128];
            ocp1_rd_string(in, buf, sizeof(buf));
            if (in->err) return AES70_BAD_FORMAT;
            if (obj->str_max && strlen(buf) > obj->str_max) return AES70_PARAMETER_OUT_OF_RANGE;
            aes70_object_commit_str(obj, buf, 5, 1, true);
            return AES70_OK;
        }
        double v;
        switch (obj->kind) {
        case AES70_K_FLOAT32: v = ocp1_rd_f32(in); break;
        case AES70_K_INT32:   v = (int32_t)ocp1_rd_u32(in); break;
        case AES70_K_UINT16:  v = ocp1_rd_u16(in); break;
        case AES70_K_UINT32:  v = ocp1_rd_u32(in); break;
        default: return AES70_NOT_IMPLEMENTED;
        }
        if (in->err) return AES70_BAD_FORMAT;
        if (v < obj->num_min || v > obj->num_max) return AES70_PARAMETER_OUT_OF_RANGE;
        aes70_object_commit_num(obj, v, 5, 1, true);
        return AES70_OK;
    }
    return AES70_BAD_METHOD;
}

/* ---- Class descriptor table --------------------------------------------- *
 * level_handlers[i] handles methods whose DefLevel == i+1 (NULL = no methods at
 * that level). The class_id length equals the number of levels. */

/* Reusable level-handler arrays. */
static const aes70_method_fn lv_block[]   = { aes70_root_dispatch, aes70_worker_dispatch, aes70_block_dispatch };
static const aes70_method_fn lv_gain[]    = { aes70_root_dispatch, aes70_worker_dispatch, NULL, gain_dispatch };
static const aes70_method_fn lv_mute[]    = { aes70_root_dispatch, aes70_worker_dispatch, NULL, state2_dispatch };
static const aes70_method_fn lv_pol[]     = { aes70_root_dispatch, aes70_worker_dispatch, NULL, state2_dispatch };
static const aes70_method_fn lv_switch[]  = { aes70_root_dispatch, aes70_worker_dispatch, NULL, switch_dispatch };
static const aes70_method_fn lv_delay[]   = { aes70_root_dispatch, aes70_worker_dispatch, NULL, delay_dispatch };
static const aes70_method_fn lv_basic[]   = { aes70_root_dispatch, aes70_worker_dispatch, NULL, NULL, basic_actuator_dispatch };
static const aes70_method_fn lv_level[]   = { aes70_root_dispatch, aes70_worker_dispatch, sensor_dispatch, level_sensor_dispatch };
static const aes70_method_fn lv_dynamics[] = { aes70_root_dispatch, aes70_worker_dispatch, NULL, aes70_dynamics_dispatch };
static const aes70_method_fn lv_filter[]  = { aes70_root_dispatch, aes70_worker_dispatch, NULL, aes70_filter_dispatch };
static const aes70_method_fn lv_devmgr[]  = { aes70_root_dispatch, NULL, aes70_devmgr_dispatch };
static const aes70_method_fn lv_submgr[]  = { aes70_root_dispatch, NULL, aes70_submgr_dispatch };

static const uint16_t cid_block[]   = { 1, 1, 3 };
static const uint16_t cid_gain[]    = { 1, 1, 1, 5 };
static const uint16_t cid_mute[]    = { 1, 1, 1, 2 };
static const uint16_t cid_pol[]     = { 1, 1, 1, 3 };
static const uint16_t cid_switch[]  = { 1, 1, 1, 4 };
static const uint16_t cid_delay[]   = { 1, 1, 1, 7 };
static const uint16_t cid_boolean[] = { 1, 1, 1, 1, 1 };
static const uint16_t cid_int32[]   = { 1, 1, 1, 1, 4 };
static const uint16_t cid_uint16[]  = { 1, 1, 1, 1, 7 };
static const uint16_t cid_uint32[]  = { 1, 1, 1, 1, 8 };
static const uint16_t cid_float32[] = { 1, 1, 1, 1, 10 };
static const uint16_t cid_string[]  = { 1, 1, 1, 1, 12 };
static const uint16_t cid_level[]    = { 1, 1, 2, 2 };
static const uint16_t cid_dynamics[] = { 1, 1, 1, 14 };
static const uint16_t cid_filter[]   = { 1, 1, 1, 9 };
static const uint16_t cid_devmgr[]   = { 1, 3, 1 };
static const uint16_t cid_submgr[]   = { 1, 3, 4 };

static const aes70_class_desc_t k_desc[AES70_K_COUNT] = {
    [AES70_K_BLOCK]      = { cid_block,   3, 3, lv_block,  "OcaBlock" },
    [AES70_K_GAIN]       = { cid_gain,    4, 3, lv_gain,   "OcaGain" },
    [AES70_K_MUTE]       = { cid_mute,    4, 3, lv_mute,   "OcaMute" },
    [AES70_K_POLARITY]   = { cid_pol,     4, 3, lv_pol,    "OcaPolarity" },
    [AES70_K_SWITCH]     = { cid_switch,  4, 3, lv_switch, "OcaSwitch" },
    [AES70_K_DELAY]      = { cid_delay,   4, 3, lv_delay,  "OcaDelay" },
    [AES70_K_BOOLEAN]    = { cid_boolean, 5, 3, lv_basic,  "OcaBooleanActuator" },
    [AES70_K_INT32]      = { cid_int32,   5, 3, lv_basic,  "OcaInt32Actuator" },
    [AES70_K_UINT16]     = { cid_uint16,  5, 3, lv_basic,  "OcaUint16Actuator" },
    [AES70_K_UINT32]     = { cid_uint32,  5, 3, lv_basic,  "OcaUint32Actuator" },
    [AES70_K_FLOAT32]    = { cid_float32, 5, 3, lv_basic,  "OcaFloat32Actuator" },
    [AES70_K_STRING]     = { cid_string,  5, 3, lv_basic,  "OcaStringActuator" },
    [AES70_K_LEVEL_SENSOR] = { cid_level, 4, 3, lv_level,  "OcaLevelSensor" },
    [AES70_K_DYNAMICS]     = { cid_dynamics, 4, 3, lv_dynamics, "OcaDynamics" },
    [AES70_K_FILTER_CLASSICAL] = { cid_filter, 4, 3, lv_filter, "OcaFilterClassical" },
    [AES70_K_DEVICE_MANAGER]       = { cid_devmgr, 3, 3, lv_devmgr, "OcaDeviceManager" },
    [AES70_K_SUBSCRIPTION_MANAGER] = { cid_submgr, 3, 4, lv_submgr, "OcaSubscriptionManager" },
};

const aes70_class_desc_t *aes70_class_for_kind(aes70_kind_t kind)
{
    if (kind >= AES70_K_COUNT) return NULL;
    return &k_desc[kind];
}
