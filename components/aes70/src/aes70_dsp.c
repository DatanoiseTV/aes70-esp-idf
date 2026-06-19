/*
 * aes70_dsp.c - Dedicated multi-parameter signal-processing control classes:
 *   OcaDynamics        (1.1.1.14) - compressor / limiter / expander / gate
 *   OcaFilterClassical (1.1.1.9)  - highpass/lowpass/... crossover filter
 *
 * Controllers (e.g. AES70 Explorer) render purpose-built widgets for these
 * classes, so a compressor is one OcaDynamics object rather than a block of
 * generic actuators. Each object keeps its parameter set in a class-specific
 * struct hung off aes70_object::priv. Method/property indices and value types
 * are verified against AES70.js controller/ControlClasses/Oca{Dynamics,
 * FilterClassical}.js.
 *
 * SPDX-License-Identifier: MIT
 */
#include <stdlib.h>

#include "aes70_internal.h"

/* OcaDynamicsFunction / OcaLevelDetectionLaw / OcaFilterPassband /
 * OcaClassicalFilterShape / OcaPresentationUnit enum values are in aes70_object.h. */

/* ---- Per-class state ---------------------------------------------------- */
typedef struct {
    uint8_t function;          /* OcaDynamicsFunction */
    uint8_t detector_law;      /* OcaLevelDetectionLaw */
    uint8_t threshold_units;   /* OcaPresentationUnit */
    float   threshold, threshold_ref, threshold_min, threshold_max;  /* OcaDBr value/ref */
    float   ratio, ratio_min, ratio_max;        /* shared by Ratio and Slope */
    float   attack, attack_min, attack_max;     /* seconds */
    float   release, release_min, release_max;  /* seconds */
    float   hold, hold_min, hold_max;           /* seconds */
    float   knee, knee_min, knee_max;
    float   gain_floor, gain_floor_min, gain_floor_max;
    float   gain_ceiling, gain_ceiling_min, gain_ceiling_max;
    /* read-only telemetry */
    float   dynamic_gain;
    bool    triggered;
} dyn_state_t;

typedef struct {
    float    frequency, frequency_min, frequency_max;
    uint8_t  passband;         /* OcaFilterPassband */
    uint8_t  shape;            /* OcaClassicalFilterShape */
    uint16_t order, order_min, order_max;
    float    parameter, parameter_min, parameter_max;
} flt_state_t;

/* Parameter selectors for the app->task set path. */
enum {
    DYN_FUNCTION = 1, DYN_THRESHOLD, DYN_RATIO, DYN_SLOPE, DYN_ATTACK, DYN_RELEASE,
    DYN_HOLD, DYN_KNEE, DYN_GAIN_FLOOR, DYN_GAIN_CEILING, DYN_DETECTOR_LAW,
    DYN_THRESHOLD_UNITS, DYN_DYNAMIC_GAIN, DYN_TRIGGERED,
    FLT_FREQUENCY = 64, FLT_PASSBAND, FLT_SHAPE, FLT_ORDER, FLT_PARAMETER,
};

bool aes70_dsp_is_dsp_kind(aes70_kind_t kind)
{
    return kind == AES70_K_DYNAMICS || kind == AES70_K_FILTER_CLASSICAL;
}

void aes70_dsp_init(struct aes70_object *obj)
{
    if (obj->kind == AES70_K_DYNAMICS) {
        dyn_state_t *d = calloc(1, sizeof(*d));
        if (!d) return;
        d->function = AES70_DYN_COMPRESS;
        d->detector_law = AES70_DETECT_PEAK;
        d->threshold_units = AES70_UNIT_DBU;
        d->threshold = -20.0f; d->threshold_ref = 0.0f; d->threshold_min = -60.0f; d->threshold_max = 0.0f;
        d->ratio = 4.0f;      d->ratio_min = 1.0f;     d->ratio_max = 20.0f;
        d->attack = 0.010f;   d->attack_min = 0.0001f; d->attack_max = 1.0f;
        d->release = 0.100f;  d->release_min = 0.001f; d->release_max = 5.0f;
        d->hold = 0.0f;       d->hold_min = 0.0f;      d->hold_max = 5.0f;
        d->knee = 0.0f;       d->knee_min = 0.0f;      d->knee_max = 20.0f;
        d->gain_floor = -40.0f;   d->gain_floor_min = -80.0f;   d->gain_floor_max = 0.0f;
        d->gain_ceiling = 0.0f;   d->gain_ceiling_min = -40.0f; d->gain_ceiling_max = 24.0f;
        obj->priv = d;
    } else if (obj->kind == AES70_K_FILTER_CLASSICAL) {
        flt_state_t *f = calloc(1, sizeof(*f));
        if (!f) return;
        f->frequency = 1000.0f; f->frequency_min = 20.0f; f->frequency_max = 20000.0f;
        f->passband = AES70_PASSBAND_LOWPASS;
        f->shape = AES70_FILTER_BUTTERWORTH;
        f->order = 2; f->order_min = 1; f->order_max = 8;
        f->parameter = 0.0f; f->parameter_min = 0.0f; f->parameter_max = 10.0f;
        obj->priv = f;
    }
}

void aes70_dsp_free(struct aes70_object *obj)
{
    free(obj->priv);
    obj->priv = NULL;
}

/* ---- Notification property mapping -------------------------------------- */
static void sel_to_prop(uint8_t sel, uint16_t *level, uint16_t *index)
{
    *level = 4;
    switch (sel) {
    case DYN_TRIGGERED:        *index = 1;  break;
    case DYN_DYNAMIC_GAIN:     *index = 2;  break;
    case DYN_FUNCTION:         *index = 3;  break;
    case DYN_RATIO:            *index = 4;  break;
    case DYN_THRESHOLD:        *index = 5;  break;
    case DYN_THRESHOLD_UNITS:  *index = 6;  break;
    case DYN_DETECTOR_LAW:     *index = 7;  break;
    case DYN_ATTACK:           *index = 8;  break;
    case DYN_RELEASE:          *index = 9;  break;
    case DYN_HOLD:             *index = 10; break;
    case DYN_GAIN_CEILING:     *index = 11; break;
    case DYN_GAIN_FLOOR:       *index = 12; break;
    case DYN_KNEE:             *index = 13; break;
    case DYN_SLOPE:            *index = 14; break;
    case FLT_FREQUENCY:        *index = 1;  break;
    case FLT_PASSBAND:         *index = 2;  break;
    case FLT_SHAPE:            *index = 3;  break;
    case FLT_ORDER:            *index = 4;  break;
    case FLT_PARAMETER:        *index = 5;  break;
    default:                   *index = 0;  break;
    }
}

/* Write one parameter (under lock), then notify subscribers and -- when the
 * change came from a controller -- the application. v2 is the OcaDBr reference. */
static void dyn_write(struct aes70_object *obj, uint8_t sel, double v, double v2, bool from_ctrl)
{
    dyn_state_t *d = obj->priv;
    if (!d) return;
    aes70_lock(obj->dev);
    switch (sel) {
    case DYN_FUNCTION:        d->function = (uint8_t)v; break;
    case DYN_DETECTOR_LAW:    d->detector_law = (uint8_t)v; break;
    case DYN_THRESHOLD_UNITS: d->threshold_units = (uint8_t)v; break;
    case DYN_THRESHOLD:       d->threshold = (float)v; d->threshold_ref = (float)v2; break;
    case DYN_RATIO: case DYN_SLOPE: d->ratio = (float)v; break;
    case DYN_ATTACK:          d->attack = (float)v; break;
    case DYN_RELEASE:         d->release = (float)v; break;
    case DYN_HOLD:            d->hold = (float)v; break;
    case DYN_KNEE:            d->knee = (float)v; break;
    case DYN_GAIN_FLOOR:      d->gain_floor = (float)v; break;
    case DYN_GAIN_CEILING:    d->gain_ceiling = (float)v; break;
    case DYN_DYNAMIC_GAIN:    d->dynamic_gain = (float)v; break;
    case DYN_TRIGGERED:       d->triggered = v != 0; break;
    default: break;
    }
    aes70_unlock(obj->dev);

    uint16_t pl, pi; sel_to_prop(sel, &pl, &pi);
    aes70_notify_property_changed(obj->dev, obj, pl, pi);
    if (from_ctrl && obj->dev->cfg.on_control_changed) {
        obj->dev->cfg.on_control_changed(obj, obj->tag, obj->dev->cfg.user);
    }
}

static void flt_write(struct aes70_object *obj, uint8_t sel, double v, bool from_ctrl)
{
    flt_state_t *f = obj->priv;
    if (!f) return;
    aes70_lock(obj->dev);
    switch (sel) {
    case FLT_FREQUENCY: f->frequency = (float)v; break;
    case FLT_PASSBAND:  f->passband = (uint8_t)v; break;
    case FLT_SHAPE:     f->shape = (uint8_t)v; break;
    case FLT_ORDER:     f->order = (uint16_t)v; break;
    case FLT_PARAMETER: f->parameter = (float)v; break;
    default: break;
    }
    aes70_unlock(obj->dev);

    uint16_t pl, pi; sel_to_prop(sel, &pl, &pi);
    aes70_notify_property_changed(obj->dev, obj, pl, pi);
    if (from_ctrl && obj->dev->cfg.on_control_changed) {
        obj->dev->cfg.on_control_changed(obj, obj->tag, obj->dev->cfg.user);
    }
}

void aes70_dsp_apply_set(struct aes70_object *obj, const aes70_set_req_t *req)
{
    if (obj->kind == AES70_K_DYNAMICS) dyn_write(obj, req->sel, req->num, req->num2, false);
    else if (obj->kind == AES70_K_FILTER_CLASSICAL) flt_write(obj, req->sel, req->num, false);
}

/* ---- Property value encoding (for notifications + getters) -------------- */
bool aes70_dsp_encode_property(struct aes70_object *obj, uint16_t level, uint16_t index,
                               ocp1_wr_t *out)
{
    if (level != 4) return false;
    if (obj->kind == AES70_K_DYNAMICS) {
        dyn_state_t *d = obj->priv;
        if (!d) return false;
        switch (index) {
        case 1:  ocp1_wr_u8(out, d->triggered ? 1 : 0); return true;
        case 2:  ocp1_wr_f32(out, d->dynamic_gain); return true;
        case 3:  ocp1_wr_u8(out, d->function); return true;
        case 4:  ocp1_wr_f32(out, d->ratio); return true;
        case 5:  ocp1_wr_f32(out, d->threshold); ocp1_wr_f32(out, d->threshold_ref); return true;
        case 6:  ocp1_wr_u8(out, d->threshold_units); return true;
        case 7:  ocp1_wr_u8(out, d->detector_law); return true;
        case 8:  ocp1_wr_f32(out, d->attack); return true;
        case 9:  ocp1_wr_f32(out, d->release); return true;
        case 10: ocp1_wr_f32(out, d->hold); return true;
        case 11: ocp1_wr_f32(out, d->gain_ceiling); return true;
        case 12: ocp1_wr_f32(out, d->gain_floor); return true;
        case 13: ocp1_wr_f32(out, d->knee); return true;
        case 14: ocp1_wr_f32(out, d->ratio); return true;
        default: return false;
        }
    }
    if (obj->kind == AES70_K_FILTER_CLASSICAL) {
        flt_state_t *f = obj->priv;
        if (!f) return false;
        switch (index) {
        case 1: ocp1_wr_f32(out, f->frequency); return true;
        case 2: ocp1_wr_u8(out, f->passband); return true;
        case 3: ocp1_wr_u8(out, f->shape); return true;
        case 4: ocp1_wr_u16(out, f->order); return true;
        case 5: ocp1_wr_f32(out, f->parameter); return true;
        default: return false;
        }
    }
    return false;
}

/* ---- OcaDynamics method dispatch (level 4, ClassID 1.1.1.14) ------------- */
aes70_status_t aes70_dynamics_dispatch(struct aes70_object *obj, uint16_t idx,
                                       ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc)
{
    dyn_state_t *d = obj->priv;
    if (!d) return AES70_DEVICE_ERROR;

    switch (idx) {
    case 1:  ocp1_wr_u8(out, d->triggered ? 1 : 0); *pc = 1; return AES70_OK;        /* GetTriggered */
    case 2:  ocp1_wr_f32(out, d->dynamic_gain); *pc = 1; return AES70_OK;            /* GetDynamicGain */
    case 3:  ocp1_wr_u8(out, d->function); *pc = 1; return AES70_OK;                 /* GetFunction */
    case 4: { uint8_t f = ocp1_rd_u8(in); if (in->err) return AES70_BAD_FORMAT;      /* SetFunction */
              dyn_write(obj, DYN_FUNCTION, f, 0, true); return AES70_OK; }
    case 5:  ocp1_wr_f32(out, d->ratio); ocp1_wr_f32(out, d->ratio_min);             /* GetRatio */
             ocp1_wr_f32(out, d->ratio_max); *pc = 3; return AES70_OK;
    case 6: { float v = ocp1_rd_f32(in); if (in->err) return AES70_BAD_FORMAT;       /* SetRatio */
              if (v < d->ratio_min || v > d->ratio_max) return AES70_PARAMETER_OUT_OF_RANGE;
              dyn_write(obj, DYN_RATIO, v, 0, true); return AES70_OK; }
    case 7:  ocp1_wr_f32(out, d->threshold); ocp1_wr_f32(out, d->threshold_ref);     /* GetThreshold (OcaDBr+min+max) */
             ocp1_wr_f32(out, d->threshold_min); ocp1_wr_f32(out, d->threshold_max);
             *pc = 3; return AES70_OK;
    case 8: { float v = ocp1_rd_f32(in); float ref = ocp1_rd_f32(in);                /* SetThreshold (OcaDBr) */
              if (in->err) return AES70_BAD_FORMAT;
              if (v < d->threshold_min || v > d->threshold_max) return AES70_PARAMETER_OUT_OF_RANGE;
              dyn_write(obj, DYN_THRESHOLD, v, ref, true); return AES70_OK; }
    case 9:  ocp1_wr_u8(out, d->threshold_units); *pc = 1; return AES70_OK;          /* GetThresholdPresentationUnits */
    case 10:{ uint8_t u = ocp1_rd_u8(in); if (in->err) return AES70_BAD_FORMAT;      /* SetThresholdPresentationUnits */
              dyn_write(obj, DYN_THRESHOLD_UNITS, u, 0, true); return AES70_OK; }
    case 11: ocp1_wr_u8(out, d->detector_law); *pc = 1; return AES70_OK;             /* GetDetectorLaw */
    case 12:{ uint8_t l = ocp1_rd_u8(in); if (in->err) return AES70_BAD_FORMAT;      /* SetDetectorLaw */
              dyn_write(obj, DYN_DETECTOR_LAW, l, 0, true); return AES70_OK; }
    case 13: ocp1_wr_f32(out, d->attack); ocp1_wr_f32(out, d->attack_min);           /* GetAttackTime */
             ocp1_wr_f32(out, d->attack_max); *pc = 3; return AES70_OK;
    case 14:{ float v = ocp1_rd_f32(in); if (in->err) return AES70_BAD_FORMAT;       /* SetAttackTime */
              if (v < d->attack_min || v > d->attack_max) return AES70_PARAMETER_OUT_OF_RANGE;
              dyn_write(obj, DYN_ATTACK, v, 0, true); return AES70_OK; }
    case 15: ocp1_wr_f32(out, d->release); ocp1_wr_f32(out, d->release_min);         /* GetReleaseTime */
             ocp1_wr_f32(out, d->release_max); *pc = 3; return AES70_OK;
    case 16:{ float v = ocp1_rd_f32(in); if (in->err) return AES70_BAD_FORMAT;       /* SetReleaseTime */
              if (v < d->release_min || v > d->release_max) return AES70_PARAMETER_OUT_OF_RANGE;
              dyn_write(obj, DYN_RELEASE, v, 0, true); return AES70_OK; }
    case 17: ocp1_wr_f32(out, d->hold); ocp1_wr_f32(out, d->hold_min);               /* GetHoldTime */
             ocp1_wr_f32(out, d->hold_max); *pc = 3; return AES70_OK;
    case 18:{ float v = ocp1_rd_f32(in); if (in->err) return AES70_BAD_FORMAT;       /* SetHoldTime */
              dyn_write(obj, DYN_HOLD, v, 0, true); return AES70_OK; }
    case 19: ocp1_wr_f32(out, d->gain_floor); ocp1_wr_f32(out, d->gain_floor_min);   /* GetDynamicGainFloor */
             ocp1_wr_f32(out, d->gain_floor_max); *pc = 3; return AES70_OK;
    case 20:{ float v = ocp1_rd_f32(in); if (in->err) return AES70_BAD_FORMAT;       /* SetDynamicGainFloor */
              dyn_write(obj, DYN_GAIN_FLOOR, v, 0, true); return AES70_OK; }
    case 21: ocp1_wr_f32(out, d->gain_ceiling); ocp1_wr_f32(out, d->gain_ceiling_min); /* GetDynamicGainCeiling */
             ocp1_wr_f32(out, d->gain_ceiling_max); *pc = 3; return AES70_OK;
    case 22:{ float v = ocp1_rd_f32(in); if (in->err) return AES70_BAD_FORMAT;       /* SetDynamicGainCeiling */
              dyn_write(obj, DYN_GAIN_CEILING, v, 0, true); return AES70_OK; }
    case 23: ocp1_wr_f32(out, d->knee); ocp1_wr_f32(out, d->knee_min);               /* GetKneeParameter */
             ocp1_wr_f32(out, d->knee_max); *pc = 3; return AES70_OK;
    case 24:{ float v = ocp1_rd_f32(in); if (in->err) return AES70_BAD_FORMAT;       /* SetKneeParameter */
              if (v < d->knee_min || v > d->knee_max) return AES70_PARAMETER_OUT_OF_RANGE;
              dyn_write(obj, DYN_KNEE, v, 0, true); return AES70_OK; }
    case 25: ocp1_wr_f32(out, d->ratio); ocp1_wr_f32(out, d->ratio_min);             /* GetSlope */
             ocp1_wr_f32(out, d->ratio_max); *pc = 3; return AES70_OK;
    case 26:{ float v = ocp1_rd_f32(in); if (in->err) return AES70_BAD_FORMAT;       /* SetSlope */
              dyn_write(obj, DYN_SLOPE, v, 0, true); return AES70_OK; }
    case 27: return AES70_NOT_IMPLEMENTED;   /* SetMultiple: use the individual setters */
    default: return AES70_BAD_METHOD;
    }
}

/* ---- OcaFilterClassical method dispatch (level 4, ClassID 1.1.1.9) ------- */
aes70_status_t aes70_filter_dispatch(struct aes70_object *obj, uint16_t idx,
                                     ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc)
{
    flt_state_t *f = obj->priv;
    if (!f) return AES70_DEVICE_ERROR;

    switch (idx) {
    case 1:  ocp1_wr_f32(out, f->frequency); ocp1_wr_f32(out, f->frequency_min);     /* GetFrequency */
             ocp1_wr_f32(out, f->frequency_max); *pc = 3; return AES70_OK;
    case 2: { float v = ocp1_rd_f32(in); if (in->err) return AES70_BAD_FORMAT;       /* SetFrequency */
              if (v < f->frequency_min || v > f->frequency_max) return AES70_PARAMETER_OUT_OF_RANGE;
              flt_write(obj, FLT_FREQUENCY, v, true); return AES70_OK; }
    case 3:  ocp1_wr_u8(out, f->passband); *pc = 1; return AES70_OK;                 /* GetPassband */
    case 4: { uint8_t p = ocp1_rd_u8(in); if (in->err) return AES70_BAD_FORMAT;      /* SetPassband */
              flt_write(obj, FLT_PASSBAND, p, true); return AES70_OK; }
    case 5:  ocp1_wr_u8(out, f->shape); *pc = 1; return AES70_OK;                    /* GetShape */
    case 6: { uint8_t s = ocp1_rd_u8(in); if (in->err) return AES70_BAD_FORMAT;      /* SetShape */
              flt_write(obj, FLT_SHAPE, s, true); return AES70_OK; }
    case 7:  ocp1_wr_u16(out, f->order); ocp1_wr_u16(out, f->order_min);             /* GetOrder */
             ocp1_wr_u16(out, f->order_max); *pc = 3; return AES70_OK;
    case 8: { uint16_t o = ocp1_rd_u16(in); if (in->err) return AES70_BAD_FORMAT;    /* SetOrder */
              if (o < f->order_min || o > f->order_max) return AES70_PARAMETER_OUT_OF_RANGE;
              flt_write(obj, FLT_ORDER, o, true); return AES70_OK; }
    case 9:  ocp1_wr_f32(out, f->parameter); ocp1_wr_f32(out, f->parameter_min);     /* GetParameter */
             ocp1_wr_f32(out, f->parameter_max); *pc = 3; return AES70_OK;
    case 10:{ float v = ocp1_rd_f32(in); if (in->err) return AES70_BAD_FORMAT;       /* SetParameter */
              flt_write(obj, FLT_PARAMETER, v, true); return AES70_OK; }
    case 11: return AES70_NOT_IMPLEMENTED;   /* SetMultiple: use the individual setters */
    default: return AES70_BAD_METHOD;
    }
}

/* ======================================================================== *
 *  Public API
 * ======================================================================== */
static struct aes70_object *make_dsp(aes70_device_t *dev, aes70_object_handle_t parent,
                                     aes70_kind_t kind, const char *role)
{
    aes70_lock(dev);
    struct aes70_object *o = aes70_object_alloc(dev, kind, parent, role);
    if (o) aes70_dsp_init(o);
    aes70_unlock(dev);
    return (o && o->priv) ? o : NULL;
}

/* ---- OcaDynamics -------------------------------------------------------- */
aes70_object_handle_t aes70_dynamics_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                            const char *role, uint8_t function)
{
    struct aes70_object *o = make_dsp(dev, parent, AES70_K_DYNAMICS, role);
    if (o) { ((dyn_state_t *)o->priv)->function = function; }
    return o;
}

#define DYN(obj) ((dyn_state_t *)((struct aes70_object *)(obj))->priv)

static esp_err_t dyn_post(aes70_object_handle_t obj, uint8_t sel, double v, double v2)
{
    if (!obj || !obj->priv) return ESP_ERR_INVALID_ARG;
    aes70_set_req_t req = { .ono = obj->ono, .sel = sel, .num = v, .num2 = v2 };
    return aes70_post_set(obj, &req);
}

uint8_t aes70_dynamics_get_function(aes70_object_handle_t o)  { return o && o->priv ? DYN(o)->function : 0; }
float   aes70_dynamics_get_threshold(aes70_object_handle_t o) { return o && o->priv ? DYN(o)->threshold : 0; }
float   aes70_dynamics_get_ratio(aes70_object_handle_t o)     { return o && o->priv ? DYN(o)->ratio : 0; }
float   aes70_dynamics_get_attack(aes70_object_handle_t o)    { return o && o->priv ? DYN(o)->attack : 0; }
float   aes70_dynamics_get_release(aes70_object_handle_t o)   { return o && o->priv ? DYN(o)->release : 0; }

esp_err_t aes70_dynamics_set_function(aes70_object_handle_t o, uint8_t f) { return dyn_post(o, DYN_FUNCTION, f, 0); }
esp_err_t aes70_dynamics_set_threshold(aes70_object_handle_t o, float db) { return dyn_post(o, DYN_THRESHOLD, db, 0); }
esp_err_t aes70_dynamics_set_ratio(aes70_object_handle_t o, float r)      { return dyn_post(o, DYN_RATIO, r, 0); }
esp_err_t aes70_dynamics_set_attack(aes70_object_handle_t o, float s)     { return dyn_post(o, DYN_ATTACK, s, 0); }
esp_err_t aes70_dynamics_set_release(aes70_object_handle_t o, float s)    { return dyn_post(o, DYN_RELEASE, s, 0); }
esp_err_t aes70_dynamics_report_gain(aes70_object_handle_t o, float gain_db, bool triggered)
{
    esp_err_t e = dyn_post(o, DYN_DYNAMIC_GAIN, gain_db, 0);
    dyn_post(o, DYN_TRIGGERED, triggered ? 1 : 0, 0);
    return e;
}

/* ---- OcaFilterClassical ------------------------------------------------- */
aes70_object_handle_t aes70_filter_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                          const char *role, uint8_t passband, uint8_t shape,
                                          float frequency, uint16_t order)
{
    struct aes70_object *o = make_dsp(dev, parent, AES70_K_FILTER_CLASSICAL, role);
    if (o) {
        flt_state_t *f = o->priv;
        f->passband = passband; f->shape = shape; f->frequency = frequency; f->order = order;
    }
    return o;
}

#define FLT(obj) ((flt_state_t *)((struct aes70_object *)(obj))->priv)

static esp_err_t flt_post(aes70_object_handle_t obj, uint8_t sel, double v)
{
    if (!obj || !obj->priv) return ESP_ERR_INVALID_ARG;
    aes70_set_req_t req = { .ono = obj->ono, .sel = sel, .num = v };
    return aes70_post_set(obj, &req);
}

float    aes70_filter_get_frequency(aes70_object_handle_t o) { return o && o->priv ? FLT(o)->frequency : 0; }
uint8_t  aes70_filter_get_passband(aes70_object_handle_t o)  { return o && o->priv ? FLT(o)->passband : 0; }
uint8_t  aes70_filter_get_shape(aes70_object_handle_t o)     { return o && o->priv ? FLT(o)->shape : 0; }
uint16_t aes70_filter_get_order(aes70_object_handle_t o)     { return o && o->priv ? FLT(o)->order : 0; }

esp_err_t aes70_filter_set_frequency(aes70_object_handle_t o, float hz)  { return flt_post(o, FLT_FREQUENCY, hz); }
esp_err_t aes70_filter_set_passband(aes70_object_handle_t o, uint8_t p)  { return flt_post(o, FLT_PASSBAND, p); }
esp_err_t aes70_filter_set_shape(aes70_object_handle_t o, uint8_t s)     { return flt_post(o, FLT_SHAPE, s); }
esp_err_t aes70_filter_set_order(aes70_object_handle_t o, uint16_t n)    { return flt_post(o, FLT_ORDER, n); }
