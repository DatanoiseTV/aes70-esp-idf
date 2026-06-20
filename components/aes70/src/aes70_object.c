/*
 * aes70_object.c - OCA object model: allocation, registry, method dispatch,
 * property-value encoding and value commit/notify plumbing.
 *
 * SPDX-License-Identifier: MIT
 */
#include <stdlib.h>
#include <string.h>

#include "lwip/sockets.h"   /* send() on the wake self-pipe */
#include "esp_log.h"
#include "aes70_internal.h"

static const char *TAG = "aes70";

/* ---- Locking ------------------------------------------------------------ */
void aes70_lock(aes70_device_t *dev)   { xSemaphoreTakeRecursive(dev->lock, portMAX_DELAY); }
void aes70_unlock(aes70_device_t *dev) { xSemaphoreGiveRecursive(dev->lock); }

/* ---- Allocation & registry ---------------------------------------------- */
struct aes70_object *aes70_object_new(aes70_device_t *dev, aes70_kind_t kind,
                                      uint32_t ono, const char *role)
{
    if (dev->object_count >= CONFIG_AES70_MAX_OBJECTS) {
        ESP_LOGE(TAG, "object table full (%d); raise AES70_MAX_OBJECTS",
                 CONFIG_AES70_MAX_OBJECTS);
        return NULL;
    }
    struct aes70_object *obj = calloc(1, sizeof(*obj));
    if (!obj) return NULL;
    obj->dev         = dev;
    obj->kind        = kind;
    obj->ono         = ono;
    obj->enabled     = true;
    obj->lock_state  = AES70_LOCK_NONE;
    obj->lock_owner  = -1;
    obj->role        = strdup(role ? role : "");
    if (!obj->role) { free(obj); return NULL; }

    dev->objects[dev->object_count++] = obj;
    return obj;
}

void aes70_block_add_member(struct aes70_object *block, struct aes70_object *child)
{
    if (!block || block->kind != AES70_K_BLOCK || !child) return;
    if (block->child_count >= block->child_cap) {
        uint16_t cap = block->child_cap ? (uint16_t)(block->child_cap * 2) : 4;
        struct aes70_object **n = realloc(block->children, cap * sizeof(*n));
        if (!n) return;
        block->children = n;
        block->child_cap = cap;
    }
    block->children[block->child_count++] = child;
    child->owner_ono = block->ono;
}

struct aes70_object *aes70_object_alloc(aes70_device_t *dev, aes70_kind_t kind,
                                        aes70_object_handle_t parent, const char *role)
{
    struct aes70_object *block = parent ? parent : dev->root_block;
    struct aes70_object *obj = aes70_object_new(dev, kind, dev->next_ono++, role);
    if (!obj) return NULL;
    aes70_block_add_member(block, obj);
    return obj;
}

void aes70_object_free(struct aes70_object *obj)
{
    if (!obj) return;
    free(obj->role);
    free(obj->label);
    free(obj->str);
    free(obj->children);
    free(obj->priv);
    if (obj->sw_names) {
        for (uint16_t i = 0; i < obj->sw_count; i++) free(obj->sw_names[i]);
        free(obj->sw_names);
    }
    free(obj);
}

struct aes70_object *aes70_device_find(aes70_device_t *dev, uint32_t ono)
{
    for (size_t i = 0; i < dev->object_count; i++) {
        if (dev->objects[i]->ono == ono) return dev->objects[i];
    }
    return NULL;
}

/* ---- Method dispatch ---------------------------------------------------- *
 * A class is the path root->...->derived; the method's DefLevel selects which
 * level handles it. Walk to that level's handler (NULL => no methods there). */
aes70_status_t aes70_object_dispatch(struct aes70_object *obj, uint16_t level, uint16_t index,
                                     ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *param_count)
{
    const aes70_class_desc_t *d = aes70_class_for_kind(obj->kind);
    if (!d) return AES70_NOT_IMPLEMENTED;
    if (level < 1 || level > d->class_id_len) return AES70_BAD_METHOD;
    aes70_method_fn h = d->level_handlers[level - 1];
    if (!h) return AES70_BAD_METHOD;

    /* Access control: only gate methods that change state, and only for objects
     * that are secured or locked (the common, fast path skips this entirely). */
    if ((obj->secured || obj->lock_state != AES70_LOCK_NONE) &&
        aes70_method_is_write(obj, level, index)) {
        int ci = obj->dev->active_conn;
        bool privileged = (ci >= 0 && ci < CONFIG_AES70_MAX_CONNECTIONS &&
                           obj->dev->conns[ci].privileged);
        /* Secured objects: an unprivileged session may read but not write. */
        if (obj->secured && !privileged) return AES70_PERMISSION_DENIED;
        /* OcaLock: a write-blocking lock is enforced against every session but
         * the one that set it (Lock/Unlock themselves are gated the same way, so
         * a different controller cannot steal or release the lock). */
        if ((obj->lock_state == AES70_LOCK_NO_WRITE ||
             obj->lock_state == AES70_LOCK_NO_READ_WRITE) &&
            ci != obj->lock_owner) {
            return AES70_LOCKED;
        }
    }
    return h(obj, index, in, out, param_count);
}

/* Setter (state-changing) method indices per class. Kept explicit rather than
 * inferred so the access-control decision is auditable; add new setters here. */
bool aes70_method_is_write(const struct aes70_object *obj, uint16_t level, uint16_t index)
{
    if (level == 1)                         /* OcaRoot: lock control is a write. */
        return index == 3 || index == 4 || index == 6;
    if (level == 2)                         /* OcaWorker: SetEnabled, SetLabel. */
        return index == 2 || index == 9;
    switch (obj->kind) {                     /* concrete-class value setters */
    case AES70_K_GAIN:     case AES70_K_MUTE:      case AES70_K_POLARITY:
    case AES70_K_SWITCH:   case AES70_K_DELAY:     case AES70_K_FREQUENCY:
    case AES70_K_IDENTIFY:
        return level == 4 && index == 2;
    case AES70_K_BOOLEAN:  case AES70_K_INT32:     case AES70_K_UINT16:
    case AES70_K_UINT32:   case AES70_K_FLOAT32:   case AES70_K_STRING:
        return level == 5 && index == 2;
    case AES70_K_DYNAMICS:               /* even indices 4..26 are the setters */
        return level == 4 && index >= 4 && index <= 26 && (index % 2) == 0;
    case AES70_K_FILTER_CLASSICAL:
        return level == 4 && (index==2||index==4||index==6||index==8||index==10);
    case AES70_K_FILTER_PARAMETRIC:
        return level == 4 && (index==2||index==4||index==6||index==8||index==10);
    case AES70_K_PANBALANCE:
        return level == 4 && (index==2||index==4);
    case AES70_K_SIGNAL_GEN:
        return level == 4 && (index==2||index==4||index==6||index==8||index==10||index==12||index==14);
    default:
        return false;   /* sensors/managers/block: no securable setters */
    }
}

void aes70_locks_drop_conn(aes70_device_t *dev, int conn_idx)
{
    for (size_t i = 0; i < dev->object_count; i++) {
        struct aes70_object *o = dev->objects[i];
        if (o->lock_owner == conn_idx) {
            o->lock_state = AES70_LOCK_NONE;
            o->lock_owner = -1;
        }
    }
}

void aes70_object_set_secured(aes70_object_handle_t obj, bool secured)
{
    if (obj) obj->secured = secured;
}

bool aes70_object_is_secured(aes70_object_handle_t obj)
{
    return obj && obj->secured;
}

/* ---- Property value encoding (for getters and notifications) ------------ *
 * Each built-in class exposes one primary value property; encode_value writes
 * its current value in wire form. */
static void encode_value(struct aes70_object *obj, ocp1_wr_t *out)
{
    switch (obj->kind) {
    case AES70_K_GAIN:
    case AES70_K_FLOAT32:
    case AES70_K_LEVEL_SENSOR:
    case AES70_K_TEMPERATURE:
    case AES70_K_FREQUENCY:
        ocp1_wr_f32(out, (float)obj->num);
        break;
    case AES70_K_DELAY:
        ocp1_wr_f64(out, obj->num);
        break;
    case AES70_K_MUTE:
    case AES70_K_POLARITY:
        ocp1_wr_u8(out, (uint8_t)obj->num);     /* OcaMuteState / OcaPolarityState */
        break;
    case AES70_K_BOOLEAN:
    case AES70_K_IDENTIFY:
        ocp1_wr_u8(out, obj->num != 0.0 ? 1 : 0);
        break;
    case AES70_K_SWITCH:
    case AES70_K_UINT16:
        ocp1_wr_u16(out, (uint16_t)obj->num);
        break;
    case AES70_K_UINT32:
        ocp1_wr_u32(out, (uint32_t)obj->num);
        break;
    case AES70_K_INT32:
        ocp1_wr_u32(out, (uint32_t)(int32_t)obj->num);
        break;
    case AES70_K_STRING:
        ocp1_wr_string(out, obj->str ? obj->str : "");
        break;
    default:
        break;
    }
}

/* Map a kind to its primary property id; returns false for classes without one
 * (e.g. OcaBlock, managers). */
static bool primary_property(struct aes70_object *obj, uint16_t *level, uint16_t *index)
{
    switch (obj->kind) {
    case AES70_K_GAIN: case AES70_K_MUTE: case AES70_K_POLARITY:
    case AES70_K_SWITCH: case AES70_K_DELAY: case AES70_K_LEVEL_SENSOR:
    case AES70_K_TEMPERATURE: case AES70_K_FREQUENCY: case AES70_K_IDENTIFY:
        *level = 4; *index = 1; return true;   /* defined at OcaActuator/OcaSensor child level */
    case AES70_K_BOOLEAN: case AES70_K_INT32: case AES70_K_UINT16:
    case AES70_K_UINT32: case AES70_K_FLOAT32: case AES70_K_STRING:
        *level = 5; *index = 1; return true;   /* OcaBasicActuator child level */
    default:
        return false;
    }
}

bool aes70_object_encode_property(struct aes70_object *obj, uint16_t level, uint16_t index,
                                  ocp1_wr_t *out)
{
    /* OcaWorker common properties (those we raise notifications for). */
    if (level == 2 && index == 1) { ocp1_wr_u8(out, obj->enabled ? 1 : 0); return true; }
    if (level == 2 && index == 3) { ocp1_wr_string(out, obj->label ? obj->label : ""); return true; }

    /* OcaDeviceManager settable identity properties. */
    if (obj->kind == AES70_K_DEVICE_MANAGER) {
        if (level == 3 && index == 4) { ocp1_wr_string(out, obj->dev->device_name); return true; }
        if (level == 3 && index == 6) { ocp1_wr_string(out, obj->dev->device_role); return true; }
    }

    /* Multi-parameter DSP classes encode their own level-4 properties. */
    if (aes70_dsp_is_dsp_kind(obj->kind) && aes70_dsp_encode_property(obj, level, index, out)) {
        return true;
    }

    /* The class's primary value property. */
    uint16_t pl, pi;
    if (primary_property(obj, &pl, &pi) && level == pl && index == pi) {
        encode_value(obj, out);
        return true;
    }
    return false;
}

/* ---- Value commit (store + notify + optional app callback) -------------- */
void aes70_object_commit_num(struct aes70_object *obj, double v,
                             uint16_t prop_level, uint16_t prop_index, bool from_controller)
{
    aes70_lock(obj->dev);
    obj->num = v;
    aes70_unlock(obj->dev);

    aes70_notify_property_changed(obj->dev, obj, prop_level, prop_index);

    if (from_controller && obj->dev->cfg.on_control_changed) {
        obj->dev->cfg.on_control_changed(obj, obj->tag, obj->dev->cfg.user);
    }
}

void aes70_object_commit_str(struct aes70_object *obj, const char *s,
                             uint16_t prop_level, uint16_t prop_index, bool from_controller)
{
    char *copy = strdup(s ? s : "");
    if (!copy) return;
    aes70_lock(obj->dev);
    char *old = obj->str;
    obj->str = copy;
    aes70_unlock(obj->dev);
    free(old);

    aes70_notify_property_changed(obj->dev, obj, prop_level, prop_index);

    if (from_controller && obj->dev->cfg.on_control_changed) {
        obj->dev->cfg.on_control_changed(obj, obj->tag, obj->dev->cfg.user);
    }
}

/* ---- Application set-request (drained from cmd_q on the server task) ----- */
void aes70_object_apply_set(aes70_device_t *dev, const aes70_set_req_t *req)
{
    struct aes70_object *obj = aes70_device_find(dev, req->ono);
    if (!obj) return;

    if (aes70_dsp_is_dsp_kind(obj->kind)) {
        aes70_dsp_apply_set(obj, req);
        return;
    }

    uint16_t pl, pi;
    if (!primary_property(obj, &pl, &pi)) return;

    if (req->is_string) {
        aes70_object_commit_str(obj, req->str, pl, pi, false);
        return;
    }

    double v = req->num;
    switch (obj->kind) {
    case AES70_K_GAIN: case AES70_K_DELAY: case AES70_K_FLOAT32:
    case AES70_K_INT32: case AES70_K_UINT16: case AES70_K_UINT32:
    case AES70_K_LEVEL_SENSOR: case AES70_K_TEMPERATURE: case AES70_K_FREQUENCY:
        if (v < obj->num_min) v = obj->num_min;   /* app path clamps silently */
        if (v > obj->num_max) v = obj->num_max;
        break;
    case AES70_K_SWITCH:
        if (v < 0) v = 0;
        if (obj->sw_count && v > obj->sw_count - 1) v = obj->sw_count - 1;
        break;
    default:
        break;
    }
    aes70_object_commit_num(obj, v, pl, pi, false);
}

esp_err_t aes70_post_set(struct aes70_object *obj, const aes70_set_req_t *req)
{
    aes70_device_t *dev = obj->dev;
    if (!dev->running || !dev->cmd_q) return ESP_ERR_INVALID_STATE;
    if (xQueueSend(dev->cmd_q, req, 0) != pdTRUE) return ESP_ERR_NO_MEM;
    /* Nudge the server task's select() so the change is applied promptly. */
    if (dev->wake_send_sock >= 0) {
        uint8_t b = 1;
        send(dev->wake_send_sock, &b, 1, 0);
    }
    return ESP_OK;
}

/* ======================================================================== *
 *  Public object API (aes70_object.h): constructors, getters, setters.
 *  Constructors run under dev->lock so the registry/block lists stay coherent
 *  if the application builds the tree while the server task is running.
 * ======================================================================== */

uint32_t aes70_object_ono(aes70_object_handle_t obj) { return obj ? obj->ono : 0; }
void     aes70_object_set_tag(aes70_object_handle_t obj, uint32_t tag) { if (obj) obj->tag = tag; }
uint32_t aes70_object_get_tag(aes70_object_handle_t obj) { return obj ? obj->tag : 0; }

esp_err_t aes70_object_set_label(aes70_object_handle_t obj, const char *label)
{
    if (!obj) return ESP_ERR_INVALID_ARG;
    char *copy = strdup(label ? label : "");
    if (!copy) return ESP_ERR_NO_MEM;
    aes70_lock(obj->dev);
    char *old = obj->label; obj->label = copy;
    aes70_unlock(obj->dev);
    free(old);
    return ESP_OK;
}

static double clampd(double v, double lo, double hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static struct aes70_object *make_num(aes70_device_t *dev, aes70_kind_t kind,
                                     aes70_object_handle_t parent, const char *role,
                                     double mn, double mx, double init)
{
    aes70_lock(dev);
    struct aes70_object *o = aes70_object_alloc(dev, kind, parent, role);
    if (o) { o->num_min = mn; o->num_max = mx; o->num = clampd(init, mn, mx); }
    aes70_unlock(dev);
    return o;
}

/* Read/write helpers shared by the typed accessors. */
static double get_num(aes70_object_handle_t obj)
{
    if (!obj) return 0;
    aes70_lock(obj->dev);
    double v = obj->num;
    aes70_unlock(obj->dev);
    return v;
}
static esp_err_t set_num(aes70_object_handle_t obj, double v)
{
    if (!obj) return ESP_ERR_INVALID_ARG;
    aes70_set_req_t req = { .ono = obj->ono, .is_string = false, .num = v };
    return aes70_post_set(obj, &req);
}

/* ---- OcaBlock ----------------------------------------------------------- */
aes70_object_handle_t aes70_block_create(aes70_device_handle_t dev,
                                         aes70_object_handle_t parent, const char *role)
{
    aes70_lock(dev);
    struct aes70_object *o = aes70_object_alloc(dev, AES70_K_BLOCK, parent, role);
    aes70_unlock(dev);
    return o;
}

/* ---- OcaGain ------------------------------------------------------------ */
aes70_object_handle_t aes70_gain_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                        const char *role, float min_db, float max_db, float init_db)
{
    return make_num(dev, AES70_K_GAIN, parent, role, min_db, max_db, init_db);
}
float     aes70_gain_get(aes70_object_handle_t obj)            { return (float)get_num(obj); }
esp_err_t aes70_gain_set(aes70_object_handle_t obj, float db)  { return set_num(obj, db); }

/* ---- OcaMute ------------------------------------------------------------ */
aes70_object_handle_t aes70_mute_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                        const char *role, bool muted)
{
    return make_num(dev, AES70_K_MUTE, parent, role, AES70_MUTE_MUTED, AES70_MUTE_UNMUTED,
                    muted ? AES70_MUTE_MUTED : AES70_MUTE_UNMUTED);
}
bool      aes70_mute_get(aes70_object_handle_t obj) { return get_num(obj) == AES70_MUTE_MUTED; }
esp_err_t aes70_mute_set(aes70_object_handle_t obj, bool muted)
{
    return set_num(obj, muted ? AES70_MUTE_MUTED : AES70_MUTE_UNMUTED);
}

/* ---- OcaPolarity -------------------------------------------------------- */
aes70_object_handle_t aes70_polarity_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                            const char *role, bool inverted)
{
    /* OcaPolarityState: NonInverted=1, Inverted=2. */
    return make_num(dev, AES70_K_POLARITY, parent, role, 1, 2, inverted ? 2 : 1);
}
bool      aes70_polarity_get(aes70_object_handle_t obj) { return get_num(obj) == 2; }
esp_err_t aes70_polarity_set(aes70_object_handle_t obj, bool inverted)
{
    return set_num(obj, inverted ? 2 : 1);
}

/* ---- OcaSwitch ---------------------------------------------------------- */
aes70_object_handle_t aes70_switch_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                          const char *role, const char *const *position_names,
                                          uint16_t count, uint16_t init)
{
    aes70_lock(dev);
    struct aes70_object *o = aes70_object_alloc(dev, AES70_K_SWITCH, parent, role);
    if (o) {
        o->num_min = 0;
        o->num_max = count ? count - 1 : 0;
        o->num     = (count && init < count) ? init : 0;
        if (count && position_names) {
            o->sw_names = calloc(count, sizeof(char *));
            if (o->sw_names) {
                o->sw_count = count;
                for (uint16_t i = 0; i < count; i++) {
                    o->sw_names[i] = strdup(position_names[i] ? position_names[i] : "");
                }
            }
        }
    }
    aes70_unlock(dev);
    return o;
}
uint16_t  aes70_switch_get(aes70_object_handle_t obj)               { return (uint16_t)get_num(obj); }
esp_err_t aes70_switch_set(aes70_object_handle_t obj, uint16_t pos) { return set_num(obj, pos); }

/* ---- OcaDelay ----------------------------------------------------------- */
aes70_object_handle_t aes70_delay_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                         const char *role, double min_s, double max_s, double init_s)
{
    return make_num(dev, AES70_K_DELAY, parent, role, min_s, max_s, init_s);
}
double    aes70_delay_get(aes70_object_handle_t obj)               { return get_num(obj); }
esp_err_t aes70_delay_set(aes70_object_handle_t obj, double s)     { return set_num(obj, s); }

/* ---- OcaBooleanActuator ------------------------------------------------- */
aes70_object_handle_t aes70_boolean_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                           const char *role, bool init)
{
    return make_num(dev, AES70_K_BOOLEAN, parent, role, 0, 1, init ? 1 : 0);
}
bool      aes70_boolean_get(aes70_object_handle_t obj)             { return get_num(obj) != 0; }
esp_err_t aes70_boolean_set(aes70_object_handle_t obj, bool v)     { return set_num(obj, v ? 1 : 0); }

/* ---- OcaFloat32Actuator ------------------------------------------------- */
aes70_object_handle_t aes70_float_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                         const char *role, float min, float max, float init)
{
    return make_num(dev, AES70_K_FLOAT32, parent, role, min, max, init);
}
float     aes70_float_get(aes70_object_handle_t obj)              { return (float)get_num(obj); }
esp_err_t aes70_float_set(aes70_object_handle_t obj, float v)     { return set_num(obj, v); }

/* ---- OcaUint16Actuator / OcaUint32Actuator / OcaInt32Actuator ----------- */
aes70_object_handle_t aes70_uint16_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                          const char *role, uint16_t min, uint16_t max, uint16_t init)
{
    return make_num(dev, AES70_K_UINT16, parent, role, min, max, init);
}
uint16_t  aes70_uint16_get(aes70_object_handle_t obj)             { return (uint16_t)get_num(obj); }
esp_err_t aes70_uint16_set(aes70_object_handle_t obj, uint16_t v) { return set_num(obj, v); }

aes70_object_handle_t aes70_uint32_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                          const char *role, uint32_t min, uint32_t max, uint32_t init)
{
    return make_num(dev, AES70_K_UINT32, parent, role, min, max, init);
}
uint32_t  aes70_uint32_get(aes70_object_handle_t obj)             { return (uint32_t)get_num(obj); }
esp_err_t aes70_uint32_set(aes70_object_handle_t obj, uint32_t v) { return set_num(obj, v); }

aes70_object_handle_t aes70_int32_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                         const char *role, int32_t min, int32_t max, int32_t init)
{
    return make_num(dev, AES70_K_INT32, parent, role, min, max, init);
}
int32_t   aes70_int32_get(aes70_object_handle_t obj)             { return (int32_t)get_num(obj); }
esp_err_t aes70_int32_set(aes70_object_handle_t obj, int32_t v)  { return set_num(obj, v); }

/* ---- OcaStringActuator -------------------------------------------------- */
aes70_object_handle_t aes70_string_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                          const char *role, const char *init, uint16_t max_len)
{
    aes70_lock(dev);
    struct aes70_object *o = aes70_object_alloc(dev, AES70_K_STRING, parent, role);
    if (o) {
        o->str_max = max_len;
        o->str = strdup(init ? init : "");
    }
    aes70_unlock(dev);
    return o;
}
const char *aes70_string_get(aes70_object_handle_t obj)
{
    if (!obj) return "";
    aes70_lock(obj->dev);
    const char *s = obj->str ? obj->str : "";
    aes70_unlock(obj->dev);
    return s;
}
esp_err_t aes70_string_set(aes70_object_handle_t obj, const char *value)
{
    if (!obj) return ESP_ERR_INVALID_ARG;
    aes70_set_req_t req = { .ono = obj->ono, .is_string = true };
    strlcpy(req.str, value ? value : "", sizeof(req.str));
    return aes70_post_set(obj, &req);
}

/* ---- OcaLevelSensor ----------------------------------------------------- */
aes70_object_handle_t aes70_level_sensor_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                                const char *role, float min_db, float max_db)
{
    struct aes70_object *o = make_num(dev, AES70_K_LEVEL_SENSOR, parent, role, min_db, max_db, min_db);
    if (o) o->reading_state = 1;   /* OcaSensorReadingState: Valid */
    return o;
}
esp_err_t aes70_level_sensor_report(aes70_object_handle_t obj, float db)
{
    return set_num(obj, db);
}

/* ---- OcaFrequencyActuator (Hz) ------------------------------------------ */
aes70_object_handle_t aes70_frequency_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                             const char *role, float min_hz, float max_hz, float init_hz)
{
    return make_num(dev, AES70_K_FREQUENCY, parent, role, min_hz, max_hz, init_hz);
}
float     aes70_frequency_get(aes70_object_handle_t obj)            { return (float)get_num(obj); }
esp_err_t aes70_frequency_set(aes70_object_handle_t obj, float hz)  { return set_num(obj, hz); }

/* ---- OcaTemperatureSensor (deg C, read-only; device reports) ------------ */
aes70_object_handle_t aes70_temperature_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                               const char *role, float min_c, float max_c)
{
    struct aes70_object *o = make_num(dev, AES70_K_TEMPERATURE, parent, role, min_c, max_c, min_c);
    if (o) o->reading_state = 1;   /* OcaSensorReadingState: Valid */
    return o;
}
esp_err_t aes70_temperature_report(aes70_object_handle_t obj, float celsius)
{
    return set_num(obj, celsius);
}

/* ---- OcaIdentificationActuator (identify on/off) ------------------------ */
aes70_object_handle_t aes70_identify_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                            const char *role)
{
    return make_num(dev, AES70_K_IDENTIFY, parent, role, 0, 1, 0);
}
bool      aes70_identify_get(aes70_object_handle_t obj)              { return get_num(obj) != 0; }
esp_err_t aes70_identify_set(aes70_object_handle_t obj, bool active) { return set_num(obj, active ? 1 : 0); }
