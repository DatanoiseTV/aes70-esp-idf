/*
 * aes70_grouper.c - OcaGrouper (ClassID 1.2.2), an OcaAgent that groups control
 * objects so one write fans out to many.
 *
 * Model: the grouper has a fixed citizen class. AddGroup creates a *proxy*
 * object of that class (a real object in the tree); writing the proxy applies
 * its value to every citizen enrolled in that group. Citizens are referenced by
 * OcaOPath; only local citizens (empty HostID) are supported here -- a remote
 * citizen (HostID set, i.e. on another device) is rejected, because acting on it
 * needs an outbound controller this responder-only component does not have.
 *
 * SPDX-License-Identifier: MIT
 */
#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "aes70_internal.h"

#define G_MAX_GROUPS   16
#define G_MAX_CITIZENS 32
#define G_CBYTES       ((G_MAX_CITIZENS + 7) / 8)

/* OcaGrouperMode */
#define G_MODE_HIERARCHICAL 1
#define G_MODE_PEER_TO_PEER 2

typedef struct { bool used; char name[40]; uint32_t proxy_ono; } g_group_t;
typedef struct { bool used; uint32_t ono; bool online; }        g_citizen_t;

typedef struct {
    aes70_kind_t citizen_kind;
    bool         actuator;                 /* ActuatorOrSensor: true = actuator */
    uint8_t      mode;
    g_group_t    groups[G_MAX_GROUPS];
    g_citizen_t  citizens[G_MAX_CITIZENS];
    uint8_t      enroll[G_MAX_GROUPS][G_CBYTES];   /* enroll[g] bit c => citizen c in group g */
} grouper_priv_t;

static inline bool enrolled(const grouper_priv_t *g, int gi, int ci)
{
    return (g->enroll[gi][ci >> 3] >> (ci & 7)) & 1;
}
static inline void set_enroll(grouper_priv_t *g, int gi, int ci, bool on)
{
    if (on) g->enroll[gi][ci >> 3] |=  (uint8_t)(1u << (ci & 7));
    else    g->enroll[gi][ci >> 3] &= (uint8_t)~(1u << (ci & 7));
}

aes70_object_handle_t aes70_grouper_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                           const char *role,
                                           aes70_grouper_citizen_class_t citizen_class)
{
    aes70_kind_t kind;
    switch (citizen_class) {
    case AES70_GROUPER_MUTE:    kind = AES70_K_MUTE; break;
    case AES70_GROUPER_BOOLEAN: kind = AES70_K_BOOLEAN; break;
    case AES70_GROUPER_GAIN:
    default:                    kind = AES70_K_GAIN; break;
    }
    struct aes70_object *o = aes70_object_alloc((aes70_device_t *)dev, AES70_K_GROUPER, parent, role);
    if (!o) return NULL;
    aes70_grouper_init(o, kind);
    return (aes70_object_handle_t)o;
}

void aes70_grouper_init(struct aes70_object *obj, aes70_kind_t citizen_kind)
{
    grouper_priv_t *g = calloc(1, sizeof(*g));
    if (!g) return;
    g->citizen_kind = citizen_kind;
    g->actuator     = true;
    g->mode         = G_MODE_PEER_TO_PEER;
    obj->priv = g;
}

/* Apply sensible bounds to a freshly-created proxy so SetX passes range checks. */
static void init_proxy(struct aes70_object *proxy)
{
    switch (proxy->kind) {
    case AES70_K_GAIN:    proxy->num_min = -200.0; proxy->num_max = 200.0; proxy->num = 0.0; break;
    case AES70_K_MUTE:    proxy->num = AES70_MUTE_UNMUTED; break;
    case AES70_K_BOOLEAN: proxy->num = 0.0; break;
    default: break;
    }
}

/* ---- Wire helpers ------------------------------------------------------- */
static void wr_group(ocp1_wr_t *o, uint16_t index, const g_group_t *grp)
{
    ocp1_wr_u16(o, index);
    ocp1_wr_string(o, grp->name);
    ocp1_wr_u32(o, grp->proxy_ono);
}
static void wr_citizen(ocp1_wr_t *o, uint16_t index, const g_citizen_t *c)
{
    ocp1_wr_u16(o, index);
    ocp1_wr_u16(o, 0);              /* OcaOPath.HostID = empty OcaBlob (local) */
    ocp1_wr_u32(o, c->ono);        /* OcaOPath.ONo */
    ocp1_wr_u8(o, c->online ? 1 : 0);
}

/* ---- Method dispatch (OcaGrouper, DefLevel 3) --------------------------- */
aes70_status_t aes70_grouper_dispatch(struct aes70_object *obj, uint16_t idx,
                                      ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc)
{
    grouper_priv_t *g = obj->priv;
    if (!g) return AES70_DEVICE_ERROR;

    switch (idx) {
    case 1: {  /* AddGroup(OcaString name) -> (OcaUint16 index, OcaUint32 proxyONo) */
        char name[40];
        ocp1_rd_string(in, name, sizeof(name));
        if (in->err) return AES70_BAD_FORMAT;
        int slot = -1;
        for (int i = 0; i < G_MAX_GROUPS; i++) if (!g->groups[i].used) { slot = i; break; }
        if (slot < 0) return AES70_OUT_OF_MEMORY;
        struct aes70_object *proxy = aes70_object_alloc(obj->dev, g->citizen_kind,
                                                        (aes70_object_handle_t)obj->dev->root_block, name);
        if (!proxy) return AES70_PROCESSING_FAILED;
        init_proxy(proxy);
        g->groups[slot].used = true;
        strlcpy(g->groups[slot].name, name, sizeof(g->groups[slot].name));
        g->groups[slot].proxy_ono = proxy->ono;
        memset(g->enroll[slot], 0, G_CBYTES);
        ocp1_wr_u16(out, (uint16_t)(slot + 1));
        ocp1_wr_u32(out, proxy->ono);
        *pc = 2;
        return AES70_OK;
    }
    case 2: {  /* DeleteGroup(OcaUint16 index) */
        uint16_t gi = ocp1_rd_u16(in);
        if (in->err) return AES70_BAD_FORMAT;
        if (gi < 1 || gi > G_MAX_GROUPS || !g->groups[gi - 1].used) return AES70_PARAMETER_OUT_OF_RANGE;
        memset(&g->groups[gi - 1], 0, sizeof(g->groups[gi - 1]));
        memset(g->enroll[gi - 1], 0, G_CBYTES);   /* proxy object is left orphaned in the tree */
        return AES70_OK;
    }
    case 3: {  /* GetGroupCount -> OcaUint16 */
        uint16_t n = 0;
        for (int i = 0; i < G_MAX_GROUPS; i++) if (g->groups[i].used) n++;
        ocp1_wr_u16(out, n); *pc = 1; return AES70_OK;
    }
    case 4: {  /* GetGroupList -> OcaList<OcaGrouperGroup> */
        uint16_t n = 0;
        for (int i = 0; i < G_MAX_GROUPS; i++) if (g->groups[i].used) n++;
        ocp1_wr_u16(out, n);
        for (int i = 0; i < G_MAX_GROUPS; i++)
            if (g->groups[i].used) wr_group(out, (uint16_t)(i + 1), &g->groups[i]);
        *pc = 1; return AES70_OK;
    }
    case 5: {  /* AddCitizen(OcaGrouperCitizen) -> OcaUint16 index */
        (void)ocp1_rd_u16(in);                         /* Index (ignored on add) */
        uint16_t host_len; (void)ocp1_rd_blob(in, &host_len);   /* OcaOPath.HostID */
        uint32_t ono = ocp1_rd_u32(in);                 /* OcaOPath.ONo */
        (void)ocp1_rd_u8(in);                           /* Online (we decide) */
        if (in->err) return AES70_BAD_FORMAT;
        if (host_len != 0) return AES70_NOT_IMPLEMENTED; /* remote citizen unsupported */
        struct aes70_object *c = aes70_device_find(obj->dev, ono);
        if (!c) return AES70_BAD_ONO;
        if (c->kind != g->citizen_kind) return AES70_PARAMETER_ERROR;
        int slot = -1;
        for (int i = 0; i < G_MAX_CITIZENS; i++) if (!g->citizens[i].used) { slot = i; break; }
        if (slot < 0) return AES70_OUT_OF_MEMORY;
        g->citizens[slot].used = true;
        g->citizens[slot].ono = ono;
        g->citizens[slot].online = true;
        ocp1_wr_u16(out, (uint16_t)(slot + 1)); *pc = 1;
        return AES70_OK;
    }
    case 6: {  /* DeleteCitizen(OcaUint16 index) */
        uint16_t ci = ocp1_rd_u16(in);
        if (in->err) return AES70_BAD_FORMAT;
        if (ci < 1 || ci > G_MAX_CITIZENS || !g->citizens[ci - 1].used) return AES70_PARAMETER_OUT_OF_RANGE;
        g->citizens[ci - 1].used = false;
        for (int i = 0; i < G_MAX_GROUPS; i++) set_enroll(g, i, ci - 1, false);
        return AES70_OK;
    }
    case 7: {  /* GetCitizenCount -> OcaUint16 */
        uint16_t n = 0;
        for (int i = 0; i < G_MAX_CITIZENS; i++) if (g->citizens[i].used) n++;
        ocp1_wr_u16(out, n); *pc = 1; return AES70_OK;
    }
    case 8: {  /* GetCitizenList -> OcaList<OcaGrouperCitizen> */
        uint16_t n = 0;
        for (int i = 0; i < G_MAX_CITIZENS; i++) if (g->citizens[i].used) n++;
        ocp1_wr_u16(out, n);
        for (int i = 0; i < G_MAX_CITIZENS; i++)
            if (g->citizens[i].used) wr_citizen(out, (uint16_t)(i + 1), &g->citizens[i]);
        *pc = 1; return AES70_OK;
    }
    case 9: {  /* GetEnrollment(OcaGrouperEnrollment) -> OcaBoolean */
        uint16_t gi = ocp1_rd_u16(in), ci = ocp1_rd_u16(in);
        if (in->err) return AES70_BAD_FORMAT;
        if (gi < 1 || gi > G_MAX_GROUPS || ci < 1 || ci > G_MAX_CITIZENS) return AES70_PARAMETER_OUT_OF_RANGE;
        ocp1_wr_u8(out, enrolled(g, gi - 1, ci - 1) ? 1 : 0); *pc = 1;
        return AES70_OK;
    }
    case 10: { /* SetEnrollment(OcaGrouperEnrollment, OcaBoolean) */
        uint16_t gi = ocp1_rd_u16(in), ci = ocp1_rd_u16(in);
        uint8_t on = ocp1_rd_u8(in);
        if (in->err) return AES70_BAD_FORMAT;
        if (gi < 1 || gi > G_MAX_GROUPS || ci < 1 || ci > G_MAX_CITIZENS) return AES70_PARAMETER_OUT_OF_RANGE;
        if (!g->groups[gi - 1].used || !g->citizens[ci - 1].used) return AES70_PARAMETER_ERROR;
        set_enroll(g, gi - 1, ci - 1, on != 0);
        return AES70_OK;
    }
    case 11: { /* GetGroupMemberList(OcaUint16 groupIndex) -> OcaList<OcaGrouperCitizen> */
        uint16_t gi = ocp1_rd_u16(in);
        if (in->err) return AES70_BAD_FORMAT;
        if (gi < 1 || gi > G_MAX_GROUPS) return AES70_PARAMETER_OUT_OF_RANGE;
        uint16_t n = 0;
        for (int i = 0; i < G_MAX_CITIZENS; i++) if (g->citizens[i].used && enrolled(g, gi - 1, i)) n++;
        ocp1_wr_u16(out, n);
        for (int i = 0; i < G_MAX_CITIZENS; i++)
            if (g->citizens[i].used && enrolled(g, gi - 1, i)) wr_citizen(out, (uint16_t)(i + 1), &g->citizens[i]);
        *pc = 1; return AES70_OK;
    }
    case 12:   ocp1_wr_u8(out, g->actuator ? 1 : 0); *pc = 1; return AES70_OK;  /* GetActuatorOrSensor */
    case 13: { uint8_t v = ocp1_rd_u8(in); if (in->err) return AES70_BAD_FORMAT; g->actuator = (v != 0); return AES70_OK; }
    case 14:   ocp1_wr_u8(out, g->mode); *pc = 1; return AES70_OK;              /* GetMode */
    case 15: { uint8_t m = ocp1_rd_u8(in); if (in->err) return AES70_BAD_FORMAT;
               if (m != G_MODE_HIERARCHICAL && m != G_MODE_PEER_TO_PEER) return AES70_PARAMETER_OUT_OF_RANGE;
               g->mode = m; return AES70_OK; }
    default:   return AES70_BAD_METHOD;
    }
}

/* ---- Proxy write fan-out ------------------------------------------------ *
 * Called after any controller value commit. If the written object is a group
 * proxy, copy its primary value to every enrolled (online, local) citizen.
 * Citizens are written directly (not via the command path), so this never
 * recurses even if a citizen is itself a proxy. */
void aes70_grouper_on_commit(aes70_device_t *dev, struct aes70_object *written)
{
    uint16_t pl, pi;
    if (!aes70_object_primary_property(written, &pl, &pi)) return;

    for (size_t i = 0; i < dev->object_count; i++) {
        struct aes70_object *go = dev->objects[i];
        if (go->kind != AES70_K_GROUPER || !go->priv) continue;
        grouper_priv_t *g = go->priv;
        for (int gi = 0; gi < G_MAX_GROUPS; gi++) {
            if (!g->groups[gi].used || g->groups[gi].proxy_ono != written->ono) continue;
            for (int ci = 0; ci < G_MAX_CITIZENS; ci++) {
                if (!g->citizens[ci].used || !g->citizens[ci].online || !enrolled(g, gi, ci)) continue;
                struct aes70_object *c = aes70_device_find(dev, g->citizens[ci].ono);
                if (!c || c->kind != written->kind || c == written) continue;
                aes70_lock(dev);
                c->num = written->num;
                aes70_unlock(dev);
                aes70_notify_property_changed(dev, c, pl, pi);
                if (dev->cfg.on_control_changed) dev->cfg.on_control_changed(c, c->tag, dev->cfg.user);
            }
        }
    }
}
