/*
 * aes70_subscription.c - OcaSubscriptionManager (1.3.4) and the PropertyChanged
 * notification emitter.
 *
 * A controller subscribes to an object's PropertyChanged event with
 * AddSubscription (EV1, method 3.1). When that object's value changes the device
 * sends an OCP.1 Notification PDU back over the same TCP connection. EV2
 * AddSubscription2 (3.8) is accepted and delivered with the same notification
 * (the EV1/EV2 difference is the subscriber-method/context bookkeeping, which we
 * carry transparently).
 *
 * SPDX-License-Identifier: MIT
 */
#include <string.h>

#include "esp_log.h"
#include "aes70_internal.h"

static const char *TAG = "aes70.sub";

/* ---- Subscription table (server-task only) ------------------------------ */
static aes70_subscription_t *find_slot(aes70_device_t *dev)
{
    for (size_t i = 0; i < CONFIG_AES70_MAX_SUBSCRIPTIONS; i++) {
        if (!dev->subs[i].in_use) return &dev->subs[i];
    }
    return NULL;
}

static bool same_sub(const aes70_subscription_t *s, int conn, uint32_t emitter,
                     uint16_t ev_l, uint16_t ev_i)
{
    return s->in_use && s->conn_idx == conn && s->emitter_ono == emitter &&
           s->event_level == ev_l && s->event_index == ev_i;
}

static aes70_status_t add_subscription(aes70_device_t *dev, ocp1_rd_t *in, bool ev2)
{
    uint32_t emitter   = ocp1_rd_u32(in);
    uint16_t ev_level  = ocp1_rd_u16(in);
    uint16_t ev_index  = ocp1_rd_u16(in);

    uint32_t sub_ono = 0;
    uint16_t m_level = 0, m_index = 0;
    uint8_t  ctx[8]; uint16_t ctx_len = 0;

    if (!ev2) {
        /* EV1: Subscriber (OcaMethod) + Context (OcaBlob) precede DeliveryMode. */
        sub_ono = ocp1_rd_u32(in);
        m_level = ocp1_rd_u16(in);
        m_index = ocp1_rd_u16(in);
        uint16_t blen = 0;
        const uint8_t *b = ocp1_rd_blob(in, &blen);
        if (b && blen) {
            ctx_len = blen > sizeof(ctx) ? sizeof(ctx) : blen;
            memcpy(ctx, b, ctx_len);
        }
    }
    uint8_t delivery = ocp1_rd_u8(in);
    /* DestinationInformation (OcaNetworkAddress) follows but is unused: OCP.1
     * delivers notifications back over the originating TCP connection. */
    if (in->err) return AES70_BAD_FORMAT;

    int conn = dev->active_conn;
    if (conn < 0) return AES70_INVALID_REQUEST;

    /* Deduplicate: identical (conn, emitter, event) is a no-op success. */
    for (size_t i = 0; i < CONFIG_AES70_MAX_SUBSCRIPTIONS; i++) {
        if (same_sub(&dev->subs[i], conn, emitter, ev_level, ev_index)) return AES70_OK;
    }

    aes70_subscription_t *s = find_slot(dev);
    if (!s) { ESP_LOGW(TAG, "subscription table full"); return AES70_OUT_OF_MEMORY; }

    memset(s, 0, sizeof(*s));
    s->in_use        = true;
    s->conn_idx      = conn;
    s->emitter_ono   = emitter;
    s->event_level   = ev_level;
    s->event_index   = ev_index;
    s->sub_ono       = sub_ono;
    s->sub_method_level = m_level;
    s->sub_method_index = m_index;
    s->context_len   = ctx_len;
    if (ctx_len) memcpy(s->context, ctx, ctx_len);
    s->delivery_mode = delivery;
    return AES70_OK;
}

static aes70_status_t remove_subscription(aes70_device_t *dev, ocp1_rd_t *in, bool ev2)
{
    uint32_t emitter  = ocp1_rd_u32(in);
    uint16_t ev_level = ocp1_rd_u16(in);
    uint16_t ev_index = ocp1_rd_u16(in);
    if (in->err) return AES70_BAD_FORMAT;
    (void)ev2;

    int conn = dev->active_conn;
    for (size_t i = 0; i < CONFIG_AES70_MAX_SUBSCRIPTIONS; i++) {
        if (same_sub(&dev->subs[i], conn, emitter, ev_level, ev_index)) {
            dev->subs[i].in_use = false;
        }
    }
    return AES70_OK;
}

aes70_status_t aes70_submgr_dispatch(struct aes70_object *obj, uint16_t idx,
                                     ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc)
{
    aes70_device_t *dev = obj->dev;
    (void)out;
    switch (idx) {
    case 1: /* AddSubscription (EV1) */
        return add_subscription(dev, in, false);
    case 2: /* RemoveSubscription (EV1) */
        return remove_subscription(dev, in, false);
    case 3: /* DisableNotifications */
        dev->notifications_disabled = true;
        return AES70_OK;
    case 4: /* ReEnableNotifications */
        dev->notifications_disabled = false;
        return AES70_OK;
    case 8: /* AddSubscription2 (EV2) */
        return add_subscription(dev, in, true);
    case 9: /* RemoveSubscription2 (EV2) */
        return remove_subscription(dev, in, true);
    default:
        return AES70_NOT_IMPLEMENTED;
    }
}

/* ---- Notification emission ---------------------------------------------- */
void aes70_subscriptions_drop_conn(aes70_device_t *dev, int conn_idx)
{
    for (size_t i = 0; i < CONFIG_AES70_MAX_SUBSCRIPTIONS; i++) {
        if (dev->subs[i].in_use && dev->subs[i].conn_idx == conn_idx) {
            dev->subs[i].in_use = false;
        }
    }
}

/* Build one OCP.1 Notification PDU for a property change and send it to the
 * subscriber. Layout (big-endian; see aes70_ocp1.h):
 *   header(10) | notificationSize u32 | targetONo u32 | methodID(4) |
 *   paramCount u8=2 | Context(OcaBlob) |
 *   EventData{ emitterONo u32, eventID(1,1), PropertyID, Value, ChangeType u8 } */
static void send_notification(aes70_device_t *dev, const aes70_subscription_t *s,
                              struct aes70_object *obj, uint16_t prop_level, uint16_t prop_index)
{
    ocp1_wr_t w;
    ocp1_wr_init(&w, dev->txbuf, sizeof(dev->txbuf));

    ocp1_wr_u8(&w, OCP1_SYNC_VAL);
    ocp1_wr_u16(&w, OCP1_PROTO_VERSION);
    size_t pdusize_at = w.off; ocp1_wr_u32(&w, 0);   /* PduSize placeholder */
    ocp1_wr_u8(&w, OCP1_NTF);
    ocp1_wr_u16(&w, 1);                              /* MessageCount = 1 */

    size_t msgsize_at = w.off; ocp1_wr_u32(&w, 0);   /* notificationSize placeholder */
    ocp1_wr_u32(&w, s->sub_ono);                     /* targetONo (subscriber method) */
    ocp1_wr_u16(&w, s->sub_method_level);
    ocp1_wr_u16(&w, s->sub_method_index);
    ocp1_wr_u8(&w, 2);                               /* parameterCount: Context + EventData */

    ocp1_wr_blob(&w, s->context, s->context_len);    /* Context (OcaBlob) */

    ocp1_wr_u32(&w, obj->ono);                       /* EventData: emitter ONo */
    ocp1_wr_u16(&w, OCP1_EVENT_PROPERTY_CHANGED_LEVEL);
    ocp1_wr_u16(&w, OCP1_EVENT_PROPERTY_CHANGED_INDEX);
    ocp1_wr_u16(&w, prop_level);                     /* PropertyID */
    ocp1_wr_u16(&w, prop_index);
    aes70_object_encode_property(obj, prop_level, prop_index, &w);  /* PropertyValue */
    ocp1_wr_u8(&w, AES70_CHANGE_CURRENT);            /* ChangeType */

    if (w.err) { ESP_LOGW(TAG, "notification overflow (raise AES70_TX_BUFFER_SIZE)"); return; }

    /* Back-patch the two length fields now that the body is complete. */
    ocp1_wr32(dev->txbuf + msgsize_at, (uint32_t)(w.off - msgsize_at));      /* inclusive */
    ocp1_wr32(dev->txbuf + pdusize_at, (uint32_t)(w.off - OCP1_PDUSIZE_PREFIX));
    aes70_conn_send(dev, s->conn_idx, dev->txbuf, w.off);
}

void aes70_notify_property_changed(aes70_device_t *dev, struct aes70_object *obj,
                                   uint16_t prop_level, uint16_t prop_index)
{
    if (dev->notifications_disabled) return;
    for (size_t i = 0; i < CONFIG_AES70_MAX_SUBSCRIPTIONS; i++) {
        aes70_subscription_t *s = &dev->subs[i];
        if (!s->in_use) continue;
        if (s->emitter_ono != obj->ono) continue;
        if (s->event_level != OCP1_EVENT_PROPERTY_CHANGED_LEVEL ||
            s->event_index != OCP1_EVENT_PROPERTY_CHANGED_INDEX) continue;
        send_notification(dev, s, obj, prop_level, prop_index);
    }
}
