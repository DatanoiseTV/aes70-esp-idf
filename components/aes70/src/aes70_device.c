/*
 * aes70_device.c - Device lifecycle (create/start/stop) and the OCP.1 command
 * router that turns an inbound Command PDU into a Response PDU.
 *
 * SPDX-License-Identifier: MIT
 */
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "aes70_internal.h"

static const char *TAG = "aes70";

/* ======================================================================== *
 *  Command routing
 * ======================================================================== */

/* Dispatch one command body and append its Response message to `resp`.
 * The response message is: size(4) | handle(4) | status(1) | paramCount(1) | params. */
static void append_response(aes70_device_t *dev, ocp1_wr_t *resp, uint32_t handle,
                            uint32_t target, uint16_t m_level, uint16_t m_index, ocp1_rd_t *params)
{
    size_t msg_at    = resp->off;  ocp1_wr_u32(resp, 0);  /* size placeholder */
    ocp1_wr_u32(resp, handle);
    size_t status_at = resp->off;  ocp1_wr_u8(resp, 0);   /* status placeholder */
    size_t pc_at     = resp->off;  ocp1_wr_u8(resp, 0);   /* paramCount placeholder */

    aes70_status_t st;
    uint8_t pc = 0;
    struct aes70_object *obj = aes70_device_find(dev, target);
    if (!obj) {
        st = AES70_BAD_ONO;
    } else {
        st = aes70_object_dispatch(obj, m_level, m_index, params, resp, &pc);
    }
    if (st != AES70_OK) {            /* discard any partially-written params */
        if (!resp->err) resp->off = pc_at + 1;
        pc = 0;
    }
    if (!resp->err) {
        resp->p[status_at] = (uint8_t)st;
        resp->p[pc_at]     = pc;
        ocp1_wr32(resp->p + msg_at, (uint32_t)(resp->off - msg_at));  /* inclusive */
    }
}

size_t aes70_route_command_pdu(aes70_device_t *dev, int conn_idx, ocp1_msg_type_t type,
                               uint16_t msg_count, const uint8_t *data, size_t data_len,
                               uint8_t *out, size_t out_cap)
{
    bool want_resp = (type == OCP1_CMD_RRQ);

    ocp1_wr_t resp;
    ocp1_wr_init(&resp, out, out_cap);
    size_t pdusize_at = 0;
    if (want_resp) {
        ocp1_wr_u8(&resp, OCP1_SYNC_VAL);
        ocp1_wr_u16(&resp, OCP1_PROTO_VERSION);
        pdusize_at = resp.off; ocp1_wr_u32(&resp, 0);     /* PduSize placeholder */
        ocp1_wr_u8(&resp, OCP1_RSP);
        ocp1_wr_u16(&resp, msg_count);                    /* one response per command */
    }

    ocp1_rd_t rd;
    ocp1_rd_init(&rd, data, data_len);
    uint8_t scratch[64];                                  /* discards output for plain CMD */

    for (uint16_t i = 0; i < msg_count; i++) {
        uint32_t cmd_size = ocp1_rd_u32(&rd);
        size_t   body_at  = rd.off;                       /* first byte after the size field */
        /* A command is >= 17 bytes (size+handle+ono+methodID+paramCount) and its
         * body (cmd_size - 4) must fit in what remains of this PDU. */
        if (rd.err || cmd_size < 17 || (cmd_size - 4) > (data_len - body_at)) break;
        size_t next = body_at + (cmd_size - 4);

        uint32_t handle  = ocp1_rd_u32(&rd);
        uint32_t target  = ocp1_rd_u32(&rd);
        uint16_t m_level = ocp1_rd_u16(&rd);
        uint16_t m_index = ocp1_rd_u16(&rd);
        (void)ocp1_rd_u8(&rd);                            /* parameterCount (informational) */
        if (rd.err) break;

        size_t params_len = cmd_size - 17;
        ocp1_rd_t pin;
        ocp1_rd_init(&pin, data + rd.off,
                     params_len <= (data_len - rd.off) ? params_len : (data_len - rd.off));

        dev->active_conn = conn_idx;
        if (want_resp) {
            append_response(dev, &resp, handle, target, m_level, m_index, &pin);
        } else {
            struct aes70_object *obj = aes70_device_find(dev, target);
            if (obj) {
                ocp1_wr_t sink; ocp1_wr_init(&sink, scratch, sizeof(scratch));
                uint8_t pc = 0;
                aes70_object_dispatch(obj, m_level, m_index, &pin, &sink, &pc);
            }
        }
        rd.off = next <= rd.len ? next : rd.len;
    }
    dev->active_conn = -1;

    if (!want_resp) return 0;
    if (resp.err) {
        ESP_LOGW(TAG, "response overflow (raise AES70_TX_BUFFER_SIZE)");
        return 0;
    }
    ocp1_wr32(resp.p + pdusize_at, (uint32_t)(resp.off - OCP1_PDUSIZE_PREFIX));
    return resp.off;
}

/* ======================================================================== *
 *  Lifecycle
 * ======================================================================== */
void aes70_device_config_default(aes70_device_config_t *cfg)
{
    if (!cfg) return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->enable_mdns   = true;
    cfg->task_core_id  = -1;
}

esp_err_t aes70_device_start(const aes70_device_config_t *cfg, aes70_device_handle_t *out)
{
    if (!cfg || !out || !cfg->device_name || !cfg->manufacturer || !cfg->model) {
        return ESP_ERR_INVALID_ARG;
    }

    aes70_device_t *dev = calloc(1, sizeof(*dev));
    if (!dev) return ESP_ERR_NO_MEM;

    dev->cfg = *cfg;
    strlcpy(dev->device_name,  cfg->device_name,                       sizeof(dev->device_name));
    strlcpy(dev->manufacturer, cfg->manufacturer,                      sizeof(dev->manufacturer));
    strlcpy(dev->model,        cfg->model,                             sizeof(dev->model));
    strlcpy(dev->version,      cfg->version       ? cfg->version : "", sizeof(dev->version));
    strlcpy(dev->serial,       cfg->serial_number ? cfg->serial_number : "", sizeof(dev->serial));
    dev->device_role[0] = '\0';
    if (cfg->model_guid) memcpy(dev->model_guid, cfg->model_guid, sizeof(dev->model_guid));

    dev->port        = cfg->tcp_port ? cfg->tcp_port : AES70_DEFAULT_TCP_PORT;
    dev->next_ono    = AES70_ONO_APP_BASE;
    dev->active_conn = -1;
    dev->listen_sock = dev->wake_recv_sock = dev->wake_send_sock = -1;

    dev->lock  = xSemaphoreCreateRecursiveMutex();
    dev->cmd_q = xQueueCreate(16, sizeof(aes70_set_req_t));
    if (!dev->lock || !dev->cmd_q) { aes70_device_stop(dev); return ESP_ERR_NO_MEM; }

    /* Required managers and the device root block. */
    dev->device_manager       = aes70_object_new(dev, AES70_K_DEVICE_MANAGER,
                                                  AES70_ONO_DEVICE_MANAGER, "DeviceManager");
    dev->subscription_manager = aes70_object_new(dev, AES70_K_SUBSCRIPTION_MANAGER,
                                                  AES70_ONO_SUBSCRIPTION_MANAGER, "SubscriptionManager");
    dev->root_block           = aes70_object_new(dev, AES70_K_BLOCK,
                                                  AES70_ONO_ROOT_BLOCK, "Root");
    if (!dev->device_manager || !dev->subscription_manager || !dev->root_block) {
        aes70_device_stop(dev);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = aes70_transport_start(dev);
    if (err != ESP_OK) { aes70_device_stop(dev); return err; }

    if (cfg->enable_mdns) {
        if (!cfg->tls.disable_plaintext) {
            aes70_mdns_start(cfg->mdns_hostname, dev->port, dev->device_name,
                             dev->manufacturer, dev->model, dev->serial);
        }
#if CONFIG_AES70_ENABLE_TLS
        if (cfg->tls.enable) {
            aes70_mdns_add_secure(cfg->mdns_hostname, dev->tls_port, dev->device_name,
                                  dev->manufacturer, dev->model, dev->serial);
        }
#endif
    }

    ESP_LOGI(TAG, "device \"%s\" up on TCP %u (%u objects)",
             dev->device_name, dev->port, (unsigned)dev->object_count);
    *out = dev;
    return ESP_OK;
}

esp_err_t aes70_device_stop(aes70_device_handle_t dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    if (dev->cfg.enable_mdns) aes70_mdns_stop();
    aes70_transport_stop(dev);

    for (size_t i = 0; i < dev->object_count; i++) aes70_object_free(dev->objects[i]);
    if (dev->cmd_q) vQueueDelete(dev->cmd_q);
    if (dev->lock)  vSemaphoreDelete(dev->lock);
    free(dev);
    return ESP_OK;
}

uint16_t aes70_device_port(aes70_device_handle_t dev) { return dev ? dev->port : 0; }

size_t aes70_device_connection_count(aes70_device_handle_t dev)
{
    if (!dev) return 0;
    size_t n = 0;
    for (size_t i = 0; i < CONFIG_AES70_MAX_CONNECTIONS; i++) if (dev->conns[i].in_use) n++;
    return n;
}

aes70_object_handle_t aes70_device_root_block(aes70_device_handle_t dev)
{
    return dev ? dev->root_block : NULL;
}
