/*
 * aes70_transport.c - OCP.1 TCP server: one task owns the listen socket, every
 * controller connection and a UDP "self-pipe" used to wake select() when the
 * application posts a value change. Frames are assembled per connection and
 * routed to the command handler; keep-alives are answered and idle connections
 * are dropped.
 *
 * SPDX-License-Identifier: MIT
 */
#include <string.h>
#include <errno.h>

#include "lwip/sockets.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "aes70_internal.h"

static const char *TAG = "aes70.tcp";

/* ---- Socket send (server task only) ------------------------------------- */
int aes70_conn_send(aes70_device_t *dev, int conn_idx, const uint8_t *buf, size_t len)
{
    aes70_conn_t *c = &dev->conns[conn_idx];
    if (!c->in_use) return -1;
    size_t sent = 0;
    while (sent < len) {
        int w = send(c->sock, buf + sent, len - sent, 0);
        if (w <= 0) {
            if (w < 0 && (errno == EINTR)) continue;
            return -1;
        }
        sent += (size_t)w;
    }
    c->last_tx_us = esp_timer_get_time();
    return 0;
}

/* ---- KeepAlive ----------------------------------------------------------- */
static void send_keepalive(aes70_device_t *dev, int conn_idx)
{
    aes70_conn_t *c = &dev->conns[conn_idx];
    uint8_t buf[OCP1_HEADER_LEN + 4];
    ocp1_wr_t w; ocp1_wr_init(&w, buf, sizeof(buf));
    ocp1_wr_u8(&w, OCP1_SYNC_VAL);
    ocp1_wr_u16(&w, OCP1_PROTO_VERSION);
    uint8_t width = (c->ka_width == 2) ? 2 : 4;
    ocp1_wr_u32(&w, (uint32_t)(9 + width));        /* PduSize = (header-1) + payload */
    ocp1_wr_u8(&w, OCP1_KEEPALIVE);
    ocp1_wr_u16(&w, 1);                            /* MessageCount */
    if (width == 2) ocp1_wr_u16(&w, (uint16_t)(c->heartbeat_ms / 1000));
    else            ocp1_wr_u32(&w, c->heartbeat_ms);
    if (!w.err) aes70_conn_send(dev, conn_idx, buf, w.off);
}

/* ---- Connection teardown ------------------------------------------------ */
static void close_conn(aes70_device_t *dev, int conn_idx)
{
    aes70_conn_t *c = &dev->conns[conn_idx];
    if (!c->in_use) return;
    if (c->sock >= 0) { shutdown(c->sock, SHUT_RDWR); close(c->sock); }
    aes70_subscriptions_drop_conn(dev, conn_idx);
    aes70_locks_drop_conn(dev, conn_idx);
    if (dev->cfg.on_connection) {
        dev->cfg.on_connection((aes70_device_handle_t)dev, c->addr, c->port,
                               AES70_CONN_CLOSED, dev->cfg.user);
    }
    ESP_LOGI(TAG, "closed %s:%u", c->addr, c->port);
    memset(c, 0, sizeof(*c));
    c->sock = -1;
    c->in_use = false;
}

/* ---- Frame handling ----------------------------------------------------- */
static void handle_frame(aes70_device_t *dev, int conn_idx, uint8_t mtype,
                         uint16_t mcount, const uint8_t *data, size_t data_len)
{
    aes70_conn_t *c = &dev->conns[conn_idx];
    switch (mtype) {
    case OCP1_CMD:
    case OCP1_CMD_RRQ: {
        size_t rlen = aes70_route_command_pdu(dev, conn_idx, (ocp1_msg_type_t)mtype, mcount,
                                              data, data_len, dev->respbuf, sizeof(dev->respbuf));
        if (rlen) aes70_conn_send(dev, conn_idx, dev->respbuf, rlen);
        break;
    }
    case OCP1_KEEPALIVE:
        if (data_len == 2)      { c->heartbeat_ms = (uint32_t)ocp1_rd16(data) * 1000; c->ka_width = 2; }
        else if (data_len >= 4) { c->heartbeat_ms = ocp1_rd32(data);                  c->ka_width = 4; }
        send_keepalive(dev, conn_idx);           /* answer so the controller sees us alive */
        break;
    default:
        /* Notifications / responses are controller-bound; ignore if received. */
        break;
    }
}

/* Parse and dispatch every complete frame buffered for a connection; compact
 * the leftover partial frame to the front. Returns false to drop the connection. */
static bool process_rx(aes70_device_t *dev, int conn_idx)
{
    aes70_conn_t *c = &dev->conns[conn_idx];
    size_t consumed = 0;
    while (c->rx_len - consumed >= OCP1_HEADER_LEN) {
        const uint8_t *p = c->rx + consumed;
        if (p[0] != OCP1_SYNC_VAL) return false;             /* unsynchronised stream */
        uint32_t pdusize = ocp1_rd32(p + 3);
        size_t frame_total = (size_t)pdusize + OCP1_PDUSIZE_PREFIX;
        if (frame_total < OCP1_HEADER_LEN || frame_total > sizeof(c->rx)) {
            ESP_LOGW(TAG, "bad PDU size %u from %s:%u", (unsigned)pdusize, c->addr, c->port);
            return false;
        }
        if (frame_total > c->rx_len - consumed) break;        /* await the rest */

        uint8_t  mtype  = p[7];
        uint16_t mcount = ocp1_rd16(p + 8);
        handle_frame(dev, conn_idx, mtype, mcount,
                     p + OCP1_HEADER_LEN, frame_total - OCP1_HEADER_LEN);
        consumed += frame_total;
    }
    if (consumed) {
        memmove(c->rx, c->rx + consumed, c->rx_len - consumed);
        c->rx_len -= consumed;
    }
    /* A full buffer with no complete frame means the message exceeds RX_BUFFER. */
    if (c->rx_len == sizeof(c->rx)) {
        ESP_LOGW(TAG, "RX buffer full, dropping %s:%u (raise AES70_RX_BUFFER_SIZE)", c->addr, c->port);
        return false;
    }
    return true;
}

/* ---- Accept ------------------------------------------------------------- */
static void accept_conn(aes70_device_t *dev)
{
    struct sockaddr_storage src;
    socklen_t slen = sizeof(src);
    int s = accept(dev->listen_sock, (struct sockaddr *)&src, &slen);
    if (s < 0) return;

    int slot = -1;
    for (int i = 0; i < CONFIG_AES70_MAX_CONNECTIONS; i++) {
        if (!dev->conns[i].in_use) { slot = i; break; }
    }
    if (slot < 0) {
        ESP_LOGW(TAG, "connection table full, rejecting");
        close(s);
        return;
    }
    int one = 1;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    aes70_conn_t *c = &dev->conns[slot];
    memset(c, 0, sizeof(*c));
    c->in_use = true;
    c->sock = s;
    c->last_rx_us = c->last_tx_us = esp_timer_get_time();
    if (src.ss_family == AF_INET) {
        struct sockaddr_in *si = (struct sockaddr_in *)&src;
        inet_ntoa_r(si->sin_addr, c->addr, sizeof(c->addr));
        c->port = ntohs(si->sin_port);
    } else if (src.ss_family == AF_INET6) {
        struct sockaddr_in6 *si = (struct sockaddr_in6 *)&src;
        inet6_ntoa_r(si->sin6_addr, c->addr, sizeof(c->addr));
        c->port = ntohs(si->sin6_port);
    }
    /* Decide whether this (plaintext) session may write secured objects. The
     * default policy leaves plaintext sessions unprivileged; an authorize
     * callback can override (e.g. trust a management subnet). TLS sessions get
     * their privilege from the client certificate in the secure accept path. */
    c->secure = false;
    c->privileged = dev->cfg.authorize
                  ? dev->cfg.authorize(c->addr, false /*secure*/, false /*authenticated*/, dev->cfg.user)
                  : false;

    ESP_LOGI(TAG, "controller connected: %s:%u%s", c->addr, c->port,
             c->privileged ? " (privileged)" : "");
    if (dev->cfg.on_connection) {
        dev->cfg.on_connection((aes70_device_handle_t)dev, c->addr, c->port,
                               AES70_CONN_OPENED, dev->cfg.user);
    }
}

/* ---- Application value-change drain -------------------------------------- */
static void drain_cmd_queue(aes70_device_t *dev)
{
    aes70_set_req_t req;
    while (xQueueReceive(dev->cmd_q, &req, 0) == pdTRUE) {
        aes70_object_apply_set(dev, &req);
    }
}

/* ---- Keep-alive / timeout maintenance ----------------------------------- */
static void service_keepalive(aes70_device_t *dev, int64_t now)
{
    int factor = CONFIG_AES70_KEEPALIVE_TIMEOUT_FACTOR;
    for (int i = 0; i < CONFIG_AES70_MAX_CONNECTIONS; i++) {
        aes70_conn_t *c = &dev->conns[i];
        if (!c->in_use || c->heartbeat_ms == 0) continue;
        int64_t hb_us = (int64_t)c->heartbeat_ms * 1000;
        if (now - c->last_rx_us > hb_us * factor) {
            ESP_LOGW(TAG, "keep-alive timeout %s:%u", c->addr, c->port);
            close_conn(dev, i);
        } else if (now - c->last_tx_us >= hb_us) {
            send_keepalive(dev, i);
        }
    }
}

/* ---- Server task -------------------------------------------------------- */
static void server_task(void *arg)
{
    aes70_device_t *dev = arg;
    uint8_t drain[64];

    while (dev->running) {
        fd_set rfds;
        FD_ZERO(&rfds);
        int maxfd = dev->listen_sock;
        FD_SET(dev->listen_sock, &rfds);
        if (dev->wake_recv_sock >= 0) {
            FD_SET(dev->wake_recv_sock, &rfds);
            if (dev->wake_recv_sock > maxfd) maxfd = dev->wake_recv_sock;
        }
        for (int i = 0; i < CONFIG_AES70_MAX_CONNECTIONS; i++) {
            if (dev->conns[i].in_use) {
                FD_SET(dev->conns[i].sock, &rfds);
                if (dev->conns[i].sock > maxfd) maxfd = dev->conns[i].sock;
            }
        }

        struct timeval tv = { .tv_sec = 0, .tv_usec = 500000 };
        int n = select(maxfd + 1, &rfds, NULL, NULL, &tv);
        if (!dev->running) break;
        int64_t now = esp_timer_get_time();

        if (n > 0) {
            if (dev->wake_recv_sock >= 0 && FD_ISSET(dev->wake_recv_sock, &rfds)) {
                recv(dev->wake_recv_sock, drain, sizeof(drain), 0);
                drain_cmd_queue(dev);
            }
            if (FD_ISSET(dev->listen_sock, &rfds)) {
                accept_conn(dev);
            }
            for (int i = 0; i < CONFIG_AES70_MAX_CONNECTIONS; i++) {
                aes70_conn_t *c = &dev->conns[i];
                if (!c->in_use || !FD_ISSET(c->sock, &rfds)) continue;
                int r = recv(c->sock, c->rx + c->rx_len, sizeof(c->rx) - c->rx_len, 0);
                if (r <= 0) { close_conn(dev, i); continue; }
                c->rx_len += (size_t)r;
                c->last_rx_us = now;
                if (!process_rx(dev, i)) close_conn(dev, i);
            }
        }
        service_keepalive(dev, now);
    }

    /* Drain and close everything on the way out. */
    for (int i = 0; i < CONFIG_AES70_MAX_CONNECTIONS; i++) close_conn(dev, i);
    vTaskDelete(NULL);
}

/* ---- Socket bring-up ---------------------------------------------------- */
static int make_listen_socket(uint16_t port)
{
    int s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s < 0) return -1;
    int one = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port = htons(port),
    };
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) { close(s); return -1; }
    if (listen(s, CONFIG_AES70_MAX_CONNECTIONS) < 0) { close(s); return -1; }
    return s;
}

/* A loopback UDP pair: aes70_post_set() sends one byte to wake select(). */
static esp_err_t make_wake_pipe(aes70_device_t *dev)
{
    int r = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (r < 0) return ESP_FAIL;
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_LOOPBACK),
        .sin_port = 0,
    };
    if (bind(r, (struct sockaddr *)&addr, sizeof(addr)) < 0) { close(r); return ESP_FAIL; }
    socklen_t alen = sizeof(addr);
    getsockname(r, (struct sockaddr *)&addr, &alen);

    int w = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (w < 0) { close(r); return ESP_FAIL; }
    if (connect(w, (struct sockaddr *)&addr, sizeof(addr)) < 0) { close(r); close(w); return ESP_FAIL; }

    dev->wake_recv_sock = r;
    dev->wake_send_sock = w;
    return ESP_OK;
}

esp_err_t aes70_transport_start(aes70_device_t *dev)
{
    dev->listen_sock = make_listen_socket(dev->port);
    if (dev->listen_sock < 0) {
        ESP_LOGE(TAG, "cannot listen on TCP %u: errno %d", dev->port, errno);
        return ESP_FAIL;
    }
    make_wake_pipe(dev);   /* best-effort; without it sets apply on the next 500 ms tick */

    dev->running = true;
    int prio  = dev->cfg.task_priority   ? dev->cfg.task_priority   : CONFIG_AES70_TASK_PRIORITY;
    int stack = dev->cfg.task_stack_size ? dev->cfg.task_stack_size : CONFIG_AES70_TASK_STACK_SIZE;
    BaseType_t core = dev->cfg.task_core_id < 0 ? tskNO_AFFINITY : dev->cfg.task_core_id;

    BaseType_t ok = xTaskCreatePinnedToCore(server_task, "aes70_tcp", stack, dev,
                                            prio, &dev->task, core);
    if (ok != pdPASS) {
        dev->running = false;
        close(dev->listen_sock); dev->listen_sock = -1;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void aes70_transport_stop(aes70_device_t *dev)
{
    if (!dev->running && !dev->task) {
        if (dev->listen_sock >= 0) { close(dev->listen_sock); dev->listen_sock = -1; }
        return;
    }
    dev->running = false;
    if (dev->wake_send_sock >= 0) { uint8_t b = 1; send(dev->wake_send_sock, &b, 1, 0); }

    /* Wait for the task to exit (it deletes itself). */
    for (int i = 0; i < 200 && dev->task; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (eTaskGetState(dev->task) == eDeleted) break;
    }
    dev->task = NULL;

    if (dev->listen_sock >= 0)    { close(dev->listen_sock);    dev->listen_sock = -1; }
    if (dev->wake_recv_sock >= 0) { close(dev->wake_recv_sock); dev->wake_recv_sock = -1; }
    if (dev->wake_send_sock >= 0) { close(dev->wake_send_sock); dev->wake_send_sock = -1; }
}
