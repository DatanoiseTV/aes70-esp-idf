/*
 * Network-layer stubs for the HOST unit-test build. The real transport and
 * mDNS modules own sockets and a FreeRTOS task; the tests drive the command
 * router directly, so these stand-ins let the device create/start/stop without
 * touching the network.
 *
 * SPDX-License-Identifier: MIT
 */
#include "aes70_internal.h"

int64_t esp_timer_get_time(void)
{
    static int64_t t = 0;
    return (t += 1000);          /* 1 ms per call, monotonic */
}

esp_err_t aes70_transport_start(aes70_device_t *dev)
{
    dev->running = true;         /* no task/sockets in the host build */
    return ESP_OK;
}

void aes70_transport_stop(aes70_device_t *dev)
{
    dev->running = false;
}

int aes70_conn_send(aes70_device_t *dev, int conn_idx, const uint8_t *buf, size_t len)
{
    (void)dev; (void)conn_idx; (void)buf; (void)len;
    return 0;                    /* notifications are discarded in tests */
}

esp_err_t aes70_mdns_start(const char *hostname, uint16_t port,
                           const char *device_name, const char *manufacturer,
                           const char *model, const char *serial)
{
    (void)hostname; (void)port; (void)device_name;
    (void)manufacturer; (void)model; (void)serial;
    return ESP_OK;
}

void aes70_mdns_stop(void) {}
