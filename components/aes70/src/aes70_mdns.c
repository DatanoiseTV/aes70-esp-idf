/*
 * aes70_mdns.c - DNS-SD advertising of the "_oca._tcp" service so controllers
 * (e.g. AES70 Explorer) discover the device automatically.
 *
 * We initialise mDNS if the application has not, publish one service with the
 * device identity in TXT records and the OCP.1 TCP port in the SRV record, and
 * on stop remove only our own service (mDNS is left running for the app).
 *
 * SPDX-License-Identifier: MIT
 */
#include <string.h>
#include <ctype.h>

#include "esp_log.h"
#include "esp_err.h"
#include "mdns.h"

#include "aes70_types.h"

static const char *TAG = "aes70.mdns";
static bool s_service_added;          /* _oca._tcp (plaintext) */
static bool s_secure_added;           /* _ocasec._tcp (TLS) */

/* Display name -> safe DNS label: lowercase alnum + '-', <= 32 chars. */
static void sanitize_hostname(const char *in, char *out, size_t outsz)
{
    size_t j = 0;
    for (size_t i = 0; in[i] && j + 1 < outsz && j < 32; i++) {
        char c = in[i];
        if (isalnum((unsigned char)c)) {
            out[j++] = (char)tolower((unsigned char)c);
        } else if ((c == ' ' || c == '-' || c == '_') && j > 0 && out[j - 1] != '-') {
            out[j++] = '-';
        }
    }
    while (j > 0 && out[j - 1] == '-') j--;
    if (j == 0) {
        const char *fb = "aes70-device";
        size_t n = strlen(fb);
        if (n >= outsz) n = outsz - 1;
        memcpy(out, fb, n);
        j = n;
    }
    out[j] = '\0';
}

static esp_err_t ensure_mdns(const char *hostname, const char *device_name)
{
    char host[33];
    if (hostname && hostname[0]) strlcpy(host, hostname, sizeof(host));
    else                         sanitize_hostname(device_name, host, sizeof(host));

    esp_err_t err = mdns_init();           /* safe if the app already called it */
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mdns_init failed: %s", esp_err_to_name(err));
        return err;
    }
    mdns_hostname_set(host);
    return ESP_OK;
}

static esp_err_t add_service(const char *device_name, const char *type, uint16_t port,
                             const char *manufacturer, const char *model, const char *serial)
{
    mdns_txt_item_t txt[] = {
        { "txtvers",      "1" },
        { "protocol",     "OCP.1" },
        { "manufacturer", (char *)(manufacturer ? manufacturer : "") },
        { "model",        (char *)(model        ? model        : "") },
        { "serial",       (char *)(serial       ? serial       : "") },
    };
    esp_err_t err = mdns_service_add(device_name, type, AES70_MDNS_SERVICE_PROTO,
                                     port, txt, sizeof(txt) / sizeof(txt[0]));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mdns_service_add(%s) failed: %s", type, esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "advertising %s.%s.%s on :%u",
             device_name, type, AES70_MDNS_SERVICE_PROTO, port);
    return ESP_OK;
}

esp_err_t aes70_mdns_start(const char *hostname, uint16_t port,
                           const char *device_name, const char *manufacturer,
                           const char *model, const char *serial)
{
    esp_err_t err = ensure_mdns(hostname, device_name);
    if (err != ESP_OK) return err;
    err = add_service(device_name, AES70_MDNS_SERVICE_TYPE, port, manufacturer, model, serial);
    if (err == ESP_OK) s_service_added = true;
    return err;
}

esp_err_t aes70_mdns_add_secure(const char *hostname, uint16_t tls_port,
                                const char *device_name, const char *manufacturer,
                                const char *model, const char *serial)
{
    esp_err_t err = ensure_mdns(hostname, device_name);
    if (err != ESP_OK) return err;
    err = add_service(device_name, AES70_MDNS_SECURE_SERVICE_TYPE, tls_port,
                      manufacturer, model, serial);
    if (err == ESP_OK) s_secure_added = true;
    return err;
}

void aes70_mdns_stop(void)
{
    if (s_service_added) {
        mdns_service_remove(AES70_MDNS_SERVICE_TYPE, AES70_MDNS_SERVICE_PROTO);
        s_service_added = false;
    }
    if (s_secure_added) {
        mdns_service_remove(AES70_MDNS_SECURE_SERVICE_TYPE, AES70_MDNS_SERVICE_PROTO);
        s_secure_added = false;
    }
    /* Intentionally not calling mdns_free(): the application may use mDNS too. */
}
