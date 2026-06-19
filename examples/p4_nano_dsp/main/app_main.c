/*
 * AES70 (OCA) DSP device demo for ESP32-P4-Nano (and any ESP-IDF target).
 *
 * Brings up a network interface, starts an AES70 device, and builds a generic
 * audio-DSP control tree that an AES70 controller (e.g. AES70 Explorer) can
 * discover over mDNS and operate:
 *
 *   Root
 *   |- Master      : OcaGain, OcaMute, OcaPolarity, OcaDelay, OcaLevelSensor (meter)
 *   |- GraphicEQ   : OcaBooleanActuator (bypass) + 10 OcaGain bands
 *   |- Compressor  : OcaDynamics (compressor), makeup OcaGain, bypass
 *   |- Limiter     : OcaDynamics (limiter), bypass
 *   `- Crossover   : Low/High sub-blocks, each an OcaFilterClassical + OcaGain + OcaMute
 *
 * The compressor and limiter are single OcaDynamics objects and the crossover
 * bands are OcaFilterClassical objects, so an AES70 controller shows dedicated
 * compressor / filter widgets rather than rows of generic sliders.
 *
 * When a controller writes a value, on_control_changed() is invoked; a real
 * product would forward the new value to its DSP there. The demo also streams a
 * synthetic output level into the meter so subscribed controllers see live
 * PropertyChanged notifications.
 *
 * SPDX-License-Identifier: MIT
 */
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"

#include "protocol_examples_common.h"

#include "aes70.h"
#include "aes70_object.h"

static const char *TAG = "aes70_dsp";

static aes70_device_handle_t s_dev;
static aes70_object_handle_t s_meter;

/* ---- Pretty logging of controller writes -------------------------------- *
 * A tiny registry maps each object handle to a name + value type so the demo
 * can print human-readable updates. A real product would instead map the
 * object (or its tag) to a DSP parameter. */
enum { T_GAIN = 1, T_FLOAT, T_MUTE, T_BOOL, T_POLARITY, T_SWITCH, T_DELAY,
       T_DYNAMICS, T_FILTER };

static aes70_object_handle_t s_compressor;   /* fed synthetic gain-reduction telemetry */

typedef struct { aes70_object_handle_t h; char name[28]; int type; } demo_param_t;
static demo_param_t s_params[64];
static size_t s_nparams;

static aes70_object_handle_t track(aes70_object_handle_t h, const char *name, int type)
{
    if (h && s_nparams < sizeof(s_params) / sizeof(s_params[0])) {
        s_params[s_nparams].h = h;
        strlcpy(s_params[s_nparams].name, name, sizeof(s_params[s_nparams].name));
        s_params[s_nparams].type = type;
        s_nparams++;
        aes70_object_set_tag(h, (uint32_t)type);
    }
    return h;
}

static const char *name_of(aes70_object_handle_t obj)
{
    for (size_t i = 0; i < s_nparams; i++) if (s_params[i].h == obj) return s_params[i].name;
    return "?";
}

static void on_control_changed(aes70_object_handle_t obj, uint32_t tag, void *user)
{
    const char *n = name_of(obj);
    switch (tag) {
    case T_GAIN:     ESP_LOGI(TAG, "%-16s = %+.2f dB", n, aes70_gain_get(obj)); break;
    case T_FLOAT:    ESP_LOGI(TAG, "%-16s = %.3f",     n, aes70_float_get(obj)); break;
    case T_MUTE:     ESP_LOGI(TAG, "%-16s = %s",       n, aes70_mute_get(obj) ? "MUTED" : "unmuted"); break;
    case T_BOOL:     ESP_LOGI(TAG, "%-16s = %s",       n, aes70_boolean_get(obj) ? "on" : "off"); break;
    case T_POLARITY: ESP_LOGI(TAG, "%-16s = %s",       n, aes70_polarity_get(obj) ? "inverted" : "normal"); break;
    case T_SWITCH:   ESP_LOGI(TAG, "%-16s = position %u", n, aes70_switch_get(obj)); break;
    case T_DELAY:    ESP_LOGI(TAG, "%-16s = %.3f ms",  n, aes70_delay_get(obj) * 1000.0); break;
    case T_DYNAMICS:
        ESP_LOGI(TAG, "%-16s = func %u, thr %.1f dB, ratio %.1f, atk %.1f ms, rel %.0f ms", n,
                 aes70_dynamics_get_function(obj), aes70_dynamics_get_threshold(obj),
                 aes70_dynamics_get_ratio(obj), aes70_dynamics_get_attack(obj) * 1000.0f,
                 aes70_dynamics_get_release(obj) * 1000.0f);
        break;
    case T_FILTER:
        ESP_LOGI(TAG, "%-16s = %.0f Hz, passband %u, shape %u, order %u", n,
                 aes70_filter_get_frequency(obj), aes70_filter_get_passband(obj),
                 aes70_filter_get_shape(obj), aes70_filter_get_order(obj));
        break;
    default: break;
    }
    /* TODO (product): push the new value into the DSP for `obj` here. */
}

static void on_connection(aes70_device_handle_t dev, const char *addr, uint16_t port,
                          aes70_conn_event_t ev, void *user)
{
    ESP_LOGI(TAG, "controller %s  %s:%u", ev == AES70_CONN_OPENED ? "connected" : "disconnected",
             addr, port);
}

/* ---- Build the control tree --------------------------------------------- */
static void build_dsp_tree(aes70_device_handle_t dev)
{
    /* Master section. */
    aes70_object_handle_t master = aes70_block_create(dev, NULL, "Master");
    track(aes70_gain_create(dev, master, "MasterGain", -80.0f, 12.0f, 0.0f), "Master Gain", T_GAIN);
    track(aes70_mute_create(dev, master, "MasterMute", false), "Master Mute", T_MUTE);
    track(aes70_polarity_create(dev, master, "MasterPolarity", false), "Master Polarity", T_POLARITY);
    track(aes70_delay_create(dev, master, "MasterDelay", 0.0, 0.100, 0.0), "Master Delay", T_DELAY);
    s_meter = aes70_level_sensor_create(dev, master, "OutputMeter", -80.0f, 0.0f);

    /* 10-band graphic EQ. */
    aes70_object_handle_t eq = aes70_block_create(dev, NULL, "GraphicEQ");
    track(aes70_boolean_create(dev, eq, "EQBypass", false), "EQ Bypass", T_BOOL);
    static const int eq_freqs[] = { 31, 63, 125, 250, 500, 1000, 2000, 4000, 8000, 16000 };
    for (size_t i = 0; i < sizeof(eq_freqs) / sizeof(eq_freqs[0]); i++) {
        char role[24], name[24];
        snprintf(role, sizeof(role), "EQBand%uHz", (unsigned)eq_freqs[i]);
        snprintf(name, sizeof(name), "EQ %uHz", (unsigned)eq_freqs[i]);
        track(aes70_gain_create(dev, eq, role, -12.0f, 12.0f, 0.0f), name, T_GAIN);
    }

    /* Compressor: one OcaDynamics object (controllers render a compressor
     * widget) plus a makeup OcaGain and a bypass toggle alongside it. */
    aes70_object_handle_t comp = aes70_block_create(dev, NULL, "Compressor");
    s_compressor = track(aes70_dynamics_create(dev, comp, "Dynamics", AES70_DYN_COMPRESS),
                         "Compressor", T_DYNAMICS);
    track(aes70_gain_create(dev, comp, "MakeupGain", -6.0f, 24.0f, 0.0f), "Comp Makeup", T_GAIN);
    track(aes70_boolean_create(dev, comp, "Bypass", false),               "Comp Bypass", T_BOOL);

    /* Limiter: an OcaDynamics configured as a limiter. */
    aes70_object_handle_t lim = aes70_block_create(dev, NULL, "Limiter");
    track(aes70_dynamics_create(dev, lim, "Dynamics", AES70_DYN_LIMIT), "Limiter", T_DYNAMICS);
    track(aes70_boolean_create(dev, lim, "Bypass", false),             "Lim Bypass", T_BOOL);

    /* 2-way crossover: an OcaFilterClassical per band (Linkwitz-Riley) plus a
     * band OcaGain and OcaMute. */
    aes70_object_handle_t xover = aes70_block_create(dev, NULL, "Crossover");
    aes70_object_handle_t low = aes70_block_create(dev, xover, "LowBand");
    track(aes70_filter_create(dev, low, "LowPass", AES70_PASSBAND_LOWPASS,
                              AES70_FILTER_LINKWITZ_RILEY, 2000.0f, 4), "Low XO Filter", T_FILTER);
    track(aes70_gain_create(dev, low, "Gain", -24.0f, 6.0f, 0.0f), "Low Gain", T_GAIN);
    track(aes70_mute_create(dev, low, "Mute", false),              "Low Mute", T_MUTE);
    aes70_object_handle_t high = aes70_block_create(dev, xover, "HighBand");
    track(aes70_filter_create(dev, high, "HighPass", AES70_PASSBAND_HIPASS,
                              AES70_FILTER_LINKWITZ_RILEY, 2000.0f, 4), "High XO Filter", T_FILTER);
    track(aes70_gain_create(dev, high, "Gain", -24.0f, 6.0f, 0.0f), "High Gain", T_GAIN);
    track(aes70_mute_create(dev, high, "Mute", false),             "High Mute", T_MUTE);

    ESP_LOGI(TAG, "built DSP control tree: %u objects", (unsigned)s_nparams + 1);
}

/* Stream a synthetic output level into the meter so subscribed controllers see
 * live PropertyChanged notifications (rate-limited). */
static void meter_task(void *arg)
{
    int v = 0, dir = 1;
    for (;;) {
        float db = -60.0f + (float)v * 0.6f;       /* triangle sweep -60..0 dB */
        if (s_meter) aes70_level_sensor_report(s_meter, db);
        if (s_compressor) {                        /* synthetic gain reduction above -20 dB */
            float gr = db > -20.0f ? -(db + 20.0f) * 0.5f : 0.0f;
            aes70_dynamics_report_gain(s_compressor, gr, gr < -0.1f);
        }
        v += dir;
        if (v >= 100) dir = -1; else if (v <= 0) dir = 1;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Bring up the network via the standard example helper. The transport
     * (Ethernet on the P4-Nano, Wi-Fi elsewhere) is chosen in menuconfig. */
    ESP_ERROR_CHECK(example_connect());

    aes70_device_config_t cfg;
    aes70_device_config_default(&cfg);
    cfg.device_name        = CONFIG_AES70_DEMO_DEVICE_NAME;
    cfg.manufacturer       = CONFIG_AES70_DEMO_MANUFACTURER;
    cfg.model              = CONFIG_AES70_DEMO_MODEL;
    cfg.version            = "1.0.0";
    cfg.serial_number      = CONFIG_AES70_DEMO_SERIAL;
    cfg.tcp_port           = CONFIG_AES70_DEMO_PORT;
    cfg.on_control_changed = on_control_changed;
    cfg.on_connection      = on_connection;

    ESP_ERROR_CHECK(aes70_device_start(&cfg, &s_dev));
    build_dsp_tree(s_dev);

    ESP_LOGI(TAG, "AES70 device \"%s\" listening on TCP %u. Discover it in AES70 Explorer.",
             cfg.device_name, aes70_device_port(s_dev));

    xTaskCreate(meter_task, "aes70_meter", 3072, NULL, 4, NULL);
}
