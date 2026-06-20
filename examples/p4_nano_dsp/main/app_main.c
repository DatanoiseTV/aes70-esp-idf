/*
 * AES70 (OCA) DSP device demo for ESP32-P4-Nano (and any ESP-IDF target).
 *
 * A full-blown audio-DSP control tree built entirely from standard OCA control
 * classes, so an AES70 controller (e.g. AES70 Explorer) discovers it over mDNS
 * and renders purpose-built widgets for each block:
 *
 *   Root
 *   |- Master        : OcaGain, OcaMute, OcaPolarity, OcaPanBalance, OcaDelay,
 *   |                  OcaLevelSensor (output meter)
 *   |- SignalGen     : OcaSignalGenerator (sine/noise/sweep test source)
 *   |- ParametricEQ  : OcaBooleanActuator (bypass) + 4 OcaFilterParametric bands
 *   |- Compressor    : OcaDynamics + makeup OcaGain + bypass
 *   |- MultibandComp : 3 bands; each = OcaFilterClassical (band split) +
 *   |                  OcaDynamics + makeup OcaGain + OcaMute
 *   |- Limiter       : OcaDynamics + bypass
 *   |- Crossover     : Low/High = OcaFilterClassical + OcaGain + OcaMute
 *   `- System        : OcaTemperatureSensor (real chip temperature) +
 *                      OcaIdentificationActuator (identify)
 *
 * AES70 has no monolithic "multiband compressor" class; the idiomatic model is
 * an OcaBlock per band holding a crossover filter + an OcaDynamics, which
 * controllers present as a band of compressor widgets. That is what MultibandComp
 * builds here.
 *
 * When a controller writes a value, on_control_changed() fires; a real product
 * would forward the value to its DSP there. The demo streams a synthetic output
 * level and compressor gain-reduction, and the real chip temperature, so
 * subscribed controllers see live meters.
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
#include "driver/temperature_sensor.h"

#include "protocol_examples_common.h"

#include "aes70.h"
#include "aes70_object.h"

static const char *TAG = "aes70_dsp";

static aes70_device_handle_t s_dev;
static aes70_object_handle_t s_meter;
static aes70_object_handle_t s_compressor;
static aes70_object_handle_t s_temp;
static temperature_sensor_handle_t s_tsens;

/* ---- Pretty logging of controller writes -------------------------------- */
enum { T_GAIN = 1, T_FLOAT, T_MUTE, T_BOOL, T_POLARITY, T_SWITCH, T_DELAY,
       T_DYNAMICS, T_FILTER, T_PEQ, T_PAN, T_SIGGEN, T_IDENT };

typedef struct { aes70_object_handle_t h; char name[28]; int type; } demo_param_t;
static demo_param_t s_params[128];
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
    case T_PEQ:
        ESP_LOGI(TAG, "%-16s = %.0f Hz, %+.1f dB, Q %.2f, shape %u", n,
                 aes70_parametric_eq_get_frequency(obj), aes70_parametric_eq_get_gain(obj),
                 aes70_parametric_eq_get_q(obj), aes70_parametric_eq_get_shape(obj));
        break;
    case T_PAN:    ESP_LOGI(TAG, "%-16s = %.2f", n, aes70_panbalance_get_position(obj)); break;
    case T_SIGGEN:
        ESP_LOGI(TAG, "%-16s = %s, %.0f Hz, %.1f dB, waveform %u", n,
                 aes70_signal_generator_is_generating(obj) ? "ON" : "off",
                 aes70_signal_generator_get_frequency(obj), aes70_signal_generator_get_level(obj),
                 aes70_signal_generator_get_waveform(obj));
        break;
    case T_IDENT:  ESP_LOGI(TAG, "%-16s = %s", n, aes70_identify_get(obj) ? "IDENTIFY" : "off"); break;
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

/* One multiband-compressor band: crossover split + dynamics + makeup + mute. */
static void make_mb_band(aes70_device_handle_t dev, aes70_object_handle_t mb,
                         const char *band, uint8_t passband, float freq)
{
    char role[24], nm[28];
    aes70_object_handle_t blk = aes70_block_create(dev, mb, band);
    snprintf(nm, sizeof(nm), "MB %s Split", band);
    track(aes70_filter_create(dev, blk, "Split", passband, AES70_FILTER_LINKWITZ_RILEY, freq, 4), nm, T_FILTER);
    snprintf(nm, sizeof(nm), "MB %s Comp", band);
    track(aes70_dynamics_create(dev, blk, "Comp", AES70_DYN_COMPRESS), nm, T_DYNAMICS);
    snprintf(nm, sizeof(nm), "MB %s Makeup", band);
    track(aes70_gain_create(dev, blk, "Makeup", -6.0f, 24.0f, 0.0f), nm, T_GAIN);
    snprintf(role, sizeof(role), "Mute");
    snprintf(nm, sizeof(nm), "MB %s Mute", band);
    track(aes70_mute_create(dev, blk, role, false), nm, T_MUTE);
}

/* ---- Build the control tree --------------------------------------------- */
static void build_dsp_tree(aes70_device_handle_t dev)
{
    /* Master section. */
    aes70_object_handle_t master = aes70_block_create(dev, NULL, "Master");
    track(aes70_gain_create(dev, master, "MasterGain", -80.0f, 12.0f, 0.0f), "Master Gain", T_GAIN);
    track(aes70_mute_create(dev, master, "MasterMute", false), "Master Mute", T_MUTE);
    track(aes70_polarity_create(dev, master, "MasterPolarity", false), "Master Polarity", T_POLARITY);
    track(aes70_panbalance_create(dev, master, "Balance"), "Master Balance", T_PAN);
    track(aes70_delay_create(dev, master, "MasterDelay", 0.0, 0.100, 0.0), "Master Delay", T_DELAY);
    s_meter = aes70_level_sensor_create(dev, master, "OutputMeter", -80.0f, 0.0f);

    /* Test signal generator. */
    aes70_object_handle_t gen = aes70_block_create(dev, NULL, "SignalGen");
    track(aes70_signal_generator_create(dev, gen, "Generator", AES70_WAVE_SINE, 1000.0f, -20.0f),
          "Signal Gen", T_SIGGEN);

    /* 4-band parametric EQ. */
    aes70_object_handle_t eq = aes70_block_create(dev, NULL, "ParametricEQ");
    track(aes70_boolean_create(dev, eq, "EQBypass", false), "EQ Bypass", T_BOOL);
    static const struct { const char *role; uint8_t shape; float f, g, q; } bands[] = {
        { "LowShelf",  AES70_PEQ_LOW_SHELF,  80.0f,   0.0f, 0.7f },
        { "LowMid",    AES70_PEQ_PEQ,        300.0f,  0.0f, 1.0f },
        { "HighMid",   AES70_PEQ_PEQ,        2500.0f, 0.0f, 1.0f },
        { "HighShelf", AES70_PEQ_HIGH_SHELF, 10000.0f, 0.0f, 0.7f },
    };
    for (size_t i = 0; i < sizeof(bands) / sizeof(bands[0]); i++) {
        char nm[28];
        snprintf(nm, sizeof(nm), "EQ %s", bands[i].role);
        track(aes70_parametric_eq_create(dev, eq, bands[i].role, bands[i].shape,
                                         bands[i].f, bands[i].g, bands[i].q), nm, T_PEQ);
    }

    /* Compressor (single OcaDynamics + makeup + bypass). */
    aes70_object_handle_t comp = aes70_block_create(dev, NULL, "Compressor");
    s_compressor = track(aes70_dynamics_create(dev, comp, "Dynamics", AES70_DYN_COMPRESS),
                         "Compressor", T_DYNAMICS);
    track(aes70_gain_create(dev, comp, "MakeupGain", -6.0f, 24.0f, 0.0f), "Comp Makeup", T_GAIN);
    track(aes70_boolean_create(dev, comp, "Bypass", false),               "Comp Bypass", T_BOOL);

    /* 3-band multiband compressor. */
    aes70_object_handle_t mb = aes70_block_create(dev, NULL, "MultibandComp");
    make_mb_band(dev, mb, "Low",  AES70_PASSBAND_LOWPASS,  300.0f);
    make_mb_band(dev, mb, "Mid",  AES70_PASSBAND_BANDPASS, 1500.0f);
    make_mb_band(dev, mb, "High", AES70_PASSBAND_HIPASS,   3000.0f);

    /* Limiter (OcaDynamics configured as a limiter). The limiter shapes the
     * output protection and is marked secured: an ordinary (unprivileged)
     * controller can read it but cannot change it. With TLS enabled a
     * mutually-authenticated controller is privileged and may. */
    aes70_object_handle_t lim = aes70_block_create(dev, NULL, "Limiter");
    aes70_object_handle_t lim_dyn =
        track(aes70_dynamics_create(dev, lim, "Dynamics", AES70_DYN_LIMIT), "Limiter", T_DYNAMICS);
    aes70_object_set_secured(lim_dyn, true);
    track(aes70_boolean_create(dev, lim, "Bypass", false),             "Lim Bypass", T_BOOL);

    /* 2-way crossover. The split filters define the system tuning, so they are
     * secured too; the per-band gain and mute stay open for normal operation. */
    aes70_object_handle_t xover = aes70_block_create(dev, NULL, "Crossover");
    aes70_object_handle_t low = aes70_block_create(dev, xover, "LowBand");
    aes70_object_handle_t lpf =
        track(aes70_filter_create(dev, low, "LowPass", AES70_PASSBAND_LOWPASS,
                                  AES70_FILTER_LINKWITZ_RILEY, 2000.0f, 4), "Low XO Filter", T_FILTER);
    aes70_object_set_secured(lpf, true);
    track(aes70_gain_create(dev, low, "Gain", -24.0f, 6.0f, 0.0f), "Low Gain", T_GAIN);
    track(aes70_mute_create(dev, low, "Mute", false),              "Low Mute", T_MUTE);
    aes70_object_handle_t high = aes70_block_create(dev, xover, "HighBand");
    aes70_object_handle_t hpf =
        track(aes70_filter_create(dev, high, "HighPass", AES70_PASSBAND_HIPASS,
                                  AES70_FILTER_LINKWITZ_RILEY, 2000.0f, 4), "High XO Filter", T_FILTER);
    aes70_object_set_secured(hpf, true);
    track(aes70_gain_create(dev, high, "Gain", -24.0f, 6.0f, 0.0f), "High Gain", T_GAIN);
    track(aes70_mute_create(dev, high, "Mute", false),             "High Mute", T_MUTE);

    /* System: real chip temperature + identify. */
    aes70_object_handle_t sys = aes70_block_create(dev, NULL, "System");
    s_temp = aes70_temperature_create(dev, sys, "ChipTemp", -10.0f, 125.0f);
    track(aes70_identify_create(dev, sys, "Identify"), "Identify", T_IDENT);

    ESP_LOGI(TAG, "built DSP control tree: %u objects", (unsigned)s_nparams + 2);
}

/* Stream synthetic meters + the real chip temperature to subscribed controllers. */
static void telemetry_task(void *arg)
{
    int v = 0, dir = 1, tick = 0;
    for (;;) {
        float db = -60.0f + (float)v * 0.6f;       /* triangle sweep -60..0 dB */
        if (s_meter) aes70_level_sensor_report(s_meter, db);
        if (s_compressor) {                        /* synthetic gain reduction above -20 dB */
            float gr = db > -20.0f ? -(db + 20.0f) * 0.5f : 0.0f;
            aes70_dynamics_report_gain(s_compressor, gr, gr < -0.1f);
        }
        if (s_temp && s_tsens && (++tick % 10) == 0) {   /* real temperature ~1 Hz */
            float c;
            if (temperature_sensor_get_celsius(s_tsens, &c) == ESP_OK)
                aes70_temperature_report(s_temp, c);
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

    /* Internal temperature sensor (drives OcaTemperatureSensor). The min/max
     * must fall within one of the sensor's predefined measurement ranges;
     * -10..80 C covers normal chip operating temperature. */
    temperature_sensor_config_t tcfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    if (temperature_sensor_install(&tcfg, &s_tsens) == ESP_OK) {
        temperature_sensor_enable(s_tsens);
    } else {
        ESP_LOGW(TAG, "temperature sensor unavailable; ChipTemp will read static");
    }

    /* Bring up the network via the standard example helper. */
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

    xTaskCreate(telemetry_task, "aes70_telemetry", 3072, NULL, 4, NULL);
}
