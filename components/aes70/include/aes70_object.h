/*
 * aes70_object.h - Build the device's OCA object tree and read/write values.
 *
 * Each constructor instantiates one OCA object of a standard control class,
 * assigns it the next free object number, and inserts it as a member of a
 * parent OcaBlock (pass NULL for the device root block, ONo 100). The returned
 * handle is owned by the device and freed by aes70_device_stop().
 *
 * Getters return the current value and are safe to call from any task (the
 * stored scalar is read atomically enough for control values). Setters change
 * the value AND emit a PropertyChanged notification to subscribed controllers;
 * they are marshalled to the internal task, so they are safe from any task and
 * never block on socket I/O. A value written by a *controller* instead invokes
 * the device's on_control_changed callback (see aes70.h).
 *
 * Class map (verified against AES70-2 / docs.deuso.de):
 *   OcaBlock            1.1.3        topology / grouping container
 *   OcaGain             1.1.1.5      gain in dB (OcaDB float32)
 *   OcaMute             1.1.1.2      signal mute
 *   OcaPolarity         1.1.1.3      phase invert
 *   OcaSwitch           1.1.1.4      n-position selector
 *   OcaDelay            1.1.1.7      delay in seconds (OcaTimeInterval float64)
 *   OcaBooleanActuator  1.1.1.1.1    on/off parameter (bypass, enable, ...)
 *   OcaInt32Actuator    1.1.1.1.4    signed integer parameter
 *   OcaUint16Actuator   1.1.1.1.7    unsigned integer parameter
 *   OcaUint32Actuator   1.1.1.1.8    unsigned integer parameter
 *   OcaFloat32Actuator  1.1.1.1.10   arbitrary float parameter (threshold, Hz, Q, ms)
 *   OcaStringActuator   1.1.1.1.12   arbitrary string parameter
 *   OcaLevelSensor      1.1.2.2      dB level meter (read-only, device reports)
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include "esp_err.h"
#include "aes70_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Generic object operations ------------------------------------------ */

/* The object's assigned OcaONo (object number). */
uint32_t aes70_object_ono(aes70_object_handle_t obj);

/* Application tag, echoed to on_control_changed so you can switch on a
 * parameter id rather than comparing handles. Default 0. */
void     aes70_object_set_tag(aes70_object_handle_t obj, uint32_t tag);
uint32_t aes70_object_get_tag(aes70_object_handle_t obj);

/* Set the OcaWorker Label (human-readable, shown by controllers). Copied. */
esp_err_t aes70_object_set_label(aes70_object_handle_t obj, const char *label);

/* ---- OcaBlock (topology container) -------------------------------------- */
aes70_object_handle_t aes70_block_create(aes70_device_handle_t dev,
                                         aes70_object_handle_t parent,
                                         const char *role);

/* ---- OcaGain (dB) -------------------------------------------------------- */
aes70_object_handle_t aes70_gain_create(aes70_device_handle_t dev,
                                        aes70_object_handle_t parent, const char *role,
                                        float min_db, float max_db, float init_db);
float     aes70_gain_get(aes70_object_handle_t obj);
esp_err_t aes70_gain_set(aes70_object_handle_t obj, float db);

/* ---- OcaMute ------------------------------------------------------------- */
aes70_object_handle_t aes70_mute_create(aes70_device_handle_t dev,
                                        aes70_object_handle_t parent, const char *role,
                                        bool muted);
bool      aes70_mute_get(aes70_object_handle_t obj);   /* true => muted */
esp_err_t aes70_mute_set(aes70_object_handle_t obj, bool muted);

/* ---- OcaPolarity (phase) ------------------------------------------------- */
aes70_object_handle_t aes70_polarity_create(aes70_device_handle_t dev,
                                            aes70_object_handle_t parent, const char *role,
                                            bool inverted);
bool      aes70_polarity_get(aes70_object_handle_t obj); /* true => inverted */
esp_err_t aes70_polarity_set(aes70_object_handle_t obj, bool inverted);

/* ---- OcaSwitch (n-position selector) ------------------------------------- *
 * position_names is an array of `count` C strings (copied); init is the start
 * position (0..count-1). Useful for crossover slope, filter type, routing. */
aes70_object_handle_t aes70_switch_create(aes70_device_handle_t dev,
                                          aes70_object_handle_t parent, const char *role,
                                          const char *const *position_names,
                                          uint16_t count, uint16_t init);
uint16_t  aes70_switch_get(aes70_object_handle_t obj);
esp_err_t aes70_switch_set(aes70_object_handle_t obj, uint16_t position);

/* ---- OcaDelay (seconds) -------------------------------------------------- */
aes70_object_handle_t aes70_delay_create(aes70_device_handle_t dev,
                                         aes70_object_handle_t parent, const char *role,
                                         double min_s, double max_s, double init_s);
double    aes70_delay_get(aes70_object_handle_t obj);
esp_err_t aes70_delay_set(aes70_object_handle_t obj, double seconds);

/* ---- OcaBooleanActuator (bypass / enable / ...) -------------------------- */
aes70_object_handle_t aes70_boolean_create(aes70_device_handle_t dev,
                                           aes70_object_handle_t parent, const char *role,
                                           bool init);
bool      aes70_boolean_get(aes70_object_handle_t obj);
esp_err_t aes70_boolean_set(aes70_object_handle_t obj, bool value);

/* ---- OcaFloat32Actuator (arbitrary scalar: threshold, ratio, Hz, Q, ms) -- */
aes70_object_handle_t aes70_float_create(aes70_device_handle_t dev,
                                         aes70_object_handle_t parent, const char *role,
                                         float min, float max, float init);
float     aes70_float_get(aes70_object_handle_t obj);
esp_err_t aes70_float_set(aes70_object_handle_t obj, float value);

/* ---- OcaUint16Actuator / OcaUint32Actuator / OcaInt32Actuator ------------ */
aes70_object_handle_t aes70_uint16_create(aes70_device_handle_t dev,
                                          aes70_object_handle_t parent, const char *role,
                                          uint16_t min, uint16_t max, uint16_t init);
uint16_t  aes70_uint16_get(aes70_object_handle_t obj);
esp_err_t aes70_uint16_set(aes70_object_handle_t obj, uint16_t value);

aes70_object_handle_t aes70_uint32_create(aes70_device_handle_t dev,
                                          aes70_object_handle_t parent, const char *role,
                                          uint32_t min, uint32_t max, uint32_t init);
uint32_t  aes70_uint32_get(aes70_object_handle_t obj);
esp_err_t aes70_uint32_set(aes70_object_handle_t obj, uint32_t value);

aes70_object_handle_t aes70_int32_create(aes70_device_handle_t dev,
                                         aes70_object_handle_t parent, const char *role,
                                         int32_t min, int32_t max, int32_t init);
int32_t   aes70_int32_get(aes70_object_handle_t obj);
esp_err_t aes70_int32_set(aes70_object_handle_t obj, int32_t value);

/* ---- OcaStringActuator --------------------------------------------------- */
aes70_object_handle_t aes70_string_create(aes70_device_handle_t dev,
                                          aes70_object_handle_t parent, const char *role,
                                          const char *init, uint16_t max_len);
/* Returns a pointer to the internal copy; valid until the next set or stop. */
const char *aes70_string_get(aes70_object_handle_t obj);
esp_err_t   aes70_string_set(aes70_object_handle_t obj, const char *value);

/* ---- OcaLevelSensor (read-only meter; device reports the reading) -------- *
 * Controllers read GetReading and may subscribe; aes70_level_sensor_report()
 * updates the dB reading and notifies subscribers. Rate-limit your reports
 * (e.g. <= 20/s) to avoid flooding controllers. */
aes70_object_handle_t aes70_level_sensor_create(aes70_device_handle_t dev,
                                                aes70_object_handle_t parent, const char *role,
                                                float min_db, float max_db);
esp_err_t aes70_level_sensor_report(aes70_object_handle_t obj, float db);

#ifdef __cplusplus
}
#endif
