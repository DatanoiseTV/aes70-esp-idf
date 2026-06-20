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
 *   OcaDynamics         1.1.1.14     compressor/limiter/expander/gate (one object)
 *   OcaFilterClassical  1.1.1.9      crossover/filter (frequency/shape/order)
 *
 * The OcaDynamics and OcaFilterClassical classes carry many parameters in a
 * single object, and AES70 controllers render purpose-built widgets for them
 * (e.g. a compressor panel), so prefer them over a block of generic actuators.
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

/* Access control: mark an object secured. A controller may always READ a
 * secured object, but any state-changing method (Set*, Lock) is rejected with
 * OcaStatus PermissionDenied unless that controller's session is privileged
 * (see aes70_authorize_cb_t / the TLS config in aes70.h). Use it to protect the
 * parameters an unprivileged operator must not touch -- crossover, limiter,
 * routing -- while leaving volume and the like open. Default: not secured. */
void aes70_object_set_secured(aes70_object_handle_t obj, bool secured);
bool aes70_object_is_secured(aes70_object_handle_t obj);

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

/* ---- OcaDynamics (compressor / limiter / expander / gate) ---------------- *
 * One object exposes Function, Threshold (dB), Ratio/Slope, Attack, Release,
 * Hold, Knee, gain floor/ceiling, DetectorLaw, plus read-only Triggered and
 * DynamicGain telemetry. `function` is an aes70_dynamics_function_t. */
aes70_object_handle_t aes70_dynamics_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                            const char *role, uint8_t function);
uint8_t   aes70_dynamics_get_function(aes70_object_handle_t obj);
float     aes70_dynamics_get_threshold(aes70_object_handle_t obj);  /* dB */
float     aes70_dynamics_get_ratio(aes70_object_handle_t obj);
float     aes70_dynamics_get_attack(aes70_object_handle_t obj);     /* seconds */
float     aes70_dynamics_get_release(aes70_object_handle_t obj);    /* seconds */
esp_err_t aes70_dynamics_set_function(aes70_object_handle_t obj, uint8_t function);
esp_err_t aes70_dynamics_set_threshold(aes70_object_handle_t obj, float db);
esp_err_t aes70_dynamics_set_ratio(aes70_object_handle_t obj, float ratio);
esp_err_t aes70_dynamics_set_attack(aes70_object_handle_t obj, float seconds);
esp_err_t aes70_dynamics_set_release(aes70_object_handle_t obj, float seconds);
/* Report live gain-reduction telemetry to subscribed controllers (rate-limit). */
esp_err_t aes70_dynamics_report_gain(aes70_object_handle_t obj, float gain_db, bool triggered);

/* ---- OcaFilterClassical (crossover / filter) ---------------------------- *
 * `passband` is an aes70_filter_passband_t, `shape` an aes70_filter_shape_t. */
aes70_object_handle_t aes70_filter_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                          const char *role, uint8_t passband, uint8_t shape,
                                          float frequency, uint16_t order);
float     aes70_filter_get_frequency(aes70_object_handle_t obj);
uint8_t   aes70_filter_get_passband(aes70_object_handle_t obj);
uint8_t   aes70_filter_get_shape(aes70_object_handle_t obj);
uint16_t  aes70_filter_get_order(aes70_object_handle_t obj);
esp_err_t aes70_filter_set_frequency(aes70_object_handle_t obj, float hz);
esp_err_t aes70_filter_set_passband(aes70_object_handle_t obj, uint8_t passband);
esp_err_t aes70_filter_set_shape(aes70_object_handle_t obj, uint8_t shape);
esp_err_t aes70_filter_set_order(aes70_object_handle_t obj, uint16_t order);

/* ---- OcaFilterParametric (parametric EQ band) --------------------------- *
 * `shape` is an aes70_parametric_eq_shape_t; gain is in dB, q is the Q factor. */
aes70_object_handle_t aes70_parametric_eq_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                                 const char *role, uint8_t shape, float frequency,
                                                 float gain_db, float q);
float     aes70_parametric_eq_get_frequency(aes70_object_handle_t obj);
float     aes70_parametric_eq_get_gain(aes70_object_handle_t obj);
float     aes70_parametric_eq_get_q(aes70_object_handle_t obj);
uint8_t   aes70_parametric_eq_get_shape(aes70_object_handle_t obj);
esp_err_t aes70_parametric_eq_set_frequency(aes70_object_handle_t obj, float hz);
esp_err_t aes70_parametric_eq_set_gain(aes70_object_handle_t obj, float db);
esp_err_t aes70_parametric_eq_set_q(aes70_object_handle_t obj, float q);
esp_err_t aes70_parametric_eq_set_shape(aes70_object_handle_t obj, uint8_t shape);

/* ---- OcaPanBalance (position -1=left .. +1=right) ------------------------ */
aes70_object_handle_t aes70_panbalance_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                              const char *role);
float     aes70_panbalance_get_position(aes70_object_handle_t obj);
esp_err_t aes70_panbalance_set_position(aes70_object_handle_t obj, float position);

/* ---- OcaSignalGenerator (test signal) ----------------------------------- *
 * `waveform` is an aes70_waveform_type_t; level is in dB. */
aes70_object_handle_t aes70_signal_generator_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                                    const char *role, uint8_t waveform,
                                                    float frequency, float level_db);
float     aes70_signal_generator_get_frequency(aes70_object_handle_t obj);
float     aes70_signal_generator_get_level(aes70_object_handle_t obj);
uint8_t   aes70_signal_generator_get_waveform(aes70_object_handle_t obj);
bool      aes70_signal_generator_is_generating(aes70_object_handle_t obj);
esp_err_t aes70_signal_generator_set_frequency(aes70_object_handle_t obj, float hz);
esp_err_t aes70_signal_generator_set_level(aes70_object_handle_t obj, float db);
esp_err_t aes70_signal_generator_set_waveform(aes70_object_handle_t obj, uint8_t waveform);
esp_err_t aes70_signal_generator_start(aes70_object_handle_t obj);
esp_err_t aes70_signal_generator_stop(aes70_object_handle_t obj);

/* ---- OcaFrequencyActuator (Hz) ------------------------------------------ */
aes70_object_handle_t aes70_frequency_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                             const char *role, float min_hz, float max_hz, float init_hz);
float     aes70_frequency_get(aes70_object_handle_t obj);
esp_err_t aes70_frequency_set(aes70_object_handle_t obj, float hz);

/* ---- OcaTemperatureSensor (deg C, read-only; device reports) ------------ */
aes70_object_handle_t aes70_temperature_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                               const char *role, float min_c, float max_c);
esp_err_t aes70_temperature_report(aes70_object_handle_t obj, float celsius);

/* ---- OcaIdentificationActuator (identify on/off) ------------------------ */
aes70_object_handle_t aes70_identify_create(aes70_device_handle_t dev, aes70_object_handle_t parent,
                                            const char *role);
bool      aes70_identify_get(aes70_object_handle_t obj);
esp_err_t aes70_identify_set(aes70_object_handle_t obj, bool active);

#ifdef __cplusplus
}
#endif
