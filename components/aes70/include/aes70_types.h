/*
 * aes70_types.h - Common AES70 / OCA constants, enums and handle types.
 *
 * Values and names follow AES70-2/-3 (the OCA Object Model and the OCP.1 binary
 * protocol). All wire fields are big-endian; see aes70_ocp1.h (internal) for the
 * framing. This public header carries only the types an application needs to
 * build a device and read/write control values.
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- OcaStatus (AES70-2, Operating Datatypes) --------------------------- *
 * Returned by every method; carried as a single byte in an OCP.1 response.   */
typedef enum {
    AES70_OK                    = 0,
    AES70_PROTOCOL_VERSION_ERROR = 1,
    AES70_DEVICE_ERROR          = 2,
    AES70_LOCKED                = 3,
    AES70_BAD_FORMAT            = 4,
    AES70_BAD_ONO               = 5,
    AES70_PARAMETER_ERROR       = 6,
    AES70_PARAMETER_OUT_OF_RANGE = 7,
    AES70_NOT_IMPLEMENTED       = 8,
    AES70_INVALID_REQUEST       = 9,
    AES70_PROCESSING_FAILED     = 10,
    AES70_BAD_METHOD            = 11,
    AES70_PARTIALLY_SUCCEEDED   = 12,
    AES70_TIMEOUT               = 13,
    AES70_BUFFER_OVERFLOW       = 14,
    AES70_PERMISSION_DENIED     = 15,
    AES70_OUT_OF_MEMORY         = 16,
    AES70_BUSY                  = 17,
} aes70_status_t;

/* ---- OcaMuteState -------------------------------------------------------- */
typedef enum {
    AES70_MUTE_MUTED   = 1,
    AES70_MUTE_UNMUTED = 2,
} aes70_mute_state_t;

/* ---- OcaLockState -------------------------------------------------------- */
typedef enum {
    AES70_LOCK_NONE            = 0,  /* NoLock */
    AES70_LOCK_NO_WRITE        = 1,  /* LockNoWrite */
    AES70_LOCK_NO_READ_WRITE   = 2,  /* LockNoReadWrite */
} aes70_lock_state_t;

/* ---- OcaPropertyChangeType (carried in PropertyChanged events) ----------- */
typedef enum {
    AES70_CHANGE_CURRENT = 1,
    AES70_CHANGE_MIN     = 2,
    AES70_CHANGE_MAX     = 3,
    AES70_CHANGE_ITEM_ADDED   = 4,
    AES70_CHANGE_ITEM_CHANGED = 5,
    AES70_CHANGE_ITEM_DELETED = 6,
} aes70_property_change_type_t;

/* OcaDB - gain / level in decibels, an IEEE-754 float32 on the wire. */
typedef float aes70_db_t;

/* ---- OcaDynamics function (compressor / limiter / expander / gate) ------- */
typedef enum {
    AES70_DYN_NONE     = 0,
    AES70_DYN_COMPRESS = 1,
    AES70_DYN_LIMIT    = 2,
    AES70_DYN_EXPAND   = 3,
    AES70_DYN_GATE     = 4,
} aes70_dynamics_function_t;

/* ---- OcaLevelDetectionLaw ------------------------------------------------ */
typedef enum {
    AES70_DETECT_NONE = 0,
    AES70_DETECT_RMS  = 1,
    AES70_DETECT_PEAK = 2,
} aes70_detection_law_t;

/* ---- OcaPresentationUnit ------------------------------------------------- */
typedef enum {
    AES70_UNIT_DBU = 0,
    AES70_UNIT_DBV = 1,
    AES70_UNIT_V   = 2,
} aes70_presentation_unit_t;

/* ---- OcaFilterPassband --------------------------------------------------- */
typedef enum {
    AES70_PASSBAND_HIPASS     = 1,
    AES70_PASSBAND_LOWPASS    = 2,
    AES70_PASSBAND_BANDPASS   = 3,
    AES70_PASSBAND_BANDREJECT = 4,
    AES70_PASSBAND_ALLPASS    = 5,
} aes70_filter_passband_t;

/* ---- OcaClassicalFilterShape --------------------------------------------- */
typedef enum {
    AES70_FILTER_BUTTERWORTH    = 1,
    AES70_FILTER_BESSEL         = 2,
    AES70_FILTER_CHEBYSHEV      = 3,
    AES70_FILTER_LINKWITZ_RILEY = 4,
} aes70_filter_shape_t;

/* ---- OcaParametricEQShape (OcaFilterParametric) -------------------------- */
typedef enum {
    AES70_PEQ_NONE       = 0,
    AES70_PEQ_PEQ        = 1,   /* peaking / bell */
    AES70_PEQ_LOW_SHELF  = 2,
    AES70_PEQ_HIGH_SHELF = 3,
    AES70_PEQ_LOWPASS    = 4,
    AES70_PEQ_HIGHPASS   = 5,
    AES70_PEQ_BANDPASS   = 6,
    AES70_PEQ_ALLPASS    = 7,
    AES70_PEQ_NOTCH      = 8,
} aes70_parametric_eq_shape_t;

/* ---- OcaWaveformType (OcaSignalGenerator) -------------------------------- */
typedef enum {
    AES70_WAVE_NONE         = 0,
    AES70_WAVE_DC           = 1,
    AES70_WAVE_SINE         = 2,
    AES70_WAVE_SQUARE       = 3,
    AES70_WAVE_IMPULSE      = 4,
    AES70_WAVE_NOISE_PINK   = 5,
    AES70_WAVE_NOISE_WHITE  = 6,
    AES70_WAVE_POLARITY_TEST = 7,
} aes70_waveform_type_t;

/* ---- OcaSweepType (OcaSignalGenerator) ----------------------------------- */
typedef enum {
    AES70_SWEEP_NONE        = 0,
    AES70_SWEEP_LINEAR      = 1,
    AES70_SWEEP_LOGARITHMIC = 2,
} aes70_sweep_type_t;

/* ---- Reserved object numbers (AES70-2) ----------------------------------- *
 * Managers occupy the low ONos; the device root block is conventionally 100;  *
 * application objects are assigned from AES70_ONO_APP_BASE upward.            */
#define AES70_ONO_DEVICE_MANAGER        1u
#define AES70_ONO_SECURITY_MANAGER      2u
#define AES70_ONO_FIRMWARE_MANAGER      3u
#define AES70_ONO_SUBSCRIPTION_MANAGER  4u
#define AES70_ONO_ROOT_BLOCK            100u
#define AES70_ONO_APP_BASE              4096u

/* Default OCP.1 TCP port. AES70 defines no fixed port (controllers learn it
 * from mDNS), but a stable default eases manual connections. */
#define AES70_DEFAULT_TCP_PORT          65000

/* Default port for secure OCP.1 (OCP.1 over TLS). */
#define AES70_DEFAULT_TLS_PORT          65001

/* DNS-SD service type advertised for discovery (insecure OCP.1 / TCP). */
#define AES70_MDNS_SERVICE_TYPE         "_oca"
#define AES70_MDNS_SERVICE_PROTO        "_tcp"
/* DNS-SD service type for secure OCP.1 (OCP.1 over TLS). */
#define AES70_MDNS_SECURE_SERVICE_TYPE  "_ocasec"

/* Opaque handles. The struct tags are also exposed as typedefs for internal
 * use; application code should treat them as opaque and use the handle types. */
typedef struct aes70_device aes70_device_t;
typedef struct aes70_object aes70_object_t;
typedef struct aes70_device *aes70_device_handle_t;
typedef struct aes70_object *aes70_object_handle_t;

#ifdef __cplusplus
}
#endif
