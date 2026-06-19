/*
 * aes70.h - AES70 (OCA) device for ESP-IDF: lifecycle and configuration.
 *
 * Implements the device (responder) side of AES70: it listens for OCP.1
 * (AES70-3) connections over TCP, presents an OCA object tree (a root OcaBlock
 * containing the application's control objects, plus the required
 * OcaDeviceManager and OcaSubscriptionManager), answers Commands with
 * Responses, and emits PropertyChanged Notifications to subscribed controllers.
 * It advertises "_oca._tcp" over mDNS so controllers (e.g. AES70 Explorer)
 * discover it automatically.
 *
 * The protocol core is class-agnostic: any standard or proprietary OCA class is
 * supported by registering a class descriptor (see aes70_object.h for the
 * built-in control classes and aes70_register.h for custom classes). This
 * header covers only bringing a device up and down.
 *
 * Threading model: a single internal task owns every socket and the entire
 * object tree. Commands are handled on that task. Application-initiated value
 * changes (aes70_gain_set(), ...) are marshalled to the task through a queue,
 * so no lock is ever held across socket I/O and the tree is never touched
 * concurrently.
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include "esp_err.h"
#include "aes70_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Reported to the application when a controller WRITES a control value (e.g.
 * calls SetGain). Runs on the internal task: do not block. Read the new value
 * with the matching getter (aes70_gain_get(), ...) and push it to your DSP.
 * `tag` is the value set on the object with aes70_object_set_tag() (0 if unset),
 * which lets the application switch on the parameter without comparing handles.
 */
typedef void (*aes70_control_changed_cb_t)(aes70_object_handle_t obj,
                                            uint32_t tag, void *user);

/* Connection lifecycle, also delivered on the internal task. */
typedef enum {
    AES70_CONN_OPENED,   /* a controller connected (TCP accepted) */
    AES70_CONN_CLOSED,   /* a controller disconnected or timed out */
} aes70_conn_event_t;

typedef void (*aes70_conn_cb_t)(aes70_device_handle_t dev,
                                const char *peer_addr, uint16_t peer_port,
                                aes70_conn_event_t event, void *user);

typedef struct {
    /* Device identity (surfaced through OcaDeviceManager and mDNS TXT). */
    const char *device_name;     /* DeviceName, settable by a controller. Required. */
    const char *manufacturer;    /* OcaModelDescription.Manufacturer. Required. */
    const char *model;           /* OcaModelDescription.Name (product). Required. */
    const char *version;         /* OcaModelDescription.Version (firmware/rev). */
    const char *serial_number;   /* OcaDeviceManager.SerialNumber. */
    const uint8_t *model_guid;   /* 8-byte OcaModelGUID: Reserved[1]+MfrCode[3]+
                                    ModelCode[4] (NULL => all zero). */

    /* Network. */
    uint16_t    tcp_port;        /* OCP.1 listen port. 0 => AES70_DEFAULT_TCP_PORT. */
    bool        enable_mdns;     /* advertise _oca._tcp via mDNS. */
    const char *mdns_hostname;   /* host part of "<name>.local"; NULL => derive. */

    /* Application callbacks. */
    aes70_control_changed_cb_t on_control_changed;
    aes70_conn_cb_t            on_connection;
    void                      *user;

    /* Internal task. Zeros => Kconfig defaults. */
    int task_priority;
    int task_stack_size;
    int task_core_id;            /* -1 => tskNO_AFFINITY */
} aes70_device_config_t;

/* Fill cfg with library defaults. The caller then sets at least device_name,
 * manufacturer and model. */
void aes70_device_config_default(aes70_device_config_t *cfg);

/*
 * Create and start a device. On success *out receives a handle, the OCP.1
 * server task is running, the required managers and the root OcaBlock exist,
 * and (if enabled) the mDNS service is published. The network interface must
 * already be up. Build the object tree with the aes70_*_create() functions in
 * aes70_object.h either before or after this call.
 */
esp_err_t aes70_device_start(const aes70_device_config_t *cfg,
                             aes70_device_handle_t *out);

/*
 * Stop the device: closes every controller connection, removes the mDNS
 * service, stops the task and frees the object tree. Safe to call once.
 */
esp_err_t aes70_device_stop(aes70_device_handle_t dev);

/* The TCP port actually being listened on (useful when tcp_port was 0). */
uint16_t aes70_device_port(aes70_device_handle_t dev);

/* Number of currently open controller connections. */
size_t aes70_device_connection_count(aes70_device_handle_t dev);

/* The device root OcaBlock (ONo 100). Pass as the parent of top-level objects,
 * or NULL to the create functions (which default to the root block). */
aes70_object_handle_t aes70_device_root_block(aes70_device_handle_t dev);

#ifdef __cplusplus
}
#endif
