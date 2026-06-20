/*
 * aes70_internal.h - Shared internal state and cross-module contracts.
 *
 * Ownership / threading: one FreeRTOS task (the OCP.1 server) owns every socket,
 * the connection table, the subscription table and the shared TX scratch buffer.
 * Object *values* are guarded by dev->lock so application getters/setters on
 * other tasks are coherent; the lock is never held across socket I/O. Application
 * setters do not touch the tree directly -- they post a request on dev->cmd_q
 * which the server task applies.
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include "aes70.h"
#include "aes70_object.h"
#include "aes70_ocp1.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Kconfig fallbacks (also lets the component build without menuconfig). */
#ifndef CONFIG_AES70_MAX_OBJECTS
#define CONFIG_AES70_MAX_OBJECTS 128
#endif
#ifndef CONFIG_AES70_MAX_CONNECTIONS
#define CONFIG_AES70_MAX_CONNECTIONS 4
#endif
#ifndef CONFIG_AES70_RX_BUFFER_SIZE
#define CONFIG_AES70_RX_BUFFER_SIZE 2048
#endif
#ifndef CONFIG_AES70_TX_BUFFER_SIZE
#define CONFIG_AES70_TX_BUFFER_SIZE 2048
#endif
#ifndef CONFIG_AES70_MAX_SUBSCRIPTIONS
#define CONFIG_AES70_MAX_SUBSCRIPTIONS 64
#endif
#ifndef CONFIG_AES70_KEEPALIVE_TIMEOUT_FACTOR
#define CONFIG_AES70_KEEPALIVE_TIMEOUT_FACTOR 3
#endif
#ifndef CONFIG_AES70_TASK_STACK_SIZE
#define CONFIG_AES70_TASK_STACK_SIZE 6144
#endif
#ifndef CONFIG_AES70_TASK_PRIORITY
#define CONFIG_AES70_TASK_PRIORITY 5
#endif

/* ---- Object kinds (one per built-in control class) ---------------------- */
typedef enum {
    AES70_K_BLOCK = 0,
    AES70_K_GAIN,
    AES70_K_MUTE,
    AES70_K_POLARITY,
    AES70_K_SWITCH,
    AES70_K_DELAY,
    AES70_K_BOOLEAN,
    AES70_K_INT32,
    AES70_K_UINT16,
    AES70_K_UINT32,
    AES70_K_FLOAT32,
    AES70_K_STRING,
    AES70_K_LEVEL_SENSOR,
    AES70_K_TEMPERATURE,      /* OcaTemperatureSensor (read-only, deg C) */
    AES70_K_FREQUENCY,        /* OcaFrequencyActuator (Hz) */
    AES70_K_IDENTIFY,         /* OcaIdentificationActuator (identify on/off) */
    AES70_K_DYNAMICS,         /* OcaDynamics (compressor/limiter/expander/gate) */
    AES70_K_FILTER_CLASSICAL, /* OcaFilterClassical (crossover/filter) */
    AES70_K_FILTER_PARAMETRIC,/* OcaFilterParametric (parametric EQ band) */
    AES70_K_PANBALANCE,       /* OcaPanBalance */
    AES70_K_SIGNAL_GEN,       /* OcaSignalGenerator */
    AES70_K_DEVICE_MANAGER,
    AES70_K_SUBSCRIPTION_MANAGER,
    AES70_K_COUNT
} aes70_kind_t;

/* ---- OCA object --------------------------------------------------------- *
 * One struct covers every built-in class; `kind` selects which fields and
 * which class descriptor apply. Numeric value/limits live in `num*` (gain dB,
 * delay seconds, float param, and integer params as exact doubles; mute and
 * polarity store their OcaMuteState/OcaPolarityState integer; boolean stores
 * 0/1). */
struct aes70_object {
    uint32_t         ono;
    aes70_kind_t     kind;
    uint32_t         owner_ono;     /* OcaWorker.Owner (container block) */
    char            *role;          /* OcaRoot.Role (heap) */
    char            *label;         /* OcaWorker.Label (heap, may be NULL) */
    bool             enabled;       /* OcaWorker.Enabled */
    uint8_t          lock_state;    /* OcaLockState */
    int              lock_owner;    /* conn_idx holding the lock, -1 = none */
    bool             secured;       /* writes require a privileged session */
    uint32_t         tag;           /* application tag */
    aes70_device_t  *dev;

    double           num;           /* current value */
    double           num_min;
    double           num_max;
    uint8_t          reading_state; /* OcaSensorReadingState (sensors) */

    /* OcaSwitch */
    uint16_t         sw_count;
    char           **sw_names;

    /* OcaStringActuator */
    char            *str;
    uint16_t         str_max;

    /* OcaBlock members */
    struct aes70_object **children;
    uint16_t         child_count;
    uint16_t         child_cap;

    /* Multi-parameter classes (OcaDynamics, OcaFilterClassical) keep their state
     * in a class-specific struct allocated here. NULL for scalar classes. */
    void            *priv;
};

/* ---- Class descriptor (drives dispatch + GetClassIdentification) -------- */
typedef aes70_status_t (*aes70_method_fn)(struct aes70_object *obj, uint16_t method_index,
                                          ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *param_count);

typedef struct {
    const uint16_t        *class_id;       /* dotted field array */
    uint8_t                class_id_len;   /* == number of inheritance levels */
    uint16_t               class_version;
    const aes70_method_fn *level_handlers; /* [class_id_len]; NULL => no methods */
    const char            *class_name;
} aes70_class_desc_t;

const aes70_class_desc_t *aes70_class_for_kind(aes70_kind_t kind);

/* ---- Subscriptions ------------------------------------------------------ */
typedef struct {
    bool     in_use;
    int      conn_idx;            /* connection that subscribed */
    uint32_t emitter_ono;
    uint16_t event_level;         /* (1,1) for PropertyChanged */
    uint16_t event_index;
    uint32_t sub_ono;             /* OcaMethod.ONo (notification target) */
    uint16_t sub_method_level;    /* OcaMethod.MethodID */
    uint16_t sub_method_index;
    uint8_t  context[8];          /* SubscriberContext (EV1, <= a few bytes) */
    uint16_t context_len;
    uint8_t  delivery_mode;       /* OcaNotificationDeliveryMode */
} aes70_subscription_t;

/* ---- Connections (owned by the server task) ----------------------------- */
typedef struct {
    bool     in_use;
    int      sock;
    bool     secure;              /* connection is over TLS */
    bool     privileged;          /* may write secured objects (see aes70_authorize_cb_t) */
    char     addr[48];
    uint16_t port;
    uint8_t  rx[CONFIG_AES70_RX_BUFFER_SIZE];
    size_t   rx_len;
    uint32_t heartbeat_ms;        /* negotiated keep-alive interval (0 = none yet) */
    uint8_t  ka_width;            /* heartbeat field width the peer uses (2 or 4 bytes) */
    int64_t  last_rx_us;
    int64_t  last_tx_us;
} aes70_conn_t;

/* ---- Application -> task value-set request ------------------------------ */
typedef struct {
    uint32_t ono;
    uint8_t  sel;        /* parameter selector for multi-parameter classes (0 = scalar) */
    bool     is_string;
    double   num;
    double   num2;       /* second value (e.g. OcaDBr reference) */
    char     str[96];
} aes70_set_req_t;

/* ---- Device ------------------------------------------------------------- */
struct aes70_device {
    aes70_device_config_t cfg;

    /* Identity (mutable copies; DeviceName/Role are settable by a controller). */
    char     device_name[64];
    char     manufacturer[64];
    char     model[64];
    char     version[32];
    char     serial[48];
    char     device_role[48];
    uint8_t  model_guid[8];      /* OcaModelGUID: Reserved[1]+MfrCode[3]+ModelCode[4] */

    uint16_t port;
    int      listen_sock;
    int      wake_recv_sock;      /* UDP self-pipe read end (in the fd_set) */
    int      wake_send_sock;      /* UDP self-pipe write end (cmd_q signaling) */
    uint16_t wake_port;

    TaskHandle_t      task;
    volatile bool     running;
    QueueHandle_t     cmd_q;
    SemaphoreHandle_t lock;       /* guards object values + registry + subs */

    struct aes70_object *objects[CONFIG_AES70_MAX_OBJECTS];
    size_t   object_count;
    uint32_t next_ono;            /* application object allocator */

    struct aes70_object *root_block;
    struct aes70_object *device_manager;
    struct aes70_object *subscription_manager;

    aes70_conn_t         conns[CONFIG_AES70_MAX_CONNECTIONS];
    aes70_subscription_t subs[CONFIG_AES70_MAX_SUBSCRIPTIONS];
    bool                 notifications_disabled; /* OcaSubscriptionManager state */

    int      active_conn;         /* connection being dispatched (for SubscriptionManager) */
    uint8_t  txbuf[CONFIG_AES70_TX_BUFFER_SIZE];   /* notification scratch (server task) */
    uint8_t  respbuf[CONFIG_AES70_TX_BUFFER_SIZE]; /* response scratch -- kept separate so a
                                                    * notification raised while a response is
                                                    * being built does not clobber it */
};

/* ---- Object model (aes70_object.c) -------------------------------------- */
/* Low-level: allocate + register an object at a specific ONo (managers/root). */
struct aes70_object *aes70_object_new(aes70_device_t *dev, aes70_kind_t kind,
                                      uint32_t ono, const char *role);
/* Allocate an application object: next free ONo, registered, linked into the
 * parent block (parent == NULL => the device root block). */
struct aes70_object *aes70_object_alloc(aes70_device_t *dev, aes70_kind_t kind,
                                        aes70_object_handle_t parent, const char *role);
void aes70_block_add_member(struct aes70_object *block, struct aes70_object *child);
void aes70_object_free(struct aes70_object *obj);
struct aes70_object *aes70_device_find(aes70_device_t *dev, uint32_t ono);
/* Store a numeric value (under lock), notify subscribers, and -- when the write
 * came from a controller -- invoke the on_control_changed callback. */
void aes70_object_commit_num(struct aes70_object *obj, double v,
                             uint16_t prop_level, uint16_t prop_index, bool from_controller);
void aes70_object_commit_str(struct aes70_object *obj, const char *s,
                             uint16_t prop_level, uint16_t prop_index, bool from_controller);
aes70_status_t aes70_object_dispatch(struct aes70_object *obj, uint16_t level, uint16_t index,
                                     ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *param_count);
/* Encode the current wire value of one property into `out` (for notifications
 * and for the various getters). Returns false if (level,index) is unknown. */
bool aes70_object_encode_property(struct aes70_object *obj, uint16_t level, uint16_t index,
                                  ocp1_wr_t *out);
/* Apply an application set-request (already on the server task), clamp, store,
 * and notify subscribers. */
void aes70_object_apply_set(aes70_device_t *dev, const aes70_set_req_t *req);
/* Lock helpers. */
void aes70_lock(aes70_device_t *dev);
void aes70_unlock(aes70_device_t *dev);
/* Post a value change from any task to the server task. */
esp_err_t aes70_post_set(struct aes70_object *obj, const aes70_set_req_t *req);

/* ---- Base-class method handlers (aes70_classes.c) ----------------------- */
aes70_status_t aes70_root_dispatch(struct aes70_object *obj, uint16_t idx,
                                   ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc);
aes70_status_t aes70_worker_dispatch(struct aes70_object *obj, uint16_t idx,
                                     ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc);

/* ---- Identification actuator (aes70_classes.c) -------------------------- */
aes70_status_t aes70_identify_dispatch(struct aes70_object *obj, uint16_t idx,
                                       ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc);

/* ---- Dedicated DSP classes (aes70_dsp.c) -------------------------------- */
aes70_status_t aes70_dynamics_dispatch(struct aes70_object *obj, uint16_t idx,
                                       ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc);
aes70_status_t aes70_filter_dispatch(struct aes70_object *obj, uint16_t idx,
                                     ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc);
aes70_status_t aes70_peq_dispatch(struct aes70_object *obj, uint16_t idx,
                                  ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc);
aes70_status_t aes70_pan_dispatch(struct aes70_object *obj, uint16_t idx,
                                  ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc);
aes70_status_t aes70_siggen_dispatch(struct aes70_object *obj, uint16_t idx,
                                     ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc);
bool aes70_dsp_is_dsp_kind(aes70_kind_t kind);
void aes70_dsp_init(struct aes70_object *obj);    /* allocate obj->priv */
void aes70_dsp_free(struct aes70_object *obj);     /* free obj->priv */
bool aes70_dsp_encode_property(struct aes70_object *obj, uint16_t level, uint16_t index,
                               ocp1_wr_t *out);
void aes70_dsp_apply_set(struct aes70_object *obj, const aes70_set_req_t *req);

/* ---- Command routing (aes70_device.c) ----------------------------------- */
/* Process an inbound Cmd/CmdRrq PDU body; if RRQ, build the response PDU into
 * `out`. Returns the response length (0 if none, e.g. plain CMD). */
size_t aes70_route_command_pdu(aes70_device_t *dev, int conn_idx, ocp1_msg_type_t type,
                               uint16_t msg_count, const uint8_t *data, size_t data_len,
                               uint8_t *out, size_t out_cap);

/* ---- Subscriptions / notifications (aes70_subscription.c) --------------- */
aes70_status_t aes70_submgr_dispatch(struct aes70_object *obj, uint16_t idx,
                                     ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc);
void aes70_notify_property_changed(aes70_device_t *dev, struct aes70_object *obj,
                                   uint16_t prop_level, uint16_t prop_index);
void aes70_subscriptions_drop_conn(aes70_device_t *dev, int conn_idx);

/* ---- Access control (aes70_object.c) ------------------------------------ */
/* True if (level,index) is a state-changing method for this object's class.
 * Only consulted for secured/locked objects, so it need only be exact for the
 * controllable actuator/DSP kinds. */
bool aes70_method_is_write(const struct aes70_object *obj, uint16_t level, uint16_t index);
/* Release any lock held by a connection that is going away. */
void aes70_locks_drop_conn(aes70_device_t *dev, int conn_idx);

/* ---- Managers (aes70_managers.c) ---------------------------------------- */
aes70_status_t aes70_devmgr_dispatch(struct aes70_object *obj, uint16_t idx,
                                     ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc);
aes70_status_t aes70_block_dispatch(struct aes70_object *obj, uint16_t idx,
                                    ocp1_rd_t *in, ocp1_wr_t *out, uint8_t *pc);

/* ---- Transport (aes70_transport.c) -------------------------------------- */
esp_err_t aes70_transport_start(aes70_device_t *dev);
void      aes70_transport_stop(aes70_device_t *dev);
/* Full-write a buffer to a connection's socket (server task only). */
int       aes70_conn_send(aes70_device_t *dev, int conn_idx, const uint8_t *buf, size_t len);

/* ---- mDNS (aes70_mdns.c) ------------------------------------------------- */
esp_err_t aes70_mdns_start(const char *hostname, uint16_t port,
                           const char *device_name, const char *manufacturer,
                           const char *model, const char *serial);
void      aes70_mdns_stop(void);

#ifdef __cplusplus
}
#endif
