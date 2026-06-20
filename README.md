# AES70 (OCA) device for ESP-IDF

An ESP-IDF component that implements the **device (responder) side of AES70** —
the AES standard for control and monitoring of networked audio devices, also
known as **OCA** (Open Control Architecture). It speaks **OCP.1** (AES70-3, the
binary protocol for TCP/IP), presents an OCA object tree, answers Commands with
Responses, emits PropertyChanged Notifications to subscribed controllers, and
advertises itself over mDNS so controllers such as **AES70 Explorer** discover
it automatically.

It is built for DSP devices: the included example exposes a parametric EQ,
compressor, multiband compressor, limiter, crossover and signal generator as
standard OCA control objects that any AES70 controller can operate, with no
controller-specific code on the device.

The component is **target-agnostic** — it uses only BSD sockets, `esp_netif`
and the `espressif/mdns` component — so it runs on any ESP-IDF chip with a
network interface. The example targets the **ESP32-P4-Nano** over wired
Ethernet and builds unchanged for the ESP32-S3, ESP32, ESP32-C6 and others.

## Status

| | |
|---|---|
| Builds (ESP-IDF v6.0, esp32p4) | yes — clean, no warnings |
| Wire format | byte-exact against the Wireshark OCP.1 dissector and the AES70-2 class definitions (docs.deuso.de); see "Protocol fidelity" below |
| Object model | OcaRoot/OcaWorker locking, GetClassIdentification, block enumeration, EV1/EV2 subscriptions and live PropertyChanged notifications |
| On-hardware OCP.1 round-trip | verified on an ESP32-P4-Nano over Ethernet (see below) |

This is a `0.x` release: the public API may still change before `1.0.0`.

The protocol was exercised end-to-end against the example running on a real
ESP32-P4-Nano (rev v1.3) over its on-board Ethernet, using the
[`tools/ocp1_smoketest.py`](tools/ocp1_smoketest.py) OCP.1 controller:
device identity (GetDeviceName / GetModelDescription), root-block
`GetClassIdentification`, recursive object-tree enumeration (all 38 demo
objects), `GetGain`/`SetGain` round-trip with range checking, `BadONo` handling,
and an `AddSubscription` → `SetGain` → PropertyChanged notification round-trip
all pass. Run it yourself against your device:

```bash
python3 tools/ocp1_smoketest.py <device-ip> 65000
```

GUI interop with AES70 Explorer (which speaks the same OCP.1 protocol) is the
recommended next check.

## What is implemented

**Protocol (OCP.1 / AES70-3) — complete for device operation**

- Frame sync, header, and the four message types: Command, Command-with-response,
  Response, Notification, plus KeepAlive (heartbeat negotiation + idle timeout).
- Big-endian marshalling of every base type the control classes use: integers,
  float32/float64, OcaString, OcaBlob, OcaList, OcaClassID/OcaClassIdentification,
  OcaObjectIdentification, OcaBlockMember.
- Multiple controller connections, each with its own framing buffer.

**Object model — class-agnostic and extensible**

- A class descriptor drives method dispatch by `(DefLevel, MethodIndex)`, walking
  the inheritance chain, so **any** OCA class — standard or proprietary — is a
  matter of registering a descriptor and a handler. The protocol core does not
  need to change to add classes.
- Per-object locking (`OcaRoot.LockState`), `GetClassIdentification`, role/label.

**Control classes (Workers)**

| Class | ClassID | Purpose |
|---|---|---|
| OcaBlock | 1.1.3 | topology / grouping container, member enumeration |
| OcaGain | 1.1.1.5 | gain in dB |
| OcaMute | 1.1.1.2 | signal mute |
| OcaPolarity | 1.1.1.3 | phase invert |
| OcaSwitch | 1.1.1.4 | n-position selector |
| OcaDelay | 1.1.1.7 | delay in seconds |
| OcaBooleanActuator | 1.1.1.1.1 | on/off parameter |
| OcaInt32Actuator | 1.1.1.1.4 | signed integer parameter |
| OcaUint16Actuator | 1.1.1.1.7 | unsigned integer parameter |
| OcaUint32Actuator | 1.1.1.1.8 | unsigned integer parameter |
| OcaFloat32Actuator | 1.1.1.1.10 | arbitrary float parameter |
| OcaStringActuator | 1.1.1.1.12 | arbitrary string parameter |
| OcaLevelSensor | 1.1.2.2 | dB level meter (device reports) |
| OcaDynamics | 1.1.1.14 | compressor / limiter / expander / gate (single object) |
| OcaFilterClassical | 1.1.1.9 | crossover / filter (frequency, shape, order) |
| OcaFilterParametric | 1.1.1.10 | parametric EQ band (frequency, gain, Q, shape) |
| OcaPanBalance | 1.1.1.6 | pan / balance |
| OcaSignalGenerator | 1.1.1.17 | test signal (waveform, frequency, level, sweep) |
| OcaFrequencyActuator | 1.1.1.8 | frequency parameter (Hz) |
| OcaTemperatureSensor | 1.1.2.5 | temperature reading (device reports) |
| OcaIdentificationActuator | 1.1.1.21 | identify (locate the device) |

`OcaDynamics`, `OcaFilterClassical`, `OcaFilterParametric`, `OcaPanBalance` and
`OcaSignalGenerator` carry their whole parameter set in one object, so a
controller renders a purpose-built compressor / filter / EQ / generator widget
rather than a row of generic sliders. Prefer them for those functions.

There is no monolithic "multiband compressor" class — the idiomatic model is an
OcaBlock per band, each holding an `OcaFilterClassical` split and an
`OcaDynamics`; controllers show that as a band of compressor widgets (the
example builds a 3-band one).

**Managers**

- **OcaDeviceManager** (1.3.1): identity (name, model description, manufacturer,
  serial, model GUID), OCA version, device state, and the manager list a
  controller reads on connect.
- **OcaSubscriptionManager** (1.3.4): `AddSubscription`/`RemoveSubscription`
  (EV1), `AddSubscription2`/`RemoveSubscription2` (EV2), and
  Disable/ReEnableNotifications. PropertyChanged events are delivered as OCP.1
  Notifications over the originating TCP connection.

### Not yet implemented (documented extension points)

AES70 is a large standard. The following parts of the repertoire are **not**
built in; the object model is structured so they can be added as additional
class descriptors without touching the protocol core. They are called out here
rather than silently omitted:

- Connection management (OcaStreamConnector / media-transport adaptations).
- Snapshot/preset library (OcaLibraryManager, dataset objects).
- Control grouping agents (OcaGrouper) and matrix (OcaMatrix).
- Reconfigurable-DSP construction (`OcaBlock.ConstructActionObject`).
- Firmware-update manager and the other optional managers.
- IPv6 listener (the server currently binds IPv4 `INADDR_ANY`).

## Access control and TLS

Plain OCP.1 is unauthenticated: by default any controller that can reach the
device may read and write every object. Two mechanisms let you tighten that.

**Secured objects.** Mark an object so writes require a privileged session:

```c
aes70_object_handle_t xover = aes70_filter_create(dev, blk, "Split", ...);
aes70_object_set_secured(xover, true);   /* installer-only from now on */
```

A secured object can still be *read* by anyone, but any `Set*` (or `Lock`) from
an unprivileged session is rejected with OcaStatus `PermissionDenied`. `OcaLock`
is also enforced: a `NoWrite`/`NoReadWrite` lock blocks writes from every session
but the one that set it.

**Who is privileged.** A connection's privilege is decided once, when it opens,
by the `authorize` callback in `aes70_device_config_t`:

```c
cfg.authorize = my_authorize;   /* bool(peer, secure, client_authenticated, user) */
```

If you leave it `NULL`, the default policy is *privileged only for a
mutually-authenticated TLS client* — plaintext controllers are never privileged.

**Secure OCP.1 (TLS).** Enable `CONFIG_AES70_ENABLE_TLS` and pass a certificate
through the config to run an OCP.1-over-TLS listener, advertised as
`_ocasec._tcp`:

```c
cfg.tls.enable          = true;
cfg.tls.server_cert_pem = server_cert;   /* PEM */
cfg.tls.server_key_pem  = server_key;    /* PEM */
cfg.tls.client_ca_pem   = client_ca;     /* PEM; set => require & verify client certs */
// cfg.tls.disable_plaintext = true;     /* optional: TLS only */
```

With `client_ca_pem` set, a controller must present a certificate that verifies
against that CA; once it does, its session is privileged and may write secured
objects. The handshake is non-blocking with a bounded timeout so a stalled
client cannot tie up a connection slot. When TLS is left off, none of the
esp-tls/mbedTLS code is compiled into the image.

## Testing

Pure logic (codec, command router, access control) has host unit tests that
build with plain `gcc` — no ESP-IDF, no hardware:

```sh
components/aes70/test/host/run.sh        # builds, runs, prints gcov coverage
```

`tools/ocp1_smoketest.py` is an integration test that drives a running device
over OCP.1.

## Protocol fidelity

Every wire field was cross-checked against two independent, freely available
sources before implementation: the **Wireshark OCP.1 dissector**
(`epan/dissectors/packet-ocp1.c`), which parses real captures and is therefore
authoritative for field order and framing, and the **AES70-2 class
definitions** (docs.deuso.de), authoritative for ClassIDs, method indices and
data-type structures. The OCP.1 header, the inclusive per-message size fields,
the Command/Response/Notification layouts, the KeepAlive heartbeat encoding, and
the EV1 PropertyChanged notification body (Context blob followed by emitter +
event id + property id + value + change type) all follow those sources.

Beyond the source cross-check, the framing and class behaviour were confirmed
on real hardware with an OCP.1 controller (`tools/ocp1_smoketest.py`): every
field the device emits — header sizes, inclusive per-message sizes, ClassID /
ClassIdentification, OcaObjectIdentification / OcaBlockMember, float32 values and
the PropertyChanged notification body — round-trips correctly.

## Using the component

Add it to a project (copy into `components/`, add as a submodule, or reference
from the component registry once published), then:

```c
#include "aes70.h"
#include "aes70_object.h"

static void on_control_changed(aes70_object_handle_t obj, uint32_t tag, void *user)
{
    // A controller wrote a value. Read it and push it to your DSP.
    if (tag == MY_MASTER_GAIN_TAG) dsp_set_master_gain(aes70_gain_get(obj));
}

void app_main(void)
{
    // ... bring up the network (Ethernet/Wi-Fi) first ...

    aes70_device_config_t cfg;
    aes70_device_config_default(&cfg);
    cfg.device_name        = "My DSP";
    cfg.manufacturer       = "Acme Audio";
    cfg.model              = "DSP-8";
    cfg.serial_number      = "SN-0001";
    cfg.on_control_changed = on_control_changed;

    aes70_device_handle_t dev;
    ESP_ERROR_CHECK(aes70_device_start(&cfg, &dev));

    // Build the control tree. Parent NULL means the device root block.
    aes70_object_handle_t master = aes70_block_create(dev, NULL, "Master");
    aes70_object_handle_t gain   = aes70_gain_create(dev, master, "MasterGain",
                                                     -80.0f, 12.0f, 0.0f);
    aes70_object_set_tag(gain, MY_MASTER_GAIN_TAG);

    // The device can also push values to controllers (emits notifications):
    aes70_gain_set(gain, -6.0f);
}
```

Getters (`aes70_gain_get`, ...) are safe from any task. Setters
(`aes70_gain_set`, ...) are marshalled to the server task, so they are safe from
any task and never block on socket I/O. A value written by a controller raises
`on_control_changed`; a value written by the application instead notifies any
subscribed controllers.

### Threading and safety

A single internal task owns every socket, the connection table, the subscription
table and the object tree. Object values are guarded by a mutex that is **never**
held across socket I/O. Application setters post a request to that task rather
than touching the tree directly.

## Example: `examples/p4_nano_dsp`

A generic full-blown DSP device, built entirely from standard OCA objects (50
of them across 14 classes): a master section (gain/mute/polarity/pan/delay/
output meter), a signal generator, a 4-band parametric EQ, a compressor, a
3-band multiband compressor, a limiter, a 2-way crossover, and a system block
with a live chip-temperature sensor and an identify button. It contains no
product-specific internals; it is a template for wiring real DSP parameters to
OCA objects.

```bash
cd examples/p4_nano_dsp
idf.py set-target esp32p4      # P4-Nano Ethernet pin map is in sdkconfig.defaults.esp32p4
idf.py build flash monitor
```

The example defaults to **wired Ethernet** (the P4-Nano has an on-board IP101
PHY). On other targets, or to use Wi-Fi, flip `CONFIG_EXAMPLE_CONNECT_*` and set
the SSID/password under *Example Connection Configuration* in `idf.py menuconfig`.

Once it is on the network, open **AES70 Explorer**, let it discover the device
(`_oca._tcp`), and browse the object tree: move a gain, toggle a mute, change the
crossover slope, and watch the device log the change. Subscribe to the output
meter to see live notifications.

Configuration (device name, manufacturer, model, serial, TCP port) is under
*AES70 DSP Demo Configuration* in `menuconfig`.

## Configuration (component)

`idf.py menuconfig` → *AES70 (OCA) device*: object/connection/subscription table
sizes, the per-connection RX buffer and shared TX buffer, the keep-alive timeout
factor, and the server task stack/priority.

## License

MIT — see [LICENSE](LICENSE).
