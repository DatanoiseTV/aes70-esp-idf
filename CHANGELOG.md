# Changelog

All notable changes to this project are documented here. The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project
adheres to [Semantic Versioning](https://semver.org/).

## [0.1.0] - 2026-06-19

### Added

- Initial AES70 (OCA) device-side component for ESP-IDF.
- **OCP.1 protocol (AES70-3)** over TCP: frame sync/header, the Command,
  Command-with-response, Response and Notification message types, and KeepAlive
  with heartbeat negotiation and idle-connection timeout. Big-endian marshalling
  for all base types (integers, float32/float64, OcaString, OcaBlob, OcaList,
  OcaClassID/OcaClassIdentification, OcaObjectIdentification, OcaBlockMember).
- **Extensible OCA object model**: class-descriptor dispatch by
  `(DefLevel, MethodIndex)` along the inheritance chain; per-object locking;
  `GetClassIdentification`; OcaBlock member enumeration
  (`GetActionObjects`/`GetActionObjectsRecursive`).
- **Control classes**: OcaBlock, OcaGain, OcaMute, OcaPolarity, OcaSwitch,
  OcaDelay, OcaBooleanActuator, OcaInt32/Uint16/Uint32/Float32/StringActuator,
  OcaLevelSensor.
- **Managers**: OcaDeviceManager (identity, model description, manager list) and
  OcaSubscriptionManager (EV1 + EV2 subscriptions, notification enable/disable).
- **Live PropertyChanged notifications** delivered as OCP.1 Notifications to
  subscribed controllers over the originating connection.
- **mDNS** advertisement of `_oca._tcp` with device-identity TXT records.
- Public API to build the object tree and read/write values from any task.
- Example `examples/p4_nano_dsp`: a generic DSP device (master, 10-band graphic
  EQ, compressor, limiter, 2-way crossover) for the ESP32-P4-Nano over Ethernet.

### Verified

- Builds clean (no warnings) on ESP-IDF v6.0 for esp32p4.
- Wire format cross-checked against the Wireshark OCP.1 dissector and the AES70-2
  class definitions.
- Exercised end-to-end on an ESP32-P4-Nano (rev v1.3) over Ethernet with
  `tools/ocp1_smoketest.py`: identity, recursive tree enumeration, gain
  get/set with range checking, error status codes, and subscription →
  PropertyChanged notification all pass.

### Known limitations

- Connection management, snapshot/preset library, control-grouping agents,
  reconfigurable-DSP construction, firmware-update manager, OCP.1-over-TLS and
  OCP.1 authentication are not yet implemented (documented extension points).
- The server listens on IPv4 only.
- GUI interop with AES70 Explorer not yet exercised (the underlying OCP.1
  protocol is verified on hardware).
