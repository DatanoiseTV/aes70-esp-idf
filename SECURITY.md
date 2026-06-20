# Security policy

## Supported versions

This is pre-1.0 software. Security fixes are applied to the latest `0.x`
release; please track the most recent tag.

## Security model

Plain OCP.1 (the default `_oca._tcp` transport) is **unauthenticated and
unencrypted**, like the protocol itself. Any controller that can reach the
device on the network can read and write every object. Treat a device running
only the plaintext listener as you would any unauthenticated control surface:
keep it on a trusted, segmented network.

The component provides two mechanisms to harden a deployment:

- **Secured objects** (`aes70_object_set_secured()`) reject writes from
  unprivileged sessions with OcaStatus `PermissionDenied`, and `OcaLock` is
  enforced. This limits *what* an unprivileged controller can change, but does
  not by itself authenticate or encrypt the link.
- **Secure OCP.1 over TLS** (`CONFIG_AES70_ENABLE_TLS`, advertised as
  `_ocasec._tcp`) encrypts the connection and, with a client CA configured,
  authenticates the controller by certificate. A mutually-authenticated
  controller becomes privileged. To require security end-to-end, enable TLS,
  configure a client CA, and set `tls.disable_plaintext = true`.

Certificates and private keys are supplied by the application; protect the
server private key as you would any device secret.

## Reporting a vulnerability

Please report suspected vulnerabilities privately through GitHub's
"Report a vulnerability" (Security → Advisories) on this repository rather than
opening a public issue. Include the affected version, a description, and a
reproduction if possible. You will receive an acknowledgement, and a fix or
mitigation will be coordinated before public disclosure.
