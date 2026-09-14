# Security Policy

## Supported versions

`tron2_env` has not published a stable release. Security fixes are prepared for
the current `main` branch and will be documented in `CHANGELOG.md` when released.

## Reporting a vulnerability

Do not report security vulnerabilities through a public issue, discussion, pull
request, log paste, or chat channel.

Use the repository hosting service's private vulnerability-reporting or security
advisory feature. If that feature is unavailable, contact the project maintainers
at `opensource@limxdynamics.com` and use the subject
`Private security report for tron2_env`. Send only a minimal initial report; the
maintainers will arrange a private channel if more sensitive detail is needed.

Include only the minimum information needed to reproduce and assess the issue:

- Affected version or commit.
- A concise description of the impact and attack conditions.
- Reproduction steps using placeholders or a mock transport where possible.
- Whether the issue can cause real-robot motion, expose observations, or cross a
  network trust boundary.
- A private way for maintainers to request additional details.

Do not include credentials, private robot addresses, camera serial numbers,
customer or site names, raw logs, images, datasets, or other deployment data.
Maintainers will acknowledge the report through the same private channel and
coordinate disclosure after the affected owners have assessed it.

## Supported network boundary

The robot-control WebSocket, TRON2 Bridge observation WebSockets, and OpenPI
policy WebSocket are supported only on a controlled robot LAN restricted to
authorised systems. They are not designed to establish a security boundary over
the Internet or an untrusted or shared network.

Do not expose these interfaces through public port forwarding or use them for
cross-site or cloud deployment without a separate security review and suitable
transport protection. The deployment owner is responsible for network
segmentation, firewalling, access control, and protecting camera images, robot
state and metadata, and policy actions.

## Security and safety scope

This process covers software vulnerabilities in the public package. Real-robot
safety, calibration, emergency-stop integration, low-level controllers, and
deployment-network policy require the applicable robot safety, control, hardware,
and security owners. The public package and CI are not a substitute for those
controls.
