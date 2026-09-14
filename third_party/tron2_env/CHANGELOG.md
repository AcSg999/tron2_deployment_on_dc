# Changelog

All notable public changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project intends to use [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added

- Public security-reporting policy.
- Issue and pull-request templates with robot-safety and sensitive-data checks.
- Software-only CI for supported Python versions.
- A no-hardware mock transport Quick Start.

### Changed

- Completed the LeRobot ActionQueue provenance, Apache-2.0 attribution, retained
  upstream copyright, LimX modification notice, and current distribution scope.
- Aligned the README provenance wording with `NOTICE` and added an auditable
  license inventory for every direct runtime, optional, development, and build
  dependency declared in `pyproject.toml`.
- Set the fallback private vulnerability-reporting address to the LimX IT
  security contact.
- Documented the supported network trust boundary for robot control, Bridge
  observations, and OpenPI policy communication.

### Known limitations

- The public CI does not connect to a TRON2 robot, TRON2 Bridge, or RealSense
  camera and does not validate calibration or real-hardware safety behaviour.
- Real-robot use requires separately approved bring-up, emergency-stop, network,
  and safety procedures.
- The public runtime currently exposes the WebSocket transport only.

## [0.1.0] - Unreleased

### Added

- Initial public-source candidate for TRON2 runtime communication, motion,
  observations, and RTC helper utilities.
