# Third-Party Dependency License Inventory

This inventory covers the direct dependencies declared in `pyproject.toml` for
the current `tron2-env` source distribution and wheel. These artifacts contain
dependency requirement metadata only: they do not bundle the source code,
binaries, or license texts of the packages listed below. Package installers
resolve and install them separately.

The version constraints below are not a lock file. The exact versions resolved
for a release environment must be captured and reviewed during release CI.

## Runtime dependencies

| Dependency | Declared constraint | License | Purpose | Upstream license |
|---|---:|---|---|---|
| NumPy | `>=1.22` | BSD-3-Clause | Array and numeric operations | [NumPy license](https://github.com/numpy/numpy/blob/main/LICENSE.txt) |
| opencv-python | `>=4.8` | MIT (Python packaging); bundled OpenCV and third-party notices also apply to the external wheel | Image processing | [Packaging license](https://github.com/opencv/opencv-python/blob/4.x/LICENSE.txt), [third-party notices](https://github.com/opencv/opencv-python/blob/4.x/LICENSE-3RD-PARTY.txt) |
| Pillow | `>=10` | MIT-CMU | Image objects and conversion | [Pillow license](https://github.com/python-pillow/Pillow/blob/main/LICENSE) |
| websocket-client | `>=1.8` | Apache-2.0 | Robot WebSocket transport | [websocket-client license](https://github.com/websocket-client/websocket-client/blob/master/LICENSE) |

## Optional runtime dependencies

| Extra | Dependency | Declared constraint | License | Purpose | Upstream license |
|---|---|---:|---|---|---|
| `bridge` | websockets | `>=12` | BSD-3-Clause | Async TRON2 Bridge transport | [websockets license](https://github.com/python-websockets/websockets/blob/main/LICENSE) |
| `camera` | pyrealsense2 | unconstrained | Apache-2.0 (librealsense) | RealSense camera integration | [librealsense license](https://github.com/IntelRealSense/librealsense/blob/master/LICENSE) |
| `openpi` | einops | `>=0.8` | MIT | Tensor reshaping for OpenPI integration | [einops license](https://github.com/arogozhnikov/einops/blob/master/LICENSE) |

The `all` extra is the union of the `bridge`, `camera`, and `openpi` optional
runtime dependencies and introduces no additional package.

## Development and build dependencies

| Dependency | Declared constraint | License | Scope | Upstream license |
|---|---:|---|---|---|
| build | `>=1.2` | MIT | Development extra; package build frontend | [build license](https://github.com/pypa/build/blob/main/LICENSE) |
| pytest | `>=8` | MIT | Development extra; tests | [pytest license](https://github.com/pytest-dev/pytest/blob/main/LICENSE) |
| setuptools | `>=77` | MIT | Isolated build-system requirement | [setuptools license](https://github.com/pypa/setuptools/blob/main/LICENSE) |
| wheel | unconstrained | MIT | Isolated build-system requirement | [wheel license](https://github.com/pypa/wheel/blob/main/LICENSE.txt) |

## Distribution boundary and release checks

- The current `tron2-env` wheel and source distribution do not redistribute the
  listed packages. They are installed separately by the user's package manager.
- The external `opencv-python` wheel has its own binary distribution obligations.
  Its upstream third-party notice states that all variants redistribute FFmpeg
  and that non-headless Linux and macOS variants redistribute Qt 5. The current
  upstream build configuration explicitly enables Qt 5 only for non-headless
  Linux CI wheels, so those upstream signals are not fully aligned. If LimX
  later redistributes an `opencv-python` wheel, a container containing it, or
  an offline bundle, inspect the exact target wheel and its included notices
  instead of inferring obligations from this inventory.
- `pyrealsense2` is also an external native/binary package. Any future
  redistribution requires a separate artifact-level license review.
- Before a release, compare this inventory with `pyproject.toml`. CI records
  installed core/development versions with `python -m pip list --format=freeze`,
  resolves every optional extra with `python -m pip install --dry-run
  ".[all,dev]"`, and logs isolated build-system versions during the verbose
  package build. Review any newly introduced or changed dependency.
- Docker images, SDK bundles, offline installers, firmware, model weights, and
  datasets are outside this inventory. Each requires its own dependency/SBOM
  and license review before publication.
