# Contributing to tron2_env

Contributions are welcome for the public TRON2 runtime package.

Please keep changes within the public scope of this package:

- WebSocket robot transport.
- Motion interpolation and command publishing.
- Bridge and legacy RealSense observation plumbing.
- RTC helper utilities.
- Documentation and public examples.

Please do not submit:

- Private robot profiles or `.local.yaml` files.
- Real robot IP addresses, camera serial numbers, credentials, or internal URLs.
- Customer data, private logs, debug image captures, datasets, or model weights.
- Undeveloped low-level SDK integrations.
- Safety-critical private controller code or unpublished robot safety policy.

Before submitting a pull request, run the software-only checks used by CI:

```bash
python -m pip install -e ".[dev]"
python -m compileall -q src tests examples
python -m pytest -q
python examples/mock_quickstart.py
python -m build
```

The public CI does not connect to a robot, TRON2 Bridge, or RealSense camera.
Describe any separately authorised hardware validation in the pull request;
never include robot addresses, serial numbers, credentials, logs, or captures.

Update README files when public behaviour changes. Update `NOTICE` when adding
or changing third-party code, and include its source, version or commit, license,
local path, and a summary of modifications.

Report security vulnerabilities using the private process in `SECURITY.md`, not
through a public issue.

Unless explicitly stated otherwise, submitted contributions are licensed under
the same license terms as this package.
