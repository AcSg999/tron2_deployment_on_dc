#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
tron2_python="${TRON2_PYTHON:-python3.10}"
if ! command -v "$tron2_python" >/dev/null 2>&1; then
    echo "Python 3.10 is required. Set TRON2_PYTHON to an installed Python 3.10 executable; see docs/deployment.md." >&2
    exit 1
fi
"$tron2_python" -I -c 'import sys; sys.exit(0 if sys.version_info[:2] == (3, 10) else "Use Python 3.10 for the tested dependency set; keep Ubuntu system Python unchanged.")'
if [[ -e .venv ]]; then
    .venv/bin/python -I -c 'import sys; sys.exit(0 if sys.version_info[:2] == (3, 10) else "Existing .venv uses a different Python version; preserve it and create a separate checkout/environment.")'
else
    "$tron2_python" -I -m venv .venv
fi
.venv/bin/python -I -m pip install --upgrade pip
.venv/bin/python -I -m pip install -c constraints-ubuntu20-py310.txt '.[bridge,dev,ros]' ./third_party/tron2_env
.venv/bin/python -I -m pip check
