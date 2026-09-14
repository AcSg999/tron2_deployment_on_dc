#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
python3.10 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -c constraints-ubuntu22-py310.txt '.[bridge,dev]' ./third_party/tron2_env
