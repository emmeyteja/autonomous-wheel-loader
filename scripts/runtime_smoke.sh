#!/usr/bin/env bash
# This verifies the UPSTREAM task, not the wheel-loader environment.
set -euo pipefail
cd "$(dirname "$0")/.."
.venv/bin/python scripts/probe_runtime.py "$@"
