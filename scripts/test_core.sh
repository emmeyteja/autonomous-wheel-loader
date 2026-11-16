#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
loader_python=python3
if [[ -x .venv/bin/python ]]; then loader_python=.venv/bin/python; fi
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 "$loader_python" -m pytest tests/unit -q "$@"
