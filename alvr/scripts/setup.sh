#!/usr/bin/env bash
set -euo pipefail
probe_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
probe_python="${PYTHON_BIN:-python3}"
"$probe_python" -c 'import sys; assert sys.version_info[:2] >= (3, 10), "Use Python 3.10 or newer."'
if [[ ! -x "$probe_root/.venv/bin/python" ]]; then
    "$probe_python" -m venv "$probe_root/.venv"
fi
"$probe_root/.venv/bin/python" -m pip install -r "$probe_root/requirements.lock.txt"
"$probe_root/.venv/bin/python" -m pip check
"$probe_root/.venv/bin/python" -c 'import openvr; print(f"OpenVR Python binding {openvr.__version__} import OK")'
