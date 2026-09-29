#!/usr/bin/env bash
set -euo pipefail
probe_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
probe_python="${PYTHON_BIN:-python3}"
"$probe_python" -c 'import sys; assert (3, 10) <= sys.version_info[:2] < (3, 13), "Use Python 3.10–3.12 (tested: 3.10)."'
if [[ ! -x "$probe_root/.venv/bin/python" ]]; then
    "$probe_python" -m venv "$probe_root/.venv"
fi
"$probe_root/.venv/bin/python" -m pip install -r "$probe_root/requirements.lock.txt"
"$probe_root/.venv/bin/python" -m pip check
"$probe_root/.venv/bin/python" -c 'from televuer import TeleVuer; print("TeleVuer import OK")'
