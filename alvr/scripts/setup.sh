#!/usr/bin/env bash
set -euo pipefail
probe_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
probe_python="${PYTHON_BIN:-python3}"

validate_probe_python() {
    local python_bin="$1"
    local label="$2"

    if ! command -v "$python_bin" >/dev/null 2>&1; then
        echo "$label not found: $python_bin" >&2
        exit 2
    fi
    if ! "$python_bin" -c 'import sys; raise SystemExit(not ((3, 10) <= sys.version_info[:2] < (3, 13)))'; then
        echo "$label must be Python 3.10, 3.11, or 3.12: $python_bin" >&2
        "$python_bin" --version >&2 || true
        exit 2
    fi
}

validate_probe_python "$probe_python" "PYTHON_BIN"

if [[ -e "$probe_root/.venv" && ! -x "$probe_root/.venv/bin/python" ]]; then
    echo "Existing probe virtual environment is incomplete: $probe_root/.venv" >&2
    echo "Move it aside, then rerun this script." >&2
    exit 2
fi
if [[ ! -x "$probe_root/.venv/bin/python" ]]; then
    "$probe_python" -m venv "$probe_root/.venv"
fi
validate_probe_python "$probe_root/.venv/bin/python" "Existing probe virtual environment"

"$probe_root/.venv/bin/python" -m pip install -r "$probe_root/requirements.lock.txt"
"$probe_root/.venv/bin/python" -m pip check
"$probe_root/.venv/bin/python" -c 'import openvr; print(f"OpenVR Python binding {openvr.__version__} import OK")'
