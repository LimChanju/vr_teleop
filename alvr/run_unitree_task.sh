#!/usr/bin/env bash
set -euo pipefail
alvr_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
launcher_python="${LAUNCHER_PYTHON:-python3}"
exec "$launcher_python" "$alvr_root/unitree_task_launcher.py" "$@"
