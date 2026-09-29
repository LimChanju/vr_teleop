#!/usr/bin/env bash
set -euo pipefail
probe_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONUNBUFFERED=1
export PYTHONDONTWRITEBYTECODE=1
cd "$probe_root"
if [[ ! -x .venv/bin/python ]]; then
    echo 'Run first: bash scripts/setup.sh' >&2
    exit 1
fi
exec .venv/bin/python quest3_probe.py "$@"
