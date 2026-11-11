#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
PYTHON="$ROOT/.venv-sparseposer/bin/python"
if [[ ! -x "$PYTHON" ]]; then
    echo "Run bash scripts/setup_sparseposer.sh first." >&2
    exit 1
fi
exec "$PYTHON" -u -m twist2_vr.sparseposer_cli "$@"
