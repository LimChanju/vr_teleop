#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
exec "$ROOT/.conda-vr/bin/pytest" "$ROOT/tests/vr_adapter" "$@"
