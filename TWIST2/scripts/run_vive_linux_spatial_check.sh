#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
exec "$ROOT/.conda-vr/bin/python" -u -m twist2_vr.spatial_check "$@"
