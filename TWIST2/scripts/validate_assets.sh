#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
exec "$ROOT/.conda-vr/bin/python" "$ROOT/scripts/validate_twist2_assets.py" \
  --output "$ROOT/runtime/asset_validation.json" "$@"
