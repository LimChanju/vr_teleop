#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
PYTHON="$ROOT/.conda-vr/bin/python"

echo "Vive full-body hardware-validation profile (dry-run; no Redis or robot output)."
echo "First copy/edit the mapping and coordinate examples after live-inventory."
exec "$PYTHON" -u -m twist2_vr vive-fullbody --frames 0 --rate-limit \
  --mapping "$ROOT/config/tracker_mapping.vive_fullbody.example.json" \
  --coordinates "$ROOT/config/coordinates.vive_fullbody.example.json" "$@"
