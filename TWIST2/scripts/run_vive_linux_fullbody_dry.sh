#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

echo "Linux VIVE Ultimate + Quest full-body validation (dry-run only)."
echo "No Redis simulation or physical G1 command will be published."
exec "$ROOT/.conda-vr/bin/python" -u -m twist2_vr vive-linux-fullbody \
  --frames 0 --rate-limit \
  --mapping "$ROOT/config/local/tracker_mapping.vive_linux.json" \
  --cross-calibration "$ROOT/config/local/calibration.quest_vive.json" \
  --coordinates "$ROOT/config/local/coordinates.vive_linux_fullbody.json" \
  --calibration "$ROOT/config/local/calibration.vive_linux.json" "$@"
