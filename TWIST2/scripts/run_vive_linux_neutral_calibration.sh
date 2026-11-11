#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
exec "$ROOT/.conda-vr/bin/python" -u -m twist2_vr vive-linux-neutral-calibrate \
  --mapping "$ROOT/config/local/tracker_mapping.vive_linux.json" \
  --cross-calibration "$ROOT/config/local/calibration.quest_vive.json" \
  --coordinates "$ROOT/config/local/coordinates.vive_linux_fullbody.json" \
  --output "$ROOT/config/local/calibration.vive_linux.json" "$@"
