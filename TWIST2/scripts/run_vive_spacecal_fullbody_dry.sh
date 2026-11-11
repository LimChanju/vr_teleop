#!/usr/bin/env bash
# Corrected SteamVR poses only: DO NOT load calibration.quest_vive.json again.
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
echo 'SpaceCalibrated OpenVR input → anatomical adapter → GMR (dry-run only).'
echo 'Requires successful xi-ve calibration; not proof of valid anatomical mounts.'
exec "$ROOT/.conda-vr/bin/python" -u -m twist2_vr live \
  --mapping "$ROOT/config/local/tracker_mapping.vive_linux.json" \
  --coordinates "$ROOT/config/local/coordinates.vive_linux_fullbody.json" \
  --calibration "$ROOT/config/local/calibration.vive_linux.json" \
  --frames 0 --rate-limit "$@"
