#!/usr/bin/env bash
# Remove ONLY the two registrations added by setup_space_calibrator.sh; keep files/backups.
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
"$ROOT/.conda-vr/bin/python" - <<'PY'
from pathlib import Path
import openvr
manifest = Path('runtime/spacecal/drivers/01spacecalibrator/bin/linux64/manifest.vrmanifest').resolve()
try:
    openvr.init(openvr.VRApplication_Background)
except Exception:
    print('SteamVR unavailable: overlay autolaunch settings were not changed.')
else:
    try:
        apps = openvr.VRApplications()
        if apps.isApplicationInstalled('spacecalibrator.linux'):
            apps.setApplicationAutoLaunch('spacecalibrator.linux', False)
            apps.removeApplicationManifest(str(manifest))
    finally:
        openvr.shutdown()
PY
STEAMVR=$("$ROOT/.conda-vr/bin/python" -c 'import json,pathlib; print(json.loads((pathlib.Path.home()/".config/openvr/openvrpaths.vrpath").read_text())["runtime"][0])')
for DRIVER in "$ROOT/runtime/spacecal/drivers/01spacecalibrator" "$ROOT/runtime/spacecal/drivers/twist2_vut"; do
  env LD_LIBRARY_PATH="$STEAMVR/bin/linux64:${LD_LIBRARY_PATH:-}" \
    "$STEAMVR/bin/linux64/vrpathreg" removedriver "$DRIVER"
done
echo 'Only project Space Calibrator/VUT driver registrations removed. ALVR and files retained.'
echo 'Close the calibration session and restart SteamVR to unload the drivers.'
