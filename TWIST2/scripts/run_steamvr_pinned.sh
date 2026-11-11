#!/usr/bin/env bash
# Launch the restored runtime without asking Steam to update/launch App 250820.
# Valve and ALVR binaries remain unchanged; only ALVR's standard wrapper link is used.
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
PIN="$ROOT/runtime/steamvr-2.12.14"
STEAM_BASE=/home/railab/.steam/debian-installation
SCOUT="$STEAM_BASE/ubuntu12_32/steam-runtime/run.sh"
SNIPER="$STEAM_BASE/steamapps/common/SteamLinuxRuntime_sniper/_v2-entry-point"
PYTHON="$ROOT/.conda-vr/bin/python"
LOG="$ROOT/runtime/spacecal/steamvr-pinned-launch.log"
MODE=${1:---start}
[[ "$MODE" == --start || "$MODE" == --check ]] || { echo 'Usage: run_steamvr_pinned.sh [--start|--check]'; exit 2; }
if [[ "$MODE" == --start ]]; then
  # Steam Library, ALVR and the desktop shortcut may request a launch together.
  # Serialize only this launcher; the lock is released when it returns.
  mkdir -p "$ROOT/runtime/spacecal"
  exec 9>"$ROOT/runtime/spacecal/steamvr-pinned-launch.lock"
  flock -w 20 9 || { echo 'Another pinned SteamVR launch is still in progress.' >&2; exit 1; }
fi
[[ -x "$PIN/bin/linux64/vrserver" && -f "$PIN/bin/version.txt" ]] || {
  echo "Pinned SteamVR installation is missing: $PIN" >&2; exit 1;
}
[[ $(tr -d '\r\n ' < "$PIN/bin/version.txt") == 1755290071 ]] || {
  echo 'Refusing to launch: runtime version marker is not the pinned 2.12.14 build.' >&2; exit 1;
}
cd "$PIN"
sha256sum --status -c "$ROOT/config/steamvr_2.12.14.sha256" || {
  echo 'Pinned runtime files changed; inspect them before launching.' >&2; exit 1;
}
"$PYTHON" - "$PIN" "$MODE" <<'PY'
import json, os, sys
from pathlib import Path
pin = Path(sys.argv[1]).resolve()
paths = Path.home()/'.config/openvr/openvrpaths.vrpath'
print('Pinned runtime:', pin)
print('Registered runtime:', json.loads(paths.read_text()).get('runtime', []))
for p in Path('/proc').iterdir():
    if not p.name.isdigit():
        continue
    try:
        exe = (p/'exe').resolve(strict=True)
        if p.stat().st_uid != os.getuid() or exe.name not in ('vrserver', 'vrmonitor', 'vrcompositor', 'vrcompositor.real'):
            continue
    except OSError:
        continue
    print('Running:', p.name, exe)
    if sys.argv[2] == '--start' and not exe.is_relative_to(pin):
        raise SystemExit('Another SteamVR runtime is running. Close SteamVR first; no processes were stopped.')
PY
[[ "$MODE" != --check ]] || exit 0
[[ -x "$SCOUT" && -x "$SNIPER" ]] || { echo 'Steam Linux Runtime is missing.' >&2; exit 1; }
pgrep -u "$(id -u)" -x steam >/dev/null || { echo 'Open the Steam client first.' >&2; exit 1; }
pgrep -u "$(id -u)" -x alvr_dashboard >/dev/null || { echo 'Open ALVR Dashboard first.' >&2; exit 1; }
if pgrep -u "$(id -u)" -x vrmonitor >/dev/null; then
  echo 'Pinned SteamVR is already running.'
  exit 0
fi
# ALVR's Restart button may return before the previous user unit is collected.
# Wait briefly for normal cleanup; do not kill an unrelated runtime or service.
for ((attempt=0; attempt<20; attempt++)); do
  pin_unit_state=$(systemctl --user show twist2-steamvr-pinned.service --property=ActiveState --value 2>/dev/null || true)
  [[ -z "$pin_unit_state" || "$pin_unit_state" == inactive || "$pin_unit_state" == failed ]] && break
  sleep 0.5
done
if [[ -n "$pin_unit_state" && "$pin_unit_state" != inactive && "$pin_unit_state" != failed ]]; then
  echo 'The previous pinned SteamVR service is still exiting; try again shortly.' >&2
  exit 1
fi
if [[ "$pin_unit_state" == failed ]]; then
  systemctl --user reset-failed twist2-steamvr-pinned.service
fi
export DISPLAY=${DISPLAY:-:1}
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run/user/$(id -u)}
export XAUTHORITY=${XAUTHORITY:-$XDG_RUNTIME_DIR/gdm/Xauthority}
export DBUS_SESSION_BUS_ADDRESS=${DBUS_SESSION_BUS_ADDRESS:-unix:path=$XDG_RUNTIME_DIR/bus}
env LD_LIBRARY_PATH="$PIN/bin/linux64" "$PIN/bin/linux64/vrpathreg" setruntime "$PIN"
export XR_RUNTIME_JSON="$PIN/steamxr_linux64.json"
mkdir -p "$(dirname "$LOG")"
# Existing machine setting steamvr.disableAsync=true is preserved. Use Valve's
# helper entrypoint to avoid an unnecessary setcap/admin setup prompt here.
systemd-run --user --unit=twist2-steamvr-pinned --collect --service-type=exec \
  --property="WorkingDirectory=$PIN" --property="StandardOutput=append:$LOG" \
  --property="StandardError=append:$LOG" \
  env -u LD_LIBRARY_PATH -u VIRTUAL_ENV -u CONDA_PREFIX \
  -u STEAMVR_VRENV -u STEAMVR_TOOLSDIR \
  DISPLAY="$DISPLAY" XAUTHORITY="$XAUTHORITY" XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  DBUS_SESSION_BUS_ADDRESS="$DBUS_SESSION_BUS_ADDRESS" XR_RUNTIME_JSON="$XR_RUNTIME_JSON" \
  __NV_PRIME_RENDER_OFFLOAD=1 __VK_LAYER_NV_optimus=NVIDIA_only \
  __GLX_VENDOR_LIBRARY_NAME=nvidia VK_DRIVER_FILES=/usr/share/vulkan/icd.d/nvidia_icd.json \
  STEAM_COMPAT_CLIENT_INSTALL_PATH="$STEAM_BASE" SRT_LAUNCHER_SERVICE_ALONGSIDE_STEAM=1 \
  "$SCOUT" "$PIN/bin/vrmonitor.sh" "$SNIPER" --verb=waitforexitandrun -- \
  "$PIN/bin/vrstartup-helper.sh"
sleep 8
if ! systemctl --user is-active --quiet twist2-steamvr-pinned.service; then
  echo "SteamVR launcher exited. See $LOG" >&2
  tail -20 "$LOG"
  exit 1
fi
echo "Started pinned SteamVR 2.12.14 (user service twist2-steamvr-pinned). Log: $LOG"
echo 'Use this launcher instead of Steam Library / ALVR Launch SteamVR to keep this version.'
echo 'This command sends no robot commands. Close SteamVR from its window when finished.'
echo 'Or stop with: systemctl --user stop twist2-steamvr-pinned.service'
