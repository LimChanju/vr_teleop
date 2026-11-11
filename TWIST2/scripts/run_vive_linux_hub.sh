#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
VERSION=${VIVE_HUB_VERSION:-v1.0.3}
HUB=${VIVE_HUB_DIR:-$ROOT/external/VIVEHub-Linux-$VERSION}
PID_FILE="$ROOT/runtime/vivehub.pid"
LAUNCH_LOG="$ROOT/runtime/vivehub-launcher.log"

if [[ ! -x "$HUB/run-vivehub.sh" ]]; then
  echo "VIVE Hub Linux is not installed at $HUB" >&2
  echo "Run: bash scripts/setup_vive_linux.sh --accept-vive-eula" >&2
  exit 1
fi

is_managed_process() {
  local pid=${1:-}
  [[ $pid =~ ^[0-9]+$ ]] || return 1
  kill -0 "$pid" 2>/dev/null || return 1
  [[ $(readlink -f "/proc/$pid/exe" 2>/dev/null || true) == "$(readlink -f "$HUB/vivehub")" ]]
}

case ${1:-foreground} in
  --background)
    mkdir -p "$ROOT/runtime"
    if [[ -f $PID_FILE ]] && is_managed_process "$(<"$PID_FILE")"; then
      echo "VIVE Hub is already running (PID $(<"$PID_FILE"))."
      exit 0
    fi
    nohup "$HUB/run-vivehub.sh" >"$LAUNCH_LOG" 2>&1 &
    pid=$!
    echo "$pid" >"$PID_FILE"
    sleep 1
    if ! is_managed_process "$pid"; then
      echo "VIVE Hub exited during startup; inspect $LAUNCH_LOG" >&2
      exit 1
    fi
    echo "Started VIVE Hub in the desktop session (PID $pid)."
    echo "Launcher log: $LAUNCH_LOG"
    ;;
  --status)
    if [[ -f $PID_FILE ]] && is_managed_process "$(<"$PID_FILE")"; then
      echo "VIVE Hub is running (PID $(<"$PID_FILE"))."
    else
      echo "VIVE Hub is not running under this launcher."
      exit 1
    fi
    ;;
  --stop)
    if [[ ! -f $PID_FILE ]] || ! is_managed_process "$(<"$PID_FILE")"; then
      echo "No managed VIVE Hub process is running."
      exit 0
    fi
    pid=$(<"$PID_FILE")
    kill -TERM "$pid"
    for _ in {1..50}; do
      is_managed_process "$pid" || break
      sleep 0.1
    done
    if is_managed_process "$pid"; then
      echo "VIVE Hub did not stop after SIGTERM; close it from the UI." >&2
      exit 1
    fi
    rm -f "$PID_FILE"
    echo "Stopped VIVE Hub."
    ;;
  foreground|--foreground)
    exec "$HUB/run-vivehub.sh"
    ;;
  *)
    echo "Usage: bash scripts/run_vive_linux_hub.sh [--background|--status|--stop|--foreground]" >&2
    exit 2
    ;;
esac
