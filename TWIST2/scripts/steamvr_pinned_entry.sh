#!/usr/bin/env bash
# Steam Launch Options passes %command% here, but it belongs to the managed
# SteamVR installation. Never execute or eval it: select our verified copy.
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
bash "$ROOT/scripts/run_steamvr_pinned.sh" --start
# Keep Steam's managed launch alive for its running/stopped state and Stop
# button, although the VR runtime itself lives in a dedicated user service.
stop_pinned() {
  systemctl --user stop twist2-steamvr-pinned.service
  exit 0
}
trap stop_pinned TERM INT
while systemctl --user is-active --quiet twist2-steamvr-pinned.service; do
  sleep 1
done
