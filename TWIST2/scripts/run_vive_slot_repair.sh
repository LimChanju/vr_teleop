#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
SDK="$ROOT/external/vive-ultimate-tracker-sdk"
SOURCE="$ROOT/tools/vive_slot_repair.cpp"
BUILD_DIR="$ROOT/runtime/bin"
BINARY="$BUILD_DIR/vive_slot_repair"
LIB="$SDK/bin/linux/x64/libultimate-tracker.so"

if [[ $(uname -m) != x86_64 ]]; then
  echo "This launcher currently targets the installed x86_64 SDK library." >&2
  exit 1
fi
if [[ ! -f $LIB ]]; then
  echo "Official VIVE Ultimate Tracker SDK library not found: $LIB" >&2
  exit 1
fi
if pgrep -x vivehub >/dev/null || pgrep -x tracker_server >/dev/null; then
  echo "VIVE Hub/tracker_server is running and owns the dongle." >&2
  echo "Stop it first with: bash scripts/run_vive_linux_hub.sh --stop" >&2
  exit 1
fi

mkdir -p "$BUILD_DIR"
if [[ ! -x $BINARY || $SOURCE -nt $BINARY || $LIB -nt $BINARY ]]; then
  g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
    -I"$SDK/include" \
    "$SOURCE" \
    -L"$(dirname "$LIB")" -lultimate-tracker -pthread \
    -Wl,-rpath,"$(dirname "$LIB")" \
    -o "$BINARY"
fi

exec "$BINARY" "$@"
