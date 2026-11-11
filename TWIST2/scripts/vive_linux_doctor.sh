#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
VERSION=${VIVE_HUB_VERSION:-v1.0.3}
HUB=${VIVE_HUB_DIR:-$ROOT/external/VIVEHub-Linux-$VERSION}
FAILED=0

check() {
  if "$@" >/dev/null 2>&1; then
    echo "OK: $*"
  else
    echo "MISSING: $*" >&2
    FAILED=1
  fi
}

case "$(uname -m)" in
  x86_64|aarch64|arm64) echo "OK: supported architecture $(uname -m)" ;;
  *) echo "MISSING: unsupported architecture $(uname -m)" >&2; FAILED=1 ;;
esac
check command -v lsusb
check test -x "$ROOT/.conda-vr/bin/python"
check test -x "$HUB/run-vivehub.sh"
check test -f /etc/udev/rules.d/70-vive-tracker.rules

if lsusb -d 0bb4:0350 >/dev/null 2>&1; then
  echo "OK: VIVE Wireless Dongle 0bb4:0350 detected"
else
  echo "MISSING: VIVE Wireless Dongle 0bb4:0350" >&2
  FAILED=1
fi

if [[ -S /tmp/vut.sock ]] && "$ROOT/.conda-vr/bin/python" -m twist2_vr \
    vive-linux-inventory --duration 0.05 >/dev/null 2>&1; then
  echo "OK: VIVE Hub VUT protocol handshake on /tmp/vut.sock"
else
  echo "MISSING: working /tmp/vut.sock handshake (start VIVE Hub as the desktop user)" >&2
  FAILED=1
fi

exit "$FAILED"
