#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [[ $# -lt 1 ]]; then
  echo "Usage: bash scripts/run_vive_linux_alignment_check.sh TRACKER_SERIAL [extra options]" >&2
  exit 2
fi
SERIAL=$1
shift
cd "$ROOT"
exec "$ROOT/.conda-vr/bin/python" -u -m twist2_vr.alignment_session --tracker-serial "$SERIAL" "$@"
