#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [[ $# -lt 1 ]]; then
  echo "Usage: $0 RECORDING.jsonl [additional options]" >&2
  exit 2
fi
RECORDING=$1
shift
exec "$ROOT/.conda-vr/bin/python" -m twist2_vr replay --input "$RECORDING" --retargeter gmr "$@"
