#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
# Live output is deliberately dry-run only until hardware identity, mount
# offsets, calibration, and a physical deadman input are verified.
exec "$ROOT/.conda-vr/bin/python" -m twist2_vr live --retargeter gmr --rate-limit "$@"
