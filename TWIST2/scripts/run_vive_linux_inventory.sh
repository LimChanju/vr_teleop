#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
exec "$ROOT/.conda-vr/bin/python" -u -m twist2_vr vive-linux-inventory "$@"
