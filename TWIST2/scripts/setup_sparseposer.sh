#!/usr/bin/env bash
# Separate wrapper environment. Never alter upstream or the working VR env.
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
UPSTREAM="$ROOT/external/SparsePoser"
PIN=6d40a03d71379d59c3b0469f87e5113c6bf0d4d2
BASE_PYTHON="${SPARSEPOSER_BASE_PYTHON:-$ROOT/.conda-vr/bin/python}"
if [[ ! -x "$BASE_PYTHON" ]]; then
    echo "Need existing Python with torch, numpy, scipy, openvr (set SPARSEPOSER_BASE_PYTHON)." >&2
    exit 1
fi
if [[ ! -e "$UPSTREAM" ]]; then
    git clone https://github.com/UPC-ViRVIG/SparsePoser.git "$UPSTREAM"
    git -C "$UPSTREAM" switch --detach "$PIN"
fi
if [[ "$(git -C "$UPSTREAM" rev-parse HEAD)" != "$PIN" || -n "$(git -C "$UPSTREAM" status --porcelain)" ]]; then
    echo "Preserving existing upstream checkout. Expected clean commit $PIN." >&2
    exit 1
fi
if [[ ! -e "$ROOT/.venv-sparseposer" ]]; then
    "$BASE_PYTHON" -m venv --system-site-packages "$ROOT/.venv-sparseposer"
fi
"$ROOT/.venv-sparseposer/bin/python" -m pip install upc-pymotion==0.1.8 matplotlib==3.10.0
echo "Upstream code: MIT; pretrained weights/data: CC BY-SA 4.0. See external/SparsePoser/README.md."
bash scripts/run_sparseposer_fullbody.sh doctor
