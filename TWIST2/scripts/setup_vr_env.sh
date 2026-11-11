#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
CONDA=${CONDA_EXE:-/home/railab/miniconda3/bin/conda}
PREFIX="$ROOT/.conda-vr"
GMR="$ROOT/external/GMR"
GMR_COMMIT=bb1bbe40774794fceb2a7c579a3464a28e68c844

if [[ ! -x "$PREFIX/bin/python" ]]; then
  "$CONDA" create -y -p "$PREFIX" python=3.10 pip
fi

if [[ ! -d "$GMR/.git" ]]; then
  mkdir -p "$ROOT/external"
  git clone https://github.com/YanjieZe/GMR.git "$GMR"
fi
git -C "$GMR" checkout "$GMR_COMMIT"

"$PREFIX/bin/python" -m pip install -r "$ROOT/requirements-vr.txt"
"$PREFIX/bin/python" -m pip install 'torch==2.5.1+cpu' --index-url https://download.pytorch.org/whl/cpu
"$PREFIX/bin/python" -m pip install -e "$ROOT" --no-deps
"$PREFIX/bin/python" -m pip install -e "$ROOT/pose" --no-deps
"$PREFIX/bin/python" -m pip install -e "$GMR" --no-deps
"$CONDA" install -y -p "$PREFIX" -c conda-forge redis

echo "Ready: $PREFIX/bin/python"
echo "Run tests: $ROOT/scripts/run_vr_tests.sh"
