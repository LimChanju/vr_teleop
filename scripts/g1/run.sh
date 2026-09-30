#!/usr/bin/env bash
set -euo pipefail
G1_REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
G1_PYTHON="${G1_PYTHON:-$HOME/anaconda3/envs/g1_teleop/bin/python}"
if [[ ! -x "$G1_PYTHON" ]]; then
  echo "G1 Python not found: $G1_PYTHON. Run scripts/g1/setup.sh or set G1_PYTHON." >&2
  exit 1
fi
export PYTHONNOUSERSITE=1
export PYTHONUNBUFFERED=1
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-8}"
export MKL_NUM_THREADS="${MKL_NUM_THREADS:-8}"
export MPLBACKEND=Agg
export PYTHONPATH="$G1_REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
cd "$G1_REPO_ROOT"
exec "$G1_PYTHON" scripts/g1/run.py "$@"
