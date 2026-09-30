#!/usr/bin/env bash
set -eo pipefail

if [[ $# -lt 2 ]]; then
    echo "Usage: $0 ENV_NAME COMMAND [ARG ...]" >&2
    exit 2
fi

env_name="$1"
shift
conda_executable="${CONDA_EXE:-/home/railab/miniconda3/bin/conda}"
if [[ ! -x "$conda_executable" ]]; then
    echo "Conda executable not found: $conda_executable" >&2
    exit 2
fi

conda_base="$("$conda_executable" info --base)"
source "$conda_base/etc/profile.d/conda.sh"
conda activate "$env_name"

# Replacing this shell keeps the simulator as the process-group leader.  That
# lets the parent launcher reliably stop Isaac Sim and all of its workers.
exec "$@"
