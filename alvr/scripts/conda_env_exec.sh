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

# VS Code terminals can have a Python venv layered on top of Conda.  Conda
# replaces its own PATH entry when activating another environment but leaves
# the nested venv first, so an unqualified `python` would still use that venv.
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
    venv_bin="${VIRTUAL_ENV%/}/bin"
    IFS=: read -r -a inherited_path_entries <<< "${PATH:-}"
    clean_path_entries=()
    for path_entry in "${inherited_path_entries[@]}"; do
        if [[ "$path_entry" != "$venv_bin" ]]; then
            clean_path_entries+=("$path_entry")
        fi
    done
    IFS=:
    PATH="${clean_path_entries[*]}"
    unset IFS
    export PATH
    unset VIRTUAL_ENV VIRTUAL_ENV_PROMPT
fi

conda_base="$("$conda_executable" info --base)"
source "$conda_base/etc/profile.d/conda.sh"
conda activate "$env_name"

# Replacing this shell keeps the simulator as the process-group leader.  That
# lets the parent launcher reliably stop Isaac Sim and all of its workers.
exec "$@"
