#!/usr/bin/env bash
# Create an isolated Sim 4.5 environment; never install into an existing source.
set -euo pipefail

G1_REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
G1_SOURCE_ENV="unitree_sim_env"
G1_SOURCE_EXPLICIT=0
G1_FRESH=0
G1_TARGET_PREFIX=""
G1_LAB_DIR="$G1_REPO_ROOT/vendor/IsaacLab-g1"
G1_LAB_COMMIT="91ad4944f2b7fad29d52c04a5264a082bcaad71d"
G1_VR_PREFIX="$G1_REPO_ROOT/.venv-alvr"
G1_DRY_RUN=0
G1_NO_VR=0
G1_VR_REUSE=0
G1_CONDA_BIN="${CONDA_EXE:-}"

usage() {
  cat <<'HELP'
Usage: bash scripts/g1/setup.sh [options]
  --fresh                     Create Python3.10 and install pinned Sim4.5/Torch/Lab from packages
  --source-env NAME_OR_PREFIX  Existing working Python3.10 / IsaacSim4.5 / Torch2.5.1cu121 env
                              (default: unitree_sim_env)
  --target-prefix PATH        New environment path (default: CONDA_BASE/envs/g1_teleop)
  --isaaclab-dir PATH         Dedicated pinned checkout (default: vendor/IsaacLab-g1)
  --vr-prefix PATH            Separate input venv (default: .venv-alvr)
  --conda PATH                Conda executable
  --no-vr                     Skip separate ALVR input venv
  --dry-run                   Show mutation commands; do not clone/install
  -h, --help                  Show help

Existing target prefixes are never overwritten. NVIDIA driver/CUDA/ALVR/SteamVR
and the source Conda environment are not modified. Fresh mode requires Linux
x86_64 with GLIBC >=2.34 and downloads the official NVIDIA/PyTorch packages.
HELP
}

while (($#)); do
  case "$1" in
    --source-env|--target-prefix|--isaaclab-dir|--vr-prefix|--conda)
      (($# >= 2)) || { echo "Missing value for $1" >&2; exit 2; }
      case "$1" in
        --source-env) G1_SOURCE_ENV="$2"; G1_SOURCE_EXPLICIT=1 ;;
        --target-prefix) G1_TARGET_PREFIX="$2" ;;
        --isaaclab-dir) G1_LAB_DIR="$2" ;;
        --vr-prefix) G1_VR_PREFIX="$2" ;;
        --conda) G1_CONDA_BIN="$2" ;;
      esac
      shift 2 ;;
    --no-vr) G1_NO_VR=1; shift ;;
    --fresh) G1_FRESH=1; shift ;;
    --dry-run) G1_DRY_RUN=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done
if ((G1_FRESH && G1_SOURCE_EXPLICIT)); then
  echo '--fresh and --source-env are mutually exclusive.' >&2; exit 2
fi

if [[ -z "$G1_CONDA_BIN" ]]; then
  G1_CONDA_BIN="$(command -v conda || true)"
fi
if [[ -z "$G1_CONDA_BIN" && -x "$HOME/anaconda3/bin/conda" ]]; then
  G1_CONDA_BIN="$HOME/anaconda3/bin/conda"
fi
if [[ -z "$G1_CONDA_BIN" && -x "$HOME/miniconda3/bin/conda" ]]; then
  G1_CONDA_BIN="$HOME/miniconda3/bin/conda"
fi
[[ -n "$G1_CONDA_BIN" && -x "$G1_CONDA_BIN" ]] || { echo "Conda executable not found; pass --conda PATH" >&2; exit 1; }
command -v git >/dev/null || { echo "git is required" >&2; exit 1; }
G1_CONDA_BASE="$("$G1_CONDA_BIN" info --base)"
G1_TARGET_PREFIX="${G1_TARGET_PREFIX:-$G1_CONDA_BASE/envs/g1_teleop}"
G1_TARGET_PREFIX="$(realpath -m -- "$G1_TARGET_PREFIX")"
G1_LAB_DIR="$(realpath -m -- "$G1_LAB_DIR")"
G1_VR_PREFIX="$(realpath -m -- "$G1_VR_PREFIX")"
[[ ! -e "$G1_TARGET_PREFIX" && ! -L "$G1_TARGET_PREFIX" ]] || {
  echo "Target already exists; unchanged: $G1_TARGET_PREFIX. Choose a new --target-prefix." >&2; exit 1;
}
[[ -f "$G1_REPO_ROOT/config/g1/requirements-supplement.txt" ]] || {
  echo "Missing config/g1/requirements-supplement.txt; obtain the complete G1 branch/bundle." >&2; exit 1;
}

run() {
  if ((G1_DRY_RUN)); then
    printf '[dry-run]'
    printf ' %q' "$@"
    printf '\n'
  else
    "$@"
  fi
}

if ((G1_FRESH)); then
  [[ -f "$G1_REPO_ROOT/config/g1/requirements-fresh.txt" ]] || { echo 'Fresh requirements file missing' >&2; exit 1; }
  "$G1_CONDA_BASE/bin/python" -I -c '
import platform
libc, version = platform.libc_ver()
if platform.system() != "Linux" or platform.machine() != "x86_64" or libc != "glibc" or tuple(map(int,version.split(".")[:2])) < (2,34):
    raise SystemExit("Fresh IsaacSim4.5 pip install requires Linux x86_64 and GLIBC>=2.34")
print(f"Fresh install host preflight: {platform.system()} {platform.machine()} {libc}{version}")
'
else
  G1_SOURCE_ARGS=(-n "$G1_SOURCE_ENV")
  if [[ "$G1_SOURCE_ENV" == */* ]]; then
    G1_SOURCE_ENV="$(realpath -e -- "$G1_SOURCE_ENV")"
    G1_SOURCE_ARGS=(-p "$G1_SOURCE_ENV")
  fi
  # Metadata-only preflight does not launch Sim or initialize the GPU.
  env -u PYTHONPATH PYTHONNOUSERSITE=1 "$G1_CONDA_BIN" run --no-capture-output "${G1_SOURCE_ARGS[@]}" python -I -c '
import importlib.metadata as m, sys
expected = {"isaacsim": "4.5.0.0", "torch": "2.5.1+cu121"}
if sys.version_info[:2] != (3,10):
    raise SystemExit("Source environment must use Python 3.10")
for package, wanted in expected.items():
    actual = m.version(package)
    if actual != wanted:
        raise SystemExit(f"Source {package}={actual}; expected {wanted}. Source unchanged.")
print("Source version preflight passed: Python3.10, IsaacSim4.5.0.0, Torch2.5.1+cu121")
'
fi

if ((!G1_NO_VR)) && [[ -e "$G1_VR_PREFIX" || -L "$G1_VR_PREFIX" ]]; then
  [[ ! -L "$G1_VR_PREFIX" && -x "$G1_VR_PREFIX/bin/python" ]] || { echo "Existing VR path is not a venv" >&2; exit 1; }
  "$G1_VR_PREFIX/bin/python" -I -c '
import importlib.metadata as m
for name, expected in {"numpy":"1.26.4","openvr":"2.12.1401"}.items():
    if m.version(name) != expected: raise SystemExit(f"Existing VR venv has incompatible {name}; choose a new --vr-prefix")
print("Compatible input venv found; it will be reused without modifications")
'
  G1_VR_REUSE=1
fi

if [[ -e "$G1_LAB_DIR" || -L "$G1_LAB_DIR" ]]; then
  [[ -d "$G1_LAB_DIR/.git" && ! -L "$G1_LAB_DIR" ]] || { echo "IsaacLab directory already exists and is not a dedicated git checkout" >&2; exit 1; }
  [[ "$(git -C "$G1_LAB_DIR" rev-parse HEAD)" == "$G1_LAB_COMMIT" ]] || { echo "IsaacLab commit differs; pass a new --isaaclab-dir" >&2; exit 1; }
  [[ -z "$(git -C "$G1_LAB_DIR" status --porcelain)" ]] || { echo "IsaacLab checkout has changes; pass a new --isaaclab-dir" >&2; exit 1; }
else
  run git clone --filter=blob:none --no-checkout https://github.com/isaac-sim/IsaacLab.git "$G1_LAB_DIR"
  run git -C "$G1_LAB_DIR" checkout --detach "$G1_LAB_COMMIT"
fi

G1_TARGET_PYTHON="$G1_TARGET_PREFIX/bin/python"
if ((G1_FRESH)); then
  run "$G1_CONDA_BIN" create --yes --prefix "$G1_TARGET_PREFIX" python=3.10 pip
  run env -u PYTHONPATH PYTHONNOUSERSITE=1 "$G1_TARGET_PYTHON" -I -m pip install \
    'pip==26.2.1' 'setuptools==80.9.0' 'wheel==0.47.0'
  run env -u PYTHONPATH PYTHONNOUSERSITE=1 "$G1_TARGET_PYTHON" -I -m pip install \
    'torch==2.5.1+cu121' 'torchvision==0.20.1+cu121' --index-url https://download.pytorch.org/whl/cu121
  run env -u PYTHONPATH PYTHONNOUSERSITE=1 "$G1_TARGET_PYTHON" -I -m pip install \
    'isaacsim[all,extscache]==4.5.0.0' --extra-index-url https://pypi.nvidia.com
  run env -u PYTHONPATH PYTHONNOUSERSITE=1 "$G1_TARGET_PYTHON" -I -m pip install \
    -r "$G1_REPO_ROOT/config/g1/requirements-fresh.txt"
else
  run "$G1_CONDA_BIN" create --yes --prefix "$G1_TARGET_PREFIX" --clone "$G1_SOURCE_ENV"
  run env -u PYTHONPATH PYTHONNOUSERSITE=1 "$G1_TARGET_PYTHON" -I -m pip install \
    -r "$G1_REPO_ROOT/config/g1/requirements-supplement.txt"
  run env -u PYTHONPATH PYTHONNOUSERSITE=1 "$G1_TARGET_PYTHON" -I -m pip install --no-deps 'rsl-rl-lib==2.3.3'
fi
for G1_PACKAGE in isaaclab isaaclab_assets isaaclab_tasks isaaclab_rl isaaclab_mimic; do
  run env -u PYTHONPATH PYTHONNOUSERSITE=1 "$G1_TARGET_PYTHON" -I -m pip install --no-deps --no-build-isolation \
    -e "$G1_LAB_DIR/source/$G1_PACKAGE"
done
run env -u PYTHONPATH PYTHONNOUSERSITE=1 "$G1_TARGET_PYTHON" -I -m pip check
run env -u PYTHONPATH PYTHONNOUSERSITE=1 "$G1_TARGET_PYTHON" -I -c '
import importlib.metadata as m
for name, expected in {"isaacsim":"4.5.0.0","torch":"2.5.1+cu121","numpy":"1.26.4","rsl-rl-lib":"2.3.3"}.items():
    actual=m.version(name)
    if actual != expected: raise SystemExit(f"{name}: expected {expected}, got {actual}")
print("Target versions verified; run the graphical/headless simulation smoke test next.")
'

if ((!G1_NO_VR)); then
  if ((!G1_VR_REUSE)); then
    run "$G1_TARGET_PYTHON" -I -m venv "$G1_VR_PREFIX"
    run "$G1_VR_PREFIX/bin/python" -I -m pip install -r "$G1_REPO_ROOT/g1_teleop/vr/requirements.txt"
  fi
fi

if ((G1_DRY_RUN)); then
  echo 'Dry run finished. No environment or checkout was created.'
else
  printf 'Setup finished. Set the interpreter before using scripts/g1/run.sh:\nexport G1_PYTHON=%q\n' "$G1_TARGET_PYTHON"
  printf 'Input interpreter: %s/bin/python\n' "$G1_VR_PREFIX"
  echo 'Next: prepare the G1 assets and run the provided checkpoint evaluation/teleop command.'
fi
