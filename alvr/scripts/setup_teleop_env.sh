#!/usr/bin/env bash
set -euo pipefail

alvr_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
repo_root="$(cd -- "$alvr_root/.." && pwd)"
vendor_root="${VENDOR_ROOT:-$repo_root/vendor}"
xr_root="${XR_TELEOPERATE_ROOT:-$vendor_root/xr_teleoperate}"
sdk_root="${UNITREE_SDK_ROOT:-$vendor_root/unitree_sdk2_python}"
teleop_python="${TELEOP_PYTHON:-/home/railab/miniconda3/envs/tv/bin/python}"

if ! command -v "$teleop_python" >/dev/null 2>&1; then
    echo "Teleop Python not found: $teleop_python" >&2
    echo "Create the official environment first:" >&2
    echo "  conda create -n tv python=3.10 pinocchio=3.1.0 casadi=3.6.7 numpy=1.26.4 -c conda-forge" >&2
    exit 2
fi
if ! "$teleop_python" -c 'import sys; raise SystemExit(not ((3, 10) <= sys.version_info[:2] < (3, 12)))'; then
    echo "Teleop Python must be Python 3.10 or 3.11 (<3.12): $teleop_python" >&2
    "$teleop_python" --version >&2 || true
    echo "The pinned teleimager package does not support Python 3.12 or newer." >&2
    exit 2
fi

for required in \
    "$xr_root/requirements.txt" \
    "$xr_root/teleop/teleimager/pyproject.toml" \
    "$xr_root/teleop/robot_control/dex-retargeting/pyproject.toml" \
    "$sdk_root/setup.py"; do
    if [[ ! -e "$required" ]]; then
        echo "Missing source: $required" >&2
        echo "Run setup_unitree_sources.sh first." >&2
        exit 2
    fi
done

if "$teleop_python" - <<'PY' >/dev/null 2>&1
from importlib.metadata import version
import cyclonedds
raise SystemExit(version("cyclonedds") != "0.10.2")
PY
then
    echo "CycloneDDS Python 0.10.2 is already available."
else
    cyclonedds_home="${CYCLONEDDS_HOME:-$vendor_root/cyclonedds/install}"
    if [[ -d "$cyclonedds_home/include/dds" ]] && \
       compgen -G "$cyclonedds_home/lib*/libddsc.so*" >/dev/null; then
        export CYCLONEDDS_HOME="$cyclonedds_home"
        echo "Using CycloneDDS C installation: $CYCLONEDDS_HOME"
        "$teleop_python" -m pip install "cyclonedds==0.10.2"
    else
        echo "CycloneDDS C SDK not found; installing the self-contained 0.10.2 wheel."
        if ! "$teleop_python" -m pip install --only-binary=:all: "cyclonedds==0.10.2"; then
            echo "CycloneDDS wheel installation failed and no C SDK is available at:" >&2
            echo "  $cyclonedds_home" >&2
            echo "Build CycloneDDS 0.10.2, then rerun with CYCLONEDDS_HOME set." >&2
            exit 2
        fi
    fi
fi

"$teleop_python" -m pip install -r "$xr_root/requirements.txt"
"$teleop_python" -m pip install \
    "numpy==1.26.4" \
    "opencv-python==4.11.0.86" \
    "logging_mp==0.2.5" \
    "pyzmq==27.0.0" \
    "openvr==2.12.1401" \
    "PyYAML>=6,<7"

# Upstream dex-retargeting declares the PyPI distribution `pin>=2.7`.  The
# official Unitree environment supplies Pinocchio 3.1.0 from conda instead,
# whose distribution metadata is intentionally different.  Installing the
# editable project with dependency resolution would therefore add a second,
# incompatible Pinocchio (currently PyPI pin 4.x) to the same environment.
if ! "$teleop_python" - <<'PY' >/dev/null 2>&1
from importlib.metadata import version
raise SystemExit(version("torch").split("+", 1)[0] != "2.3.0")
PY
then
    "$teleop_python" -m pip install "torch==2.3.0+cpu" \
        --index-url https://download.pytorch.org/whl/cpu
fi
"$teleop_python" -m pip install \
    "pytransform3d>=3.5.0" \
    "nlopt>=2.6.1,<2.8.0" \
    "trimesh>=4.4.0" \
    "anytree>=2.12.0" \
    "lxml>=5.2.2"
"$teleop_python" -m pip install --no-deps -e "$xr_root/teleop/teleimager"
"$teleop_python" -m pip install --no-deps -e "$xr_root/teleop/robot_control/dex-retargeting"
"$teleop_python" -m pip install --no-deps -e "$sdk_root"
pip_check_output="$("$teleop_python" -m pip check 2>&1 || true)"
unexpected_pip_check="$(printf '%s\n' "$pip_check_output" | \
    grep -vFx 'dex-retargeting 0.4.7 requires pin, which is not installed.' | \
    grep -vFx 'No broken requirements found.' || true)"
if [[ -n "$unexpected_pip_check" ]]; then
    echo "$pip_check_output" >&2
    exit 1
fi
"$teleop_python" - <<'PY'
from importlib.metadata import PackageNotFoundError, version
import casadi
import cyclonedds
import pinocchio

assert version("cyclonedds") == "0.10.2"
assert version("logging_mp") == "0.2.5"
assert version("numpy") == "1.26.4"
assert version("opencv-python") == "4.11.0.86"
assert version("torch").split("+", 1)[0] == "2.3.0"
assert pinocchio.__version__ == "3.1.0"
import pinocchio.casadi
try:
    version("pin")
except PackageNotFoundError:
    pass
else:
    raise AssertionError("PyPI pin must not be installed over conda Pinocchio 3.1.0")
PY

(
    cd "$xr_root/teleop"
    PYTHONPATH="$alvr_root/shim:$alvr_root:$xr_root${PYTHONPATH:+:$PYTHONPATH}" \
        "$teleop_python" - <<'PY'
import casadi
import cv2
import dex_retargeting
import logging_mp
import numpy
import openvr
import pinocchio
import unitree_sdk2py
import yaml
import zmq
from teleimager.image_client import ImageClient
from teleop.robot_control.robot_arm import G1_29_ArmController
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK
from teleop.robot_control.robot_hand_inspire import Inspire_Controller_DFX
from teleop.robot_control.robot_hand_unitree import Dex1_1_Gripper_Controller, Dex3_1_Controller
from televuer import TeleVuerWrapper

print("teleop environment and runtime imports: OK")
PY
)
