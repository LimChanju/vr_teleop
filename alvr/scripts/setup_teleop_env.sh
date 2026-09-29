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
    echo "  conda create -n tv python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge" >&2
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
    if [[ ! -d "$cyclonedds_home/include/dds" ]] || \
       ! compgen -G "$cyclonedds_home/lib*/libddsc.so*" >/dev/null; then
        echo "CycloneDDS 0.10.x C installation not found or incomplete: $cyclonedds_home" >&2
        echo "Run unitree_sim_isaaclab/auto_setup_env.sh first; it normally builds:" >&2
        echo "  $vendor_root/cyclonedds/install" >&2
        echo "Alternatively build the CycloneDDS releases/0.10.x branch, then rerun:" >&2
        echo "  CYCLONEDDS_HOME=/path/to/cyclonedds/install TELEOP_PYTHON=$teleop_python bash $alvr_root/scripts/setup_teleop_env.sh" >&2
        exit 2
    fi
    export CYCLONEDDS_HOME="$cyclonedds_home"
    echo "Using CycloneDDS C installation: $CYCLONEDDS_HOME"
fi

"$teleop_python" -m pip install -r "$xr_root/requirements.txt"
"$teleop_python" -m pip install -e "$xr_root/teleop/teleimager"
"$teleop_python" -m pip install -e "$xr_root/teleop/robot_control/dex-retargeting"
"$teleop_python" -m pip install -e "$sdk_root"
"$teleop_python" -m pip install "numpy==1.26.4" "openvr==2.12.1401" "PyYAML>=6,<7"
"$teleop_python" -m pip check
"$teleop_python" - <<'PY'
from importlib.metadata import version
import cyclonedds
assert version("cyclonedds") == "0.10.2"
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
