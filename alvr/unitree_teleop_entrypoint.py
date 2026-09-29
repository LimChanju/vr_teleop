#!/usr/bin/env python3
"""Run pinned xr_teleoperate with an ALVR arm-command safety overlay.

The upstream hand controllers honor ``motion_data_ready``, but its main loop
still solves/sends IK while that flag is false and its arm publisher repeats the
last target if the main loop stalls.  This verified in-memory overlay adds a
measured-position hold and an independent command heartbeat watchdog.
"""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import sys


UPSTREAM_NUMPY_ANCHOR = "import time\n"
GATED_NUMPY_ANCHOR = (
    "import time\n"
    "import numpy as np  # ALVR safety overlay\n"
    "from arm_safety import ArmCommandSafety  # ALVR safety overlay\n"
)
UPSTREAM_ARM_INIT_ANCHOR = "\n        # end-effector\n"
GATED_ARM_INIT_ANCHOR = (
    "\n        arm_safety = ArmCommandSafety(arm_ctrl, xr_motion_data_ready)\n"
    "\n        # end-effector\n"
)
UPSTREAM_READY_ASSIGNMENT = (
    "            with xr_motion_data_ready.get_lock():\n"
    "                xr_motion_data_ready.value = tele_data.motion_data_ready\n"
)
GATED_READY_ASSIGNMENT = (
    "            # ALVR overlay owns readiness; hands enable only after a safe arm command.\n"
    "            if not tele_data.motion_data_ready:\n"
    "                arm_safety.hold(released=True)\n"
)
UPSTREAM_IK_BLOCK = """            # get current robot state data.
            current_lr_arm_q  = arm_ctrl.get_current_dual_arm_q()
            current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()

            # solve ik using motor data and wrist pose, then use ik results to control arms.
            time_ik_start = time.time()
            sol_q, sol_tauff  = arm_ik.solve_ik(tele_data.left_wrist_pose, tele_data.right_wrist_pose, current_lr_arm_q, current_lr_arm_dq)
            time_ik_end = time.time()
            logger_mp.debug(f"ik:\\t{round(time_ik_end - time_ik_start, 6)}")
            arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)

"""
GATED_IK_BLOCK = """            # ALVR overlay: skip IK and hold measured joints unless safely armed.
            current_lr_arm_q  = arm_ctrl.get_current_dual_arm_q()
            current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()
            arm_safety.observe_measured(current_lr_arm_q)
            if tele_data.motion_data_ready and arm_safety.can_command():
                time_ik_start = time.time()
                sol_q, sol_tauff = arm_ik.solve_ik(tele_data.left_wrist_pose, tele_data.right_wrist_pose, current_lr_arm_q, current_lr_arm_dq)
                time_ik_end = time.time()
                logger_mp.debug(f"ik:\\t{round(time_ik_end - time_ik_start, 6)}")
                if not arm_safety.command(sol_q, sol_tauff):
                    sol_q = current_lr_arm_q.copy()
                    sol_tauff = np.zeros_like(current_lr_arm_q)
            else:
                sol_q = current_lr_arm_q.copy()
                sol_tauff = np.zeros_like(current_lr_arm_q)
                arm_safety.hold(current_lr_arm_q, released=not tele_data.motion_data_ready)

"""
UPSTREAM_GO_HOME = "            arm_ctrl.ctrl_dual_arm_go_home()\n"
GATED_GO_HOME = "            arm_safety.close()\n"
PINNED_UPSTREAM_SHA256 = "41e50227bb6973a7e12d3b5a1f118c2bf5391c06f3bb62b33d710c7cee9c1261"


def apply_safety_overlay(source: str) -> str:
    counts = {
        "import": source.count(UPSTREAM_NUMPY_ANCHOR),
        "arm-init": source.count(UPSTREAM_ARM_INIT_ANCHOR),
        "readiness": source.count(UPSTREAM_READY_ASSIGNMENT),
        "ik": source.count(UPSTREAM_IK_BLOCK),
        "go-home": source.count(UPSTREAM_GO_HOME),
    }
    if any(count != 1 for count in counts.values()):
        raise RuntimeError(
            "unsupported xr_teleoperate revision: safety overlay anchors changed: "
            + ", ".join(f"{name}={count}" for name, count in counts.items())
        )
    return (
        source.replace(UPSTREAM_NUMPY_ANCHOR, GATED_NUMPY_ANCHOR)
        .replace(UPSTREAM_ARM_INIT_ANCHOR, GATED_ARM_INIT_ANCHOR)
        .replace(UPSTREAM_READY_ASSIGNMENT, GATED_READY_ASSIGNMENT)
        .replace(UPSTREAM_IK_BLOCK, GATED_IK_BLOCK)
        .replace(UPSTREAM_GO_HOME, GATED_GO_HOME)
    )


def verified_safety_overlay(source: str) -> str:
    digest = hashlib.sha256(source.encode("utf-8")).hexdigest()
    if digest != PINNED_UPSTREAM_SHA256:
        raise RuntimeError(
            "unsupported xr_teleoperate source: expected pinned teleop_hand_and_arm.py "
            f"sha256 {PINNED_UPSTREAM_SHA256}, got {digest}"
        )
    return apply_safety_overlay(source)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument("--upstream-script", type=Path, required=True)
    options, upstream_args = parser.parse_known_args(argv)
    script = options.upstream_script.expanduser().resolve()
    if not script.is_file():
        parser.error(f"upstream teleop script not found: {script}")
    if "--sim" not in upstream_args:
        parser.error("the ALVR safety overlay is simulator-only and requires --sim")

    source = script.read_text(encoding="utf-8")
    overlaid = verified_safety_overlay(source)
    sys.argv = [str(script), *upstream_args]
    os.chdir(script.parent)
    namespace = {
        "__name__": "__main__",
        "__file__": str(script),
        "__package__": None,
        "__cached__": None,
    }
    exec(compile(overlaid, str(script), "exec"), namespace)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
