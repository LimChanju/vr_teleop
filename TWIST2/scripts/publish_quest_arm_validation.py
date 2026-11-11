#!/usr/bin/env python3
"""Publish deterministic unclipped Quest-arm requests for headless validation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from twist2_vr.config import load_json  # noqa: E402
from twist2_vr.controller import RedisSimulationSink  # noqa: E402
from twist2_vr.models import BodyPoseFrame, PoseSample  # noqa: E402
from twist2_vr.providers.mock import FULL_TARGET_POSES  # noqa: E402
from twist2_vr.reconstruction import GMR_BODY_JOINTS  # noqa: E402
from twist2_vr.retargeting import DEFAULT_G1_Q29, QuestArmIKRetargeter  # noqa: E402


def smoothstep(value: float) -> float:
    value = float(np.clip(value, 0.0, 1.0))
    return value * value * (3.0 - 2.0 * value)


def arm_delta(elapsed: float) -> tuple[np.ndarray, np.ndarray]:
    # Exercise requests well beyond the removed Cartesian box.  The second
    # phase replays the peak direction observed in the 2026-10-12 Quest log.
    # IK, rather than software shaping, solves the closest pose permitted by
    # the physical XML joint ranges and reports any remaining residual.
    vertical = np.array([0.0, 0.0, 0.60], dtype=np.float64)
    live_peak_left = np.array([0.238, -0.213, 0.565], dtype=np.float64)
    live_peak_right = np.array([0.117, 0.140, 0.464], dtype=np.float64)
    zero = np.zeros(3, dtype=np.float64)
    if elapsed < 1.0:
        return zero, zero
    if elapsed < 3.0:
        return smoothstep((elapsed - 1.0) / 2.0) * vertical, zero
    if elapsed < 4.0:
        return vertical, zero
    if elapsed < 6.0:
        return (1.0 - smoothstep((elapsed - 4.0) / 2.0)) * vertical, zero
    if elapsed < 7.0:
        return zero, zero
    if elapsed < 9.0:
        amount = smoothstep((elapsed - 7.0) / 2.0)
        return amount * live_peak_left, amount * live_peak_right
    if elapsed < 10.0:
        return live_peak_left, live_peak_right
    if elapsed < 12.0:
        amount = 1.0 - smoothstep((elapsed - 10.0) / 2.0)
        return amount * live_peak_left, amount * live_peak_right
    return zero, zero


def body_frame(timestamp_s: float, left_delta: np.ndarray, right_delta: np.ndarray) -> BodyPoseFrame:
    joints = {}
    provenance = {}
    for name in (*GMR_BODY_JOINTS, "Head"):
        position, orientation = FULL_TARGET_POSES[name]
        delta = left_delta if name == "Left_Wrist" else right_delta if name == "Right_Wrist" else 0.0
        joints[name] = PoseSample(
            f"validation:{name}", name, np.asarray(position) + delta,
            orientation, timestamp_s, tracking_state="synthetic_validation",
        )
        provenance[name] = "synthetic_validation" if name.endswith("Wrist") else "fixed_neutral"
    return BodyPoseFrame(timestamp_s, joints, provenance, True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=14.0)
    parser.add_argument("--frequency", type=float, default=50.0)
    parser.add_argument("--redis-port", type=int, default=16378)
    parser.add_argument("--output", type=Path, default=ROOT / "runtime/quest_arm_reference_validation.json")
    parser.add_argument("--no-realtime", action="store_true")
    args = parser.parse_args()
    if args.seconds <= 0 or args.frequency <= 0:
        raise ValueError("seconds and frequency must be positive")

    retargeter = QuestArmIKRetargeter(
        ROOT / "assets/g1/g1_mocap_29dof.xml", load_json(ROOT / "config/quest_arms.json")
    )
    sink = RedisSimulationSink(port=args.redis_port, allow_simulation_publish=True)
    started = time.monotonic()
    previous = None
    max_step = 0.0
    max_residual = 0.0
    max_joint_limit_violation = 0.0
    max_abs_requested_delta = np.zeros(3, dtype=np.float64)
    protected_drift = 0.0
    frames = int(round(args.seconds * args.frequency))
    protected = np.ones(29, dtype=bool)
    protected[15:19] = False
    protected[22:26] = False
    try:
        for index in range(frames):
            deadline = started + index / args.frequency
            if not args.no_realtime:
                time.sleep(max(0.0, deadline - time.monotonic()))
            timestamp = time.monotonic()
            left, right = arm_delta(index / args.frequency)
            max_abs_requested_delta = np.maximum(
                max_abs_requested_delta,
                np.maximum(np.abs(left), np.abs(right)),
            )
            reference = retargeter.retarget(body_frame(timestamp, left, right))
            sink.publish(reference)
            q29 = reference.mimic_obs[6:]
            if previous is not None:
                max_step = max(max_step, float(np.max(np.abs(q29 - previous))))
            previous = q29.copy()
            protected_drift = max(
                protected_drift, float(np.max(np.abs(q29[protected] - DEFAULT_G1_Q29[protected])))
            )
            max_residual = max(max_residual, *retargeter.last_residual_m.values())
            for side, arm_slice in (("Left", slice(15, 19)), ("Right", slice(22, 26))):
                lower, upper = retargeter.joint_limits[side]
                arm_q = q29[arm_slice]
                max_joint_limit_violation = max(
                    max_joint_limit_violation,
                    float(np.max(np.maximum(lower - arm_q, 0.0))),
                    float(np.max(np.maximum(arm_q - upper, 0.0))),
                )
    finally:
        sink.close()
    finite = bool(np.isfinite([
        max_step, max_residual, max_joint_limit_violation, protected_drift,
        *max_abs_requested_delta,
    ]).all())
    removed_box = np.array([0.10, 0.06, 0.30], dtype=np.float64)
    removed_cap_exercised = bool(np.all(max_abs_requested_delta > removed_box))
    trajectory_passed = bool(
        finite
        and removed_cap_exercised
        and max_joint_limit_violation <= 1e-6
        and protected_drift <= 1e-7
    )
    report = {
        "frames": frames,
        "frequency_hz": args.frequency,
        "max_q_step_rad": max_step,
        "max_ik_residual_m": max_residual,
        "max_physical_joint_limit_violation_rad": max_joint_limit_violation,
        "max_abs_requested_delta_m": max_abs_requested_delta.tolist(),
        "removed_cartesian_cap_exercised": removed_cap_exercised,
        "protected_root_leg_waist_wrist_drift_rad": protected_drift,
        "finite": finite,
        "unshaped_trajectory_passed": trajectory_passed,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if trajectory_passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
