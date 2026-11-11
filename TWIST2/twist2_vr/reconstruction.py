"""Tracker-to-XRobot body reconstruction with explicit provenance.

The sparse reconstructor is a configurable kinematic estimate, not measured
full-body tracking.  ``StrictDirectReconstructor`` is provided for setups with
one calibrated source for every GMR target.
"""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from .math3d import quaternion_to_matrix, yaw_quaternion
from .models import BodyPoseFrame, PoseSample, TrackerFrame


GMR_BODY_JOINTS = (
    "Pelvis",
    "Left_Hip", "Right_Hip",
    "Left_Knee", "Right_Knee",
    "Left_Foot", "Right_Foot",
    "Spine3",
    "Left_Shoulder", "Right_Shoulder",
    "Left_Elbow", "Right_Elbow",
    "Left_Wrist", "Right_Wrist",
)


def _estimated_pose(name: str, position: np.ndarray, quat: np.ndarray, timestamp: float) -> PoseSample:
    return PoseSample(
        device_id=f"estimate:{name}",
        role=name,
        position_m=position,
        orientation_xyzw=quat,
        timestamp_s=timestamp,
        valid=True,
        tracking_state="estimated",
        metadata={"measured": False},
    )


def _two_bone_joint(
    start: np.ndarray,
    end: np.ndarray,
    upper_length: float,
    lower_length: float,
    bend_hint: np.ndarray,
) -> np.ndarray:
    delta = end - start
    distance = float(np.linalg.norm(delta))
    if distance < 1e-6:
        raise ValueError("two-bone endpoints coincide")
    direction = delta / distance
    reachable = float(np.clip(distance, abs(upper_length - lower_length) + 1e-5,
                              upper_length + lower_length - 1e-5))
    along = (upper_length * upper_length - lower_length * lower_length + reachable * reachable) / (2 * reachable)
    height = float(np.sqrt(max(0.0, upper_length * upper_length - along * along)))
    perpendicular = bend_hint - direction * float(np.dot(bend_hint, direction))
    if np.linalg.norm(perpendicular) < 1e-6:
        perpendicular = np.cross(direction, [0.0, 0.0, 1.0])
    if np.linalg.norm(perpendicular) < 1e-6:
        perpendicular = np.cross(direction, [0.0, 1.0, 0.0])
    perpendicular /= np.linalg.norm(perpendicular)
    return start + direction * along + perpendicular * height


class StrictDirectReconstructor:
    def __init__(self, require_head: bool = False) -> None:
        self.required = GMR_BODY_JOINTS + (("Head",) if require_head else ())

    def __call__(self, frame: TrackerFrame) -> BodyPoseFrame:
        missing = [name for name in self.required if name not in frame.poses or not frame.poses[name].valid]
        if missing:
            return BodyPoseFrame(frame.timestamp_s, {}, {}, False, f"missing direct trackers: {', '.join(missing)}")
        joints = {name: frame.poses[name] for name in self.required}
        return BodyPoseFrame(frame.timestamp_s, joints, {name: "measured" for name in joints}, True)


class SparseKinematicReconstructor:
    """Fill unmeasured joints from a calibrated neutral skeleton and two-bone IK."""

    def __init__(self, config: Mapping[str, Any]) -> None:
        self.neutral = {
            name: np.asarray(value, dtype=np.float64)
            for name, value in config["neutral_positions_m"].items()
        }
        missing_template = [name for name in (*GMR_BODY_JOINTS, "Head") if name not in self.neutral]
        if missing_template:
            raise ValueError(f"neutral skeleton missing joints: {missing_template}")
        self.minimum_measured = tuple(config.get(
            "minimum_measured_roles", ["Head", "Pelvis", "Left_Wrist", "Right_Wrist", "Left_Foot", "Right_Foot"]
        ))
        self.arm_bend = {
            "Left": np.asarray(config.get("left_elbow_bend_hint", [0.0, 1.0, -0.2]), dtype=np.float64),
            "Right": np.asarray(config.get("right_elbow_bend_hint", [0.0, -1.0, -0.2]), dtype=np.float64),
        }
        self.knee_bend = np.asarray(config.get("knee_bend_hint", [1.0, 0.0, 0.0]), dtype=np.float64)

    def __call__(self, frame: TrackerFrame) -> BodyPoseFrame:
        measured = {name: pose for name, pose in frame.poses.items() if pose.valid}
        missing = [name for name in self.minimum_measured if name not in measured]
        if missing:
            return BodyPoseFrame(frame.timestamp_s, {}, {}, False, f"minimum trackers missing: {', '.join(missing)}")

        if "Pelvis" in measured:
            root_position = measured["Pelvis"].position_m
            root_quat = yaw_quaternion(measured["Pelvis"].orientation_xyzw)
            root_source = "measured"
        elif "Head" in measured:
            root_quat = yaw_quaternion(measured["Head"].orientation_xyzw)
            root_rotation = quaternion_to_matrix(root_quat)
            root_position = measured["Head"].position_m - root_rotation @ (
                self.neutral["Head"] - self.neutral["Pelvis"]
            )
            root_source = "estimated_from_head"
        else:
            return BodyPoseFrame(frame.timestamp_s, {}, {}, False, "Pelvis or Head anchor required")

        root_rotation = quaternion_to_matrix(root_quat)
        neutral_root = self.neutral["Pelvis"]
        joints: dict[str, PoseSample] = {}
        provenance: dict[str, str] = {}
        for name in (*GMR_BODY_JOINTS, "Head"):
            position = root_position + root_rotation @ (self.neutral[name] - neutral_root)
            joints[name] = _estimated_pose(name, position, root_quat, frame.timestamp_s)
            provenance[name] = "estimated_neutral"
        provenance["Pelvis"] = root_source

        for name, pose in measured.items():
            if name in joints:
                joints[name] = pose
                provenance[name] = "measured"

        for side in ("Left", "Right"):
            shoulder_name, elbow_name, wrist_name = (
                f"{side}_Shoulder", f"{side}_Elbow", f"{side}_Wrist"
            )
            if provenance[elbow_name] != "measured" and provenance[wrist_name] == "measured":
                l1 = float(np.linalg.norm(self.neutral[elbow_name] - self.neutral[shoulder_name]))
                l2 = float(np.linalg.norm(self.neutral[wrist_name] - self.neutral[elbow_name]))
                hint = root_rotation @ self.arm_bend[side]
                position = _two_bone_joint(
                    joints[shoulder_name].position_m, joints[wrist_name].position_m, l1, l2, hint
                )
                joints[elbow_name] = _estimated_pose(elbow_name, position, root_quat, frame.timestamp_s)
                provenance[elbow_name] = "estimated_two_bone_ik"

            hip_name, knee_name, foot_name = f"{side}_Hip", f"{side}_Knee", f"{side}_Foot"
            if provenance[knee_name] != "measured" and provenance[foot_name] == "measured":
                l1 = float(np.linalg.norm(self.neutral[knee_name] - self.neutral[hip_name]))
                l2 = float(np.linalg.norm(self.neutral[foot_name] - self.neutral[knee_name]))
                position = _two_bone_joint(
                    joints[hip_name].position_m,
                    joints[foot_name].position_m,
                    l1,
                    l2,
                    root_rotation @ self.knee_bend,
                )
                joints[knee_name] = _estimated_pose(knee_name, position, root_quat, frame.timestamp_s)
                provenance[knee_name] = "estimated_two_bone_ik"

        return BodyPoseFrame(frame.timestamp_s, joints, provenance, True)
