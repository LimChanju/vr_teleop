"""Deterministic synthetic OpenVR-like tracker source for offline validation."""

from __future__ import annotations

import math
import time
from typing import Mapping, Sequence

import numpy as np

from ..math3d import matrix_to_quaternion, quaternion_to_matrix
from ..models import PoseSample, TrackerFrame
from .base import VRInputProvider


OPENVR_TO_TWIST2 = np.array(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64
)

SPARSE_TARGET_POSES: dict[str, tuple[Sequence[float], Sequence[float]]] = {
    "Head": ([0.00, 0.00, 1.70], [0, 0, 0, 1]),
    "Pelvis": ([0.00, 0.00, 0.95], [0, 0, 0, 1]),
    "Left_Wrist": ([0.25, 0.42, 1.25], [0, 0, 0, 1]),
    "Right_Wrist": ([0.25, -0.42, 1.25], [0, 0, 0, 1]),
    "Left_Foot": ([0.03, 0.10, 0.05], [0, 0, 0, 1]),
    "Right_Foot": ([0.03, -0.10, 0.05], [0, 0, 0, 1]),
}

# Synthetic XRobot body frame obtained by taking TWIST2's canonical standing
# G1 qpos through the official GMR robot FK and analytically inverting the
# documented xrobot scale/rotation offsets.  This is a deterministic test
# fixture, not captured human data and not evidence of hardware tracking.
FULL_TARGET_POSES: dict[str, tuple[Sequence[float], Sequence[float]]] = {
    "Pelvis": ([0.0, 0.0, 1.016675864], [-0.5, 0.5, 0.5, -0.5]),
    "Spine3": ([-0.004662941, 0.0, 1.068440570], [-0.5, 0.5, 0.5, -0.5]),
    "Left_Hip": ([0.087961403, 0.137002353, 0.735614655], [-0.584418399, 0.584418399, 0.398064234, -0.398064234]),
    "Right_Hip": ([0.087961403, -0.137002353, 0.735614655], [-0.584418399, 0.584418399, 0.398064234, -0.398064234]),
    "Left_Knee": ([0.078669375, 0.139530471, 0.507750322], [-0.447585374, 0.447585374, 0.547418791, -0.547418791]),
    "Right_Knee": ([0.078669375, -0.139530471, 0.507750322], [-0.447585374, 0.447585374, 0.547418791, -0.547418791]),
    "Left_Foot": ([0.067371979, 0.139419359, 0.117647059], [-0.5, 0.5, 0.5, -0.5]),
    "Right_Foot": ([0.067371979, -0.139419359, 0.117647059], [-0.5, 0.5, 0.5, -0.5]),
    "Left_Shoulder": ([0.000005408, 0.246839902, 1.276538223], [0.140430471, -0.693002026, -0.140570965, 0.693013188]),
    "Right_Shoulder": ([0.000005408, -0.246826666, 1.276538223], [-0.693002026, 0.140430471, 0.693013188, -0.140570965]),
    "Left_Elbow": ([0.020897231, 0.288341157, 1.178383931], [0.112111350, -0.808874591, -0.164048647, 0.553390444]),
    "Right_Elbow": ([0.020897231, -0.288327922, 1.178383931], [-0.808874591, 0.112111350, 0.553390444, -0.164048647]),
    "Left_Wrist": ([0.096812248, 0.380897889, 0.965879454], [0.112111350, -0.808874591, -0.164048647, 0.553390444]),
    "Right_Wrist": ([0.096812248, -0.380884654, 0.965879454], [-0.808874591, 0.112111350, 0.553390444, -0.164048647]),
    "Head": ([0.0, 0.0, 1.766675864], [0.0, 0.0, 0.0, 1.0]),
}

DEFAULT_TARGET_POSES = FULL_TARGET_POSES


class MockVRProvider(VRInputProvider):
    """Generate tracked poses in OpenVR axes (+X right, +Y up, -Z forward)."""

    def __init__(
        self,
        target_poses: Mapping[str, tuple[Sequence[float], Sequence[float]]] | None = None,
        frequency_hz: float = 50.0,
        clock=time.monotonic,
    ) -> None:
        if frequency_hz <= 0:
            raise ValueError("frequency_hz must be positive")
        self.target_poses = dict(target_poses or DEFAULT_TARGET_POSES)
        self.frequency_hz = frequency_hz
        self.clock = clock
        self.sequence = 0
        self.start = float(clock())

    def read(self) -> TrackerFrame:
        now = float(self.clock())
        elapsed = self.sequence / self.frequency_hz
        poses: dict[str, PoseSample] = {}
        for role, (target_position, target_quat) in self.target_poses.items():
            position = np.asarray(target_position, dtype=np.float64).copy()
            if role in {"Left_Wrist", "Right_Wrist"}:
                phase = 0.0 if role == "Left_Wrist" else math.pi
                position[0] += 0.06 * math.sin(2 * math.pi * 0.25 * elapsed + phase)
                position[2] += 0.04 * math.sin(2 * math.pi * 0.20 * elapsed + phase)
            source_position = OPENVR_TO_TWIST2.T @ position
            target_rotation = quaternion_to_matrix(target_quat)
            source_rotation = OPENVR_TO_TWIST2.T @ target_rotation @ OPENVR_TO_TWIST2
            poses[role] = PoseSample(
                device_id=f"mock:{role}",
                role=role,
                position_m=source_position,
                orientation_xyzw=matrix_to_quaternion(source_rotation),
                timestamp_s=now,
                valid=True,
                tracking_state="mock",
                metadata={"synthetic": True},
            )
        frame = TrackerFrame(
            timestamp_s=now,
            poses=poses,
            source="mock",
            coordinate_frame="openvr_standing",
            sequence=self.sequence,
        )
        self.sequence += 1
        return frame
