"""Neutral-pose calibration stored independently from source code."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from .math3d import matrix_to_quaternion, quaternion_to_matrix, yaw_quaternion
from .models import PoseSample, TrackerFrame


class NeutralCalibration:
    def __init__(self, config: Mapping[str, Any]) -> None:
        matrix = np.asarray(config.get("tracking_to_neutral_matrix", np.eye(4)), dtype=np.float64)
        if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
            raise ValueError("tracking_to_neutral_matrix must be finite 4x4")
        if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8):
            raise ValueError("invalid homogeneous calibration transform")
        self.rotation = matrix[:3, :3]
        self.translation = matrix[:3, 3]
        self.calibrated = bool(config.get("calibrated", False))
        self.anchor_role = str(config.get("anchor_role", "Pelvis"))
        self.neutral_positions = {
            name: np.asarray(value, dtype=np.float64)
            for name, value in config.get("neutral_positions_m", {}).items()
        }

    def calibrate_from_frame(self, frame: TrackerFrame, anchor_role: str | None = None) -> np.ndarray:
        role = anchor_role or self.anchor_role
        sample = frame.poses.get(role)
        if sample is None or not sample.valid:
            raise ValueError(f"calibration requires a valid {role!r} tracker")
        if role not in self.neutral_positions:
            raise ValueError(f"neutral position missing for calibration anchor {role!r}")
        source_yaw = quaternion_to_matrix(yaw_quaternion(sample.orientation_xyzw))
        self.rotation = source_yaw.T
        self.translation = self.neutral_positions[role] - self.rotation @ sample.position_m
        self.calibrated = True
        result = np.eye(4)
        result[:3, :3] = self.rotation
        result[:3, 3] = self.translation
        return result

    def apply(self, frame: TrackerFrame) -> TrackerFrame:
        if not self.calibrated:
            raise RuntimeError("neutral calibration has not been completed")
        poses: dict[str, PoseSample] = {}
        for role, pose in frame.poses.items():
            source_rotation = quaternion_to_matrix(pose.orientation_xyzw)
            poses[role] = pose.replaced(
                position_m=self.rotation @ pose.position_m + self.translation,
                orientation_xyzw=matrix_to_quaternion(self.rotation @ source_rotation),
                metadata={**pose.metadata, "neutral_calibrated": True},
            )
        return TrackerFrame(
            timestamp_s=frame.timestamp_s,
            poses=poses,
            source=frame.source,
            coordinate_frame="twist2_neutral",
            controls=frame.controls,
            sequence=frame.sequence,
        )
