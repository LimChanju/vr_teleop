"""Explicit runtime-axis, unit, and tracker-mount normalization."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from .math3d import matrix_to_quaternion, quaternion_to_matrix
from .models import PoseSample, TrackerFrame


class PoseNormalizer:
    def __init__(self, config: Mapping[str, Any]) -> None:
        self.axis = np.asarray(config["axis_matrix"], dtype=np.float64)
        if self.axis.shape != (3, 3) or not np.all(np.isfinite(self.axis)):
            raise ValueError("axis_matrix must be finite 3x3")
        if not np.allclose(self.axis.T @ self.axis, np.eye(3), atol=1e-6) or np.linalg.det(self.axis) < 0.999:
            raise ValueError("axis_matrix must be a proper orthonormal rotation")
        self.scale = float(config.get("position_scale", 1.0))
        if not np.isfinite(self.scale) or self.scale <= 0:
            raise ValueError("position_scale must be positive")
        self.translation = np.asarray(config.get("translation_m", [0, 0, 0]), dtype=np.float64)
        if self.translation.shape != (3,) or not np.all(np.isfinite(self.translation)):
            raise ValueError("translation_m must be a finite 3-vector")
        self.expected_source = str(config.get("source_frame", "openvr_standing"))
        self.target_frame = str(config.get("target_frame", "twist2_world"))
        self.offsets = config.get("tracker_to_body", {})

    @staticmethod
    def _offset_transform(value: Mapping[str, Any] | None) -> tuple[np.ndarray, np.ndarray]:
        value = value or {}
        translation = np.asarray(value.get("translation_m", [0, 0, 0]), dtype=np.float64)
        rotation = quaternion_to_matrix(value.get("orientation_xyzw", [0, 0, 0, 1]))
        if translation.shape != (3,) or not np.all(np.isfinite(translation)):
            raise ValueError("tracker offset translation must be a finite 3-vector")
        return translation, rotation

    def normalize_pose(self, pose: PoseSample) -> PoseSample:
        source_rotation = quaternion_to_matrix(pose.orientation_xyzw)
        rotation = self.axis @ source_rotation @ self.axis.T
        position = self.scale * (self.axis @ pose.position_m) + self.translation
        offset_translation, offset_rotation = self._offset_transform(self.offsets.get(pose.role))
        # tracker pose T_world_tracker * fixed T_tracker_body
        position = position + rotation @ offset_translation
        rotation = rotation @ offset_rotation
        return pose.replaced(
            position_m=position,
            orientation_xyzw=matrix_to_quaternion(rotation),
            metadata={**pose.metadata, "normalized_from": self.expected_source},
        )

    def __call__(self, frame: TrackerFrame) -> TrackerFrame:
        if frame.coordinate_frame == self.target_frame:
            return frame
        if frame.coordinate_frame != self.expected_source:
            raise ValueError(
                f"expected coordinate frame {self.expected_source!r}, got {frame.coordinate_frame!r}"
            )
        return TrackerFrame(
            timestamp_s=frame.timestamp_s,
            poses={role: self.normalize_pose(pose) for role, pose in frame.poses.items()},
            source=frame.source,
            coordinate_frame=self.target_frame,
            controls=frame.controls,
            sequence=frame.sequence,
        )
