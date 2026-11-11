"""Combine Quest OpenVR poses with Linux VIVE Ultimate Tracker poses."""

from __future__ import annotations

import time
from typing import Any, Mapping

from ..models import TrackerFrame
from ..spatial_alignment import transform_pose, validate_transform
from .base import VRInputProvider


class QuestViveHybridProvider(VRInputProvider):
    """Merge two independently tracked spaces only after explicit calibration."""

    def __init__(
        self,
        quest: VRInputProvider,
        vive: VRInputProvider,
        calibration: Mapping[str, Any],
    ) -> None:
        if not bool(calibration.get("calibrated", False)):
            raise RuntimeError(
                "Quest/VIVE spatial alignment is not calibrated; run vive-linux-align first"
            )
        self.quest = quest
        self.vive = vive
        self.source_frame = str(calibration.get("source_frame", "vive_hub_world"))
        self.target_frame = str(calibration.get("target_frame", "openvr_standing"))
        if self.target_frame != "openvr_standing":
            raise ValueError("Quest/VIVE alignment target_frame must be 'openvr_standing'")
        self.vive_to_quest = validate_transform(
            calibration.get("source_to_target_matrix"),
            "source_to_target_matrix",
        )
        self.calibration_id = str(calibration.get("calibration_id", "unknown"))
        self.sequence = 0
        self._closed = False

    def read(self) -> TrackerFrame:
        if self._closed:
            raise RuntimeError("hybrid provider is closed")
        quest_frame = self.quest.read()
        vive_frame = self.vive.read()
        if quest_frame.coordinate_frame != self.target_frame:
            raise ValueError(
                f"expected Quest frame {self.target_frame!r}, got {quest_frame.coordinate_frame!r}"
            )
        if vive_frame.coordinate_frame != self.source_frame:
            raise ValueError(
                f"expected VIVE frame {self.source_frame!r}, got {vive_frame.coordinate_frame!r}"
            )

        poses = dict(quest_frame.poses)
        for role, pose in vive_frame.poses.items():
            if role in poses:
                raise RuntimeError(
                    f"role collision while merging Quest and VIVE sources: {role!r}"
                )
            aligned = transform_pose(pose, self.vive_to_quest, source_frame=self.source_frame)
            poses[role] = aligned.replaced(
                metadata={
                    **aligned.metadata,
                    "spatial_calibration_id": self.calibration_id,
                }
            )

        now_s = time.monotonic()
        frame = TrackerFrame(
            timestamp_s=now_s,
            poses=poses,
            source="quest_openvr+vive_hub_linux",
            coordinate_frame=self.target_frame,
            controls=quest_frame.controls,
            sequence=self.sequence,
        )
        self.sequence += 1
        return frame

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        errors = []
        for provider in (self.vive, self.quest):
            try:
                provider.close()
            except Exception as error:  # pragma: no cover - best-effort shutdown
                errors.append(error)
        if errors:
            raise errors[0]


__all__ = ["QuestViveHybridProvider"]
