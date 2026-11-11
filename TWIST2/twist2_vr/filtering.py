"""Per-device freshness, timestamp, and reconnect validity checks."""

from __future__ import annotations

from typing import Any, Mapping

from .models import PoseSample, TrackerFrame


class PoseFilter:
    def __init__(self, config: Mapping[str, Any]) -> None:
        self.max_stale_s = float(config.get("max_stale_s", 0.20))
        if self.max_stale_s <= 0:
            raise ValueError("max_stale_s must be positive")
        self._last: dict[str, PoseSample] = {}

    def __call__(self, frame: TrackerFrame, now_s: float | None = None) -> TrackerFrame:
        now = frame.timestamp_s if now_s is None else float(now_s)
        output: dict[str, PoseSample] = {}
        # A missing role is a disconnect, not a sample to bridge across.  Drop
        # its filter history so a controller that wakes at another pose can be
        # accepted as a fresh device and forced through teleop recalibration.
        for missing_role in set(self._last) - set(frame.poses):
            self._last.pop(missing_role, None)
        for role, pose in frame.poses.items():
            age = now - pose.timestamp_s
            if age < -1e-6 or age > self.max_stale_s:
                output[role] = pose.replaced(valid=False, tracking_state="stale")
                continue
            previous = self._last.get(role)
            if previous is not None:
                previous_generation = previous.metadata.get("connection_generation", 0)
                current_generation = pose.metadata.get("connection_generation", 0)
                if pose.device_id != previous.device_id or current_generation != previous_generation:
                    previous = None
                    self._last.pop(role, None)
            if not pose.valid or previous is None or not previous.valid:
                output[role] = pose
                if pose.valid:
                    self._last[role] = pose
                continue
            dt = pose.timestamp_s - previous.timestamp_s
            if dt <= 0:
                output[role] = pose.replaced(valid=False, tracking_state="non_monotonic")
                continue
            self._last[role] = pose
            output[role] = pose
        return TrackerFrame(
            timestamp_s=frame.timestamp_s,
            poses=output,
            source=frame.source,
            coordinate_frame=frame.coordinate_frame,
            controls=frame.controls,
            sequence=frame.sequence,
        )
