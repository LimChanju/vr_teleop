"""JSONL recording and deterministic replay through the same input contract."""

from __future__ import annotations

import json
from pathlib import Path
import time
from typing import IO, Any

import numpy as np

from ..models import PoseSample, TrackerFrame
from .base import VRInputProvider


SCHEMA_VERSION = 1


def frame_to_dict(frame: TrackerFrame) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "timestamp_s": frame.timestamp_s,
        "sequence": frame.sequence,
        "source": frame.source,
        "coordinate_frame": frame.coordinate_frame,
        "controls": dict(frame.controls),
        "poses": {
            role: {
                "device_id": pose.device_id,
                "position_m": pose.position_m.tolist(),
                "orientation_xyzw": pose.orientation_xyzw.tolist(),
                "valid": pose.valid,
                "tracking_state": pose.tracking_state,
                "metadata": dict(pose.metadata),
            }
            for role, pose in frame.poses.items()
        },
    }


class JSONLWriter:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._stream: IO[str] = self.path.open("w", encoding="utf-8")

    def write(self, frame: TrackerFrame) -> None:
        self._stream.write(json.dumps(frame_to_dict(frame), separators=(",", ":")) + "\n")
        self._stream.flush()

    def close(self) -> None:
        self._stream.close()

    def __enter__(self) -> "JSONLWriter":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


class JSONLReplayProvider(VRInputProvider):
    def __init__(self, path: str | Path, realtime: bool = False, loop: bool = False) -> None:
        self.path = Path(path)
        self.realtime = realtime
        self.loop = loop
        self._records = [json.loads(line) for line in self.path.read_text().splitlines() if line.strip()]
        if not self._records:
            raise ValueError(f"replay file is empty: {self.path}")
        if any(record.get("schema_version") != SCHEMA_VERSION for record in self._records):
            raise ValueError("unsupported replay schema version")
        self._index = 0
        self._start = time.monotonic()
        self._source_start = float(self._records[0]["timestamp_s"])

    def read(self) -> TrackerFrame:
        if self._index >= len(self._records):
            if not self.loop:
                raise EOFError("end of replay")
            self._index = 0
            self._start = time.monotonic()
        record = self._records[self._index]
        source_timestamp = float(record["timestamp_s"])
        if self.realtime:
            due = self._start + (source_timestamp - self._source_start)
            delay = due - time.monotonic()
            if delay > 0:
                time.sleep(delay)
        now = time.monotonic()
        poses = {
            role: PoseSample(
                device_id=value["device_id"],
                role=role,
                position_m=np.asarray(value["position_m"]),
                orientation_xyzw=np.asarray(value["orientation_xyzw"]),
                timestamp_s=now,
                valid=bool(value.get("valid", True)),
                tracking_state=value.get("tracking_state", "replay"),
                metadata={**value.get("metadata", {}), "source_timestamp_s": source_timestamp},
            )
            for role, value in record["poses"].items()
        }
        frame = TrackerFrame(
            timestamp_s=now,
            poses=poses,
            source="replay",
            coordinate_frame=record["coordinate_frame"],
            controls=record.get("controls", {}),
            sequence=int(record.get("sequence", self._index)),
        )
        self._index += 1
        return frame
