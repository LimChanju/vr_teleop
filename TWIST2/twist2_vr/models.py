"""Validated data contracts shared by every VR adapter stage."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

import numpy as np


def _finite_vector(value: Any, size: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite {size}-vector")
    return result


@dataclass(frozen=True)
class PoseSample:
    """One tracked rigid body pose.

    Quaternions always use ``[x, y, z, w]`` inside this package.  The GMR
    adapter performs the explicit conversion to its scalar-first convention.
    """

    device_id: str
    role: str
    position_m: np.ndarray
    orientation_xyzw: np.ndarray
    timestamp_s: float
    valid: bool = True
    tracking_state: str = "tracked"
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "position_m", _finite_vector(self.position_m, 3, "position_m"))
        quat = _finite_vector(self.orientation_xyzw, 4, "orientation_xyzw")
        norm = float(np.linalg.norm(quat))
        if norm < 1e-8:
            raise ValueError("orientation quaternion has zero norm")
        object.__setattr__(self, "orientation_xyzw", quat / norm)
        if not np.isfinite(self.timestamp_s):
            raise ValueError("timestamp_s must be finite")
        if not self.device_id or not self.role:
            raise ValueError("device_id and role must be non-empty")

    def replaced(self, **changes: Any) -> "PoseSample":
        values = {
            "device_id": self.device_id,
            "role": self.role,
            "position_m": self.position_m.copy(),
            "orientation_xyzw": self.orientation_xyzw.copy(),
            "timestamp_s": self.timestamp_s,
            "valid": self.valid,
            "tracking_state": self.tracking_state,
            "metadata": dict(self.metadata),
        }
        values.update(changes)
        return PoseSample(**values)


@dataclass(frozen=True)
class TrackerFrame:
    timestamp_s: float
    poses: Mapping[str, PoseSample]
    source: str
    coordinate_frame: str
    controls: Mapping[str, Any] = field(default_factory=dict)
    sequence: int = 0

    def __post_init__(self) -> None:
        if not np.isfinite(self.timestamp_s):
            raise ValueError("frame timestamp_s must be finite")
        for role, pose in self.poses.items():
            if role != pose.role:
                raise ValueError(f"pose mapping key {role!r} does not match role {pose.role!r}")

    @property
    def valid_roles(self) -> set[str]:
        return {role for role, pose in self.poses.items() if pose.valid}


@dataclass(frozen=True)
class BodyPoseFrame:
    timestamp_s: float
    joints: Mapping[str, PoseSample]
    provenance: Mapping[str, str]
    valid: bool
    reason: str = ""


@dataclass(frozen=True)
class RobotReference:
    timestamp_s: float
    mimic_obs: np.ndarray
    qpos: np.ndarray | None
    valid: bool
    source: str
    reason: str = ""

    def __post_init__(self) -> None:
        obs = np.asarray(self.mimic_obs, dtype=np.float32)
        if obs.shape != (35,):
            raise ValueError(f"TWIST2 mimic observation must have shape (35,), got {obs.shape}")
        if not np.all(np.isfinite(obs)):
            raise ValueError("TWIST2 mimic observation contains NaN/Inf")
        object.__setattr__(self, "mimic_obs", obs)
        if self.qpos is not None:
            qpos = np.asarray(self.qpos, dtype=np.float64)
            if qpos.shape != (36,) or not np.all(np.isfinite(qpos)):
                raise ValueError("G1 qpos must be a finite 36-vector (free root + 29 joints)")
            object.__setattr__(self, "qpos", qpos)
