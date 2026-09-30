"""Relative three-point tracking, explicit calibration and clutch controls."""

from __future__ import annotations

import dataclasses
import math
import time
import uuid

import numpy as np

from .protocol import DEFAULT_NOMINAL, TargetFrame

OPENVR_TO_ROBOT = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


@dataclasses.dataclass
class RawFrame:
    # 3x4 row-major OpenVR device-to-standing-world matrices; head, left, right.
    poses: np.ndarray
    tracking_valid: bool
    inputs_valid: bool = False
    left_stick: tuple = (0.0, 0.0)
    right_stick: tuple = (0.0, 0.0)
    grips: tuple = (0.0, 0.0)
    triggers: tuple = (0.0, 0.0)
    calibrate: bool = False
    arm: bool = False
    stop: bool = False
    reset: bool = False
    reference_changed: bool = False
    sampled_monotonic: float = dataclasses.field(default_factory=time.monotonic)
    source: str = "openvr"


def valid_poses(poses):
    try:
        matrices = np.asarray(poses, dtype=float)
        if matrices.shape != (3, 3, 4) or not np.isfinite(matrices).all():
            return False
        rotations = matrices[:, :, :3]
        return bool(np.allclose(rotations.transpose(0, 2, 1) @ rotations, np.eye(3), atol=0.02)
                    and np.allclose(np.linalg.det(rotations), 1.0, atol=0.02)
                    and np.max(np.abs(matrices[:, :, 3])) < 20)
    except (ValueError, TypeError):
        return False


def _deadzone(value, threshold=0.2):
    value = float(value)
    return 0.0 if abs(value) <= threshold else math.copysign((abs(value) - threshold) / (1 - threshold), value)


def validate_target_bounds(delta_min, delta_max):
    """Validate independent robot-frame displacement limits, including neutral."""
    lower, upper = np.asarray(delta_min), np.asarray(delta_max)
    if (lower.shape != (3, 3) or upper.shape != (3, 3)
            or lower.dtype.kind not in "iuf" or upper.dtype.kind not in "iuf"):
        raise ValueError("delta_min and delta_max must be numeric 3x3 arrays in metres")
    lower, upper = lower.astype(float, copy=True), upper.astype(float, copy=True)
    if (not np.isfinite(lower).all() or not np.isfinite(upper).all()
            or np.any(lower > 0) or np.any(upper < 0) or np.any(lower >= upper)):
        raise ValueError("target bounds must be finite and satisfy delta_min <= 0 <= delta_max and delta_min < delta_max")
    return lower, upper


class TargetMapper:
    """Map neutral-relative motion onto trained robot landmarks.

    The head neutral yaw defines forward. Human absolute height and the initial
    wrist placement do not get mistaken for the robot yaw/floor reference frame.
    Orientation is recorded in RawFrame, but this policy contract uses positions.
    """

    def __init__(self, nominal=None, scale=0.65, max_delta=None, velocity_limits=(0.5, 0.25, 0.6),
                 delta_min=None, delta_max=None):
        self.nominal = np.asarray(DEFAULT_NOMINAL if nominal is None else nominal, dtype=float).copy()
        self.max_delta = np.asarray([[0.12] * 3, [0.30] * 3, [0.30] * 3] if max_delta is None else max_delta, dtype=float)
        self.velocity_limits = np.asarray(velocity_limits, dtype=float)
        if self.nominal.shape != (3, 3) or not np.isfinite(self.nominal).all() or np.abs(self.nominal).max() > 2:
            raise ValueError("nominal must be a finite 3x3 matrix in metres")
        if self.max_delta.shape != (3, 3) or not np.isfinite(self.max_delta).all() or np.any(self.max_delta <= 0):
            raise ValueError("max_delta must be positive finite 3x3")
        if delta_min is not None or delta_max is not None:
            if delta_min is None or delta_max is None:
                raise ValueError("delta_min and delta_max must be supplied together")
            if max_delta is not None:
                raise ValueError("choose max_delta or asymmetric delta_min/delta_max, not both")
            self.delta_min, self.delta_max = validate_target_bounds(delta_min, delta_max)
            self.max_delta = np.maximum(-self.delta_min, self.delta_max)
        else:
            self.delta_min, self.delta_max = -self.max_delta.copy(), self.max_delta.copy()
        if self.velocity_limits.shape != (3,) or not np.isfinite(self.velocity_limits).all() or np.any(self.velocity_limits < 0):
            raise ValueError("invalid velocity limits")
        if not 0 < scale <= 2:
            raise ValueError("scale must be between 0 and 2")
        self.scale = scale
        self.neutral = None
        self.basis = None
        self.armed = False
        self.session = str(uuid.uuid4())
        self.sequence = 0
        self.reset_id = 0
        self.calibration_id = 0
        self.last_status = "UNCALIBRATED"
        self._previous = dict(calibrate=False, arm=False, reset=False)
        self._arm_release_required = False
        self._last_update = None

    def calibrate(self, raw):
        if not raw.tracking_valid or not valid_poses(raw.poses):
            raise ValueError("calibration requires valid head and both controller poses")
        forward = OPENVR_TO_ROBOT @ -raw.poses[0, :, 2]
        if np.linalg.norm(forward[:2]) < 0.25:
            raise ValueError("look approximately forward while calibrating")
        yaw = math.atan2(forward[1], forward[0])
        c, s = math.cos(yaw), math.sin(yaw)
        self.basis = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]]) @ OPENVR_TO_ROBOT
        self.neutral = raw.poses[:, :, 3].copy()
        self.armed = False
        self.calibration_id += 1

    def update(self, raw: RawFrame, command=None) -> TargetFrame:
        now = time.monotonic()
        interrupted = self._last_update is not None and now - self._last_update > 0.25
        self._last_update = now
        recent = 0 <= now - raw.sampled_monotonic <= 0.25
        valid = bool(raw.tracking_valid and valid_poses(raw.poses) and recent)
        try:
            buttons = np.asarray([*raw.left_stick, *raw.right_stick, *raw.grips, *raw.triggers], dtype=float)
            valid = valid and buttons.shape == (8,) and np.isfinite(buttons).all() and np.max(np.abs(buttons[:4])) <= 1.01 and np.all((buttons[4:] >= 0) & (buttons[4:] <= 1.01))
        except (ValueError, TypeError):
            valid = False
        valid = bool(valid and raw.inputs_valid)
        edges = {name: bool(getattr(raw, name)) and not self._previous[name] for name in self._previous}
        self._previous = {name: bool(getattr(raw, name)) for name in self._previous}
        if interrupted or not valid:
            self.armed = False
            self._arm_release_required = True
        elif not raw.arm:
            self._arm_release_required = False
        if raw.reference_changed:
            self.neutral = None
            self.basis = None
            self.armed = False
        # A reset or stop in a sample always wins over an arm in that sample.
        if not valid or raw.stop or command in ("stop", "quit"):
            self.armed = False
        if edges["reset"] or command == "reset":
            self.reset_id += 1
            self.armed = False
        elif valid and (edges["calibrate"] or command == "calibrate"):
            try:
                self.calibrate(raw)
            except ValueError:
                self.armed = False
        elif (valid and self.neutral is not None and not raw.stop and command not in ("stop", "quit")
              and ((edges["arm"] and not self._arm_release_required) or command == "arm")):
            self.armed = True
            self._arm_release_required = False
        positions = self.nominal.copy()
        if valid and self.neutral is not None:
            delta = (raw.poses[:, :, 3] - self.neutral) @ self.basis.T * self.scale
            positions += np.clip(delta, self.delta_min, self.delta_max)
        enabled = bool(valid and self.armed and self.neutral is not None and min(raw.grips) >= 0.7)
        velocity = np.zeros(3)
        if enabled:
            velocity = np.array([_deadzone(raw.left_stick[1]), -_deadzone(raw.left_stick[0]), -_deadzone(raw.right_stick[0])]) * self.velocity_limits
        self.last_status = ("TRACKING_LOST" if not valid else "UNCALIBRATED" if self.neutral is None
                            else "ACTIVE" if enabled else "HOLD_GRIPS" if self.armed else "STOPPED")
        frame = TargetFrame(positions=positions, velocity=velocity, enabled=enabled,
                            tracking_valid=valid, calibrated=self.neutral is not None,
                            source=raw.source, session=self.session, seq=self.sequence,
                            reset_id=self.reset_id, calibration_id=self.calibration_id,
                            timestamp_unix_ns=time.time_ns())
        self.sequence += 1
        return frame
