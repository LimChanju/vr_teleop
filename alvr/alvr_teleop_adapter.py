#!/usr/bin/env python3
"""ALVR/OpenVR input adapter compatible with Unitree ``TeleVuerWrapper``.

The adapter intentionally contains no robot command loop.  ``xr_teleoperate``
keeps ownership of G1 IK and arm/hand DDS commands.  This module only replaces
its browser/WebXR input provider.  Whole-body simulator tasks are the one
exception: their documented simulator interface is a small DDS velocity topic,
which is published here from the Quest thumbsticks when explicitly enabled.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any

import numpy as np

from alvr_probe import DEFAULT_MANIFEST, OpenVRProbe


# Same OpenXR -> Unitree basis change used by Unitree TeleVuer.
T_ROBOT_OPENXR = np.array(
    [
        [0.0, 0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=float,
)
T_OPENXR_ROBOT = T_ROBOT_OPENXR.T
WAIST_FROM_HEAD = np.array([0.15, 0.0, 0.45], dtype=float)

LEFT_NEUTRAL = np.eye(4, dtype=float)
LEFT_NEUTRAL[:3, 3] = (0.25, 0.15, 0.08)
RIGHT_NEUTRAL = np.eye(4, dtype=float)
RIGHT_NEUTRAL[:3, 3] = (0.25, -0.15, 0.08)

WORKSPACE_MIN = np.array([-0.05, -0.70, -0.45], dtype=float)
WORKSPACE_MAX = np.array([0.80, 0.70, 0.80], dtype=float)


def _env_float(name: str, default: float, minimum: float, maximum: float) -> float:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        value = float(raw)
    except ValueError as error:
        raise ValueError(f"{name} must be a number, got {raw!r}") from error
    if not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}")
    return value


def _env_int(name: str, default: int, minimum: int, maximum: int) -> int:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        value = int(raw)
    except ValueError as error:
        raise ValueError(f"{name} must be an integer, got {raw!r}") from error
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}")
    return value


def _clamp01(value: Any) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    return min(1.0, max(0.0, number)) if math.isfinite(number) else 0.0


def matrix34_to_homogeneous(rows: Any) -> np.ndarray:
    """Convert JSON/OpenVR 3x4 rows to a finite rigid 4x4 transform."""

    matrix = np.asarray(rows, dtype=float)
    if matrix.shape != (3, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError("pose matrix must be a finite 3x4 array")
    result = np.eye(4, dtype=float)
    result[:3, :] = matrix
    result[:3, :3] = project_rotation(result[:3, :3])
    return result


def project_rotation(rotation: Any) -> np.ndarray:
    """Project a noisy 3x3 matrix onto SO(3)."""

    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("rotation must be a finite 3x3 array")
    u, _, vt = np.linalg.svd(matrix)
    projected = u @ vt
    if np.linalg.det(projected) < 0:
        u[:, -1] *= -1.0
        projected = u @ vt
    return projected


def change_basis(transform: np.ndarray) -> np.ndarray:
    """Change an OpenVR/OpenXR transform into Unitree robot axes."""

    result = T_ROBOT_OPENXR @ transform @ T_OPENXR_ROBOT
    result[:3, :3] = project_rotation(result[:3, :3])
    result[3, :] = (0.0, 0.0, 0.0, 1.0)
    return result


def head_yaw_rotation(head_rotation: np.ndarray) -> np.ndarray:
    """Return the head-yaw-only frame used by Unitree TeleVuer."""

    x_axis = np.asarray(head_rotation, dtype=float)[:, 0].copy()
    x_axis[2] = 0.0
    norm = float(np.linalg.norm(x_axis))
    if not math.isfinite(norm) or norm < 1e-6:
        return np.eye(3, dtype=float)
    x_axis /= norm
    z_axis = np.array([0.0, 0.0, 1.0], dtype=float)
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    return np.column_stack((x_axis, y_axis, z_axis))


def official_wrist_pose(head_xr: np.ndarray, controller_aim_xr: np.ndarray) -> np.ndarray:
    """Apply the controller branch of Unitree TeleVuer's wrist transform."""

    head = change_basis(head_xr)
    arm = change_basis(controller_aim_xr)
    yaw = head_yaw_rotation(head[:3, :3])
    result = np.eye(4, dtype=float)
    result[:3, :3] = yaw.T @ arm[:3, :3]
    result[:3, 3] = yaw.T @ (arm[:3, 3] - head[:3, 3]) + WAIST_FROM_HEAD
    return result


def head_relative_pose(head_xr: np.ndarray, controller_xr: np.ndarray) -> np.ndarray:
    """Return a controller pose in the yaw-only head frame, without waist offset."""

    head = change_basis(head_xr)
    controller = change_basis(controller_xr)
    yaw = head_yaw_rotation(head[:3, :3])
    result = np.eye(4, dtype=float)
    result[:3, :3] = yaw.T @ controller[:3, :3]
    result[:3, 3] = yaw.T @ (controller[:3, 3] - head[:3, 3])
    return result


def _average_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    return project_rotation(np.mean(np.stack(rotations), axis=0))


def _clamp_workspace(transform: np.ndarray) -> np.ndarray:
    result = transform.copy()
    result[:3, 3] = np.clip(result[:3, 3], WORKSPACE_MIN, WORKSPACE_MAX)
    result[:3, :3] = project_rotation(result[:3, :3])
    return result


def _slew_pose(
    previous: np.ndarray,
    desired: np.ndarray,
    dt: float,
    linear_speed: float,
    angular_speed: float,
) -> np.ndarray:
    """Bound target motion to avoid a reconnect/calibration jump."""

    if not math.isfinite(dt) or dt <= 0.0:
        return previous.copy()
    result = previous.copy()
    delta = desired[:3, 3] - previous[:3, 3]
    distance = float(np.linalg.norm(delta))
    max_distance = linear_speed * max(0.0, dt)
    if distance > max_distance > 0.0:
        delta *= max_distance / distance
    result[:3, 3] = previous[:3, 3] + delta

    relative = desired[:3, :3] @ previous[:3, :3].T
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    angle = math.acos(cosine)
    max_angle = angular_speed * max(0.0, dt)
    if angle <= max_angle:
        result[:3, :3] = desired[:3, :3]
    elif angle < 1e-8 or max_angle <= 0.0:
        result[:3, :3] = previous[:3, :3]
    else:
        if abs(math.pi - angle) < 1e-5:
            # At pi the skew part is zero. The +1 eigenvector is the stable
            # rotation axis, including for an exact 180-degree reconnect jump.
            _, vectors = np.linalg.eigh((relative + relative.T) * 0.5)
            axis = vectors[:, -1]
        else:
            axis = np.array(
                [
                    relative[2, 1] - relative[1, 2],
                    relative[0, 2] - relative[2, 0],
                    relative[1, 0] - relative[0, 1],
                ],
                dtype=float,
            )
            axis /= 2.0 * math.sin(angle)
        axis /= np.linalg.norm(axis)
        x, y, z = axis
        skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
        step = np.eye(3) + math.sin(max_angle) * skew + (1.0 - math.cos(max_angle)) * (skew @ skew)
        result[:3, :3] = project_rotation(step @ previous[:3, :3])
    return _clamp_workspace(result)


def _pose_from_device(device: dict[str, Any], *, aim: bool) -> np.ndarray | None:
    source: dict[str, Any] | None = device
    if aim:
        source = device.get("input", {}).get("aim_pose")
        if not source or not source.get("active") or not source.get("pose_valid"):
            return None
    elif not device.get("pose_valid"):
        return None
    rows = source.get("matrix_3x4") if source else None
    if rows is None:
        return None
    try:
        return matrix34_to_homogeneous(rows)
    except (ValueError, np.linalg.LinAlgError):
        return None


def _input(snapshot: dict[str, Any], hand: str) -> dict[str, Any]:
    return snapshot.get("devices", {}).get(hand, {}).get("input", {})


def missing_required_actions(snapshot: dict[str, Any], *, wholebody: bool) -> tuple[str, ...]:
    required = ("trigger", "grip", "secondary", "thumbstick_click")
    if wholebody:
        required += ("thumbstick",)
    missing: list[str] = []
    for hand in ("left", "right"):
        controls = _input(snapshot, hand)
        available = controls.get("available", {})
        if not controls.get("active"):
            missing.append(f"{hand}:inactive")
            continue
        missing.extend(f"{hand}:{name}" for name in required if not available.get(name, False))
    return tuple(missing)


def required_actions_available(snapshot: dict[str, Any], *, wholebody: bool) -> bool:
    return not missing_required_actions(snapshot, wholebody=wholebody)


class PoseMapper:
    """Prefer OpenXR aim actions; calibrate raw device poses as a fallback."""

    def __init__(self) -> None:
        mode = os.environ.get("ALVR_POSE_MODE", "auto").strip().lower()
        if mode not in {"auto", "aim", "calibrated"}:
            raise ValueError("ALVR_POSE_MODE must be auto, aim, or calibrated")
        self.requested_mode = mode
        self.source: str | None = None
        self.position_scale = _env_float("ALVR_POSITION_SCALE", 1.0, 0.1, 3.0)
        self.calibration_frames = _env_int("ALVR_CALIBRATION_FRAMES", 15, 1, 300)
        self.calibration_min_duration = _env_float(
            "ALVR_CALIBRATION_MIN_DURATION", 0.40, 0.0, 5.0
        )
        self.aim_wait_frames = _env_int("ALVR_AIM_WAIT_FRAMES", 90, 0, 900)
        self.aim_wait_seconds = _env_float("ALVR_AIM_WAIT_SECONDS", 3.0, 0.0, 30.0)
        self.calibration_position_std = _env_float(
            "ALVR_CALIBRATION_POSITION_STD", 0.015, 0.001, 0.10
        )
        self.calibration_angle_range = _env_float(
            "ALVR_CALIBRATION_ANGLE_RANGE", 0.10, 0.01, 0.75
        )
        self._aim_waited = 0
        self._aim_wait_started_at: float | None = None
        self._left_samples: list[np.ndarray] = []
        self._right_samples: list[np.ndarray] = []
        self._calibration_times: list[float] = []
        self._left_zero: np.ndarray | None = None
        self._right_zero: np.ndarray | None = None
        self._device_signature: tuple[Any, ...] | None = None

    @property
    def calibrated(self) -> bool:
        return self.source == "aim" or (self._left_zero is not None and self._right_zero is not None)

    def reset(self) -> None:
        self.source = None
        self._aim_waited = 0
        self._aim_wait_started_at = None
        self._left_samples.clear()
        self._right_samples.clear()
        self._calibration_times.clear()
        self._left_zero = None
        self._right_zero = None

    @staticmethod
    def _signature(devices: dict[str, Any]) -> tuple[Any, ...]:
        return tuple(
            (
                devices.get(name, {}).get("device_index"),
                devices.get(name, {}).get("connection_generation"),
                devices.get(name, {}).get("serial"),
                devices.get(name, {}).get("tracking_system"),
            )
            for name in ("head", "left", "right")
        )

    def _calibration_is_stable(self) -> bool:
        left_positions = np.stack([item[:3, 3] for item in self._left_samples])
        right_positions = np.stack([item[:3, 3] for item in self._right_samples])
        max_position_std = max(
            float(np.max(np.std(left_positions, axis=0))),
            float(np.max(np.std(right_positions, axis=0))),
        )
        if max_position_std > self.calibration_position_std:
            return False
        left_rotation = _average_rotation([item[:3, :3] for item in self._left_samples])
        right_rotation = _average_rotation([item[:3, :3] for item in self._right_samples])
        for samples, mean_rotation in (
            (self._left_samples, left_rotation),
            (self._right_samples, right_rotation),
        ):
            for sample in samples:
                relative = sample[:3, :3] @ mean_rotation.T
                angle = math.acos(float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)))
                if angle > self.calibration_angle_range:
                    return False
        left_mean = np.mean(left_positions, axis=0)
        right_mean = np.mean(right_positions, axis=0)
        separation = float(np.linalg.norm(left_mean - right_mean))
        return bool(left_mean[1] > right_mean[1] + 0.08 and 0.12 <= separation <= 1.50)

    def map(self, snapshot: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, bool]:
        devices = snapshot.get("devices", {})
        signature = self._signature(devices)
        if self._device_signature is None:
            self._device_signature = signature
        elif signature != self._device_signature:
            self.reset()
            self._device_signature = signature
            return LEFT_NEUTRAL.copy(), RIGHT_NEUTRAL.copy(), False
        head = _pose_from_device(devices.get("head", {}), aim=False)
        if head is None:
            return LEFT_NEUTRAL.copy(), RIGHT_NEUTRAL.copy(), False

        left_aim = _pose_from_device(devices.get("left", {}), aim=True)
        right_aim = _pose_from_device(devices.get("right", {}), aim=True)
        aim_ready = left_aim is not None and right_aim is not None

        left_raw = _pose_from_device(devices.get("left", {}), aim=False)
        right_raw = _pose_from_device(devices.get("right", {}), aim=False)
        raw_ready = left_raw is not None and right_raw is not None

        if self.source in {None, "waiting_aim", "calibrating"}:
            if self.requested_mode != "calibrated" and aim_ready:
                self.source = "aim"
                self._left_samples.clear()
                self._right_samples.clear()
                print("[ALVR] Using OpenXR aim pose actions (TeleVuer-compatible).", file=sys.stderr)
            elif self.requested_mode == "aim":
                return LEFT_NEUTRAL.copy(), RIGHT_NEUTRAL.copy(), False
            elif self.source in {None, "waiting_aim"} and raw_ready:
                self._aim_waited += 1
                if self.requested_mode == "auto":
                    now = time.monotonic()
                    if self._aim_wait_started_at is None:
                        self._aim_wait_started_at = now
                    if (
                        self._aim_waited <= self.aim_wait_frames
                        or now - self._aim_wait_started_at < self.aim_wait_seconds
                    ):
                        self.source = "waiting_aim"
                        return LEFT_NEUTRAL.copy(), RIGHT_NEUTRAL.copy(), False
                self.source = "calibrating"

        if self.source == "aim":
            if not aim_ready:
                return LEFT_NEUTRAL.copy(), RIGHT_NEUTRAL.copy(), False
            return (
                _clamp_workspace(official_wrist_pose(head, left_aim)),
                _clamp_workspace(official_wrist_pose(head, right_aim)),
                True,
            )

        if not raw_ready:
            return LEFT_NEUTRAL.copy(), RIGHT_NEUTRAL.copy(), False
        left_relative = head_relative_pose(head, left_raw)
        right_relative = head_relative_pose(head, right_raw)

        if self._left_zero is None or self._right_zero is None:
            self._left_samples.append(left_relative)
            self._right_samples.append(right_relative)
            self._calibration_times.append(time.monotonic())
            self._left_samples = self._left_samples[-self.calibration_frames :]
            self._right_samples = self._right_samples[-self.calibration_frames :]
            self._calibration_times = self._calibration_times[-self.calibration_frames :]
            if len(self._left_samples) < self.calibration_frames:
                return LEFT_NEUTRAL.copy(), RIGHT_NEUTRAL.copy(), False
            if self._calibration_times[-1] - self._calibration_times[0] < self.calibration_min_duration:
                return LEFT_NEUTRAL.copy(), RIGHT_NEUTRAL.copy(), False
            if not self._calibration_is_stable():
                return LEFT_NEUTRAL.copy(), RIGHT_NEUTRAL.copy(), False
            self._left_zero = np.eye(4, dtype=float)
            self._right_zero = np.eye(4, dtype=float)
            self._left_zero[:3, 3] = np.mean([item[:3, 3] for item in self._left_samples], axis=0)
            self._right_zero[:3, 3] = np.mean([item[:3, 3] for item in self._right_samples], axis=0)
            self._left_zero[:3, :3] = _average_rotation([item[:3, :3] for item in self._left_samples])
            self._right_zero[:3, :3] = _average_rotation([item[:3, :3] for item in self._right_samples])
            self.source = "calibrated"
            print(
                "[ALVR] Aim pose unavailable; calibrated raw controller poses. "
                "Restart after fixing SteamVR bindings to prefer aim poses.",
                file=sys.stderr,
            )

        assert self._left_zero is not None and self._right_zero is not None
        left = self._relative_target(left_relative, self._left_zero, LEFT_NEUTRAL)
        right = self._relative_target(right_relative, self._right_zero, RIGHT_NEUTRAL)
        return _clamp_workspace(left), _clamp_workspace(right), True

    def _relative_target(
        self,
        current: np.ndarray,
        zero: np.ndarray,
        neutral: np.ndarray,
    ) -> np.ndarray:
        result = neutral.copy()
        result[:3, 3] += self.position_scale * (current[:3, 3] - zero[:3, 3])
        result[:3, :3] = current[:3, :3] @ zero[:3, :3].T
        return result


class SafetyGate:
    """Dead-man and soft e-stop state for simulator teleoperation."""

    def __init__(self) -> None:
        mode = os.environ.get("ALVR_DEADMAN", "both_grips").strip().lower()
        if mode not in {"both_grips", "off"}:
            raise ValueError("ALVR_DEADMAN must be both_grips or off")
        self.mode = mode
        self.threshold = _env_float("ALVR_DEADMAN_THRESHOLD", 0.55, 0.05, 1.0)
        self.armed = False
        self.estop_latched = False
        self.release_seen = False
        self._reported_state: tuple[bool, bool, bool, bool, bool] | None = None

    def update(self, snapshot: dict[str, Any], ready: bool) -> bool:
        left = _input(snapshot, "left")
        right = _input(snapshot, "right")
        both_grips = _clamp01(left.get("grip")) >= self.threshold and _clamp01(right.get("grip")) >= self.threshold
        both_sticks = bool(left.get("thumbstick_click")) and bool(right.get("thumbstick_click"))
        reset_estop = bool(left.get("secondary")) and bool(right.get("secondary")) and not both_grips

        if both_sticks:
            self.estop_latched = True
            self.armed = False
            self.release_seen = False
        elif self.estop_latched and reset_estop:
            self.estop_latched = False
            self.release_seen = True

        if not ready or self.estop_latched:
            self.armed = False
            if not ready:
                self.release_seen = False
        elif self.mode == "off":
            self.armed = True
        elif not both_grips:
            self.armed = False
            self.release_seen = True
        elif self.release_seen:
            self.armed = True

        state = (self.armed, self.estop_latched, ready, both_grips, self.release_seen)
        if state != self._reported_state:
            if self.estop_latched:
                detail = "E_STOP (release grips and hold Y+B to reset)"
            elif self.armed:
                detail = "ARMED"
            elif not ready:
                detail = "DISARMED (pose/input/base not ready)"
            elif both_grips:
                detail = "DISARMED (release both grips once, then hold both)"
            else:
                detail = "DISARMED (ready; hold both side grip buttons to arm)"
            print(f"[ALVR] Teleop state: {detail}", file=sys.stderr)
            self._reported_state = state
        return self.armed


def _synthetic_finger(
    positions: np.ndarray,
    indices: tuple[int, int, int, int, int],
    z: float,
    curl: float,
) -> None:
    """Create one WebXR-order finger in Unitree's wrist-relative hand frame."""

    lengths = (0.030, 0.025, 0.020, 0.016)
    positions[indices[0]] = (0.0, -0.030, z)
    point = positions[indices[0]].copy()
    for segment, (index, length) in enumerate(zip(indices[1:], lengths), start=1):
        angle = curl * (0.35 + 0.28 * segment) * (math.pi / 2.0)
        point = point + np.array([-math.sin(angle) * length, -math.cos(angle) * length, 0.0])
        positions[index] = point


def synthetic_hand_positions(curl: float, side: str) -> np.ndarray:
    """Return a deterministic 25-joint grasp synergy for controller-only hands.

    This is intentionally labelled a synergy, not hand tracking.  It lets the
    official Dex3/Inspire retargeters operate for all simulator tasks while a
    Quest Touch trigger controls a coordinated open/close grasp.
    """

    amount = _clamp01(curl)
    if side not in {"left", "right"}:
        raise ValueError("side must be left or right")
    sign = 1.0 if side == "left" else -1.0
    positions = np.zeros((25, 3), dtype=float)
    _synthetic_finger(positions, (5, 6, 7, 8, 9), sign * 0.030, amount)
    _synthetic_finger(positions, (10, 11, 12, 13, 14), sign * 0.010, amount)
    _synthetic_finger(positions, (15, 16, 17, 18, 19), sign * -0.012, amount)
    _synthetic_finger(positions, (20, 21, 22, 23, 24), sign * -0.033, amount)

    thumb_open = np.array(
        [
            [0.0, -0.018, sign * 0.040],
            [-0.004, -0.030, sign * 0.055],
            [-0.007, -0.044, sign * 0.067],
            [-0.009, -0.058, sign * 0.078],
        ]
    )
    thumb_closed = np.array(
        [
            [0.0, -0.018, sign * 0.040],
            [-0.012, -0.030, sign * 0.030],
            [-0.025, -0.038, sign * 0.016],
            [-0.034, -0.043, sign * 0.002],
        ]
    )
    positions[1:5] = (1.0 - amount) * thumb_open + amount * thumb_closed
    return positions


@dataclass
class TeleData:
    """Field-compatible subset of Unitree televuer.tv_wrapper.TeleData."""

    head_pose: np.ndarray
    left_wrist_pose: np.ndarray
    right_wrist_pose: np.ndarray
    left_hand_pos: np.ndarray | None = None
    right_hand_pos: np.ndarray | None = None
    left_hand_rot: np.ndarray | None = None
    right_hand_rot: np.ndarray | None = None
    left_hand_pinch: bool = False
    left_hand_pinchValue: float = 10.0
    left_hand_squeeze: bool = False
    left_hand_squeezeValue: float = 0.0
    right_hand_pinch: bool = False
    right_hand_pinchValue: float = 10.0
    right_hand_squeeze: bool = False
    right_hand_squeezeValue: float = 0.0
    motion_data_ready: bool = False
    left_ctrl_trigger: bool = False
    left_ctrl_triggerValue: float = 10.0
    left_ctrl_squeeze: bool = False
    left_ctrl_squeezeValue: float = 0.0
    left_ctrl_aButton: bool = False
    left_ctrl_bButton: bool = False
    left_ctrl_thumbstick: bool = False
    left_ctrl_thumbstickValue: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=float))
    right_ctrl_trigger: bool = False
    right_ctrl_triggerValue: float = 10.0
    right_ctrl_squeeze: bool = False
    right_ctrl_squeezeValue: float = 0.0
    right_ctrl_aButton: bool = False
    right_ctrl_bButton: bool = False
    right_ctrl_thumbstick: bool = False
    right_ctrl_thumbstickValue: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=float))


class WholebodyCommandPublisher:
    """Publish the command format consumed by unitree_sim_isaaclab Wholebody tasks."""

    def __init__(self, enabled: bool) -> None:
        self.enabled = enabled
        self.publisher: Any = None
        self.message_type: Any = None
        self.speed = _env_float("ALVR_BASE_SPEED", 0.30, 0.0, 1.0)
        self.yaw_speed = _env_float("ALVR_YAW_SPEED", 0.30, 0.0, 2.0)
        self.deadzone = _env_float("ALVR_STICK_DEADZONE", 0.10, 0.0, 0.95)
        self.height = _env_float("ALVR_BASE_HEIGHT", 0.80, 0.3, 1.2)
        self.publish_rate = _env_float("ALVR_BASE_PUBLISH_RATE", 100.0, 10.0, 240.0)
        self.command_timeout = _env_float("ALVR_COMMAND_TIMEOUT", 0.20, 0.05, 2.0)
        self._state_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._stop = threading.Event()
        self._writer_stop = threading.Event()
        self._pending_lock = threading.Lock()
        self._pending_event = threading.Event()
        self._latest_payload = [0.0, 0.0, 0.0, self.height]
        self._pending_payload = list(self._latest_payload)
        self._state_revision = 0
        self._pending_revision = 0
        self._last_command_at = time.monotonic()
        self._last_active = False
        self._timeout_latched = False
        self._writer_error: Exception | None = None
        self._writer_busy = False
        self._write_started_at = 0.0
        self._last_write_completed_at = time.monotonic()
        self._thread: threading.Thread | None = None
        self._writer_thread: threading.Thread | None = None
        if not enabled:
            return
        try:
            from unitree_sdk2py.core.channel import ChannelPublisher
            from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
        except ImportError as error:
            raise RuntimeError(
                "Wholebody task requires unitree_sdk2py in the xr_teleoperate environment"
            ) from error
        self.message_type = String_
        self.publisher = ChannelPublisher("rt/run_command/cmd", String_)
        self.publisher.Init()
        # The simulator consumes and clears each whole-body command at 100 Hz.
        # Keep watchdog/scheduling separate from the potentially blocking DDS
        # Write call.  This lets a release or heartbeat timeout replace a
        # queued command with zero even while a previous write is stalled.
        self._writer_thread = threading.Thread(
            target=self._writer_loop,
            name="alvr-wholebody-dds-writer",
            daemon=True,
        )
        self._thread = threading.Thread(
            target=self._repeat_latest,
            name="alvr-wholebody-scheduler",
            daemon=True,
        )
        self._writer_thread.start()
        self._thread.start()

    def _axis(self, value: Any) -> float:
        number = float(np.clip(float(value), -1.0, 1.0))
        if abs(number) <= self.deadzone:
            return 0.0
        return math.copysign((abs(number) - self.deadzone) / (1.0 - self.deadzone), number)

    def publish(self, left_stick: np.ndarray, right_stick: np.ndarray, active: bool) -> None:
        if not self.enabled:
            return
        if active:
            vx = -self._axis(left_stick[1]) * self.speed
            vy = -self._axis(left_stick[0]) * self.speed
            yaw = -self._axis(right_stick[0]) * self.yaw_speed
        else:
            vx = vy = yaw = 0.0
        payload = [vx, vy, yaw, self.height]
        state_lock = getattr(self, "_state_lock", None)
        if state_lock is None:  # Supports small dependency-free unit-test fakes.
            self._latest_payload = payload
            self._send(payload)
        else:
            with state_lock:
                if not active:
                    # A physical dead-man release acknowledges a prior command
                    # heartbeat timeout. Never auto-resume from a stalled loop.
                    self._timeout_latched = False
                elif self._timeout_latched:
                    payload = [0.0, 0.0, 0.0, self.height]
                self._last_active = bool(active)
                self._latest_payload = payload
                self._last_command_at = time.monotonic()
                self._state_revision += 1
                revision = self._state_revision
            self._queue_payload(payload, revision)

    def healthy(self) -> bool:
        if not self.enabled:
            return True
        pending_lock = getattr(self, "_pending_lock", None)
        if pending_lock is None:
            return True
        with pending_lock:
            failed = self._writer_error is not None
            stalled = self._writer_busy and (
                time.monotonic() - self._write_started_at > self.command_timeout
            )
        writer = self._writer_thread
        return not failed and not stalled and writer is not None and writer.is_alive()

    def _queue_payload(self, values: list[float], revision: int) -> None:
        pending_lock = getattr(self, "_pending_lock", None)
        pending_event = getattr(self, "_pending_event", None)
        if pending_lock is None or pending_event is None:
            self._send(values)
            return
        with pending_lock:
            # A scheduler sample copied before a dead-man release must never
            # replace the newer zero command after the state lock is released.
            if revision < self._pending_revision:
                return
            self._pending_payload = list(values)
            self._pending_revision = revision
            pending_event.set()

    def _send(self, values: list[float]) -> None:
        payload = json.dumps(values, separators=(",", ":"))
        write_lock = getattr(self, "_write_lock", None)
        if write_lock is None:
            self.publisher.Write(self.message_type(data=payload))
        else:
            with write_lock:
                self.publisher.Write(self.message_type(data=payload))

    def _writer_loop(self) -> None:
        while True:
            self._pending_event.wait()
            with self._pending_lock:
                payload = list(self._pending_payload)
                revision = self._pending_revision
                self._pending_event.clear()
                self._writer_busy = True
                self._write_started_at = time.monotonic()
            try:
                self._send(payload)
            except Exception as error:  # DDS failures must disarm the adapter.
                with self._pending_lock:
                    self._writer_error = error
                    self._writer_busy = False
                print(f"[ALVR] Whole-body DDS writer failed: {error}", file=sys.stderr)
                return
            with self._pending_lock:
                self._writer_busy = False
                self._last_write_completed_at = time.monotonic()
                pending_changed = (
                    self._pending_revision > revision or self._pending_event.is_set()
                )
            if self._writer_stop.is_set() and not pending_changed:
                return

    def _repeat_latest(self) -> None:
        interval = 1.0 / self.publish_rate
        while not self._stop.wait(interval):
            with self._state_lock:
                if (
                    self._last_active
                    and time.monotonic() - self._last_command_at > self.command_timeout
                ):
                    if not self._timeout_latched:
                        print(
                            "[ALVR] Base command heartbeat timed out; release both grips before re-arming.",
                            file=sys.stderr,
                        )
                    self._timeout_latched = True
                    self._latest_payload = [0.0, 0.0, 0.0, self.height]
                    self._state_revision += 1
                payload = list(self._latest_payload)
                revision = self._state_revision
            self._queue_payload(payload, revision)

    def close(self) -> None:
        if self.enabled:
            self.publish(np.zeros(2), np.zeros(2), False)
            self._stop.set()
            if self._thread is not None:
                self._thread.join(timeout=1.0)
            self._writer_stop.set()
            self._pending_event.set()
            if self._writer_thread is not None:
                self._writer_thread.join(timeout=1.0)
                if self._writer_thread.is_alive():
                    print(
                        "[ALVR] Whole-body DDS writer did not stop; zero remains queued.",
                        file=sys.stderr,
                    )


class TeleVuerWrapper:
    """Drop-in input wrapper selected through the local ``televuer`` shim."""

    def __init__(
        self,
        use_hand_tracking: bool,
        binocular: bool = True,
        img_shape: tuple[int, int] = (480, 1280),
        display_fps: float = 30.0,
        display_mode: str = "immersive",
        zmq: bool = False,
        webrtc: bool = False,
        webrtc_url: str | None = None,
        cert_file: str | None = None,
        key_file: str | None = None,
        return_hand_rot_data: bool = False,
        arm_reference_mode: str = "head_yaw",
        **_: Any,
    ) -> None:
        del binocular, img_shape, display_fps, display_mode, zmq, webrtc, webrtc_url
        del cert_file, key_file
        if return_hand_rot_data:
            raise ValueError("The ALVR adapter does not provide return_hand_rot_data")
        if arm_reference_mode != "head_yaw":
            raise ValueError("The ALVR adapter currently supports arm_reference_mode=head_yaw only")
        self.use_hand_tracking = bool(use_hand_tracking)
        self.wholebody = os.environ.get("ALVR_WHOLEBODY", "0") == "1"
        self.pose_mapper = PoseMapper()
        self.safety = SafetyGate()
        self.left_target = LEFT_NEUTRAL.copy()
        self.right_target = RIGHT_NEUTRAL.copy()
        self.head_target = np.eye(4, dtype=float)
        self.left_curl = 0.0
        self.right_curl = 0.0
        self.left_grip_value = 0.0
        self.right_grip_value = 0.0
        self.linear_speed = _env_float("ALVR_MAX_WRIST_SPEED", 1.0, 0.05, 5.0)
        self.angular_speed = _env_float("ALVR_MAX_WRIST_ANGULAR_SPEED", 4.0, 0.1, 20.0)
        self._last_update = time.monotonic()
        self._render_warning_printed = False
        self._reported_readiness: tuple[str | None, bool, tuple[str, ...], bool] | None = None
        self._closed = False

        try:
            import openvr
        except ImportError as error:
            raise RuntimeError("openvr is required in the xr_teleoperate Python environment") from error
        self.openvr = openvr
        manifest = os.environ.get("ALVR_ACTION_MANIFEST")
        manifest_path = DEFAULT_MANIFEST if not manifest else Path(manifest).expanduser().resolve()
        self.probe = OpenVRProbe(
            openvr,
            openvr.TrackingUniverseStanding,
            "standing",
            manifest_path,
        )
        try:
            self.probe.start()
        except Exception:
            self.probe.close()
            raise
        try:
            self.base = WholebodyCommandPublisher(self.wholebody)
        except Exception:
            self.probe.close()
            raise
        hand_label = "synthetic grasp synergy" if self.use_hand_tracking else "controller gripper"
        print(
            f"[ALVR] Unitree input adapter connected ({hand_label}, deadman={self.safety.mode}).",
            file=sys.stderr,
        )

    def get_tele_data(self) -> TeleData:
        now = time.monotonic()
        dt = min(0.1, max(0.0, now - self._last_update))
        self._last_update = now
        snapshot = self.probe.sample()

        left_desired, right_desired, poses_ready = self.pose_mapper.map(snapshot)
        missing_actions = missing_required_actions(snapshot, wholebody=self.wholebody)
        actions_ready = not missing_actions
        base_ready = self.base.healthy() if hasattr(self.base, "healthy") else True
        pose_source = getattr(self.pose_mapper, "source", None)
        readiness = (pose_source, poses_ready, missing_actions, base_ready)
        if readiness != getattr(self, "_reported_readiness", None):
            action_status = "ready" if actions_ready else f"missing={','.join(missing_actions)}"
            print(
                f"[ALVR] Readiness: pose={pose_source or 'waiting'}"
                f"/{'ready' if poses_ready else 'waiting'}; actions={action_status}; "
                f"base={'ready' if base_ready else 'blocked'}",
                file=sys.stderr,
            )
            self._reported_readiness = readiness
        ready = poses_ready and actions_ready and base_ready
        armed = self.safety.update(snapshot, ready)
        if armed:
            self.left_target = _slew_pose(
                self.left_target, left_desired, dt, self.linear_speed, self.angular_speed
            )
            self.right_target = _slew_pose(
                self.right_target, right_desired, dt, self.linear_speed, self.angular_speed
            )

        head_device = snapshot.get("devices", {}).get("head", {})
        head_xr = _pose_from_device(head_device, aim=False)
        if head_xr is not None:
            self.head_target = change_basis(head_xr)

        left = _input(snapshot, "left")
        right = _input(snapshot, "right")
        left_trigger = _clamp01(left.get("trigger"))
        right_trigger = _clamp01(right.get("trigger"))
        left_grip = _clamp01(left.get("grip"))
        right_grip = _clamp01(right.get("grip"))
        left_stick = _stick(left)
        right_stick = _stick(right)

        if armed:
            # Grip buttons are reserved for the dead-man switch.  Triggers alone
            # drive the synthetic Dex3/Inspire grasp so arming does not close it.
            self.left_curl = left_trigger
            self.right_curl = right_trigger
            self.left_grip_value = left_grip
            self.right_grip_value = right_grip
        self.base.publish(left_stick, right_stick, armed and ready)

        left_hand = synthetic_hand_positions(self.left_curl, "left") if self.use_hand_tracking else None
        right_hand = synthetic_hand_positions(self.right_curl, "right") if self.use_hand_tracking else None
        motion_ready = bool(armed and ready)
        # Freeze all end-effector analog values while disarmed. Upstream writes
        # these shared values just before it writes motion_data_ready; keeping
        # the last armed values closes that one-cycle cross-process race.
        public_left_trigger = self.left_curl
        public_right_trigger = self.right_curl
        public_left_grip = self.left_grip_value
        public_right_grip = self.right_grip_value
        public_left_stick = left_stick if motion_ready else np.zeros(2, dtype=float)
        public_right_stick = right_stick if motion_ready else np.zeros(2, dtype=float)
        return TeleData(
            head_pose=self.head_target.copy(),
            left_wrist_pose=self.left_target.copy(),
            right_wrist_pose=self.right_target.copy(),
            left_hand_pos=left_hand,
            right_hand_pos=right_hand,
            left_hand_pinch=public_left_trigger > 0.5,
            left_hand_pinchValue=10.0 * (1.0 - public_left_trigger),
            left_hand_squeeze=public_left_grip > 0.5,
            left_hand_squeezeValue=public_left_grip,
            right_hand_pinch=public_right_trigger > 0.5,
            right_hand_pinchValue=10.0 * (1.0 - public_right_trigger),
            right_hand_squeeze=public_right_grip > 0.5,
            right_hand_squeezeValue=public_right_grip,
            motion_data_ready=motion_ready,
            left_ctrl_trigger=public_left_trigger > 0.5,
            left_ctrl_triggerValue=10.0 * (1.0 - public_left_trigger),
            left_ctrl_squeeze=public_left_grip > 0.5,
            left_ctrl_squeezeValue=public_left_grip,
            left_ctrl_aButton=motion_ready and bool(left.get("primary")),
            left_ctrl_bButton=motion_ready and bool(left.get("secondary")),
            left_ctrl_thumbstick=motion_ready and bool(left.get("thumbstick_click")),
            left_ctrl_thumbstickValue=public_left_stick,
            right_ctrl_trigger=public_right_trigger > 0.5,
            right_ctrl_triggerValue=10.0 * (1.0 - public_right_trigger),
            right_ctrl_squeeze=public_right_grip > 0.5,
            right_ctrl_squeezeValue=public_right_grip,
            right_ctrl_aButton=motion_ready and bool(right.get("primary")),
            right_ctrl_bButton=motion_ready and bool(right.get("secondary")),
            right_ctrl_thumbstick=motion_ready and bool(right.get("thumbstick_click")),
            right_ctrl_thumbstickValue=public_right_stick,
        )

    def render_to_xr(self, _image: Any) -> None:
        # ALVR owns display transport.  The user can view the Isaac window from
        # the SteamVR desktop; a robot-camera compositor is a separate backend.
        if not self._render_warning_printed:
            print(
                "[ALVR] render_to_xr is a no-op; view the Isaac Sim window through SteamVR Desktop.",
                file=sys.stderr,
            )
            self._render_warning_printed = True

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self.base.close()
        finally:
            self.probe.close()


def _stick(controls: dict[str, Any]) -> np.ndarray:
    raw = np.asarray(controls.get("thumbstick", (0.0, 0.0)), dtype=float)
    if raw.shape != (2,) or not np.all(np.isfinite(raw)):
        return np.zeros(2, dtype=float)
    # WebXR gamepads use negative Y for forward, while SteamVR reports positive
    # Y for stick-up on Touch.  Match TeleVuer's public TeleData convention.
    return np.array([np.clip(raw[0], -1.0, 1.0), -np.clip(raw[1], -1.0, 1.0)])
__all__ = [
    "LEFT_NEUTRAL",
    "RIGHT_NEUTRAL",
    "PoseMapper",
    "SafetyGate",
    "TeleData",
    "TeleVuerWrapper",
    "change_basis",
    "head_relative_pose",
    "matrix34_to_homogeneous",
    "official_wrist_pose",
    "required_actions_available",
    "synthetic_hand_positions",
]
