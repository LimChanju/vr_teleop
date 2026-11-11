"""Safe Quest-controller clutching for simulation-only G1 arm teleoperation.

This module deliberately does not infer pelvis or leg motion from the HMD.
The HMD yaw is sampled once during neutral calibration and is then held fixed,
so looking around cannot move the robot arms.  Extra tracked devices remain in
``TrackerFrame`` for the separate full-body profile but are ignored here.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np

from .math3d import quaternion_to_matrix, yaw_quaternion
from .models import BodyPoseFrame, PoseSample, TrackerFrame
from .providers.mock import FULL_TARGET_POSES
from .reconstruction import GMR_BODY_JOINTS


@dataclass(frozen=True)
class UpperBodyStatus:
    state: str
    detail: str
    armed: bool
    calibrated: bool


def _canonical_pose(name: str, timestamp_s: float, *, position: np.ndarray | None = None,
                    provenance: str = "fixed_neutral") -> PoseSample:
    canonical_position, canonical_quaternion = FULL_TARGET_POSES[name]
    return PoseSample(
        device_id=f"canonical:{name}",
        role=name,
        position_m=np.asarray(canonical_position if position is None else position, dtype=np.float64),
        orientation_xyzw=np.asarray(canonical_quaternion, dtype=np.float64),
        timestamp_s=timestamp_s,
        valid=True,
        tracking_state=provenance,
        metadata={"measured": False, "provenance": provenance},
    )


class QuestUpperBodyReconstructor:
    """Map two Touch controllers to clutchable wrist displacement targets.

    The output is a complete, valid XRobot-shaped frame.  All non-wrist joints
    use the FK-derived standing fixture.  ``QuestArmIKRetargeter`` consumes the
    two wrist displacements and changes only the G1's two proximal arms.
    """

    REQUIRED_ROLES = ("Head", "Left_Wrist", "Right_Wrist")

    def __init__(self, config: Mapping[str, Any]) -> None:
        self.press_threshold = float(config.get("grip_press_threshold", 0.65))
        self.release_threshold = float(config.get("grip_release_threshold", 0.35))
        if not 0 <= self.release_threshold < self.press_threshold <= 1:
            raise ValueError("grip thresholds must satisfy 0 <= release < press <= 1")
        self.min_samples = int(config.get("neutral_samples", 15))
        self.min_duration_s = float(config.get("neutral_duration_s", 0.30))
        self.max_position_std_m = float(config.get("neutral_max_position_std_m", 0.015))
        if self.min_samples < 2 or self.min_duration_s <= 0 or self.max_position_std_m <= 0:
            raise ValueError("neutral calibration limits must be positive")

        self.samples: deque[tuple[float, np.ndarray, np.ndarray, np.ndarray]] = deque(
            maxlen=max(self.min_samples * 4, self.min_samples + 1)
        )
        self.calibrated = False
        self.armed = False
        self.release_seen = False
        self.estop_latched = False
        self.signature: tuple[tuple[str, int], ...] | None = None
        self.fixed_room_to_body = np.eye(3)
        self.clutch_positions: dict[str, np.ndarray] = {}
        self.clutch_base_delta = {"Left": np.zeros(3), "Right": np.zeros(3)}
        self.delta = {"Left": np.zeros(3), "Right": np.zeros(3)}
        self.status = UpperBodyStatus("WAITING_TRACKING", "HMD와 양손 컨트롤러를 기다리는 중", False, False)

    def _invalid_roles(self, frame: TrackerFrame) -> list[str]:
        return [
            role for role in self.REQUIRED_ROLES
            if frame.poses.get(role) is None or not frame.poses[role].valid
        ]

    @staticmethod
    def _generation(pose: PoseSample) -> int:
        try:
            return int(pose.metadata.get("connection_generation", 0))
        except (TypeError, ValueError):
            return 0

    def _frame_signature(self, frame: TrackerFrame) -> tuple[tuple[str, int], ...] | None:
        result = []
        for role in self.REQUIRED_ROLES:
            pose = frame.poses.get(role)
            if pose is None or not pose.valid:
                return None
            result.append((pose.device_id, self._generation(pose)))
        return tuple(result)

    @staticmethod
    def _hand_control(frame: TrackerFrame, side: str) -> Mapping[str, Any] | None:
        value = frame.controls.get(side.lower())
        return value if isinstance(value, Mapping) else None

    def _control_state(self, frame: TrackerFrame) -> tuple[bool, float, float, bool, bool]:
        left = self._hand_control(frame, "left")
        right = self._hand_control(frame, "right")
        if left is None or right is None:
            return False, 0.0, 0.0, False, False

        def grip_available(value: Mapping[str, Any]) -> bool:
            available = value.get("available", {})
            if isinstance(available, Mapping):
                return bool(value.get("active")) and bool(available.get("grip", False))
            return bool(value.get("active")) and bool(value.get("grip_available", False))

        ready = grip_available(left) and grip_available(right)
        left_grip = float(np.clip(float(left.get("grip", 0.0)), 0.0, 1.0))
        right_grip = float(np.clip(float(right.get("grip", 0.0)), 0.0, 1.0))
        both_sticks = bool(left.get("thumbstick_click")) and bool(right.get("thumbstick_click"))
        reset_estop = bool(left.get("secondary")) and bool(right.get("secondary"))
        return ready, left_grip, right_grip, both_sticks, reset_estop

    def _invalidate(self, state: str, detail: str) -> None:
        self.samples.clear()
        self.calibrated = False
        self.armed = False
        self.release_seen = False
        self.signature = None
        self.clutch_positions.clear()
        self.status = UpperBodyStatus(state, detail, False, False)

    def _collect_neutral(self, frame: TrackerFrame, signature: tuple[tuple[str, int], ...],
                         left_grip: float, right_grip: float) -> None:
        if left_grip > self.release_threshold or right_grip > self.release_threshold:
            self.samples.clear()
            self.status = UpperBodyStatus(
                "WAIT_RELEASE", "양쪽 그립을 완전히 놓고 손을 편한 중립 위치에 두세요", False, False
            )
            return
        head = frame.poses["Head"]
        left = frame.poses["Left_Wrist"]
        right = frame.poses["Right_Wrist"]
        if self.samples and signature != self.signature:
            self.samples.clear()
        self.signature = signature
        self.samples.append((frame.timestamp_s, head.orientation_xyzw.copy(),
                             left.position_m.copy(), right.position_m.copy()))
        if len(self.samples) < self.min_samples:
            self.status = UpperBodyStatus(
                "CALIBRATING", f"중립 자세 유지 {len(self.samples)}/{self.min_samples}", False, False
            )
            return
        duration = self.samples[-1][0] - self.samples[0][0]
        left_samples = np.stack([sample[2] for sample in self.samples])
        right_samples = np.stack([sample[3] for sample in self.samples])
        spread = max(float(np.max(np.std(left_samples, axis=0))),
                     float(np.max(np.std(right_samples, axis=0))))
        if duration < self.min_duration_s:
            self.status = UpperBodyStatus("CALIBRATING", "중립 자세를 조금 더 유지하세요", False, False)
            return
        if spread > self.max_position_std_m:
            self.samples.clear()
            self.status = UpperBodyStatus(
                "CALIBRATING", f"컨트롤러를 고정하세요 (흔들림 {spread * 100:.1f} cm)", False, False
            )
            return
        head_yaw = yaw_quaternion(self.samples[-1][1])
        self.fixed_room_to_body = quaternion_to_matrix(head_yaw).T
        self.calibrated = True
        self.release_seen = True
        self.status = UpperBodyStatus(
            "READY", "준비 완료: 양쪽 그립을 계속 누르면 팔 텔레옵이 시작됩니다", False, True
        )

    def _body(self, timestamp_s: float) -> BodyPoseFrame:
        joints: dict[str, PoseSample] = {}
        provenance: dict[str, str] = {}
        for name in (*GMR_BODY_JOINTS, "Head"):
            source = "fixed_neutral"
            position = None
            if name in {"Left_Wrist", "Right_Wrist"}:
                side = name.split("_", 1)[0]
                position = np.asarray(FULL_TARGET_POSES[name][0], dtype=np.float64) + self.delta[side]
                source = "controller_delta" if self.armed else "held_safe"
            joints[name] = _canonical_pose(name, timestamp_s, position=position, provenance=source)
            provenance[name] = source
        return BodyPoseFrame(timestamp_s, joints, provenance, True)

    def __call__(self, frame: TrackerFrame) -> BodyPoseFrame:
        signature = self._frame_signature(frame)
        controls_ready, left_grip, right_grip, both_sticks, reset_estop = self._control_state(frame)

        if both_sticks:
            self.estop_latched = True
            self.armed = False
            self.release_seen = False
        if self.estop_latched:
            released = left_grip <= self.release_threshold and right_grip <= self.release_threshold
            if controls_ready and released and reset_estop:
                self.estop_latched = False
                self._invalidate("CALIBRATING", "비상정지 해제: 중립 자세를 유지하세요")
            else:
                self.status = UpperBodyStatus(
                    "E_STOP", "비상정지: 그립을 놓고 Y+B를 함께 눌러 해제하세요", False, self.calibrated
                )
                return self._body(frame.timestamp_s)

        if signature is None:
            missing = ", ".join(self._invalid_roles(frame))
            self._invalidate(
                "WAITING_TRACKING",
                f"tracking lost/invalid: {missing or 'unknown device'}",
            )
            return self._body(frame.timestamp_s)
        if not controls_ready:
            self._invalidate("WAITING_INPUT", "양쪽 grip 입력을 기다리는 중입니다")
            return self._body(frame.timestamp_s)
        if self.calibrated and signature != self.signature:
            self._invalidate("RECONNECTED", "장치가 다시 연결되어 중립 보정이 필요합니다")
            return self._body(frame.timestamp_s)
        if not self.calibrated:
            self._collect_neutral(frame, signature, left_grip, right_grip)
            return self._body(frame.timestamp_s)

        both_pressed = left_grip >= self.press_threshold and right_grip >= self.press_threshold
        either_released = left_grip <= self.release_threshold or right_grip <= self.release_threshold
        if either_released:
            self.armed = False
            self.release_seen = True
            self.status = UpperBodyStatus(
                "READY", "팔 목표를 유지합니다; 양쪽 그립을 계속 누르면 다시 시작됩니다", False, True
            )
        elif both_pressed and (self.armed or self.release_seen):
            if not self.armed:
                self.clutch_positions = {
                    "Left": frame.poses["Left_Wrist"].position_m.copy(),
                    "Right": frame.poses["Right_Wrist"].position_m.copy(),
                }
                self.clutch_base_delta = {side: value.copy() for side, value in self.delta.items()}
                self.armed = True
                self.release_seen = False
            for side in ("Left", "Right"):
                current = frame.poses[f"{side}_Wrist"].position_m
                room_delta = current - self.clutch_positions[side]
                self.delta[side] = self.clutch_base_delta[side] + (
                    self.fixed_room_to_body @ room_delta
                )
            self.status = UpperBodyStatus(
                "ARMED", "양쪽 그립 유지 중: 손을 천천히 움직이세요", True, True
            )
        elif not self.armed:
            self.status = UpperBodyStatus(
                "WAIT_RELEASE", "재활성화 전에 양쪽 그립을 한 번 완전히 놓으세요", False, True
            )
        return self._body(frame.timestamp_s)
