"""Official GMR adapter and the exact TWIST2 35D reference encoder."""

from __future__ import annotations

from pathlib import Path
import sys
from typing import Any, Mapping, Protocol

import numpy as np

from .math3d import (
    quaternion_inverse,
    quaternion_multiply,
    quaternion_to_matrix,
    roll_pitch,
    rotation_vector,
)
from .models import BodyPoseFrame, RobotReference
from .reconstruction import GMR_BODY_JOINTS
from .providers.mock import FULL_TARGET_POSES


DEFAULT_G1_Q29 = np.array([
    -0.2, 0, 0, 0.4, -0.2, 0, -0.2, 0, 0, 0.4, -0.2, 0,
    0, 0, 0,
    0, 0.4, 0, 1.2, 0, 0, 0,
    0, -0.4, 0, 1.2, 0, 0, 0,
], dtype=np.float64)
DEFAULT_MIMIC_OBS = np.concatenate(([0, 0, 0.8, 0, 0, 0], DEFAULT_G1_Q29)).astype(np.float32)


def body_frame_to_gmr(frame: BodyPoseFrame) -> dict[str, list[np.ndarray]]:
    if not frame.valid:
        raise ValueError(f"invalid body frame: {frame.reason}")
    missing = [name for name in GMR_BODY_JOINTS if name not in frame.joints]
    if missing:
        raise ValueError(f"GMR body frame missing: {missing}")
    result: dict[str, list[np.ndarray]] = {}
    for name in GMR_BODY_JOINTS:
        pose = frame.joints[name]
        x, y, z, w = pose.orientation_xyzw
        result[name] = [pose.position_m.copy(), np.array([w, x, y, z])]
    return result


class Retargeter(Protocol):
    def retarget(self, frame: BodyPoseFrame) -> RobotReference: ...


class Twist2ReferenceEncoder:
    def __init__(self, nominal_dt_s: float = 0.02, min_dt_s: float = 0.005, max_dt_s: float = 0.10) -> None:
        self.nominal_dt_s = nominal_dt_s
        self.min_dt_s = min_dt_s
        self.max_dt_s = max_dt_s
        self.last_qpos: np.ndarray | None = None
        self.last_timestamp: float | None = None

    def encode(self, qpos: np.ndarray, timestamp_s: float, source: str) -> RobotReference:
        qpos = np.asarray(qpos, dtype=np.float64)
        if qpos.shape != (36,) or not np.all(np.isfinite(qpos)):
            raise ValueError("official GMR must return finite qpos[36]")
        root_pos = qpos[:3]
        root_quat_xyzw = qpos[[4, 5, 6, 3]]  # MuJoCo free joint stores wxyz.
        if self.last_qpos is None:
            linear_local = np.zeros(3)
            angular_local = np.zeros(3)
        else:
            raw_dt = timestamp_s - float(self.last_timestamp)
            dt = float(np.clip(raw_dt, self.min_dt_s, self.max_dt_s)) if raw_dt > 0 else self.nominal_dt_s
            last_quat_xyzw = self.last_qpos[[4, 5, 6, 3]]
            linear_world = (root_pos - self.last_qpos[:3]) / dt
            relative = quaternion_multiply(root_quat_xyzw, quaternion_inverse(last_quat_xyzw))
            angular_world = rotation_vector(relative) / dt
            world_to_root = quaternion_to_matrix(root_quat_xyzw).T
            linear_local = world_to_root @ linear_world
            angular_local = world_to_root @ angular_world
        roll, pitch = roll_pitch(root_quat_xyzw)
        mimic = np.concatenate((
            linear_local[:2], [root_pos[2]], [roll, pitch], [angular_local[2]], qpos[7:]
        )).astype(np.float32)
        self.last_qpos = qpos.copy()
        self.last_timestamp = timestamp_s
        return RobotReference(timestamp_s, mimic, qpos, True, source)


class OfficialGMRRetargeter:
    def __init__(self, gmr_checkout: str | Path, actual_human_height: float = 1.7) -> None:
        checkout = Path(gmr_checkout).resolve()
        if not checkout.is_dir():
            raise ValueError(f"GMR checkout not found: {checkout}")
        sys.path.insert(0, str(checkout))
        try:
            from general_motion_retargeting import GeneralMotionRetargeting
        except Exception as error:
            raise RuntimeError(f"official GMR import failed: {error}") from error
        self.gmr = GeneralMotionRetargeting(
            src_human="xrobot",
            tgt_robot="unitree_g1",
            actual_human_height=actual_human_height,
            verbose=False,
        )
        self.encoder = Twist2ReferenceEncoder()

    def retarget(self, frame: BodyPoseFrame) -> RobotReference:
        qpos = self.gmr.retarget(body_frame_to_gmr(frame), offset_to_ground=True)
        return self.encoder.encode(qpos, frame.timestamp_s, "official_gmr")


class NeutralTestRetargeter:
    """Test-only fallback; never represents successful GMR/VR integration."""

    def __init__(self) -> None:
        self.encoder = Twist2ReferenceEncoder()

    def retarget(self, frame: BodyPoseFrame) -> RobotReference:
        if not frame.valid:
            raise ValueError(frame.reason)
        qpos = np.concatenate(([0.0, 0.0, 0.8], [1.0, 0.0, 0.0, 0.0], DEFAULT_G1_Q29))
        return self.encoder.encode(qpos, frame.timestamp_s, "neutral_test_only")


class QuestArmIKRetargeter:
    """Position-only G1 arm IK for the isolated Quest simulation profile.

    The official XRobot→G1 GMR configuration intentionally gives arm joint
    positions zero cost and therefore cannot turn controller translations into
    arm motion by itself.  This small solver uses the official G1 MuJoCo model
    and changes only shoulder pitch/roll/yaw and elbow.  Root, legs, waist, and
    all wrist joints are overwritten with their canonical values every frame.
    """

    ARM_JOINTS = {
        "Left": (
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint", "left_elbow_joint",
        ),
        "Right": (
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint", "right_elbow_joint",
        ),
    }
    WRIST_BODIES = {"Left": "left_wrist_yaw_link", "Right": "right_wrist_yaw_link"}

    def __init__(self, robot_xml: str | Path, config: Mapping[str, Any]) -> None:
        try:
            import mujoco
        except ImportError as error:
            raise RuntimeError("MuJoCo is required for Quest arm IK") from error
        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(Path(robot_xml).resolve()))
        self.data = mujoco.MjData(self.model)
        if self.model.nq != 36 or self.model.nv != 35:
            raise ValueError(f"expected free-root G1 29-DoF model, got nq={self.model.nq}, nv={self.model.nv}")
        self.neutral_q29 = DEFAULT_G1_Q29.copy()
        self.base_qpos = np.concatenate((
            [0.0, 0.0, float(config.get("ik_root_height_m", 0.793))],
            [1.0, 0.0, 0.0, 0.0],
            self.neutral_q29,
        ))
        self.current_q29 = self.neutral_q29.copy()
        self.damping = float(config.get("ik_damping", 0.025))
        self.iterations = int(config.get("ik_iterations", 30))
        self.reference_root_height = float(config.get("reference_root_height_m", 0.8))
        if self.damping <= 0 or self.iterations < 1:
            raise ValueError("invalid Quest arm IK solver configuration")

        self.qpos_addresses: dict[str, np.ndarray] = {}
        self.dof_addresses: dict[str, np.ndarray] = {}
        self.joint_limits: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self.wrist_body_ids: dict[str, int] = {}
        for side, names in self.ARM_JOINTS.items():
            joints = [self.model.joint(name) for name in names]
            qpos = np.asarray([int(joint.qposadr[0]) for joint in joints], dtype=np.int32)
            dofs = np.asarray([int(joint.dofadr[0]) for joint in joints], dtype=np.int32)
            lower = np.asarray([float(joint.range[0]) for joint in joints])
            upper = np.asarray([float(joint.range[1]) for joint in joints])
            self.qpos_addresses[side] = qpos
            self.dof_addresses[side] = dofs
            self.joint_limits[side] = (lower, upper)
            self.wrist_body_ids[side] = int(self.model.body(self.WRIST_BODIES[side]).id)

        self.data.qpos[:] = self.base_qpos
        self.mujoco.mj_forward(self.model, self.data)
        self.neutral_wrist_positions = {
            side: self.data.xpos[body_id].copy() for side, body_id in self.wrist_body_ids.items()
        }
        self.last_residual_m = {"Left": 0.0, "Right": 0.0}

    def _solve_side(self, side: str, target: np.ndarray, seed_q29: np.ndarray) -> tuple[np.ndarray, float]:
        qpos = self.base_qpos.copy()
        qpos[7:] = seed_q29
        addresses = self.qpos_addresses[side]
        dofs = self.dof_addresses[side]
        lower, upper = self.joint_limits[side]
        body_id = self.wrist_body_ids[side]
        jacobian_position = np.zeros((3, self.model.nv), dtype=np.float64)
        jacobian_rotation = np.zeros((3, self.model.nv), dtype=np.float64)

        for _ in range(self.iterations):
            self.data.qpos[:] = qpos
            self.mujoco.mj_forward(self.model, self.data)
            error = target - self.data.xpos[body_id]
            if float(np.linalg.norm(error)) < 1e-4:
                break
            jacobian_position.fill(0.0)
            jacobian_rotation.fill(0.0)
            self.mujoco.mj_jacBody(
                self.model, self.data, jacobian_position, jacobian_rotation, body_id
            )
            jacobian = jacobian_position[:, dofs]
            system = jacobian.T @ jacobian + (
                self.damping * self.damping
            ) * np.eye(len(addresses))
            rhs = jacobian.T @ error
            try:
                step = np.linalg.solve(system, rhs)
            except np.linalg.LinAlgError:
                step = np.linalg.lstsq(system, rhs, rcond=None)[0]
            qpos[addresses] = np.clip(qpos[addresses] + step, lower, upper)

        self.data.qpos[:] = qpos
        self.mujoco.mj_forward(self.model, self.data)
        residual = float(np.linalg.norm(target - self.data.xpos[body_id]))
        return qpos[7:].copy(), residual

    def retarget(self, frame: BodyPoseFrame) -> RobotReference:
        if not frame.valid:
            raise ValueError(frame.reason)
        missing = [name for name in ("Left_Wrist", "Right_Wrist") if name not in frame.joints]
        if missing:
            raise ValueError(f"Quest arm frame missing: {missing}")

        solved = self.current_q29.copy()
        for side in ("Left", "Right"):
            wrist_name = f"{side}_Wrist"
            canonical = np.asarray(FULL_TARGET_POSES[wrist_name][0], dtype=np.float64)
            delta = frame.joints[wrist_name].position_m - canonical
            target = self.neutral_wrist_positions[side] + delta
            solved_side, residual = self._solve_side(side, target, solved)
            arm_slice = slice(15, 19) if side == "Left" else slice(22, 26)
            solved[arm_slice] = solved_side[arm_slice]
            self.last_residual_m[side] = residual

        # The first live test is translation-only.  Explicitly lock every
        # non-proximal-arm DoF, including wrist roll/pitch/yaw.
        protected = np.ones(29, dtype=bool)
        protected[15:19] = False
        protected[22:26] = False
        solved[protected] = self.neutral_q29[protected]

        self.current_q29 = solved
        self.current_q29[protected] = self.neutral_q29[protected]

        mimic = np.concatenate((
            [0.0, 0.0, self.reference_root_height, 0.0, 0.0, 0.0], self.current_q29
        )).astype(np.float32)
        qpos = np.concatenate((
            [0.0, 0.0, self.reference_root_height], [1.0, 0.0, 0.0, 0.0], self.current_q29
        ))
        return RobotReference(frame.timestamp_s, mimic, qpos, True, "quest_arm_ik_sim_only")
