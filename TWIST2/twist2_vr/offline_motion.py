"""CPU-compatible adapter for TWIST2's official offline motion format.

The upstream motion server currently hard-codes CUDA and always opens a
viewer.  This module reuses its ``MotionLib`` and reproduces its documented
35-value reference construction so example motions can be exercised in the
same isolated, headless simulation used by the VR adapter.
"""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import torch

# deploy_real is an official script tree rather than an installable package.
_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT / "deploy_real"))

from data_utils.rot_utils import (  # noqa: E402
    euler_from_quaternion_torch,
    quat_rotate_inverse_torch,
)
from pose.utils.motion_lib_pkl import MotionLib  # noqa: E402

from .models import RobotReference


class OfficialExampleMotionStream:
    """Read one official G1-retargeted pickle at the controller's 50 Hz rate."""

    def __init__(self, motion_file: str | Path, control_dt_s: float = 0.02) -> None:
        self.motion_file = Path(motion_file).resolve()
        if not self.motion_file.is_file():
            raise FileNotFoundError(self.motion_file)
        if control_dt_s <= 0:
            raise ValueError("control_dt_s must be positive")
        self.control_dt_s = float(control_dt_s)
        self.motion_lib = MotionLib(str(self.motion_file), device="cpu")
        motion_id = torch.tensor([0], dtype=torch.long)
        self.motion_length_s = float(self.motion_lib.get_motion_length(motion_id).item())
        self.num_steps = int(self.motion_length_s / self.control_dt_s)
        # Avoid MotionLib's modulo wrap if a clip length is an exact multiple
        # of the controller period and the +1 official lookahead lands at EOF.
        if self.num_steps * self.control_dt_s >= self.motion_length_s:
            self.num_steps -= 1
        if self.num_steps < 1:
            raise ValueError("motion is too short for the official one-step lookahead")

        rotations = self.motion_lib._motion_root_rot.detach().cpu().numpy()
        norm_error = float(np.max(np.abs(np.linalg.norm(rotations, axis=-1) - 1.0)))
        if norm_error > 1e-3:
            raise ValueError(
                f"motion root quaternions are not unit length (max norm error {norm_error:.3g})"
            )

    def reference(self, step: int, timestamp_s: float) -> RobotReference:
        if step < 0 or step >= self.num_steps:
            raise IndexError(f"motion step {step} outside [0, {self.num_steps})")

        # The official server's default --steps=1 requests one frame 20 ms in
        # the future.  Keeping this offset is part of its deployed contract.
        sample_time = torch.tensor([(step + 1) * self.control_dt_s], dtype=torch.float32)
        motion_id = torch.zeros(1, dtype=torch.long)
        (
            root_pos,
            root_rot_xyzw,
            root_vel,
            root_ang_vel,
            dof_pos,
            _dof_vel,
            _local_body_pos,
            _root_pos_delta_local,
            _root_rot_delta_local,
        ) = self.motion_lib.calc_motion_frame(motion_id, sample_time)

        roll, pitch, _yaw = euler_from_quaternion_torch(root_rot_xyzw, scalar_first=False)
        root_vel_local = quat_rotate_inverse_torch(
            root_rot_xyzw, root_vel, scalar_first=False
        )
        root_ang_vel_local = quat_rotate_inverse_torch(
            root_rot_xyzw, root_ang_vel, scalar_first=False
        )
        mimic_obs = torch.cat(
            (
                root_vel_local[:, :2],
                root_pos[:, 2:3],
                roll[:, None],
                pitch[:, None],
                root_ang_vel_local[:, 2:3],
                dof_pos,
            ),
            dim=-1,
        )[0].detach().cpu().numpy()

        root = root_pos[0].detach().cpu().numpy()
        quat_xyzw = root_rot_xyzw[0].detach().cpu().numpy()
        joints = dof_pos[0].detach().cpu().numpy()
        qpos = np.concatenate((root, quat_xyzw[[3, 0, 1, 2]], joints))
        return RobotReference(
            timestamp_s=float(timestamp_s),
            mimic_obs=mimic_obs,
            qpos=qpos,
            valid=True,
            source=f"official_example_motion:{self.motion_file.name}",
        )
