"""Isaac Lab direct G1 sparse-target whole-body control environment.

This is an adaptation, not a claim of reproducing OmniH2O's published results.
The same class performs learning, evaluation, and interactive policy deployment.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensor, ContactSensorCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import quat_apply, quat_apply_inverse, yaw_quat

from .asset import BODY_JOINT_NAMES, HAND_JOINT_NAMES, TRACKED_BODY_NAMES, FOOT_BODY_NAMES, make_robot_cfg


def _failure_snapshot(*, env_id, global_step, episode_seconds, root_height, gravity_z,
                      finite_state, joint_names, requested_targets, soft_limits,
                      clip_index=None, clip_name=None, reflected_phase=None):
    """Build a JSON-safe immutable CPU snapshot of one terminal state."""
    finite = bool(finite_state and math.isfinite(root_height) and math.isfinite(gravity_z))
    clamped, invalid_targets = [], []
    for name, requested, (lower, upper) in zip(joint_names, requested_targets, soft_limits):
        if not math.isfinite(requested):
            invalid_targets.append(name)
        elif requested < lower or requested > upper:
            clamped.append({"joint": name, "requested_target_rad": float(requested),
                            "applied_target_rad": float(min(max(requested, lower), upper)),
                            "soft_lower_rad": float(lower), "soft_upper_rad": float(upper)})
    return {"env_id": int(env_id), "global_step": int(global_step),
            "episode_seconds": float(episode_seconds),
            "clip_index": None if clip_index is None else int(clip_index), "clip_name": clip_name,
            "reflected_phase_frame": float(reflected_phase) if reflected_phase is not None and math.isfinite(reflected_phase) else None,
            "root_height_m": float(root_height) if math.isfinite(root_height) else None,
            "tilt_deg": math.degrees(math.acos(min(1.0, max(-1.0, -gravity_z)))) if math.isfinite(gravity_z) else None,
            "finite_state": finite, "soft_limit_clamped_joints": clamped,
            "nonfinite_target_joints": invalid_targets}


@configclass
class G1WholeBodyEnvCfg(DirectRLEnvCfg):
    decimation = 4
    episode_length_s = 20.0
    action_space = 29
    observation_space = 105
    state_space = 138
    action_scale = 0.5
    action_clip = 4.0
    teacher = False
    rich_observations = False
    motion_file: str | None = None
    motion_split = "train"
    randomize_reset = True
    reference_state_initialization = True
    command_max = (0.5, 0.25, 0.6)
    reference_height = 0.76792282
    tracking_reward_weight = 4.0
    tracking_error_variance = 0.04
    head_tracking_weight = 1.0
    height_reward_weight = 1.0
    height_error_variance = 0.01
    fall_cost = 2.0
    record_failure_events = False
    sim: SimulationCfg = SimulationCfg(dt=0.005, render_interval=4,
        physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0,
                                                        restitution=0.0))
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=1024, env_spacing=3.0, replicate_physics=True)


class G1WholeBodyEnv(DirectRLEnv):
    cfg: G1WholeBodyEnvCfg

    def __init__(self, cfg: G1WholeBodyEnvCfg, render_mode=None, **kwargs):
        if not math.isfinite(cfg.head_tracking_weight) or not 0 < cfg.head_tracking_weight <= 10:
            raise ValueError("head_tracking_weight must be finite and in (0,10]")
        for name in ("tracking_reward_weight", "height_reward_weight", "fall_cost"):
            value = getattr(cfg, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        for name in ("tracking_error_variance", "height_error_variance"):
            value = getattr(cfg, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        actor_size, critic_size = (126, 156) if cfg.rich_observations else (105, 138)
        cfg.observation_space = critic_size if cfg.teacher else actor_size
        cfg.state_space = critic_size
        super().__init__(cfg, render_mode, **kwargs)
        # Explicit semantic mapping, independent of PhysX tree traversal order.
        self.body_joint_ids = self._resolve_names(self.robot.joint_names, BODY_JOINT_NAMES)
        self.hand_joint_ids = self._resolve_names(self.robot.joint_names, HAND_JOINT_NAMES)
        self.tracked_body_ids = self._resolve_names(self.robot.body_names, TRACKED_BODY_NAMES)
        self.foot_body_ids = self._resolve_names(self.robot.body_names, FOOT_BODY_NAMES)
        self.foot_contact_ids = self._resolve_names(self.contact_sensor.body_names, FOOT_BODY_NAMES)
        self.nominal_q = self.robot.data.default_joint_pos[0, self.body_joint_ids].clone()
        self.default_q = self.nominal_q
        from g1_teleop.motion.kinematics import KinematicModel
        fk = KinematicModel()
        nominal_numpy = self.nominal_q.cpu().numpy()
        self.nominal_root_height = float(fk.nominal_root_height(nominal_numpy))
        self.joint_limits = self.robot.data.soft_joint_pos_limits[0, self.body_joint_ids].clone()
        self.actions = torch.zeros(self.num_envs, 29, device=self.device)
        self.previous_actions = torch.zeros_like(self.actions)
        self.joint_targets = self.robot.data.default_joint_pos.clone()
        self.reference_q = self.nominal_q.repeat(self.num_envs, 1)
        self.command_velocity = torch.zeros(self.num_envs, 3, device=self.device)
        self.reference_height = torch.full((self.num_envs,), self.nominal_root_height, device=self.device)
        self.nominal_keypoints = torch.as_tensor(fk.keypoints(nominal_numpy), dtype=torch.float32, device=self.device)
        self.target_positions = self.nominal_keypoints.repeat(self.num_envs, 1, 1)
        self.target_velocity = torch.zeros_like(self.target_positions)
        self._previous_target_positions = self.target_positions.clone()
        self._target_history_valid = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self._external_target_initialized = False
        self.external_mode = False
        self.metrics = {}
        self.failure_events = []
        self.failure_events_total_count = 0
        self.failure_events_truncated_count = 0
        self._failure_reference_phase = None
        self._motion = None
        self._clip_selection = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self._clip_phase = torch.zeros(self.num_envs, device=self.device)
        self._episode_sums = {}
        if cfg.motion_file:
            self.load_motion(cfg.motion_file, cfg.motion_split)

    @staticmethod
    def _resolve_names(actual, expected):
        missing = set(expected) - set(actual)
        if missing:
            raise ValueError(f"G1 model contract mismatch: missing {sorted(missing)}")
        return [actual.index(name) for name in expected]

    def _setup_scene(self):
        self.robot = Articulation(make_robot_cfg())
        # The authored dense left hip-yaw hull fails PhysX 4.5 cooking on this
        # asset. Use deterministic symmetric bounds, with unchanged inertias.
        # This writes the active stage only; source USD files stay untouched.
        from pxr import UsdPhysics
        for side in ("left", "right"):
            prim = self.scene.stage.GetPrimAtPath(f"/World/envs/env_0/Robot/{side}_hip_yaw_link/collisions")
            if not prim.IsValid():
                raise ValueError(f"Missing {side} hip collision prim in G1 USD")
            UsdPhysics.MeshCollisionAPI.Apply(prim).GetApproximationAttr().Set("boundingCube")
        self.scene.articulations["robot"] = self.robot
        self.contact_sensor = ContactSensor(ContactSensorCfg(
            prim_path="/World/envs/env_.*/Robot/.*", history_length=3, update_period=0.0,
            track_air_time=True))
        self.scene.sensors["contact_sensor"] = self.contact_sensor
        # GroundPlaneCfg in this Isaac Sim version references a remote grid USD.
        # A locally authored static cuboid keeps offline server launches bounded.
        ground = sim_utils.CuboidCfg(size=(240., 240., 0.10),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=self.cfg.sim.physics_material,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.25, 0.25, 0.25)))
        ground.func("/World/ground", ground, translation=(0., 0., -0.05))
        self.scene.clone_environments(copy_from_source=False)
        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=["/World/ground"])
        light = sim_utils.DomeLightCfg(intensity=1800.0, color=(0.8, 0.8, 0.8))
        light.func("/World/Light", light)

    def _marker_world_positions(self):
        # USD head_link frame is near the torso base, not at the head geometry.
        # The +0.45 m marker matches motion/kinematics.py's sparse contract.
        offsets = torch.tensor([[0., 0., 0.45], [0., 0., 0.], [0., 0., 0.]], device=self.device)
        body_quat = self.robot.data.body_quat_w[:, self.tracked_body_ids]
        marker_offset = quat_apply(body_quat.reshape(-1, 4), offsets.expand(self.num_envs, -1, -1).reshape(-1, 3)).reshape(-1, 3, 3)
        return self.robot.data.body_pos_w[:, self.tracked_body_ids] + marker_offset

    def raw_root_keypoints(self):
        """Root-body-frame marker positions for exact USD/FK comparison only."""
        delta = self._marker_world_positions() - self.robot.data.root_pos_w[:, None]
        quat = self.robot.data.root_quat_w[:, None].expand(-1, 3, -1)
        return quat_apply_inverse(quat.reshape(-1, 4), delta.reshape(-1, 3)).reshape(-1, 3, 3)

    def _tracked_positions(self):
        # Translating the origin vertically with the pelvis would erase crouch.
        # Keep its Z at the neutral standing height above each env's ground.
        origin = self.robot.data.root_pos_w.clone()
        origin[:, 2] = self.scene.env_origins[:, 2] + self.nominal_root_height
        delta = self._marker_world_positions() - origin[:, None]
        quat = yaw_quat(self.robot.data.root_quat_w)[:, None].expand(-1, 3, -1)
        return quat_apply_inverse(quat.reshape(-1, 4), delta.reshape(-1, 3)).reshape(-1, 3, 3)

    def current_keypoints(self):
        """Markers in pelvis-yaw frame with a fixed nominal-standing-height Z origin."""
        return self._tracked_positions()

    def load_motion(self, path, split="train"):
        """Load bounded tensor motion data; source joint order must match canonical29."""
        with np.load(Path(path), allow_pickle=False) as data:
            if "joint_names" in data and list(data["joint_names"].astype(str)) != BODY_JOINT_NAMES:
                raise ValueError("Motion joint names do not match canonical G1 29-body order")
            q = np.asarray(data["q"], dtype=np.float32)
            points = np.array(data["keypoints"], dtype=np.float32, copy=True)
            heights = np.asarray(data["root_height"], dtype=np.float32)
            if q.ndim != 2 or q.shape[1] != 29 or points.shape != (len(q), 3, 3):
                raise ValueError("Motion must contain q[T,29], keypoints[T,3,3]")
            if not np.isfinite(q).all() or not np.isfinite(points).all():
                raise ValueError("Non-finite motion values")
            if heights.shape != (len(q),) or not np.isfinite(heights).all():
                raise ValueError("Motion root_height must be finite [T]")
            # Raw FK data is pelvis-relative. Shift Z to the teleop observation
            # frame so crouching lowers all three targets rather than vanishing.
            points[:, :, 2] += heights[:, None] - self.nominal_root_height
            starts = np.asarray(data["clip_start"], dtype=np.int64)
            lengths = np.asarray(data["clip_length"], dtype=np.int64)
            split_values = np.asarray(data["split"])
            mask = (split_values == (0 if split == "train" else 1)) if np.issubdtype(split_values.dtype, np.integer) else (split_values.astype(str) == split)
            if not mask.any():
                raise ValueError(f"No motion clips for split {split!r}")
            self._motion = {
                "q": torch.as_tensor(q, device=self.device),
                "keypoints": torch.as_tensor(points, device=self.device),
                "root_height": torch.as_tensor(heights, device=self.device),
                "starts": torch.as_tensor(starts[mask], device=self.device),
                "lengths": torch.as_tensor(lengths[mask], device=self.device),
                "fps": float(data["fps"]),
                "path": str(Path(path).resolve()), "split": split,
                "clip_indices": np.flatnonzero(mask).tolist(),
                "clip_names": np.asarray(data["clip_names"]).astype(str)[mask].tolist() if "clip_names" in data else None,
            }
            if "command_velocity" in data:
                self._motion["command_velocity"] = torch.as_tensor(np.asarray(data["command_velocity"], dtype=np.float32), device=self.device)
        self._sample_reference(torch.arange(self.num_envs, device=self.device))
        self._update_reference()

    def _sample_reference(self, ids):
        if self.external_mode:
            return
        if self._motion is not None:
            self._clip_selection[ids] = torch.randint(len(self._motion["starts"]), (len(ids),), device=self.device)
            self._clip_phase[ids] = torch.rand(len(ids), device=self.device) * self._motion["lengths"][self._clip_selection[ids]]
        else:
            self.reference_q[ids] = self.nominal_q
            self.target_positions[ids] = self.nominal_keypoints
            self.reference_height[ids] = self.nominal_root_height
        # With no motion clips, train balance and commanded walking around neutral arms.
        scale = torch.tensor(self.cfg.command_max, device=self.device)
        self.command_velocity[ids] = (torch.rand(len(ids), 3, device=self.device) * 2. - 1.) * scale
        stand = torch.rand(len(ids), device=self.device) < 0.35
        self.command_velocity[ids[stand]] = 0.

    def _update_reference(self):
        if self.external_mode or self._motion is None:
            return
        motion = self._motion
        length = motion["lengths"][self._clip_selection]
        phase = self._clip_phase + self.episode_length_buf * self.step_dt * motion["fps"]
        # Recorded punch clips are not periodic. Reflect at clip boundaries,
        # keeping positional continuity rather than jumping last -> first.
        span = (length - 1).clamp(min=1)
        phase = torch.remainder(phase, 2 * span)
        reflected = torch.where(phase <= span, phase, 2 * span - phase)
        if getattr(self.cfg, "record_failure_events", False):
            # Keep the exact target phase used by the actor. Recomputing after
            # the physics step would advance this label by one control step.
            self._failure_reference_phase = reflected
        low = reflected.long()
        high = torch.minimum(low + 1, length - 1)
        weight = reflected - low
        low = motion["starts"][self._clip_selection] + low
        high = motion["starts"][self._clip_selection] + high
        def interpolate(name):
            values = motion[name]
            blend = weight.reshape((-1,) + (1,) * (values.ndim - 1))
            return torch.lerp(values[low], values[high], blend)
        self.reference_q.copy_(interpolate("q"))
        self.target_positions.copy_(interpolate("keypoints"))
        self.reference_height.copy_(interpolate("root_height"))
        if "command_velocity" in motion:
            self.command_velocity.copy_(interpolate("command_velocity"))
        else:
            self.command_velocity.zero_()

    def set_external_targets(self, positions, commands, *, reset_velocity=False):
        """Positions [N,3,3] in g1-yaw-floor-relative-v1, commands [N,3]."""
        points = torch.as_tensor(positions, dtype=torch.float32, device=self.device)
        velocity = torch.as_tensor(commands, dtype=torch.float32, device=self.device)
        if points.shape != self.target_positions.shape or velocity.shape != self.command_velocity.shape:
            raise ValueError("External targets require [num_envs,3,3] positions and [num_envs,3] commands")
        if not torch.isfinite(points).all() or not torch.isfinite(velocity).all():
            raise ValueError("External targets contain non-finite values")
        self.external_mode = True
        self.target_positions.copy_(points)
        if reset_velocity or not self._external_target_initialized:
            self.reset_target_history()
        self._external_target_initialized = True
        # No full-body reference is available from Quest; head-height change is
        # the observable crouch command. The sparse marker reward also tracks it.
        self.reference_height.copy_((self.nominal_root_height + points[:, 0, 2] - self.nominal_keypoints[0, 2]).clamp(0.55, 0.85))
        scale = torch.tensor(self.cfg.command_max, device=self.device)
        self.command_velocity.copy_(velocity.clamp(-scale, scale))

    def reset_target_history(self, env_ids=None):
        """Clear derivative on reset/rearm/disarm so a new target has no spike.

        Call after replacing targets at a teleop state transition. The first
        following control step establishes a baseline with zero target velocity.
        """
        ids = slice(None) if env_ids is None else env_ids
        self.target_velocity[ids] = 0.0
        self._previous_target_positions[ids] = self.target_positions[ids]
        self._target_history_valid[ids] = False

    def _update_target_velocity(self):
        # Updated exactly once per executed control step, never on observation
        # reads. A policy sees this filtered estimate with one control-step lag.
        derivative = (self.target_positions - self._previous_target_positions) / self.step_dt
        derivative = torch.nan_to_num(derivative, nan=0., posinf=2., neginf=-2.).clamp(-2., 2.)
        filtered = 0.5 * self.target_velocity + 0.5 * derivative
        self.target_velocity.copy_(torch.where(self._target_history_valid[:, None, None], filtered, 0.0))
        self._previous_target_positions.copy_(self.target_positions)
        self._target_history_valid.fill_(True)

    def _pre_physics_step(self, actions):
        if actions.shape != self.actions.shape:
            raise ValueError(f"Expected actions {self.actions.shape}; got {actions.shape}")
        if self.cfg.rich_observations:
            self._update_target_velocity()
        self.previous_actions.copy_(self.actions)
        limit = self.cfg.action_clip
        self.actions.copy_(torch.nan_to_num(actions, nan=0., posinf=limit, neginf=-limit).clamp(-limit, limit))
        target = self.nominal_q + self.cfg.action_scale * self.actions
        self.joint_targets[:, self.body_joint_ids] = target.clamp(self.joint_limits[:, 0], self.joint_limits[:, 1])
        self.joint_targets[:, self.hand_joint_ids] = 0.024

    def _apply_action(self):
        self.robot.set_joint_position_target(self.joint_targets)

    def _get_observations(self):
        self._update_reference()
        data = self.robot.data
        sparse = torch.cat((data.root_ang_vel_b * 0.25, data.projected_gravity_b,
            data.joint_pos[:, self.body_joint_ids] - self.nominal_q,
            data.joint_vel[:, self.body_joint_ids] * 0.05, self.actions,
            self.target_positions.flatten(1), self.command_velocity), dim=-1)
        root_height = data.root_pos_w[:, 2:3] - self.scene.env_origins[:, 2:3]
        if self.cfg.rich_observations:
            sparse = torch.cat((sparse, data.root_lin_vel_b,
                (self.target_positions - self.current_keypoints()).flatten(1),
                self.target_velocity.flatten(1)), dim=-1)
            full = torch.cat((sparse, root_height, self.reference_q - self.nominal_q), dim=-1)
        else:
            # Exact legacy order and values for existing 105/138 checkpoints.
            full = torch.cat((sparse, data.root_lin_vel_b, root_height, self.reference_q - self.nominal_q), dim=-1)
        return {"policy": full if self.cfg.teacher else sparse, "critic": full}

    def _get_rewards(self):
        data = self.robot.data
        point_error = torch.linalg.vector_norm(self._tracked_positions() - self.target_positions, dim=-1)
        velocity_error = (data.root_lin_vel_b[:, :2] - self.command_velocity[:, :2]).square().sum(-1)
        yaw_error = (data.root_ang_vel_b[:, 2] - self.command_velocity[:, 2]).square()
        height = data.root_pos_w[:, 2] - self.scene.env_origins[:, 2]
        tilt = data.projected_gravity_b[:, :2].square().sum(-1)
        qerr = (data.joint_pos[:, self.body_joint_ids] - self.reference_q).square().mean(-1)
        foot_contact = torch.linalg.vector_norm(self.contact_sensor.data.net_forces_w[:, self.foot_contact_ids], dim=-1) > 5.
        slide = (data.body_lin_vel_w[:, self.foot_body_ids, :2].square().sum(-1) * foot_contact).sum(-1)
        point_tracking = torch.exp(-point_error.square() / self.cfg.tracking_error_variance)
        tracking = point_tracking.mean(-1)
        head_weight = getattr(self.cfg, "head_tracking_weight", 1.0)
        if head_weight != 1.0:
            # Preserve each hand's coefficient. At the default, retain the
            # original reduction exactly rather than reweighting all markers.
            tracking = tracking + (head_weight - 1.0) * point_tracking[:, 0] / 3.0
        terms = {
            "alive": torch.ones_like(height),
            "sparse_tracking": self.cfg.tracking_reward_weight * tracking,
            "linear_velocity": 2. * torch.exp(-velocity_error / 0.25),
            "yaw_velocity": torch.exp(-yaw_error / 0.25),
            "height": self.cfg.height_reward_weight * torch.exp(
                -(height - self.reference_height).square() / self.cfg.height_error_variance),
            "upright": torch.exp(-tilt / 0.10),
            "reference_q": torch.exp(-qerr / 0.25),
            "foot_slide": -0.5 * slide,
            "vertical_velocity": -0.5 * data.root_lin_vel_b[:, 2].square(),
            "angular_xy": -0.05 * data.root_ang_vel_b[:, :2].square().sum(-1),
            "torque": -1.e-5 * data.applied_torque[:, self.body_joint_ids].square().sum(-1),
            "joint_velocity": -1.e-3 * data.joint_vel[:, self.body_joint_ids].square().sum(-1),
            "action_rate": -0.02 * (self.actions - self.previous_actions).square().sum(-1),
        }
        reward = torch.stack(list(terms.values())).sum(0) * self.step_dt - self.cfg.fall_cost * self.reset_terminated.float()
        for key, value in terms.items():
            if key not in self._episode_sums:
                self._episode_sums[key] = torch.zeros_like(value)
            self._episode_sums[key].add_(value * self.step_dt)
        # Snapshot before automatic reset: evaluator must never score reset poses.
        self.metrics = {"tracking_error_m": point_error.mean(-1).clone(),
            "head_error_m": point_error[:, 0].clone(), "hand_error_m": point_error[:, 1:].mean(-1).clone(),
            "velocity_error": velocity_error.sqrt().clone(), "root_height": height.clone(),
            "fallen": self.reset_terminated.clone(), "episode_steps": self.episode_length_buf.clone()}
        if getattr(self.cfg, "record_failure_events", False):
            self._record_failure_events()
        return reward

    def _record_failure_events(self):
        """Opt-in terminal snapshots before automatic reset; no normal-path sync."""
        ids = self.reset_terminated.nonzero(as_tuple=False).flatten()
        count = len(ids)
        if not count:
            return
        self.failure_events_total_count += count
        capacity = max(0, 10000 - len(self.failure_events))
        self.failure_events_truncated_count += max(0, count - capacity)
        if capacity == 0:
            return
        ids = ids[:capacity]
        data = self.robot.data
        root_height = data.root_pos_w[ids, 2] - self.scene.env_origins[ids, 2]
        finite = torch.isfinite(data.root_state_w[ids]).all(-1) & torch.isfinite(data.joint_pos[ids]).all(-1)
        summary = torch.stack((root_height, data.projected_gravity_b[ids, 2],
                               self.episode_length_buf[ids] * self.step_dt, finite), dim=-1).cpu().tolist()
        requested = (self.nominal_q + self.cfg.action_scale * self.actions[ids]).cpu().tolist()
        limits = self.joint_limits.cpu().tolist()
        selected = self._clip_selection[ids].cpu().tolist() if self._motion is not None and not self.external_mode else None
        phases = self._failure_reference_phase[ids].cpu().tolist() if selected is not None and self._failure_reference_phase is not None else None
        for row, env_id in enumerate(ids.cpu().tolist()):
            clip_index, clip_name = None, None
            if selected is not None:
                selection = selected[row]
                source_indices = self._motion.get("clip_indices")
                clip_index = source_indices[selection] if source_indices is not None else selection
                names = self._motion.get("clip_names")
                clip_name = names[selection] if names is not None else None
            height, gravity_z, seconds, is_finite = summary[row]
            self.failure_events.append(_failure_snapshot(
                env_id=env_id, global_step=self.common_step_counter, episode_seconds=seconds,
                root_height=height, gravity_z=gravity_z, finite_state=bool(is_finite),
                joint_names=BODY_JOINT_NAMES, requested_targets=requested[row], soft_limits=limits,
                clip_index=clip_index, clip_name=clip_name,
                reflected_phase=phases[row] if phases is not None else None))

    def _get_dones(self):
        data = self.robot.data
        height = data.root_pos_w[:, 2] - self.scene.env_origins[:, 2]
        fallen = (height < 0.40) | (data.projected_gravity_b[:, 2] > -0.5)
        finite = torch.isfinite(data.root_state_w).all(-1) & torch.isfinite(data.joint_pos).all(-1)
        return fallen | ~finite, self.episode_length_buf >= self.max_episode_length - 1

    def _reset_idx(self, env_ids):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)
        self.robot.reset(env_ids)
        self._sample_reference(env_ids)
        self._update_reference()
        if self.external_mode:
            # Reset clearance is an initial physics condition, not a requested
            # extra head height. Keep the policy's target at grounded neutral.
            self.target_positions[env_ids] = self.nominal_keypoints
            self.command_velocity[env_ids] = 0.
            self.reference_height[env_ids] = self.nominal_root_height
        self.reset_target_history(env_ids)
        q = self.robot.data.default_joint_pos[env_ids].clone()
        use_reference = self.cfg.reference_state_initialization and self._motion is not None and not self.external_mode
        if use_reference:
            q[:, self.body_joint_ids] = self.reference_q[env_ids].clamp(self.joint_limits[:, 0], self.joint_limits[:, 1])
        if self.cfg.randomize_reset:
            q[:, self.body_joint_ids] += (torch.rand(len(env_ids), 29, device=self.device) - 0.5) * 0.04
        dq = torch.zeros_like(q)
        root = self.robot.data.default_root_state[env_ids].clone()
        reset_height = self.reference_height[env_ids] if use_reference else self.nominal_root_height
        root[:, 2] = reset_height + 0.015
        root[:, :3] += self.scene.env_origins[env_ids]
        self.robot.write_root_pose_to_sim(root[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(root[:, 7:], env_ids)
        self.robot.write_joint_state_to_sim(q, dq, env_ids=env_ids)
        self.actions[env_ids] = ((q[:, self.body_joint_ids] - self.nominal_q) / self.cfg.action_scale).clamp(-self.cfg.action_clip, self.cfg.action_clip)
        self.previous_actions[env_ids] = self.actions[env_ids]
        self.joint_targets[env_ids] = q
        self.robot.set_joint_position_target(q, env_ids=env_ids)
        self.extras["log"] = {f"Episode_Reward/{name}": values[env_ids].mean() / self.cfg.episode_length_s
                              for name, values in self._episode_sums.items()}
        for values in self._episode_sums.values():
            values[env_ids] = 0.

    def policy_metadata(self):
        sparse_order = ["root_angular_velocity*0.25", "projected_gravity", "q-nominal",
            "joint_velocity*0.05", "previous_action", "head_leftwrist_rightwrist_target_positions", "vx_vy_yawrate"]
        privileged_order = ["root_linear_velocity", "root_height", "reference_q-nominal"]
        if self.cfg.rich_observations:
            sparse_order += ["root_linear_velocity", "target_minus_current_keypoints", "filtered_target_velocity"]
            privileged_order = ["root_height", "reference_q-nominal"]
        return {"model": "unitree_g1_29dof_dex1_freebase", "usd_path": self.robot.cfg.spawn.usd_path,
            "body_joint_names": BODY_JOINT_NAMES, "body_joint_ids": self.body_joint_ids,
            "hand_joint_names": HAND_JOINT_NAMES, "hand_position_m": 0.024,
            "nominal_q": self.nominal_q.cpu().tolist(), "soft_joint_limits": self.joint_limits.cpu().tolist(),
            "body_joint_stiffness": self.robot.data.joint_stiffness[0, self.body_joint_ids].cpu().tolist(),
            "body_joint_damping": self.robot.data.joint_damping[0, self.body_joint_ids].cpu().tolist(),
            "body_joint_armature": self.robot.data.joint_armature[0, self.body_joint_ids].cpu().tolist(),
            "body_joint_effort_limits": self.robot.data.joint_effort_limits[0, self.body_joint_ids].cpu().tolist(),
            "tracked_body_names": TRACKED_BODY_NAMES, "tracked_body_ids": self.tracked_body_ids,
            "tracked_local_offsets_m": [[0., 0., 0.45], [0., 0., 0.], [0., 0., 0.]],
            "nominal_keypoints_root_m": self.nominal_keypoints.cpu().tolist(),
            "nominal_root_height_m": self.nominal_root_height,
            "collision_overrides": {"left_hip_yaw_link/collisions": "boundingCube",
                                    "right_hip_yaw_link/collisions": "boundingCube"},
            "physics_dt": self.physics_dt, "control_dt": self.step_dt, "decimation": self.cfg.decimation,
            "action_scale": self.cfg.action_scale, "action_clip": [-self.cfg.action_clip, self.cfg.action_clip],
            "reference_state_initialization": self.cfg.reference_state_initialization,
            "reset_contract": {"version": "grounded_nominal_v1", "clearance_m": 0.015,
                "root_height": "reference height for RSI; otherwise FK nominal root height; plus clearance",
                "external_reset_targets": "grounded nominal markers; zero commanded velocity",
                "external_disarm_targets": "current reachable markers; zero commanded velocity"},
            "reward_contract": {name: getattr(self.cfg, name) for name in (
                "tracking_reward_weight", "tracking_error_variance", "height_reward_weight",
                "height_error_variance", "fall_cost", "head_tracking_weight")},
            "motion_playback": "linear interpolation with reflected clip endpoints",
            "teacher": self.cfg.teacher, "actor_observations": self.cfg.observation_space,
            "critic_observations": self.cfg.state_space, "coordinate_version": "g1-yaw-floor-relative-v1",
            "observation_version": "sparse_tracking_v2" if self.cfg.rich_observations else "sparse_positions_v1",
            "coordinates": "pelvis yaw frame; origin XY=root XY, origin Z=ground+nominal_root_height; +X forward +Y left +Z up; quaternion wxyz",
            "observation_order": sparse_order,
            "privileged_order": privileged_order,
            "actor_observation_order": sparse_order + privileged_order if self.cfg.teacher else sparse_order,
            "critic_observation_order": sparse_order + privileged_order,
            "target_velocity_estimator": {"enabled": self.cfg.rich_observations,
                "update": "once per pre_physics_step; one control-step observation lag",
                "difference_dt_s": self.step_dt, "clip_mps": [-2., 2.], "ema_alpha": 0.5,
                "reset": "zero; first subsequent control step establishes baseline"},
            "motion_file": self._motion["path"] if self._motion else None,
            "motion_split": self._motion["split"] if self._motion else None}

    def validate_kinematics(self, tolerance=0.002):
        """Compare exact-USD offline FK against measured PhysX body markers.

        Call after env.reset() or an actual physics step, not during construction.
        This returns evidence for the runner to enforce and save.
        """
        from g1_teleop.motion.kinematics import KinematicModel
        count = min(self.num_envs, 4)
        q = self.robot.data.joint_pos[:count, self.body_joint_ids].detach().cpu().numpy()
        expected = KinematicModel().keypoints(q)
        measured = self.raw_root_keypoints()[:count].detach().cpu().numpy()
        errors = np.linalg.norm(expected - measured, axis=-1)
        return {"environment_count": count, "max_marker_error_m": float(errors.max()),
                "mean_marker_error_m": float(errors.mean()), "tolerance_m": tolerance,
                "passed": bool(np.isfinite(errors).all() and errors.max() <= tolerance),
                "marker_names": TRACKED_BODY_NAMES}
