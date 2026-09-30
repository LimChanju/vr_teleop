"""Optional commanded-locomotion curriculum on the identical G1 articulation.

Motion clips provide upper/body reference targets. Independently sampled
velocities train locomotion with PPO; no fabricated leg trajectory or scripted
root displacement is applied. This module does not change the base policy's
observation layout, action interpretation, or physical model.
"""
from __future__ import annotations

import math
import torch

from isaaclab.utils import configclass

from g1_teleop.velocity_evaluation import VelocityEvaluationSchedule
from .env import G1WholeBodyEnv, G1WholeBodyEnvCfg


@configclass
class G1VelocityEnvCfg(G1WholeBodyEnvCfg):
    command_max = (0.30, 0.15, 0.40)
    standing_fraction = 0.30
    command_resampling_s = 5.0
    command_sampling = "mixed"
    moving_xy_threshold = 0.08
    feet_air_time_reward_scale = 0.50
    both_feet_air_penalty_scale = 0.15
    moving_linear_velocity_reward_scale = 4.0
    moving_linear_velocity_error_variance = 0.04
    moving_yaw_velocity_reward_scale = 2.0
    moving_yaw_velocity_error_variance = 0.10
    velocity_evaluation = False


class G1VelocityEnv(G1WholeBodyEnv):
    cfg: G1VelocityEnvCfg

    def __init__(self, cfg: G1VelocityEnvCfg, render_mode=None, **kwargs):
        if cfg.command_sampling not in ("mixed", "pure_axis"):
            raise ValueError("command_sampling must be mixed or pure_axis")
        if not math.isfinite(cfg.moving_xy_threshold) or not 0 < cfg.moving_xy_threshold <= 0.3:
            raise ValueError("moving_xy_threshold must be finite and in (0,0.3]")
        if len(cfg.command_max) != 3 or any(not math.isfinite(v) or v < 0 for v in cfg.command_max):
            raise ValueError("command_max must contain three finite nonnegative limits")
        if cfg.command_sampling == "pure_axis" and any(
                lower > upper for lower, upper in zip((0.10, 0.08, 0.15), cfg.command_max)):
            raise ValueError("pure_axis command maxima must be at least (0.10,0.08,0.15)")
        if not 0.0 <= cfg.standing_fraction <= 1.0:
            raise ValueError("standing_fraction must be in [0,1]")
        if cfg.command_resampling_s <= 0.0:
            raise ValueError("command_resampling_s must be positive")
        if (cfg.moving_linear_velocity_error_variance <= 0.0
                or cfg.moving_yaw_velocity_error_variance <= 0.0):
            raise ValueError("Velocity reward error variances must be positive")
        if (cfg.moving_linear_velocity_reward_scale < 0.0
                or cfg.moving_yaw_velocity_reward_scale < 0.0):
            raise ValueError("Velocity reward scales must be nonnegative")
        super().__init__(cfg, render_mode, **kwargs)
        self._ensure_velocity_buffers()

    def _ensure_velocity_buffers(self):
        # Base construction can load a dataset and dispatch _sample_reference
        # before this subclass's __init__ has returned.
        if not hasattr(self, "_velocity_commands"):
            self._velocity_commands = torch.zeros(self.num_envs, 3, device=self.device)
            self._velocity_last_sample_step = torch.full(
                (self.num_envs,), -10**9, dtype=torch.long, device=self.device)
            self._velocity_interval_steps = max(1, round(self.cfg.command_resampling_s / self.step_dt))
        if self.cfg.velocity_evaluation and not hasattr(self, "_velocity_evaluation_schedule"):
            self._velocity_evaluation_schedule = VelocityEvaluationSchedule(step_dt=self.step_dt)
            self._velocity_evaluation_block = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
            self._velocity_evaluation_age = torch.zeros(self.num_envs, device=self.device)
            self._velocity_evaluation_settled = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

    def _set_evaluation_commands(self, ids=None):
        self._ensure_velocity_buffers()
        state = self._velocity_evaluation_schedule.state(self.common_step_counter, self.num_envs, self.device)
        ids = slice(None) if ids is None else ids
        self._velocity_commands[ids] = state["command"][ids]
        self.command_velocity[ids] = state["command"][ids]
        self._velocity_evaluation_block[ids] = state["block_id"][ids]
        self._velocity_evaluation_age[ids] = state["block_elapsed_s"][ids]
        self._velocity_evaluation_settled[ids] = state["settled"][ids]

    def _sample_velocity(self, ids):
        if self.external_mode or len(ids) == 0:
            return
        self._ensure_velocity_buffers()
        if self.cfg.velocity_evaluation:
            self._set_evaluation_commands(ids)
            return
        limits = torch.tensor(self.cfg.command_max, device=self.device)
        if self.cfg.command_sampling == "mixed":
            # Keep the legacy RNG calls and arithmetic exactly unchanged.
            commands = (2.0 * torch.rand(len(ids), 3, device=self.device) - 1.0) * limits
        else:
            draw = torch.rand(len(ids), device=self.device)
            axis = (draw >= 5.0 / 14.0).long() + (draw >= 10.0 / 14.0).long()
            minima = torch.tensor((0.10, 0.08, 0.15), device=self.device)[axis]
            magnitude = minima + torch.rand(len(ids), device=self.device) * (limits[axis] - minima)
            sign = torch.where(torch.rand(len(ids), device=self.device) < 0.5, -1.0, 1.0)
            commands = torch.zeros(len(ids), 3, device=self.device)
            commands.scatter_(1, axis[:, None], (magnitude * sign)[:, None])
        standing = torch.rand(len(ids), device=self.device) < self.cfg.standing_fraction
        commands[standing] = 0.0
        self._velocity_commands[ids] = commands
        self._velocity_last_sample_step[ids] = self.common_step_counter
        self.command_velocity[ids] = commands

    def _sample_reference(self, ids):
        super()._sample_reference(ids)
        if not self.external_mode:
            self._sample_velocity(ids)

    def _update_reference(self):
        super()._update_reference()
        if self.external_mode:
            return
        self._ensure_velocity_buffers()
        if self.cfg.velocity_evaluation:
            self._set_evaluation_commands()
            return
        due = (self.common_step_counter - self._velocity_last_sample_step >= self._velocity_interval_steps)
        self._sample_velocity(due.nonzero(as_tuple=False).flatten())
        # Base motion loading may supply zero recorded velocities. This variant
        # deliberately owns velocity commands independently of those clips.
        self.command_velocity.copy_(self._velocity_commands)

    def _get_rewards(self):
        reward = super()._get_rewards()
        moving = (torch.linalg.vector_norm(self.command_velocity[:, :2], dim=-1) > self.cfg.moving_xy_threshold) | (
            self.command_velocity[:, 2].abs() > 0.10)
        actual_linear = self.robot.data.root_lin_vel_b[:, :2]
        actual_yaw = self.robot.data.root_ang_vel_b[:, 2]
        linear_error_squared = (actual_linear - self.command_velocity[:, :2]).square().sum(-1)
        yaw_error_squared = (actual_yaw - self.command_velocity[:, 2]).square()
        # Replace, rather than add to, the broad base tracking terms. The base
        # class has already accumulated its contribution to the episode sums;
        # applying the same delta to both paths keeps logging and PPO aligned.
        # Zero/small commands retain the base standing reward exactly.
        replacements = (
            ("linear_velocity", 2.0 * torch.exp(-linear_error_squared / 0.25),
             self.cfg.moving_linear_velocity_reward_scale * torch.exp(
                 -linear_error_squared / self.cfg.moving_linear_velocity_error_variance)),
            ("yaw_velocity", torch.exp(-yaw_error_squared / 0.25),
             self.cfg.moving_yaw_velocity_reward_scale * torch.exp(
                 -yaw_error_squared / self.cfg.moving_yaw_velocity_error_variance)),
        )
        for name, previous, replacement in replacements:
            delta = torch.where(moving, replacement - previous, torch.zeros_like(previous)) * self.step_dt
            reward = reward + delta
            self._episode_sums[name].add_(delta)
        first_contact = self.contact_sensor.compute_first_contact(self.step_dt)[:, self.foot_contact_ids]
        last_air_time = self.contact_sensor.data.last_air_time[:, self.foot_contact_ids]
        # Reward a genuine swing only on touchdown. A dragging foot gets no
        # airtime bonus; the base contact-weighted foot-slide penalty stays on.
        useful_airtime = (last_air_time - 0.20).clamp(min=0.0, max=0.30)
        air_reward = self.cfg.feet_air_time_reward_scale * (useful_airtime * first_contact).sum(-1) * moving
        contact = torch.linalg.vector_norm(
            self.contact_sensor.data.net_forces_w[:, self.foot_contact_ids], dim=-1) > 5.0
        flight_penalty = -self.cfg.both_feet_air_penalty_scale * (~contact.any(-1)) * moving
        for name, value in (("velocity_feet_airtime", air_reward), ("velocity_both_feet_air", flight_penalty)):
            reward = reward + value * self.step_dt
            if name not in self._episode_sums:
                self._episode_sums[name] = torch.zeros_like(value)
            self._episode_sums[name].add_(value * self.step_dt)
        self.metrics.update({
            "command_vx_mps": self.command_velocity[:, 0].clone(),
            "command_vy_mps": self.command_velocity[:, 1].clone(),
            "command_yaw_rate_radps": self.command_velocity[:, 2].clone(),
            "actual_vx_mps": actual_linear[:, 0].clone(),
            "actual_vy_mps": actual_linear[:, 1].clone(),
            "actual_yaw_rate_radps": actual_yaw.clone(),
            "vx_absolute_error_mps": (actual_linear[:, 0] - self.command_velocity[:, 0]).abs(),
            "vy_absolute_error_mps": (actual_linear[:, 1] - self.command_velocity[:, 1]).abs(),
            "linear_velocity_error_mps": torch.linalg.vector_norm(actual_linear - self.command_velocity[:, :2], dim=-1),
            "yaw_rate_error_radps": (actual_yaw - self.command_velocity[:, 2]).abs(),
            "commanded_moving": moving.clone(),
        })
        if self.cfg.velocity_evaluation and not self.external_mode:
            # These tags were captured with the command before this physics
            # step. Recomputing from the now-incremented global step would
            # mislabel the last action at every block transition.
            self.metrics.update({
                "velocity_evaluation_block_id": self._velocity_evaluation_block.clone(),
                "velocity_evaluation_block_elapsed_s": self._velocity_evaluation_age.clone(),
                "velocity_evaluation_settled": self._velocity_evaluation_settled.clone(),
            })
        return reward

    def policy_metadata(self):
        metadata = super().policy_metadata()
        metadata.update({
            "environment_variant": "g1_commanded_velocity_v1",
            "training_velocity_limits": list(self.cfg.command_max),
            "training_standing_fraction": self.cfg.standing_fraction,
            "velocity_command_resampling_s": self.cfg.command_resampling_s,
            "velocity_command_sampling": {"version": "velocity_command_sampling_v1",
                "mode": self.cfg.command_sampling,
                "pure_axis_probabilities_given_moving": [5.0 / 14.0, 5.0 / 14.0, 4.0 / 14.0],
                "pure_axis_minimum_absolute_commands": [0.10, 0.08, 0.15],
                "pure_axis_sign": "uniform independent positive/negative",
                "standing_fraction": self.cfg.standing_fraction},
            "feet_air_time_reward_scale": self.cfg.feet_air_time_reward_scale,
            "both_feet_air_penalty_scale": self.cfg.both_feet_air_penalty_scale,
            "velocity_reward_contract": {
                "version": "moving_velocity_replacement_v2",
                "moving_condition": f"norm(command_xy)>{self.cfg.moving_xy_threshold:g} m/s OR abs(command_yaw)>0.10 rad/s",
                "moving_thresholds": {"xy_norm_mps": self.cfg.moving_xy_threshold,
                    "yaw_abs_radps": 0.10, "comparison": ">"},
                "linear_velocity": {"scale": self.cfg.moving_linear_velocity_reward_scale,
                    "squared_error_denominator_m2_s2": self.cfg.moving_linear_velocity_error_variance},
                "yaw_velocity": {"scale": self.cfg.moving_yaw_velocity_reward_scale,
                    "squared_error_denominator_rad2_s2": self.cfg.moving_yaw_velocity_error_variance},
                "standing_terms": "unchanged base linear 2*exp(-e2/.25), yaw exp(-e2/.25)",
                "application": "replace base terms for moving commands; integrate every term with control dt",
            },
            "velocity_training_note": "Commanded velocities are sampled independently; learning does not guarantee tracking performance.",
        })
        if self.cfg.velocity_evaluation:
            metadata["velocity_evaluation_schedule"] = VelocityEvaluationSchedule(step_dt=self.step_dt).metadata()
        return metadata
