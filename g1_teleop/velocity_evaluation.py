"""Deterministic held-out velocity commands and CPU metrics; no simulator import."""
from __future__ import annotations

import math
import numpy as np


class VelocityEvaluationSchedule:
    """Phase-offset command blocks driven by global executed control steps.

    Episode resets do not reset the schedule. The selected command and phase
    must be captured together before stepping, then scored against that step's
    pre-reset state. Each environment sees all commands in one 45 second cycle.
    """
    NAMES = ("stand", "forward", "backward", "left", "right", "yaw_left",
             "yaw_right", "diagonal_forward_left", "diagonal_backward_right")
    COMMANDS = ((0., 0., 0.), (.20, 0., 0.), (-.20, 0., 0.),
                (0., .10, 0.), (0., -.10, 0.), (0., 0., .30),
                (0., 0., -.30), (.15, .08, 0.), (-.15, -.08, 0.))

    def __init__(self, step_dt=0.02, block_seconds=5.0, settling_seconds=1.0):
        values = (step_dt, block_seconds, settling_seconds)
        if not all(math.isfinite(v) for v in values) or step_dt <= 0 or block_seconds <= 0:
            raise ValueError("Schedule times must be finite; dt and block duration must be positive")
        if not 0 <= settling_seconds < block_seconds:
            raise ValueError("Settling duration must be nonnegative and shorter than a block")
        self.step_dt = float(step_dt)
        self.block_steps = round(block_seconds / step_dt)
        self.settling_steps = round(settling_seconds / step_dt)
        if self.block_steps < 1 or self.settling_steps >= self.block_steps:
            raise ValueError("Schedule durations are not representable at this control rate")

    def state(self, step, num_envs, device=None):
        step = int(step)
        if step < 0 or num_envs < 1:
            raise ValueError("Expected nonnegative step and positive environment count")
        age = step % self.block_steps
        if device is None:
            block = (np.arange(num_envs, dtype=np.int64) + step // self.block_steps) % len(self.NAMES)
            return {"command": np.asarray(self.COMMANDS, dtype=np.float32)[block],
                    "block_id": block, "block_elapsed_s": np.full(num_envs, age * self.step_dt),
                    "settled": np.full(num_envs, age >= self.settling_steps, dtype=bool)}
        import torch
        block = (torch.arange(num_envs, device=device) + step // self.block_steps) % len(self.NAMES)
        return {"command": torch.tensor(self.COMMANDS, dtype=torch.float32, device=device)[block],
                "block_id": block,
                "block_elapsed_s": torch.full((num_envs,), age * self.step_dt, device=device),
                "settled": torch.full((num_envs,), age >= self.settling_steps, dtype=torch.bool, device=device)}

    def commands(self, step, num_envs, device=None):
        return self.state(step, num_envs, device)["command"]

    def metadata(self):
        return {"version": "heldout_velocity_blocks_v1", "step_dt": self.step_dt,
                "block_seconds": self.block_steps * self.step_dt,
                "settling_seconds": self.settling_steps * self.step_dt,
                "cycle_seconds": len(self.NAMES) * self.block_steps * self.step_dt,
                "phase_rule": "block=(env_id+global_control_step//block_steps)%9; resets do not rewind",
                "falls_include_settling": True,
                "blocks": [{"id": i, "name": name, "command_vx_vy_yaw": list(command)}
                           for i, (name, command) in enumerate(zip(self.NAMES, self.COMMANDS))]}


def _array(value):
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value)


class VelocityEvaluationAccumulator:
    """Accumulate metrics captured before auto-reset; report JSON-safe numbers.

    update() accepts the env's metrics mapping. Falls count in every phase,
    including the first second. Speed/error scores exclude the settling phase
    but do not discard reset recovery or fallen states. Nonfinite samples are
    reported explicitly and excluded from numeric averages, never hidden.
    """
    def __init__(self, step_dt=0.02, schedule=None):
        self.schedule = schedule or VelocityEvaluationSchedule(step_dt=step_dt)
        self.step_dt = self.schedule.step_dt
        count = len(self.schedule.NAMES)
        self._samples = np.zeros(count, dtype=np.int64)
        self._scored = np.zeros(count, dtype=np.int64)
        self._falls = np.zeros(count, dtype=np.int64)
        self._invalid = np.zeros(count, dtype=np.int64)
        self._rows = [[] for _ in range(count)]

    def update(self, metrics):
        block = _array(metrics["velocity_evaluation_block_id"]).astype(np.int64)
        settled = _array(metrics["velocity_evaluation_settled"]).astype(bool)
        fallen = _array(metrics["fallen"]).astype(bool)
        actual = np.stack([_array(metrics[k]) for k in
                           ("actual_vx_mps", "actual_vy_mps", "actual_yaw_rate_radps")], axis=-1)
        command = np.stack([_array(metrics[k]) for k in
                            ("command_vx_mps", "command_vy_mps", "command_yaw_rate_radps")], axis=-1)
        if block.ndim != 1 or settled.shape != block.shape or fallen.shape != block.shape:
            raise ValueError("Velocity evaluation block/settled/fallen metrics must have shape [N]")
        if actual.shape != (len(block), 3) or command.shape != actual.shape:
            raise ValueError("Velocity command and actual metrics must have shape [N,3]")
        if np.any(block < 0) or np.any(block >= len(self.schedule.NAMES)):
            raise ValueError("Unknown velocity evaluation block id")
        expected = np.asarray(self.schedule.COMMANDS)[block]
        if not np.allclose(command, expected, atol=1e-6, rtol=0):
            raise ValueError("Recorded command does not match its held-out block; phase snapshot may be wrong")
        for idx in range(len(self.schedule.NAMES)):
            selected = block == idx
            scored = selected & settled
            finite = np.isfinite(actual).all(-1)
            self._samples[idx] += selected.sum()
            self._falls[idx] += (selected & fallen).sum()
            self._scored[idx] += scored.sum()
            self._invalid[idx] += (scored & ~finite).sum()
            valid = scored & finite
            if np.any(valid):
                self._rows[idx].append(actual[valid].astype(np.float64, copy=True))

    @staticmethod
    def _scores(actual, command):
        if len(actual) == 0:
            return {"valid_samples": 0, "axis_mae": None, "axis_rmse": None,
                    "actual_mean": None, "axis_gain": None, "axis_sign_agreement": None,
                    "xy_error_mean_mps": None, "xy_error_p95_mps": None,
                    "yaw_error_mean_radps": None, "yaw_error_p95_radps": None,
                    "actual_xy_speed_mean_mps": None, "actual_abs_yaw_mean_radps": None}
        error = actual - command
        xy_error = np.linalg.norm(error[:, :2], axis=-1)
        gain, sign = [], []
        for axis in range(3):
            active = np.abs(command[:, axis]) > 1e-8
            if not np.any(active):
                gain.append(None)
                sign.append(None)
            else:
                # Least-squares gain remains meaningful when signs are mixed.
                c, a = command[active, axis], actual[active, axis]
                gain.append(float(np.dot(c, a) / np.dot(c, c)))
                sign.append(float(np.mean(c * a > 0)))
        return {"valid_samples": len(actual), "axis_mae": np.abs(error).mean(0).tolist(),
                "axis_rmse": np.sqrt(np.square(error).mean(0)).tolist(),
                "actual_mean": actual.mean(0).tolist(), "axis_gain": gain, "axis_sign_agreement": sign,
                "xy_error_mean_mps": float(xy_error.mean()), "xy_error_p95_mps": float(np.percentile(xy_error, 95)),
                "yaw_error_mean_radps": float(np.abs(error[:, 2]).mean()),
                "yaw_error_p95_radps": float(np.percentile(np.abs(error[:, 2]), 95)),
                "actual_xy_speed_mean_mps": float(np.linalg.norm(actual[:, :2], axis=-1).mean()),
                "actual_abs_yaw_mean_radps": float(np.abs(actual[:, 2]).mean())}

    def summary(self):
        blocks, aggregate_actual, aggregate_command = [], [], []
        for idx, (name, target) in enumerate(zip(self.schedule.NAMES, self.schedule.COMMANDS)):
            actual = np.concatenate(self._rows[idx]) if self._rows[idx] else np.empty((0, 3))
            command = np.broadcast_to(target, actual.shape)
            scores = self._scores(actual, command)
            blocks.append({"id": idx, "name": name, "command_vx_vy_yaw": list(target),
                           "all_samples": int(self._samples[idx]), "scored_samples": int(self._scored[idx]),
                           "nonfinite_scored_samples": int(self._invalid[idx]), "falls": int(self._falls[idx]),
                           "exposure_env_seconds": float(self._samples[idx] * self.step_dt), **scores})
            if idx:
                aggregate_actual.append(actual)
                aggregate_command.append(command)
        moving = self._scores(np.concatenate(aggregate_actual), np.concatenate(aggregate_command))
        return {"schedule": self.schedule.metadata(), "axis_order": ["vx_mps", "vy_mps", "yaw_radps"],
                "blocks": blocks, "moving": moving, "standing": blocks[0],
                "falls_all_phases": int(self._falls.sum()),
                "nonfinite_scored_samples": int(self._invalid.sum()),
                "total_exposure_env_seconds": float(self._samples.sum() * self.step_dt),
                "note": "Speed scores omit block settling only. Falls include all phases. No automatic success claim."}
