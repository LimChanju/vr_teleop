#!/usr/bin/env python3
"""Train, resume, evaluate, export, and run G1 policies in the same simulator."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import shutil
import sys
import time
import traceback
from datetime import datetime

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

CONTRACT_KEYS = (
    "model", "body_joint_names", "observation_dim", "observation_order", "action_scale", "action_clip",
    "nominal_q", "soft_joint_limits", "control_dt", "physics_dt", "coordinate_version",
    "body_joint_stiffness", "body_joint_damping", "body_joint_armature", "body_joint_effort_limits",
    "collision_overrides", "nominal_root_height_m", "tracked_local_offsets_m", "hand_position_m", "observation_version",
)


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(data, indent=2, ensure_ascii=False, default=str) + "\n")
    temp.replace(path)


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_linear_options(mode, velocity_variant, sigma=None, weight=None):
    """Validate optional moving linear reward overrides before simulator startup."""
    if sigma is None and weight is None:
        return
    if mode != "train" or not velocity_variant:
        raise ValueError("Linear reward overrides require train mode and the velocity variant")
    if sigma is not None and (isinstance(sigma, bool) or not math.isfinite(sigma)
                              or not 0 < sigma <= 2 or sigma ** 2 == 0):
        raise ValueError("--linear-tracking-sigma must be finite and in (0,2] m/s")
    if weight is not None and (isinstance(weight, bool) or not math.isfinite(weight) or not 0 < weight <= 20):
        raise ValueError("--linear-reward-weight must be finite and in (0,20]")


def validate_yaw_options(mode, velocity_variant, sigma=None, weight=None):
    """Validate optional training overrides before starting the simulator."""
    if sigma is None and weight is None:
        return
    if mode != "train" or not velocity_variant:
        raise ValueError("Yaw reward overrides require train mode and the velocity variant (--velocity-training or a velocity --checkpoint)")
    if sigma is not None and (not math.isfinite(sigma) or not 0 < sigma <= 2 or sigma ** 2 == 0):
        raise ValueError("--yaw-tracking-sigma must be finite and in (0,2] rad/s")
    if weight is not None and (not math.isfinite(weight) or not 0 < weight <= 20):
        raise ValueError("--yaw-reward-weight must be finite and in (0,20]")


def validate_standing_yaw_options(mode, velocity_variant, sigma=None, weight=None):
    """Validate opt-in standing yaw rewards without changing inference actions."""
    if sigma is None and weight is None:
        return
    if mode != "train" or not velocity_variant:
        raise ValueError("Standing yaw reward overrides require train mode and the velocity variant")
    if sigma is not None and (isinstance(sigma, bool) or not math.isfinite(sigma)
                              or not 0 < sigma <= 2 or sigma ** 2 == 0):
        raise ValueError("--standing-yaw-tracking-sigma must be finite and in (0,2] rad/s")
    if weight is not None and (isinstance(weight, bool) or not math.isfinite(weight) or not 0 < weight <= 20):
        raise ValueError("--standing-yaw-reward-weight must be finite and in (0,20]")


def validate_velocity_command_options(mode, velocity_variant, sampling=None, xy_threshold=None, command_max=None):
    """Validate opt-in command curriculum without starting the simulator."""
    if sampling is None and xy_threshold is None:
        return
    if mode != "train" or not velocity_variant:
        raise ValueError("Command sampling/threshold overrides require train mode and the velocity variant")
    if sampling is not None and sampling not in ("mixed", "pure_axis"):
        raise ValueError("--command-sampling must be mixed or pure_axis")
    if xy_threshold is not None and (isinstance(xy_threshold, bool) or not math.isfinite(xy_threshold)
                                    or not 0 < xy_threshold <= 0.3):
        raise ValueError("--moving-xy-threshold must be finite and in (0,0.3] m/s")
    if sampling == "pure_axis":
        limits = (0.30, 0.15, 0.40) if command_max is None else command_max
        if (not isinstance(limits, (list, tuple)) or len(limits) != 3
                or any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v)
                       or v < lower for v, lower in zip(limits, (0.10, 0.08, 0.15)))):
            raise ValueError("pure_axis command maxima must be finite and at least (0.10,0.08,0.15)")


def restore_velocity_settings(cfg, metadata):
    """Restore velocity training semantics before any explicit new overrides.

    Older velocity metadata predates the moving-only replacement: its reward
    was the base linear 2*exp(-e2/.25), yaw exp(-e2/.25). Restoring those values
    makes the replacement delta zero, preserving that legacy behavior.
    """
    if not metadata or metadata.get("environment_variant") != "g1_commanded_velocity_v1":
        return
    # Absence of the optional standing term means the original base yaw reward,
    # even if this configuration object previously held a nondefault value.
    changes = {"standing_yaw_velocity_reward_scale": 1.0,
               "standing_yaw_velocity_error_variance": 0.25}
    def number(value, name, *, positive=False, maximum=None):
        if isinstance(value, bool):
            raise ValueError(f"Saved {name} must be a number")
        try:
            value = float(value)
        except (ValueError, TypeError) as error:
            raise ValueError(f"Invalid saved {name}") from error
        if not math.isfinite(value) or value < 0 or (positive and value == 0) or (maximum is not None and value > maximum):
            raise ValueError(f"Invalid saved {name}: {value}")
        return value
    if "velocity_reward_contract" not in metadata:
        changes.update(moving_linear_velocity_reward_scale=2., moving_linear_velocity_error_variance=.25,
                       moving_yaw_velocity_reward_scale=1., moving_yaw_velocity_error_variance=.25)
    else:
        contract = metadata["velocity_reward_contract"]
        if not isinstance(contract, dict) or contract.get("version") != "moving_velocity_replacement_v2":
            raise ValueError("Unsupported saved velocity_reward_contract version")
        for prefix, term, variance_key in (
            ("moving_linear_velocity", "linear_velocity", "squared_error_denominator_m2_s2"),
            ("moving_yaw_velocity", "yaw_velocity", "squared_error_denominator_rad2_s2"),
        ):
            values = contract.get(term)
            if not isinstance(values, dict) or "scale" not in values or variance_key not in values:
                raise ValueError(f"Incomplete saved velocity reward term: {term}")
            changes[prefix + "_reward_scale"] = number(values["scale"], term + ".scale")
            changes[prefix + "_error_variance"] = number(values[variance_key], term + ".variance", positive=True)
        if "standing_yaw_velocity" in contract:
            values = contract["standing_yaw_velocity"]
            variance_key = "squared_error_denominator_rad2_s2"
            if not isinstance(values, dict) or "scale" not in values or variance_key not in values:
                raise ValueError("Incomplete saved velocity reward term: standing_yaw_velocity")
            changes["standing_yaw_velocity_reward_scale"] = number(
                values["scale"], "standing_yaw_velocity.scale", positive=True)
            changes["standing_yaw_velocity_error_variance"] = number(
                values[variance_key], "standing_yaw_velocity.variance", positive=True)
    for key, field, positive, maximum in (
        ("training_standing_fraction", "standing_fraction", False, 1.),
        ("velocity_command_resampling_s", "command_resampling_s", True, None),
        ("feet_air_time_reward_scale", "feet_air_time_reward_scale", False, None),
        ("both_feet_air_penalty_scale", "both_feet_air_penalty_scale", False, None),
    ):
        if key in metadata:
            changes[field] = number(metadata[key], key, positive=positive, maximum=maximum)
    if "training_velocity_limits" in metadata:
        limits = metadata["training_velocity_limits"]
        if not isinstance(limits, (list, tuple)) or len(limits) != 3:
            raise ValueError("Saved training_velocity_limits must have three entries")
        changes["command_max"] = tuple(number(value, "training_velocity_limits") for value in limits)
    # Missing structured fields are legacy mixed sampling with the original
    # .08/.10 moving mask; preserve the existing reward replacement semantics.
    changes.update(command_sampling="mixed", moving_xy_threshold=0.08)
    sampling = metadata.get("velocity_command_sampling")
    if sampling is not None:
        if not isinstance(sampling, dict) or sampling.get("version") != "velocity_command_sampling_v1":
            raise ValueError("Unsupported saved velocity_command_sampling")
        if sampling.get("mode") not in ("mixed", "pure_axis"):
            raise ValueError("Unsupported saved velocity command sampling mode")
        expected = {"pure_axis_probabilities_given_moving": [5.0 / 14.0, 5.0 / 14.0, 4.0 / 14.0],
                    "pure_axis_minimum_absolute_commands": [0.10, 0.08, 0.15],
                    "pure_axis_sign": "uniform independent positive/negative"}
        if any(sampling.get(key) != value for key, value in expected.items()):
            raise ValueError("Unsupported saved pure-axis sampling constants")
        standing = changes.get("standing_fraction", cfg.standing_fraction)
        if sampling.get("standing_fraction") != standing:
            raise ValueError("Conflicting saved standing fractions")
        changes["command_sampling"] = sampling.get("mode")
    thresholds = metadata.get("velocity_reward_contract", {}).get("moving_thresholds")
    if thresholds is not None:
        if (not isinstance(thresholds, dict) or thresholds.get("yaw_abs_radps") != 0.10
                or thresholds.get("comparison") != ">" or "xy_norm_mps" not in thresholds):
            raise ValueError("Unsupported saved moving thresholds")
        changes["moving_xy_threshold"] = number(thresholds["xy_norm_mps"], "moving_thresholds.xy_norm_mps", positive=True, maximum=.3)
    validate_velocity_command_options("train", True, changes["command_sampling"],
        changes["moving_xy_threshold"], changes.get("command_max", cfg.command_max))
    # Reject malformed metadata before mutating any configuration fields.
    for field, value in changes.items():
        setattr(cfg, field, value)


def configure_head_tracking_weight(cfg, metadata=None, *, override=None, mode="train"):
    """Restore the optional head-only multiplier; legacy metadata means 1."""
    if override is not None and mode != "train":
        raise ValueError("--head-tracking-weight is a training option; evaluation restores the saved value")
    contract = (metadata or {}).get("reward_contract") or {}
    if not isinstance(contract, dict):
        raise ValueError("Saved reward_contract must be a mapping")
    value = contract.get("head_tracking_weight", 1.0) if override is None else override
    if isinstance(value, bool):
        raise ValueError("head_tracking_weight must be a number")
    try:
        value = float(value)
    except (ValueError, TypeError) as error:
        raise ValueError("Invalid head_tracking_weight") from error
    if not math.isfinite(value) or not 0 < value <= 10:
        raise ValueError("--head-tracking-weight must be finite and in (0,10]")
    if cfg is not None:
        cfg.head_tracking_weight = value
    return value


def set_teleop_hold_targets(raw, *, after_reset=False):
    """Reset to grounded neutral; an ordinary disarm holds the reached pose."""
    positions = (raw.nominal_keypoints.expand(raw.num_envs, -1, -1) if after_reset
                 else raw.current_keypoints().detach())
    raw.set_external_targets(positions, raw.command_velocity.new_zeros(raw.command_velocity.shape),
                             reset_velocity=True)


def main():
    from isaaclab.app import AppLauncher

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("train", "evaluate", "teleop", "export"))
    parser.add_argument("--stage", choices=("sparse", "teacher", "student"), default=None)
    parser.add_argument("--num-envs", type=int, default=None)
    parser.add_argument("--iterations", type=int, default=3000,
                        help="Additional updates, including when resuming")
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--teacher-checkpoint", type=Path)
    parser.add_argument("--warm-start", type=Path,
                        help="Initialize actor/critic weights with an explicitly checked observation migration")
    parser.add_argument("--warm-start-noise", type=float, default=None)
    parser.add_argument("--rich-observations", action="store_true")
    parser.add_argument("--precision-training", action="store_true",
                        help="Use tighter head/hand tracking rewards and a stronger fall penalty")
    parser.add_argument("--learning-rate", type=float,
                        help="Explicit training optimizer learning rate (saved in run configuration)")
    parser.add_argument("--tracking-sigma", type=float,
                        help="Training-only marker reward length scale in metres; variance=sigma**2")
    parser.add_argument("--head-tracking-weight", type=float,
                        help="Training-only head reward multiplier in (0,10]; hand coefficients stay unchanged")
    parser.add_argument("--motion-file", type=Path)
    parser.add_argument("--motion-split", choices=("train", "eval"), default=None)
    parser.add_argument("--velocity-training", action="store_true",
                        help="Use commanded locomotion curriculum with the same sparse policy inputs")
    parser.add_argument("--command-sampling", choices=("mixed", "pure_axis"),
                        help="Velocity training only: independent mixed commands or a single moving axis")
    parser.add_argument("--moving-xy-threshold", type=float,
                        help="Velocity training only: moving XY speed threshold in (0,0.3] m/s")
    parser.add_argument("--velocity-evaluation", action="store_true",
                        help="Evaluate deterministic held-out velocity commands with settled block metrics")
    parser.add_argument("--yaw-tracking-sigma", type=float,
                        help="Velocity training only: yaw reward width in rad/s, (0,2]; variance=sigma**2")
    parser.add_argument("--linear-tracking-sigma", type=float,
                        help="Velocity training only: moving XY reward width in m/s, (0,2]; variance=sigma**2")
    parser.add_argument("--linear-reward-weight", type=float,
                        help="Velocity training only: moving XY reward weight in (0,20]")
    parser.add_argument("--yaw-reward-weight", type=float,
                        help="Velocity training only: moving yaw reward weight in (0,20]")
    parser.add_argument("--standing-yaw-tracking-sigma", type=float,
                        help="Velocity training only: nonmoving yaw reward width in rad/s, (0,2]; legacy default 0.5")
    parser.add_argument("--standing-yaw-reward-weight", type=float,
                        help="Velocity training only: nonmoving yaw reward weight in (0,20]; legacy default 1")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--steps", type=int, default=2000,
                        help="Evaluation/teleop steps; 0 = continuous teleop")
    parser.add_argument("--max-hours", type=float, default=0,
                        help="Stop training at this wall time; 0 uses iterations only")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--input-timeout", type=float, default=0.25)
    parser.add_argument("--real-time", action="store_true")
    parser.add_argument("--record-trace", action="store_true",
                        help="Save environment-zero target/state/action trace (evaluation/teleop)")
    parser.add_argument("--capture-frame", action="store_true",
                        help="Save a final third-person simulator render as preview.png")
    parser.add_argument("--use-exported-policy", action="store_true",
                        help="Run policy.pt beside --checkpoint through TorchScript")
    parser.add_argument("--zero-policy", action="store_true",
                        help="Evaluation baseline: hold nominal joint targets")
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.precision_training and args.mode != "train":
        parser.error("--precision-training is a training option; evaluation restores the saved reward settings")
    if args.learning_rate is not None and (args.mode != "train" or not 0 < args.learning_rate <= 0.01):
        parser.error("--learning-rate requires train mode and a value in (0,0.01]")
    if args.tracking_sigma is not None and (args.mode != "train" or not 0 < args.tracking_sigma <= 1):
        parser.error("--tracking-sigma requires train mode and a value in (0,1] metres")
    try:
        configure_head_tracking_weight(None, override=args.head_tracking_weight, mode=args.mode)
    except ValueError as error:
        parser.error(str(error))
    if args.velocity_evaluation:
        if args.mode != "evaluate":
            parser.error("--velocity-evaluation requires evaluate mode")
        args.velocity_training = True
    if args.capture_frame:
        if args.mode not in ("evaluate", "teleop"):
            parser.error("--capture-frame is for evaluation or teleop")
        args.enable_cameras = True
    if args.iterations < 1 or args.steps < 0 or args.max_hours < 0:
        parser.error("Invalid iteration/step/time limit")
    if args.xr and args.headless:
        parser.error("--xr requires a graphical session, not --headless")
    if args.mode != "train" and args.checkpoint is None and not args.zero_policy:
        parser.error("--checkpoint is required")
    if args.mode != "teleop" and args.mode != "train" and args.steps == 0:
        parser.error("Evaluation needs a finite --steps count")
    if args.record_trace and args.steps == 0:
        parser.error("--record-trace requires a finite --steps count")
    if args.use_exported_policy and args.checkpoint is None:
        parser.error("--use-exported-policy requires --checkpoint")
    args.num_envs = args.num_envs or (2048 if args.mode == "train" else 64 if args.mode == "evaluate" else 1)
    if args.mode == "teleop" and args.num_envs != 1:
        parser.error("Teleoperation uses one robot (--num-envs 1)")

    checkpoint_meta = None
    if args.checkpoint:
        args.checkpoint = args.checkpoint.expanduser().resolve()
        sidecar = args.checkpoint.parent / "run_config.json"
        if not sidecar.is_file():
            parser.error(f"Missing model contract: {sidecar}")
        checkpoint_meta = json.loads(sidecar.read_text())
        checkpoint_meta.setdefault("observation_version", "sparse_positions_v1")
        if checkpoint_meta["observation_version"] == "sparse_tracking_v2":
            args.rich_observations = True
        if checkpoint_meta.get("environment_variant") == "g1_commanded_velocity_v1":
            args.velocity_training = True
        saved_stage = checkpoint_meta["stage"]
        if args.stage and args.stage != saved_stage:
            parser.error("Resuming/evaluating must use the saved stage")
        args.stage = saved_stage
        if args.mode in ("train", "evaluate") and args.motion_file is None and checkpoint_meta.get("motion_file"):
            saved_path = Path(checkpoint_meta["motion_file"])
            portable = ROOT / "data" / "motions" / saved_path.name
            args.motion_file = saved_path if saved_path.exists() else portable
    args.stage = args.stage or "sparse"
    try:
        validate_linear_options(args.mode, args.velocity_training, args.linear_tracking_sigma, args.linear_reward_weight)
        validate_yaw_options(args.mode, args.velocity_training, args.yaw_tracking_sigma, args.yaw_reward_weight)
        validate_standing_yaw_options(args.mode, args.velocity_training,
            args.standing_yaw_tracking_sigma, args.standing_yaw_reward_weight)
        validate_velocity_command_options(args.mode, args.velocity_training, args.command_sampling,
            args.moving_xy_threshold, (checkpoint_meta or {}).get("training_velocity_limits"))
    except ValueError as error:
        parser.error(str(error))
    if args.warm_start and (args.mode != "train" or args.stage != "sparse" or args.checkpoint or args.teacher_checkpoint):
        parser.error("--warm-start is for sparse training initialization, separately from resume/distillation")
    if args.warm_start_noise is not None and not args.warm_start:
        parser.error("--warm-start-noise requires --warm-start")
    if args.teacher_checkpoint and (args.stage != "student" or args.mode != "train"):
        parser.error("--teacher-checkpoint is only for --stage student training")
    if args.mode == "teleop" and args.stage == "teacher":
        parser.error("Privileged teacher cannot run from Quest inputs; train/export a sparse or student policy")
    if args.mode == "train" and args.stage == "student" and not (args.checkpoint or args.teacher_checkpoint):
        parser.error("Student distillation requires --teacher-checkpoint or a student --checkpoint")
    if args.motion_file and not args.motion_file.is_file():
        parser.error(f"Motion file missing: {args.motion_file}")
    args.motion_split = args.motion_split or ("eval" if args.mode == "evaluate" else "train")
    output = (args.output or ROOT / "runs" / (datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + args.mode)).resolve()
    if output.exists() and any(output.iterdir()):
        parser.error(f"Output already contains files; choose a new --output: {output}")
    output.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    stopping = False
    stop_reason = None

    def request_stop(reason):
        nonlocal stopping, stop_reason
        if not stopping:
            print(f"[G1] Stop requested: {reason}", flush=True)
        stopping, stop_reason = True, reason

    xr_requested = args.xr  # AppLauncher consumes/removes this Namespace field.
    launcher = AppLauncher(args)
    app = launcher.app
    env = runner = receiver = xr = None
    report = {"mode": args.mode, "stage": args.stage, "status": "starting",
              "physical_quest_verified": False, "output": str(output)}
    try:
        import builtins
        import torch
        import isaaclab.utils.math as math_utils
        from isaaclab.sim import SimulationContext
        from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
        from rsl_rl.runners import OnPolicyRunner
        from g1_teleop.sim.env import G1WholeBodyEnv, G1WholeBodyEnvCfg
        from g1_teleop.training import runner_config

        # These compatibility fixes apply only to this interpreter.
        math_utils.quat_rotate = math_utils.quat_apply
        original_step, original_render = SimulationContext.step, SimulationContext.render

        def guarded(method):
            def call(sim, *positional, **keywords):
                if stopping:
                    raise KeyboardInterrupt
                return method(sim, *positional, **keywords)
            return call

        def on_stop(sim, event):
            if not sim._disable_app_control_on_stop_handle:
                request_stop("window closed or simulator stopped")

        SimulationContext._app_control_on_stop_handle_fn = on_stop
        SimulationContext.step = guarded(original_step)
        SimulationContext.render = guarded(original_render)
        signal.signal(signal.SIGINT, lambda *_: request_stop("SIGINT"))
        signal.signal(signal.SIGTERM, lambda *_: request_stop("SIGTERM"))
        torch.set_num_threads(8)
        torch.backends.cuda.matmul.allow_tf32 = True
        env_class, cfg_class = G1WholeBodyEnv, G1WholeBodyEnvCfg
        if args.velocity_training:
            from g1_teleop.sim.velocity_env import G1VelocityEnv, G1VelocityEnvCfg
            env_class, cfg_class = G1VelocityEnv, G1VelocityEnvCfg
        cfg = cfg_class()
        cfg.scene.num_envs = args.num_envs
        cfg.sim.device = args.device
        cfg.seed = args.seed
        cfg.teacher = args.stage == "teacher"
        cfg.rich_observations = args.rich_observations
        cfg.record_failure_events = args.mode == "evaluate"
        reward_fields = ("tracking_reward_weight", "tracking_error_variance", "height_reward_weight",
                         "height_error_variance", "fall_cost")
        if checkpoint_meta and checkpoint_meta.get("reward_contract"):
            for key in reward_fields:
                setattr(cfg, key, checkpoint_meta["reward_contract"][key])
        if args.precision_training:
            for key, value in zip(reward_fields, (6.0, 0.01, 2.0, 0.0025, 5.0)):
                setattr(cfg, key, value)
        if args.tracking_sigma is not None:
            cfg.tracking_error_variance = args.tracking_sigma ** 2
        configure_head_tracking_weight(cfg, checkpoint_meta, override=args.head_tracking_weight, mode=args.mode)
        if args.velocity_training:
            restore_velocity_settings(cfg, checkpoint_meta)
            if args.command_sampling is not None:
                cfg.command_sampling = args.command_sampling
            if args.moving_xy_threshold is not None:
                cfg.moving_xy_threshold = args.moving_xy_threshold
            validate_velocity_command_options("train", True, cfg.command_sampling,
                cfg.moving_xy_threshold, cfg.command_max)
            if args.linear_tracking_sigma is not None:
                cfg.moving_linear_velocity_error_variance = args.linear_tracking_sigma ** 2
            if args.linear_reward_weight is not None:
                cfg.moving_linear_velocity_reward_scale = args.linear_reward_weight
            if args.yaw_tracking_sigma is not None:
                cfg.moving_yaw_velocity_error_variance = args.yaw_tracking_sigma ** 2
            if args.yaw_reward_weight is not None:
                cfg.moving_yaw_velocity_reward_scale = args.yaw_reward_weight
            if args.standing_yaw_tracking_sigma is not None:
                cfg.standing_yaw_velocity_error_variance = args.standing_yaw_tracking_sigma ** 2
            if args.standing_yaw_reward_weight is not None:
                cfg.standing_yaw_velocity_reward_scale = args.standing_yaw_reward_weight
            cfg.velocity_evaluation = args.velocity_evaluation
        cfg.motion_file = str(args.motion_file.resolve()) if args.motion_file else None
        cfg.motion_split = args.motion_split
        if args.mode == "teleop":
            cfg.motion_file = None  # Runtime sparse actor does not need a motion database.
            cfg.episode_length_s = 3600.0
        if args.capture_frame:
            cfg.viewer.eye = (2.2, 2.2, 1.6)
            cfg.viewer.lookat = (0.0, 0.0, 0.8)
            cfg.viewer.resolution = (960, 720)
        raw = env_class(cfg=cfg, render_mode="rgb_array" if args.capture_frame else None)
        if args.capture_frame:
            raw.render(recompute=True)  # Attach the RGB annotator before physics renders warm it up.
        if args.mode == "teleop":
            raw.external_mode = True
        if getattr(builtins, "ISAACLAB_CALLBACK_EXCEPTION", None) is not None:
            raise builtins.ISAACLAB_CALLBACK_EXCEPTION

        class SparseWrapper(RslRlVecEnvWrapper):
            def get_observations(self):
                obs, extras = super().get_observations()
                extras["observations"]["teacher"] = extras["observations"]["critic"]
                return obs, extras

            def step(self, actions):
                obs, reward, done, extras = super().step(actions)
                extras["observations"]["teacher"] = extras["observations"]["critic"]
                return obs, reward, done, extras

        env = SparseWrapper(raw, clip_actions=cfg.action_clip)
        obs, _ = env.get_observations()
        agent_cfg = runner_config(args.stage, args.seed, args.device)
        if checkpoint_meta and args.mode == "train":
            agent_cfg = copy.deepcopy(checkpoint_meta["runner"])
            agent_cfg.update(seed=args.seed, device=args.device)
        if args.learning_rate is not None:
            agent_cfg["algorithm"]["learning_rate"] = args.learning_rate
        metadata = raw.policy_metadata()
        if checkpoint_meta and checkpoint_meta.get("reset_contract") != metadata["reset_contract"]:
            # Old checkpoints remain usable: this fixes reset initialization,
            # without changing their observation/action or physical parameters.
            metadata["reset_contract_migration"] = {
                "checkpoint": checkpoint_meta.get("reset_contract", "legacy_default_height_and_measured_reset_targets"),
                "runtime": metadata["reset_contract"],
            }
        metadata.setdefault("training_velocity_limits", [0.0, 0.0, 0.0])
        if args.mode != "train" and checkpoint_meta:
            metadata["scenario_velocity_limits"] = metadata["training_velocity_limits"]
            metadata["training_velocity_limits"] = checkpoint_meta.get("training_velocity_limits", [0.0, 0.0, 0.0])
        metadata["kinematics_validation"] = raw.validate_kinematics()
        if not metadata["kinematics_validation"]["passed"]:
            raise RuntimeError(f"USD and retarget FK disagree: {metadata['kinematics_validation']}")
        snapshot_dir = output / "source"
        for source_name in ("g1_teleop/sim/env.py", "g1_teleop/sim/asset.py", "g1_teleop/training.py",
                            "g1_teleop/motion/g1_29dof_kinematics.json", "g1_teleop/sim/velocity_env.py",
                            "g1_teleop/warm_start.py", "g1_teleop/velocity_evaluation.py", "scripts/g1/run.py"):
            destination = snapshot_dir / source_name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / source_name, destination)
        metadata["source_sha256"] = {str(p.relative_to(snapshot_dir)): sha256(p)
                                     for p in snapshot_dir.rglob("*") if p.is_file()}
        metadata.update(stage=args.stage, runner=agent_cfg, seed=args.seed,
                        motion_file=cfg.motion_file, motion_split=cfg.motion_split,
                        motion_sha256=sha256(args.motion_file) if args.motion_file else None,
                        observation_dim=obs.shape[1], num_envs=args.num_envs,
                        source_checkpoint=str(args.checkpoint) if args.checkpoint else None,
                        physical_quest_verified=False)
        write_json(output / "run_config.json", metadata)
        write_json(output / "nominal_targets.json", {
            "nominal_targets": metadata["nominal_keypoints_root_m"],
            "coordinates": metadata["coordinates"],
        })
        if checkpoint_meta:
            for key in CONTRACT_KEYS:
                if metadata.get(key) != checkpoint_meta.get(key):
                    raise ValueError(f"Model contract mismatch in {key}: {metadata.get(key)} vs {checkpoint_meta.get(key)}")
            if metadata["observation_version"] == "sparse_tracking_v2" and metadata["target_velocity_estimator"] != checkpoint_meta.get("target_velocity_estimator"):
                raise ValueError("Target velocity filter/timing contract mismatch")
        runner = OnPolicyRunner(env, copy.deepcopy(agent_cfg),
                                log_dir=str(output) if args.mode == "train" else None,
                                device=args.device)
        runner.logger_type = "tensorboard"
        if args.warm_start:
            from g1_teleop.warm_start import warm_start_policy
            warm_meta = json.loads((args.warm_start.parent / "run_config.json").read_text())
            if warm_meta["stage"] != "sparse":
                raise ValueError("Warm-start source must be a sparse PPO policy")
            for key in CONTRACT_KEYS:
                if key in ("observation_dim", "observation_order", "observation_version"):
                    continue
                if warm_meta.get(key) != metadata.get(key):
                    raise ValueError(f"Warm-start physical contract mismatch: {key}")
            metadata["warm_start"] = warm_start_policy(runner.alg.policy, str(args.warm_start), args.warm_start_noise)
            metadata["warm_start"]["sha256"] = sha256(args.warm_start)
            write_json(output / "run_config.json", metadata)
            print(f"[G1] WARM START {metadata['warm_start']}", flush=True)
        if args.checkpoint:
            saved_info = runner.load(str(args.checkpoint), load_optimizer=args.mode == "train")
            if args.mode == "train":
                runner.current_learning_iteration = saved_info["next_iteration"] if isinstance(saved_info, dict) and "next_iteration" in saved_info else runner.current_learning_iteration + 1
                # RSL restores optimizer groups but not PPO's separate adaptive-LR scalar.
                restored_lr = runner.alg.optimizer.param_groups[0]["lr"] if args.learning_rate is None else args.learning_rate
                runner.alg.learning_rate = restored_lr
                for group in runner.alg.optimizer.param_groups:
                    group["lr"] = restored_lr
                metadata["resume_learning_rate"] = restored_lr
                write_json(output / "run_config.json", metadata)
        if args.teacher_checkpoint:
            teacher_meta = json.loads((args.teacher_checkpoint.parent / "run_config.json").read_text())
            teacher_meta.setdefault("observation_version", "sparse_positions_v1")
            if teacher_meta["stage"] != "teacher":
                raise ValueError("--teacher-checkpoint must be a privileged teacher")
            for key in (*CONTRACT_KEYS, "critic_observations", "privileged_order"):
                if key == "observation_dim":
                    continue
                if teacher_meta.get(key) != metadata.get(key):
                    raise ValueError(f"Teacher/student model contract mismatch: {key}")
            runner.load(str(args.teacher_checkpoint), load_optimizer=False)
        print(f"[G1] READY: stage={args.stage}, obs={tuple(obs.shape)}, actions={env.num_actions}", flush=True)
        report.update(status="running", metadata=str(output / "run_config.json"))
        write_json(output / "result.json", report)

        def export_policy():
            policy = runner.alg.policy
            actor = policy.student if args.stage == "student" else policy.actor
            scripted = torch.jit.script(copy.deepcopy(actor).cpu().eval())
            scripted.save(str(output / "policy.pt"))
            old_tf32 = torch.backends.cuda.matmul.allow_tf32
            torch.backends.cuda.matmul.allow_tf32 = False
            with torch.inference_mode():
                expected = actor(obs[:4]).cpu()
                actual = scripted(obs[:4].cpu())
                error = (actual - expected).abs().max().item()
            torch.backends.cuda.matmul.allow_tf32 = old_tf32
            if error > 1e-4:
                raise RuntimeError(f"Export discrepancy: {error}")
            write_json(output / "policy.json", {**metadata, "torchscript": "policy.pt",
                       "export_max_abs_error": error, "sha256": sha256(output / "policy.pt")})
            return error

        if args.mode == "train":
            initial = {name: p.detach().clone() for name, p in runner.alg.policy.named_parameters()}
            first_iter = runner.current_learning_iteration
            final_iter = first_iter + args.iterations
            while runner.current_learning_iteration < final_iter and not stopping:
                count = min(100, final_iter - runner.current_learning_iteration)
                runner.learn(count, init_at_random_ep_len=runner.current_learning_iteration == 0)
                runner.current_learning_iteration += 1
                runner.save(str(output / "model_latest.pt"), infos={**metadata, "next_iteration": runner.current_learning_iteration})
                elapsed = time.monotonic() - started
                completed = runner.current_learning_iteration - first_iter
                remaining = elapsed * (final_iter - runner.current_learning_iteration) / completed
                write_json(output / "progress.json", {
                    "completed_iterations": runner.current_learning_iteration,
                    "additional_iterations": completed,
                    "training_seconds": elapsed,
                    "estimated_remaining_seconds": remaining,
                    "metrics": {k: float(v.float().mean().item()) for k, v in raw.metrics.items()},
                })
                print(f"[G1] TRAIN additional={completed}/{args.iterations}, elapsed={elapsed:.1f}s, estimated_remaining={remaining:.1f}s", flush=True)
                if args.max_hours and time.monotonic() - started >= args.max_hours * 3600:
                    stop_reason = "time budget reached"
                    break
            delta = max((p - initial[name]).abs().max().item() for name, p in runner.alg.policy.named_parameters())
            actor_prefix = "student." if args.stage == "student" else "actor."
            actor_delta = max((p - initial[name]).abs().max().item()
                              for name, p in runner.alg.policy.named_parameters() if name.startswith(actor_prefix))
            if not actor_delta > 0:
                raise RuntimeError("Training did not update actor parameters")
            runner.save(str(output / "model_final.pt"), infos={**metadata, "next_iteration": runner.current_learning_iteration})
            export_error = export_policy()
            report.update(status="training_finished_evaluation_required", checkpoint=str(output / "model_final.pt"),
                          completed_iterations=runner.current_learning_iteration,
                          additional_iterations=runner.current_learning_iteration - first_iter,
                          actor_max_parameter_change=actor_delta, policy_max_parameter_change=delta,
                          export_max_abs_error=export_error,
                          stop_reason=stop_reason)
        elif args.mode == "export":
            report.update(status="exported", export_max_abs_error=export_policy())
        else:
            policy = runner.get_inference_policy(device=env.device)
            if args.use_exported_policy:
                exported_path = args.checkpoint.parent / "policy.pt"
                exported_meta = json.loads((args.checkpoint.parent / "policy.json").read_text())
                exported_meta.setdefault("observation_version", "sparse_positions_v1")
                if sha256(exported_path) != exported_meta["sha256"]:
                    raise ValueError("Exported policy SHA256 mismatch")
                for key in CONTRACT_KEYS:
                    if exported_meta.get(key) != metadata.get(key):
                        raise ValueError(f"Exported policy contract mismatch: {key}")
                if metadata["observation_version"] == "sparse_tracking_v2" and metadata["target_velocity_estimator"] != exported_meta.get("target_velocity_estimator"):
                    raise ValueError("Exported target velocity filter/timing contract mismatch")
                policy = torch.jit.load(str(exported_path), map_location=env.device).eval()
                reference_actor = runner.alg.policy.student if args.stage == "student" else runner.alg.policy.actor
                expected_state, exported_state = reference_actor.state_dict(), policy.state_dict()
                if expected_state.keys() != exported_state.keys() or any(
                    not torch.equal(value, exported_state[name]) for name, value in expected_state.items()
                ):
                    raise ValueError("Exported policy weights do not match the selected checkpoint")
            xr = None
            if xr_requested:
                from g1_teleop.sim.xr_view import G1XRView
                xr = G1XRView(raw, app, backend="openxr")
            if args.mode == "teleop":
                from g1_teleop.vr.protocol import UDPReceiver
                from g1_teleop.runtime import TeleopGate
                receiver = UDPReceiver(host=args.host, port=args.port, timeout_s=args.input_timeout)
                gate = TeleopGate()
                raw.external_mode = True
                trained_velocity_limits = torch.tensor(metadata["training_velocity_limits"], device=env.device)
                print(f"[G1] Waiting for ALVR targets at {args.host}:{args.port}", flush=True)
                print(f"[G1] Policy joystick limits (vx,vy,yaw): {metadata['training_velocity_limits']}", flush=True)
            durations = []
            falls = timeouts = accepted_frames = enabled_steps = manual_resets = 0
            elapsed_steps = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
            sums, samples = {}, 0
            last_print = time.monotonic()
            last_print_step = 0
            loop_started = last_print
            velocity_stats = None
            if args.velocity_evaluation:
                from g1_teleop.velocity_evaluation import VelocityEvaluationAccumulator
                velocity_stats = VelocityEvaluationAccumulator(step_dt=raw.step_dt)
            active = False
            schedule = time.monotonic()
            trace = {key: [] for key in ("target", "actual", "q", "action", "command", "enabled", "done",
                                        "wall_time", "input_sequence", "input_fresh", "manual_reset",
                                        "root_position_w", "root_quaternion_wxyz", "root_linear_velocity_b", "root_angular_velocity_b",
                                        "foot_contact", "foot_linear_velocity_w")}
            input_sources = set()
            xr_calibration = None
            with torch.inference_mode():
                step = 0
                while app.is_running() and not stopping and (args.steps == 0 or step < args.steps):
                    frame = None
                    if receiver:
                        frame = receiver.poll()
                        now_active = gate.update(frame)
                        if xr and frame and frame.fresh and frame.calibrated and frame.calibration_id > 0:
                            calibration = (frame.session, frame.calibration_id)
                            if calibration != xr_calibration:
                                xr.recenter()
                                xr_calibration = calibration
                        if frame and frame.reset:
                            manual_resets += 1
                            env.reset()
                            elapsed_steps.zero_()
                            if xr:
                                xr.recenter()
                        if now_active:
                            positions = torch.as_tensor(frame.positions, device=env.device, dtype=torch.float32).reshape(1, 3, 3)
                            velocity = torch.as_tensor(frame.velocity, device=env.device, dtype=torch.float32).reshape(1, 3)
                            velocity = velocity.clamp(-trained_velocity_limits, trained_velocity_limits)
                            raw.set_external_targets(positions, velocity, reset_velocity=not active)
                            accepted_frames += 1
                            enabled_steps += 1
                            input_sources.add(frame.source)
                        elif active or step == 0 or (frame and frame.reset):
                            set_teleop_hold_targets(raw, after_reset=step == 0 or bool(frame and frame.reset))
                        if active != now_active:
                            print(f"[G1] {'ARMED' if now_active else 'STOPPED / stale input'}", flush=True)
                        active = now_active
                        obs, _ = env.get_observations()
                    actions = torch.zeros((env.num_envs, env.num_actions), device=env.device) if args.zero_policy else policy(obs)
                    if not torch.isfinite(actions).all():
                        raise RuntimeError("Non-finite policy action")
                    obs, _, dones, extras = env.step(actions)
                    if velocity_stats is not None:
                        velocity_stats.update(raw.metrics)
                    if args.record_trace:
                        trace["target"].append(raw.target_positions[0].cpu().numpy().copy())
                        trace["actual"].append(raw.current_keypoints()[0].cpu().numpy().copy())
                        trace["q"].append(raw.robot.data.joint_pos[0, raw.body_joint_ids].cpu().numpy().copy())
                        trace["action"].append(actions[0].cpu().numpy().copy())
                        trace["command"].append(raw.command_velocity[0].cpu().numpy().copy())
                        trace["enabled"].append(active)
                        trace["done"].append(bool(dones[0].item()))
                        trace["wall_time"].append(time.time())
                        trace["input_sequence"].append(frame.seq if frame else -1)
                        trace["input_fresh"].append(bool(frame and frame.fresh))
                        trace["manual_reset"].append(bool(frame and frame.reset))
                        robot_data = raw.robot.data
                        trace["root_position_w"].append(robot_data.root_pos_w[0].cpu().numpy().copy())
                        trace["root_quaternion_wxyz"].append(robot_data.root_quat_w[0].cpu().numpy().copy())
                        trace["root_linear_velocity_b"].append(robot_data.root_lin_vel_b[0].cpu().numpy().copy())
                        trace["root_angular_velocity_b"].append(robot_data.root_ang_vel_b[0].cpu().numpy().copy())
                        foot_force = raw.contact_sensor.data.net_forces_w[0, raw.foot_contact_ids]
                        trace["foot_contact"].append((torch.linalg.vector_norm(foot_force, dim=-1) > 5.).cpu().numpy().copy())
                        trace["foot_linear_velocity_w"].append(robot_data.body_lin_vel_w[0, raw.foot_body_ids].cpu().numpy().copy())
                    if xr:
                        xr.update()
                    elapsed_steps += 1
                    timeout_mask = extras.get("time_outs", torch.zeros_like(dones)).bool()
                    done_mask = dones.bool()
                    # A fall on the time-limit step remains a fall, counted once.
                    fallen = raw.metrics["fallen"].bool()
                    if receiver and done_mask.any():
                        gate.trip("fall" if fallen.any() else "episode_end")
                        set_teleop_hold_targets(raw, after_reset=True)
                        active = False  # Do not replace neutral with the airborne pose next loop.
                        print("[G1] Simulator reset; release controls and explicitly arm again.", flush=True)
                    falls += int(fallen.sum().item())
                    timeouts += int((done_mask & timeout_mask & ~fallen).sum().item())
                    durations.extend((elapsed_steps[done_mask].float() * raw.step_dt).cpu().tolist())
                    elapsed_steps[done_mask] = 0
                    for key, value in raw.metrics.items():
                        sums[key] = sums.get(key, 0.0) + value.float().sum().item()
                    samples += env.num_envs
                    step += 1
                    if time.monotonic() - last_print >= 10:
                        printed_at = time.monotonic()
                        frequency = (step - last_print_step) / (printed_at - last_print)
                        print(f"[G1] {args.mode.upper()} step={step}, falls={falls}, timeouts={timeouts}, input_steps={accepted_frames}, control_hz={frequency:.1f}", flush=True)
                        last_print, last_print_step = printed_at, step
                    if args.real_time or args.mode == "teleop":
                        schedule += raw.step_dt
                        delay = schedule - time.monotonic()
                        if delay > 0:
                            time.sleep(min(delay, raw.step_dt))
                        elif delay < -0.2:
                            schedule = time.monotonic()
            episodes = falls + timeouts
            wall_loop_seconds = time.monotonic() - loop_started
            if args.capture_frame:
                from PIL import Image
                for _ in range(5):
                    raw.sim.render()
                    rendered = raw.render(recompute=True)
                if rendered is None or rendered.size == 0 or rendered.max() == rendered.min():
                    raise RuntimeError("Simulator returned an empty or uniform rendered frame")
                Image.fromarray(rendered).save(output / "preview.png")
                report["preview"] = str(output / "preview.png")
            if args.record_trace and step:
                import numpy as np
                np.savez_compressed(output / "trace.npz", **{k: np.asarray(v) for k, v in trace.items()}, dt=raw.step_dt)
            report.update(status="evaluated" if args.mode == "evaluate" else "teleop_stopped",
                          steps=step, seed=args.seed, motion_split=args.motion_split,
                          wall_loop_seconds=wall_loop_seconds,
                          mean_control_hz=step / wall_loop_seconds,
                          simulation_to_wall_time_ratio=step * raw.step_dt / wall_loop_seconds,
                          simulated_seconds=step * raw.step_dt * env.num_envs,
                          completed_episodes=episodes, falls=falls, timeouts=timeouts,
                          fall_rate_completed_episodes=falls / episodes if episodes else None,
                          mean_completed_episode_seconds=sum(durations) / len(durations) if durations else None,
                          incomplete_episode_seconds=(elapsed_steps.float() * raw.step_dt).cpu().tolist(),
                          mean_metrics={k: v / samples for k, v in sums.items()} if samples else {},
                          accepted_input_steps=accepted_frames, enabled_steps=enabled_steps,
                          manual_resets=manual_resets,
                          input_sources=sorted(input_sources),
                          policy_backend="torchscript" if args.use_exported_policy else "rsl_rl",
                          checkpoint=str(args.checkpoint), zero_policy=args.zero_policy)
            if velocity_stats is not None:
                report["velocity_evaluation"] = velocity_stats.summary()
            if cfg.record_failure_events:
                write_json(output / "failure_events.json", {
                    "scope": "terminated states captured before automatic reset; all environments",
                    "total_count": raw.failure_events_total_count,
                    "truncated_count": raw.failure_events_truncated_count,
                    "events": raw.failure_events,
                })
                report["failure_events"] = str(output / "failure_events.json")
        report["elapsed_seconds"] = time.monotonic() - started
        write_json(output / "result.json", report)
        print("[G1] RESULT " + json.dumps(report, default=str), flush=True)
    except KeyboardInterrupt:
        report.update(status="interrupted", stop_reason=stop_reason)
        if runner is not None and args.mode == "train":
            runner.save(str(output / "model_interrupted.pt"))
            report["checkpoint"] = str(output / "model_interrupted.pt")
        write_json(output / "result.json", report)
    except Exception as error:
        report.update(status="failed", error=repr(error), traceback=traceback.format_exc())
        write_json(output / "result.json", report)
        traceback.print_exc()
        raise
    finally:
        if xr:
            xr.close()
        if receiver:
            receiver.close()
        if env:
            env.close()
        app.close(wait_for_replicator=False)


if __name__ == "__main__":
    main()
