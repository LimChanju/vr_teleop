"""CPU verification of actual pre-reset failure snapshots and bounded storage."""
import ast
import json
import math
from pathlib import Path
from types import MethodType, SimpleNamespace
import unittest

try:
    import torch
except ImportError:
    torch = None


SOURCE = Path(__file__).resolve().parents[2] / "g1_teleop/sim/env.py"
JOINT_NAMES = [f"joint_{idx}" for idx in range(29)]


def load_actual_methods():
    tree = ast.parse(SOURCE.read_text())
    snapshot = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "_failure_snapshot")
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "G1WholeBodyEnv")
    methods = [node for node in cls.body if isinstance(node, ast.FunctionDef)
               and node.name in ("_get_rewards", "_record_failure_events")]
    scope = {"torch": torch, "math": math, "BODY_JOINT_NAMES": JOINT_NAMES}
    exec(compile(ast.Module(body=[snapshot, *methods], type_ignores=[]), str(SOURCE), "exec"), scope)
    return scope


def make_env(methods):
    z = lambda *shape: torch.zeros(*shape)
    data = SimpleNamespace(root_state_w=z(3, 13), joint_pos=z(3, 29),
        root_pos_w=torch.tensor([[0., 0., .8], [0., 0., 10.3], [0., 0., .7]]),
        projected_gravity_b=torch.tensor([[0., 0., -1.], [0., 0., -.5], [0., 0., -1.]]),
        root_lin_vel_b=z(3, 3), root_ang_vel_b=z(3, 3), joint_vel=z(3, 29),
        applied_torque=z(3, 29), body_lin_vel_w=z(3, 2, 3))
    origins = z(3, 3)
    origins[1, 2] = 10.
    cfg = SimpleNamespace(action_scale=.5, record_failure_events=True, tracking_reward_weight=4.,
        tracking_error_variance=.04, height_reward_weight=1., height_error_variance=.01, fall_cost=2.)
    env = SimpleNamespace(cfg=cfg, robot=SimpleNamespace(data=data), scene=SimpleNamespace(env_origins=origins),
        reset_terminated=torch.tensor([False, True, True]), common_step_counter=731,
        episode_length_buf=torch.tensor([100, 80, 50]), step_dt=.02,
        nominal_q=z(29), actions=z(3, 29), previous_actions=z(3, 29),
        joint_limits=torch.tensor([[-.5, .5]] * 29),
        _motion={"clip_indices": [4, 8], "clip_names": ["clip-four", "clip-eight"]},
        _clip_selection=torch.tensor([0, 1, 0]), _failure_reference_phase=torch.tensor([2., 7.5, 15.]),
        external_mode=False, failure_events=[], failure_events_total_count=0, failure_events_truncated_count=0,
        target_positions=z(3, 3, 3), command_velocity=z(3, 3), reference_height=torch.full((3,), .8),
        reference_q=z(3, 29), body_joint_ids=list(range(29)), foot_body_ids=[0, 1], foot_contact_ids=[0, 1],
        contact_sensor=SimpleNamespace(data=SimpleNamespace(net_forces_w=z(3, 2, 3))), _episode_sums={})
    env._tracked_positions = lambda: z(3, 3, 3)
    env.actions[1, :2] = torch.tensor([2., -2.])
    env._record_failure_events = MethodType(methods["_record_failure_events"], env)
    return env


@unittest.skipIf(torch is None, "PyTorch required; no simulator or GPU needed")
class FailureDiagnosticsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.methods = load_actual_methods()

    def test_only_terminated_envs_capture_exact_phase_and_local_height(self):
        env = make_env(self.methods)
        env._record_failure_events()
        self.assertEqual([row["env_id"] for row in env.failure_events], [1, 2])
        event = env.failure_events[0]
        self.assertEqual(event["global_step"], 731)
        self.assertAlmostEqual(event["episode_seconds"], 1.6, places=6)
        self.assertAlmostEqual(event["root_height_m"], .3, places=5)
        self.assertAlmostEqual(event["tilt_deg"], 60., places=7)
        self.assertEqual((event["clip_index"], event["clip_name"], event["reflected_phase_frame"]), (8, "clip-eight", 7.5))
        self.assertEqual([row["joint"] for row in event["soft_limit_clamped_joints"]], ["joint_0", "joint_1"])
        self.assertEqual([row["requested_target_rad"] for row in event["soft_limit_clamped_joints"]], [1., -1.])
        self.assertEqual([row["applied_target_rad"] for row in event["soft_limit_clamped_joints"]], [.5, -.5])

    def test_snapshot_survives_subsequent_reset_mutation(self):
        env = make_env(self.methods)
        env._record_failure_events()
        before = json.dumps(env.failure_events, allow_nan=False, sort_keys=True)
        env.robot.data.root_pos_w.zero_()
        env.actions.zero_()
        env._clip_selection.zero_()
        env._failure_reference_phase.zero_()
        self.assertEqual(before, json.dumps(env.failure_events, allow_nan=False, sort_keys=True))

    def test_cap_counts_all_failures_without_unbounded_records(self):
        env = make_env(self.methods)
        env.failure_events = [None] * 9999
        env._record_failure_events()
        self.assertEqual(len(env.failure_events), 10000)
        self.assertEqual(env.failure_events[-1]["env_id"], 1)
        self.assertEqual(env.failure_events_total_count, 2)
        self.assertEqual(env.failure_events_truncated_count, 1)
        # With no storage left, state transfer must not be attempted.
        env.robot = None
        env._record_failure_events()
        self.assertEqual(len(env.failure_events), 10000)
        self.assertEqual(env.failure_events_total_count, 4)
        self.assertEqual(env.failure_events_truncated_count, 3)

    def test_nonfinite_failure_is_explicit_and_json_safe(self):
        env = make_env(self.methods)
        env.robot.data.root_state_w[1, 0] = float("nan")
        env.robot.data.root_pos_w[1, 2] = float("nan")
        env.robot.data.projected_gravity_b[1, 2] = float("nan")
        env.actions[1, 0] = float("nan")
        env._record_failure_events()
        event = env.failure_events[0]
        self.assertFalse(event["finite_state"])
        self.assertIsNone(event["root_height_m"])
        self.assertIsNone(event["tilt_deg"])
        self.assertEqual(event["nonfinite_target_joints"], ["joint_0"])
        json.dumps(env.failure_events, allow_nan=False)

    def test_external_mode_has_no_stale_motion_attribution(self):
        env = make_env(self.methods)
        env.external_mode = True
        env._record_failure_events()
        for event in env.failure_events:
            self.assertIsNone(event["clip_index"])
            self.assertIsNone(event["clip_name"])
            self.assertIsNone(event["reflected_phase_frame"])

    def test_disabled_diagnostics_never_enter_recording_method(self):
        env = make_env(self.methods)
        env.cfg.record_failure_events = False
        def forbidden():
            raise AssertionError("Disabled diagnostics entered the tensor/CPU extraction path")
        env._record_failure_events = forbidden
        reward = self.methods["_get_rewards"](env)
        self.assertTrue(torch.isfinite(reward).all())
        self.assertEqual(env.failure_events, [])
        self.assertEqual(env.failure_events_total_count, 0)

    def test_enabling_diagnostics_does_not_change_rewards_or_metrics(self):
        disabled, enabled = make_env(self.methods), make_env(self.methods)
        disabled.cfg.record_failure_events = False
        a = self.methods["_get_rewards"](disabled)
        b = self.methods["_get_rewards"](enabled)
        torch.testing.assert_close(a, b, atol=0, rtol=0)
        for key in disabled.metrics:
            torch.testing.assert_close(disabled.metrics[key], enabled.metrics[key], atol=0, rtol=0)
        for key in disabled._episode_sums:
            torch.testing.assert_close(disabled._episode_sums[key], enabled._episode_sums[key], atol=0, rtol=0)
        self.assertEqual(len(enabled.failure_events), 2)

    def test_no_failure_returns_before_state_access(self):
        env = make_env(self.methods)
        env.reset_terminated.zero_()
        env.robot = None
        env._record_failure_events()
        self.assertEqual(env.failure_events_total_count, 0)


if __name__ == "__main__":
    unittest.main()
