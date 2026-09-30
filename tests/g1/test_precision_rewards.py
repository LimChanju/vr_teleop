"""Execute the actual reward method on CPU, without importing Isaac Sim."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

try:
    import torch
except ImportError:
    torch = None


SOURCE = Path(__file__).resolve().parents[2] / "g1_teleop/sim/env.py"
REWARD_FIELDS = ("tracking_reward_weight", "tracking_error_variance", "height_reward_weight",
                 "height_error_variance", "fall_cost")


def reward_method(path=SOURCE):
    tree = ast.parse(path.read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "G1WholeBodyEnv")
    method = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == "_get_rewards")
    namespace = {"torch": torch}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), "exec"), namespace)
    return namespace["_get_rewards"]


def default_cfg():
    tree = ast.parse(SOURCE.read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "G1WholeBodyEnvCfg")
    fields = {node.targets[0].id: ast.literal_eval(node.value)
              for node in cls.body if isinstance(node, ast.Assign)
              and isinstance(node.targets[0], ast.Name) and node.targets[0].id in REWARD_FIELDS}
    return SimpleNamespace(**fields)


def fixture(*, precision=False, point_error=0.0, height_error=0.0, fallen=False, dt=.02):
    cfg = default_cfg()
    if precision:
        cfg.tracking_reward_weight = 6.0
        cfg.tracking_error_variance = .01
        cfg.height_reward_weight = 2.0
        cfg.height_error_variance = .0025
        cfg.fall_cost = 5.0
    # float64 makes analytic identities precise enough to expose dt mistakes.
    zeros = lambda *shape: torch.zeros(*shape, dtype=torch.float64)
    root_position = zeros(1, 3)
    root_position[:, 2] = .75 + height_error
    marker_position = zeros(1, 3, 3)
    marker_position[:, :, 0] = point_error
    physics = SimpleNamespace(root_lin_vel_b=zeros(1, 3), root_ang_vel_b=zeros(1, 3),
        root_pos_w=root_position, projected_gravity_b=torch.tensor([[0., 0., -1.]], dtype=torch.float64),
        joint_pos=zeros(1, 29), joint_vel=zeros(1, 29), applied_torque=zeros(1, 29),
        body_lin_vel_w=zeros(1, 2, 3))
    env = SimpleNamespace(cfg=cfg, robot=SimpleNamespace(data=physics),
        scene=SimpleNamespace(env_origins=zeros(1, 3)), target_positions=zeros(1, 3, 3),
        command_velocity=zeros(1, 3), reference_height=torch.tensor([.75], dtype=torch.float64),
        reference_q=zeros(1, 29), body_joint_ids=list(range(29)), foot_body_ids=[0, 1],
        foot_contact_ids=[0, 1], contact_sensor=SimpleNamespace(data=SimpleNamespace(net_forces_w=zeros(1, 2, 3))),
        actions=zeros(1, 29), previous_actions=zeros(1, 29), step_dt=dt,
        reset_terminated=torch.tensor([fallen]), episode_length_buf=torch.tensor([13]), _episode_sums={})
    env._tracked_positions = lambda: marker_position
    return env


@unittest.skipIf(torch is None, "PyTorch required; no simulator or GPU needed")
class PrecisionRewardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.calculate = staticmethod(reward_method())

    def test_defaults_preserve_original_configuration(self):
        self.assertEqual(vars(default_cfg()), dict(tracking_reward_weight=4., tracking_error_variance=.04,
            height_reward_weight=1., height_error_variance=.01, fall_cost=2.))

    def test_ideal_standing_has_known_default_and_precision_rewards(self):
        # alive(1)+tracking(4/6)+linear(2)+yaw(1)+height(1/2)+upright(1)+qref(1)
        for precision, per_second in [(False, 11.), (True, 14.)]:
            env = fixture(precision=precision)
            result = self.calculate(env)
            self.assertAlmostEqual(float(result[0]), per_second * .02, places=12)
            self.assertAlmostEqual(float(sum(env._episode_sums.values())[0]), float(result[0]), places=12)

    def test_known_point_and_height_errors_use_requested_widths(self):
        # 10cm point error / 5cm height error => exponent -.25 default, -1 precision.
        for precision, expected in [(False, 6. + 5. * math.exp(-.25)),
                                    (True, 6. + 8. * math.exp(-1.))]:
            env = fixture(precision=precision, point_error=.1, height_error=.05)
            result = self.calculate(env)
            self.assertAlmostEqual(float(result[0]), expected * .02, places=12)
            self.assertAlmostEqual(float(env.metrics["tracking_error_m"][0]), .1, places=12)

    def test_fall_cost_is_per_event_not_scaled_by_dt(self):
        for precision, fall_cost in [(False, 2.), (True, 5.)]:
            for dt in [.02, .04]:
                intact = self.calculate(fixture(precision=precision, dt=dt))
                fallen = self.calculate(fixture(precision=precision, dt=dt, fallen=True))
                self.assertAlmostEqual(float(intact[0] - fallen[0]), fall_cost, places=12)

    def test_reward_variant_does_not_change_reported_tracking_metrics(self):
        normal = fixture(point_error=.09, height_error=.04, fallen=True)
        precision = fixture(precision=True, point_error=.09, height_error=.04, fallen=True)
        self.calculate(normal)
        self.calculate(precision)
        for name in normal.metrics:
            torch.testing.assert_close(normal.metrics[name], precision.metrics[name], atol=0, rtol=0)

    def test_episode_tracking_and_height_sums_follow_configured_terms(self):
        env = fixture(precision=True, point_error=.1, height_error=.05)
        for _ in range(3):
            self.calculate(env)
        self.assertAlmostEqual(float(env._episode_sums["sparse_tracking"][0]), 3 * .02 * 6 * math.exp(-1), places=12)
        self.assertAlmostEqual(float(env._episode_sums["height"][0]), 3 * .02 * 2 * math.exp(-1), places=12)


if __name__ == "__main__":
    unittest.main()
