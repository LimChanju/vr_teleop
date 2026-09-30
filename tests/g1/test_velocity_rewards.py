"""CPU tests of the actual velocity subclass, using a simulator-free base double."""
import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

try:
    import torch
except ImportError:
    torch = None


def load_velocity_module():
    class Base:
        def _get_rewards(self):
            linear_error = (self.robot.data.root_lin_vel_b[:, :2] - self.command_velocity[:, :2]).square().sum(-1)
            yaw_error = (self.robot.data.root_ang_vel_b[:, 2] - self.command_velocity[:, 2]).square()
            terms = {"linear_velocity": 2.0 * torch.exp(-linear_error / 0.25),
                     "yaw_velocity": torch.exp(-yaw_error / 0.25)}
            self.metrics = {}
            for name, value in terms.items():
                self._episode_sums.setdefault(name, torch.zeros_like(value)).add_(value * self.step_dt)
            return sum(terms.values()) * self.step_dt

        def policy_metadata(self):
            return {}

        def _update_reference(self):
            pass

        def _sample_reference(self, ids):
            self.command_velocity[ids] = 0.0

    base_module = ModuleType("g1_teleop.sim.env")
    base_module.G1WholeBodyEnv = Base
    base_module.G1WholeBodyEnvCfg = object
    utils = ModuleType("isaaclab.utils")
    utils.configclass = lambda cls: cls
    source = Path(__file__).resolve().parents[2] / "g1_teleop/sim/velocity_env.py"
    spec = importlib.util.spec_from_file_location("g1_teleop.sim._velocity_reward_test", source)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {"g1_teleop.sim.env": base_module,
                                  "isaaclab": ModuleType("isaaclab"), "isaaclab.utils": utils}):
        spec.loader.exec_module(module)
    return module


@unittest.skipIf(torch is None, "PyTorch is required; no simulator or GPU is needed")
class VelocityRewardsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_velocity_module()

    def make_env(self, commands, actual=None, yaw=None):
        env = self.module.G1VelocityEnv.__new__(self.module.G1VelocityEnv)
        env.cfg = self.module.G1VelocityEnvCfg()
        env.command_velocity = torch.tensor(commands, dtype=torch.float32)
        count = len(commands)
        env.step_dt = 0.02
        env.num_envs = count
        env.device = "cpu"
        env.common_step_counter = 0
        env.external_mode = False
        env._episode_sums = {}
        env.foot_contact_ids = [0, 1]
        linear = torch.zeros(count, 3)
        angular = torch.zeros(count, 3)
        if actual is not None:
            linear[:, :2] = torch.tensor(actual)
        if yaw is not None:
            angular[:, 2] = torch.tensor(yaw)
        env.robot = SimpleNamespace(data=SimpleNamespace(root_lin_vel_b=linear, root_ang_vel_b=angular))
        force = torch.zeros(count, 2, 3)
        force[:, :, 2] = 100.0
        env.contact_sensor = SimpleNamespace(
            compute_first_contact=lambda dt: torch.zeros(count, 2, dtype=torch.bool),
            data=SimpleNamespace(last_air_time=torch.zeros(count, 2), net_forces_w=force))
        return env

    def test_zero_command_keeps_original_reward_including_drift(self):
        env = self.make_env([[0, 0, 0], [0, 0, 0]], actual=[[0, 0], [.2, -.1]], yaw=[0, .3])
        reward = env._get_rewards()
        expected = torch.tensor([3.0, 2.0 * math.exp(-.05 / .25) + math.exp(-.09 / .25)]) * .02
        torch.testing.assert_close(reward, expected)
        self.assertFalse(env.metrics["commanded_moving"].any())

    def test_stationary_robot_with_forward_command_gets_sharper_reward(self):
        env = self.make_env([[.2, 0, 0]])
        reward = env._get_rewards()
        self.assertAlmostEqual(float(reward[0]), (4.0 * math.exp(-1.0) + 2.0) * .02, places=7)
        self.assertAlmostEqual(float(env._episode_sums["linear_velocity"][0]), 4.0 * math.exp(-1.0) * .02, places=7)
        self.assertAlmostEqual(float(env.metrics["linear_velocity_error_mps"][0]), .2, places=6)

    def test_exact_moving_tracking_replaces_terms_without_double_counting(self):
        env = self.make_env([[.2, -.1, .3]], actual=[[.2, -.1]], yaw=[.3])
        reward = env._get_rewards()
        self.assertAlmostEqual(float(reward[0]), 6.0 * .02, places=7)
        self.assertAlmostEqual(float(env._episode_sums["linear_velocity"][0]), 4.0 * .02, places=7)
        self.assertAlmostEqual(float(env._episode_sums["yaw_velocity"][0]), 2.0 * .02, places=7)

    def test_yaw_only_and_small_command_mask_is_per_environment(self):
        env = self.make_env([[0, 0, .2], [.04, .03, .05], [.15, 0, 0]])
        reward = env._get_rewards()
        expected = torch.tensor([
            4.0 + 2.0 * math.exp(-.04 / .1),
            2.0 * math.exp(-.0025 / .25) + math.exp(-.0025 / .25),
            4.0 * math.exp(-.0225 / .04) + 2.0,
        ]) * .02
        torch.testing.assert_close(reward, expected)
        self.assertEqual(env.metrics["commanded_moving"].tolist(), [True, False, True])

    def test_episode_sums_equal_integrated_reward_over_repeated_steps(self):
        env = self.make_env([[.2, -.1, .3], [0, 0, 0]])
        total = torch.zeros(2)
        for _ in range(7):
            total += env._get_rewards()
        torch.testing.assert_close(sum(env._episode_sums.values()), total)

    def test_contact_bonuses_remain_gated_by_moving_command(self):
        env = self.make_env([[.2, 0, 0], [0, 0, 0]])
        base_reward = env._get_rewards()
        env.contact_sensor.compute_first_contact = lambda dt: torch.ones(2, 2, dtype=torch.bool)
        env.contact_sensor.data.last_air_time[:] = .5
        with_airtime = env._get_rewards()
        torch.testing.assert_close(with_airtime - base_reward, torch.tensor([.3 * .02, 0]))

    def test_configured_xy_threshold_changes_small_lateral_gate_but_not_yaw_gate(self):
        commands = [[0, .06, 0], [0, .03, 0], [0, 0, .09], [0, 0, .11]]
        legacy = self.make_env(commands)
        legacy._get_rewards()
        self.assertEqual(legacy.metrics["commanded_moving"].tolist(), [False, False, False, True])
        small_xy = self.make_env(commands)
        small_xy.cfg.moving_xy_threshold = .04
        small_xy._get_rewards()
        self.assertEqual(small_xy.metrics["commanded_moving"].tolist(), [True, False, False, True])
        expected = 4.0 * math.exp(-.06 ** 2 / .04) * .02
        self.assertAlmostEqual(float(small_xy._episode_sums["linear_velocity"][0]), expected, places=7)

    def test_metadata_records_exact_replacement_contract(self):
        env = self.make_env([[0, 0, 0]])
        contract = env.policy_metadata()["velocity_reward_contract"]
        self.assertEqual(contract["version"], "moving_velocity_replacement_v2")
        self.assertEqual(contract["linear_velocity"], {"scale": 4.0, "squared_error_denominator_m2_s2": .04})
        self.assertEqual(contract["yaw_velocity"], {"scale": 2.0, "squared_error_denominator_rad2_s2": .1})

    def test_default_standing_configuration_preserves_legacy_bits_and_metadata(self):
        env = self.make_env([[0, 0, 0], [.04, .02, .05], [.2, 0, .3]],
                            actual=[[.01, .02], [-.03, .04], [.1, .03]], yaw=[.25, -.1, .2])
        cmd = env.command_velocity
        linear_error = (env.robot.data.root_lin_vel_b[:, :2] - cmd[:, :2]).square().sum(-1)
        yaw_error = (env.robot.data.root_ang_vel_b[:, 2] - cmd[:, 2]).square()
        base_linear = 2. * torch.exp(-linear_error / .25)
        base_yaw = torch.exp(-yaw_error / .25)
        moving = (torch.linalg.vector_norm(cmd[:, :2], dim=-1) > .08) | (cmd[:, 2].abs() > .1)
        expected = (base_linear + base_yaw) * .02
        expected += torch.where(moving, 4. * torch.exp(-linear_error / .04) - base_linear, 0.) * .02
        expected += torch.where(moving, 2. * torch.exp(-yaw_error / .1) - base_yaw, 0.) * .02
        self.assertTrue(torch.equal(env._get_rewards(), expected))
        contract = env.policy_metadata()['velocity_reward_contract']
        self.assertNotIn('standing_yaw_velocity', contract)
        self.assertEqual(contract['standing_terms'], 'unchanged base linear 2*exp(-e2/.25), yaw exp(-e2/.25)')
        self.assertEqual(contract['application'], 'replace base terms for moving commands; integrate every term with control dt')

    def test_standing_yaw_replacement_only_changes_complement_of_moving_mask(self):
        commands = [[0, 0, 0], [.02, 0, .05], [.03, 0, .1], [.031, 0, 0], [0, 0, .11]]
        base = self.make_env(commands, yaw=[.2, -.1, .2, .2, .2])
        tuned = self.make_env(commands, yaw=[.2, -.1, .2, .2, .2])
        for env in (base, tuned):
            env.cfg.moving_xy_threshold = .03
            env.cfg.moving_yaw_velocity_reward_scale = 3.
            env.contact_sensor.compute_first_contact = lambda dt: torch.ones(5, 2, dtype=torch.bool)
            env.contact_sensor.data.last_air_time[:] = .5
        tuned.cfg.standing_yaw_velocity_reward_scale = 3.
        tuned.cfg.standing_yaw_velocity_error_variance = .05
        old_reward, new_reward = base._get_rewards(), tuned._get_rewards()
        moving = base.metrics['commanded_moving']
        self.assertEqual(moving.tolist(), [False, False, False, True, True])
        error = (tuned.robot.data.root_ang_vel_b[:, 2] - tuned.command_velocity[:, 2]).square()
        expected_delta = torch.where(~moving, 3. * torch.exp(-error / .05) - torch.exp(-error / .25), 0.) * .02
        torch.testing.assert_close(new_reward - old_reward, expected_delta)
        torch.testing.assert_close(tuned._episode_sums['yaw_velocity'] - base._episode_sums['yaw_velocity'], expected_delta)
        self.assertTrue(torch.equal(new_reward[moving], old_reward[moving]))
        for name in ('linear_velocity', 'velocity_feet_airtime', 'velocity_both_feet_air'):
            self.assertTrue(torch.equal(tuned._episode_sums[name], base._episode_sums[name]))
        total = new_reward.clone()
        for _ in range(6):
            total += tuned._get_rewards()
        torch.testing.assert_close(sum(tuned._episode_sums.values()), total)

    def test_standing_yaw_metadata_roundtrip_and_invalid_direct_config(self):
        from scripts.g1.run import restore_velocity_settings
        env = self.make_env([[0, 0, 0]])
        env.cfg.standing_yaw_velocity_reward_scale = 3.
        env.cfg.standing_yaw_velocity_error_variance = .05
        saved = env.policy_metadata()
        self.assertEqual(saved['velocity_reward_contract']['standing_yaw_velocity'],
                         {'scale': 3., 'squared_error_denominator_rad2_s2': .05})
        restored = self.module.G1VelocityEnvCfg()
        restore_velocity_settings(restored, saved)
        self.assertEqual(restored.standing_yaw_velocity_reward_scale, 3.)
        self.assertEqual(restored.standing_yaw_velocity_error_variance, .05)
        for name in ('standing_yaw_velocity_reward_scale', 'standing_yaw_velocity_error_variance'):
            for value in (True, 0., -1., float('nan'), float('inf')):
                cfg = self.module.G1VelocityEnvCfg()
                setattr(cfg, name, value)
                with self.assertRaises(ValueError):
                    self.module.G1VelocityEnv(cfg)

    def test_evaluation_commands_survive_reset_and_use_global_phase(self):
        env = self.make_env([[0, 0, 0]] * 9)
        env.cfg.velocity_evaluation = True
        env.common_step_counter = 300
        env._update_reference()
        commands = env.command_velocity.clone()
        env._sample_reference(torch.tensor([0, 3]))
        torch.testing.assert_close(env.command_velocity, commands)
        self.assertEqual(env._velocity_evaluation_block.tolist(), [1, 2, 3, 4, 5, 6, 7, 8, 0])
        self.assertTrue(env._velocity_evaluation_settled.all())

    def test_reward_tags_keep_pre_step_command_at_block_boundary(self):
        env = self.make_env([[0, 0, 0]] * 9)
        env.cfg.velocity_evaluation = True
        env.common_step_counter = 249
        env._update_reference()
        env.common_step_counter = 250
        env._get_rewards()
        self.assertEqual(env.metrics["velocity_evaluation_block_id"].tolist(), list(range(9)))
        self.assertTrue(env.metrics["velocity_evaluation_settled"].all())
        env._update_reference()
        self.assertFalse(env._velocity_evaluation_settled.any())

    def test_external_mode_disables_evaluation_command_override(self):
        env = self.make_env([[.12, -.06, .2]] * 9)
        env.cfg.velocity_evaluation = True
        env.external_mode = True
        commands = env.command_velocity.clone()
        env._update_reference()
        env._sample_velocity(torch.arange(9))
        torch.testing.assert_close(env.command_velocity, commands)


if __name__ == "__main__":
    unittest.main()
