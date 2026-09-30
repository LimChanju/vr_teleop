"""Held-out command timing and metrics, independent of Isaac Sim and GPU."""
import json
import unittest

import numpy as np

from g1_teleop.velocity_evaluation import VelocityEvaluationAccumulator, VelocityEvaluationSchedule


def metrics(state, actual=None, fallen=None):
    command = state["command"]
    actual = command.copy() if actual is None else np.asarray(actual)
    return {"velocity_evaluation_block_id": state["block_id"],
            "velocity_evaluation_settled": state["settled"],
            "command_vx_mps": command[:, 0], "command_vy_mps": command[:, 1],
            "command_yaw_rate_radps": command[:, 2],
            "actual_vx_mps": actual[:, 0], "actual_vy_mps": actual[:, 1],
            "actual_yaw_rate_radps": actual[:, 2],
            "fallen": np.zeros(len(command), bool) if fallen is None else fallen}


class VelocityScheduleTests(unittest.TestCase):
    def test_every_environment_sees_each_command_in_one_cycle(self):
        schedule = VelocityEvaluationSchedule()
        blocks = np.stack([schedule.state(step * 250, 13)["block_id"] for step in range(9)])
        for column in blocks.T:
            self.assertEqual(sorted(column), list(range(9)))
        np.testing.assert_array_equal(schedule.commands(0, 13), schedule.commands(2250, 13))

    def test_block_and_settling_boundaries_follow_global_steps(self):
        schedule = VelocityEvaluationSchedule()
        self.assertFalse(schedule.state(49, 9)["settled"].any())
        self.assertTrue(schedule.state(50, 9)["settled"].all())
        np.testing.assert_array_equal(schedule.commands(0, 9), schedule.commands(249, 9))
        self.assertFalse(schedule.state(250, 9)["settled"].any())
        self.assertEqual(schedule.state(250, 9)["block_id"].tolist(), [1, 2, 3, 4, 5, 6, 7, 8, 0])
        self.assertEqual(schedule.metadata()["cycle_seconds"], 45.0)

    def test_commands_do_not_depend_on_episode_reset_or_random_state(self):
        schedule = VelocityEvaluationSchedule()
        before = schedule.state(937, 5)
        np.random.seed(483)
        np.random.rand(500)
        after = VelocityEvaluationSchedule().state(937, 5)
        for key in before:
            np.testing.assert_array_equal(before[key], after[key])

    def test_invalid_timing_is_rejected(self):
        for kwargs in ({"step_dt": 0}, {"block_seconds": float("nan")},
                       {"settling_seconds": 5}, {"settling_seconds": -1}):
            with self.assertRaises(ValueError):
                VelocityEvaluationSchedule(**kwargs)

    def test_torch_cpu_schedule_matches_numpy(self):
        try:
            import torch
        except ImportError:
            self.skipTest("PyTorch not installed")
        schedule = VelocityEvaluationSchedule()
        for step in [0, 49, 50, 249, 250, 2249, 2250]:
            a, b = schedule.state(step, 18), schedule.state(step, 18, "cpu")
            for key in a:
                np.testing.assert_allclose(a[key], b[key].numpy(), atol=1e-6)


class VelocityAccumulatorTests(unittest.TestCase):
    def test_perfect_tracking_scores_one_gain_zero_error(self):
        schedule = VelocityEvaluationSchedule()
        result = VelocityEvaluationAccumulator()
        result.update(metrics(schedule.state(50, 9)))
        summary = result.summary()
        np.testing.assert_allclose(summary["moving"]["axis_gain"], [1., 1., 1.], atol=1e-7)
        self.assertEqual(summary["moving"]["axis_sign_agreement"], [1., 1., 1.])
        np.testing.assert_allclose(summary["moving"]["axis_rmse"], [0., 0., 0.], atol=1e-7)
        self.assertEqual(summary["standing"]["actual_xy_speed_mean_mps"], 0.)

    def test_stationary_policy_has_zero_gain_and_nonzero_error(self):
        schedule = VelocityEvaluationSchedule()
        result = VelocityEvaluationAccumulator()
        state = schedule.state(50, 9)
        result.update(metrics(state, actual=np.zeros((9, 3))))
        summary = result.summary()
        self.assertEqual(summary["moving"]["axis_gain"], [0., 0., 0.])
        self.assertEqual(summary["moving"]["axis_sign_agreement"], [0., 0., 0.])
        self.assertAlmostEqual(summary["blocks"][1]["axis_mae"][0], .2)
        self.assertAlmostEqual(summary["blocks"][5]["axis_mae"][2], .3)

    def test_opposite_directions_do_not_cancel_errors_or_gain(self):
        result = VelocityEvaluationAccumulator()
        state = VelocityEvaluationSchedule().state(50, 9)
        result.update(metrics(state, actual=-state["command"]))
        moving = result.summary()["moving"]
        np.testing.assert_allclose(moving["axis_gain"], [-1., -1., -1.], atol=1e-7)
        self.assertEqual(moving["axis_sign_agreement"], [0., 0., 0.])
        self.assertTrue(all(value > 0 for value in moving["axis_rmse"]))

    def test_falls_during_settling_count_but_velocity_is_not_scored(self):
        result = VelocityEvaluationAccumulator()
        state = VelocityEvaluationSchedule().state(20, 9)
        fallen = np.zeros(9, bool)
        fallen[1] = True
        result.update(metrics(state, actual=np.full((9, 3), 99), fallen=fallen))
        summary = result.summary()
        self.assertEqual(summary["falls_all_phases"], 1)
        self.assertEqual(summary["blocks"][1]["falls"], 1)
        self.assertEqual(summary["moving"]["valid_samples"], 0)
        self.assertIsNone(summary["moving"]["axis_rmse"])
        self.assertAlmostEqual(summary["total_exposure_env_seconds"], .18)

    def test_nonfinite_samples_are_reported_and_summary_is_json_safe(self):
        result = VelocityEvaluationAccumulator()
        state = VelocityEvaluationSchedule().state(50, 9)
        actual = state["command"].copy()
        actual[1, 0] = np.nan
        actual[2, 0] = np.inf
        result.update(metrics(state, actual=actual))
        summary = result.summary()
        self.assertEqual(summary["nonfinite_scored_samples"], 2)
        self.assertEqual(summary["blocks"][1]["scored_samples"], 1)
        self.assertEqual(summary["blocks"][1]["valid_samples"], 0)
        json.dumps(summary, allow_nan=False)

    def test_mislabeled_block_is_rejected_at_transition(self):
        sample = metrics(VelocityEvaluationSchedule().state(249, 9))
        sample["velocity_evaluation_block_id"] = VelocityEvaluationSchedule().state(250, 9)["block_id"]
        with self.assertRaisesRegex(ValueError, "phase snapshot"):
            VelocityEvaluationAccumulator().update(sample)


if __name__ == "__main__":
    unittest.main()
