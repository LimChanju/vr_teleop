"""Independent CPU tests for policy migration and failure without mutation."""

import copy
from pathlib import Path
import sys
import tempfile
import unittest

import torch
from torch import nn

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from g1_teleop.warm_start import warm_start_policy


class SmallActorCritic(nn.Module):
    def __init__(self, actor_obs=105, critic_obs=138, hidden=16):
        super().__init__()
        self.actor = nn.Sequential(nn.Linear(actor_obs, hidden), nn.ELU(), nn.Linear(hidden, 29))
        self.critic = nn.Sequential(nn.Linear(critic_obs, hidden), nn.ELU(), nn.Linear(hidden, 1))
        self.std = nn.Parameter(torch.full((29,), 0.45))


class WarmStartTests(unittest.TestCase):
    def setUp(self):
        torch.manual_seed(731)
        self.temporary = tempfile.TemporaryDirectory()
        self.path = Path(self.temporary.name) / "model.pt"
        self.old = SmallActorCritic()
        self.new = SmallActorCritic(126, 156)
        self.save(self.old)

    def tearDown(self):
        self.temporary.cleanup()

    def save(self, policy, transform=None):
        checkpoint = {"model_state_dict": copy.deepcopy(policy.state_dict()), "iter": 42,
                      "optimizer_state_dict": {"test": "must not be loaded by policy migration"}}
        if transform:
            transform(checkpoint)
        torch.save(checkpoint, self.path)

    def assert_unchanged(self, before):
        for name, value in self.new.state_dict().items():
            self.assertTrue(torch.equal(value, before[name]), f"Rejected migration changed {name}")

    def test_expanded_semantic_observations_preserve_action_and_value(self):
        result = warm_start_policy(self.new, self.path)
        # Independent construction of old/new semantic fields, including nonzero
        # new features. This does not call the helper's internal mapping check.
        generator = torch.Generator().manual_seed(1685)
        base = torch.randn(127, 105, generator=generator)
        velocity = torch.randn(127, 3, generator=generator)
        error = torch.randn(127, 9, generator=generator) * 4
        target_velocity = torch.randn(127, 9, generator=generator) * 5
        height = torch.randn(127, 1, generator=generator)
        reference_q = torch.randn(127, 29, generator=generator)
        old_critic = torch.cat((base, velocity, height, reference_q), dim=1)
        new_actor = torch.cat((base, velocity, error, target_velocity), dim=1)
        new_critic = torch.cat((new_actor, height, reference_q), dim=1)
        with torch.inference_mode():
            torch.testing.assert_close(self.new.actor(new_actor), self.old.actor(base), atol=1e-6, rtol=1e-5)
            torch.testing.assert_close(self.new.critic(new_critic), self.old.critic(old_critic), atol=1e-6, rtol=1e-5)
        self.assertEqual(result["source_iteration"], 42)
        self.assertFalse(result["optimizer_transferred"])
        self.assertLess(result["actor_preservation_error"], 2e-5)
        self.assertLess(result["critic_preservation_error"], 2e-5)
        self.assertTrue(self.new.training)

    def test_nonmigrating_parameters_are_copied_exactly_and_new_weights_are_zero(self):
        warm_start_policy(self.new, self.path)
        original = self.old.state_dict()
        current = self.new.state_dict()
        for name in original:
            if name not in ("actor.0.weight", "critic.0.weight"):
                self.assertTrue(torch.equal(current[name], original[name]), name)
        self.assertEqual(torch.count_nonzero(current["actor.0.weight"][:, 105:]).item(), 0)
        self.assertEqual(torch.count_nonzero(current["critic.0.weight"][:, 108:126]).item(), 0)
        self.assertTrue(torch.equal(current["critic.0.weight"][:, 126:], original["critic.0.weight"][:, 108:]))

    def test_same_dimensions_copy_and_optional_noise_override(self):
        for dimensions in ((105, 138), (126, 156)):
            with self.subTest(dimensions=dimensions):
                source = SmallActorCritic(*dimensions)
                self.save(source)
                destination = SmallActorCritic(*dimensions)
                result = warm_start_policy(destination, self.path, noise_std=0.2)
                for name, value in source.state_dict().items():
                    if name != "std":
                        self.assertTrue(torch.equal(destination.state_dict()[name], value))
                torch.testing.assert_close(destination.std, torch.full((29,), 0.2))
                self.assertEqual(result["noise_std_override"], 0.2)

    def test_invalid_noise_rejected_without_loading_any_parameter(self):
        before = copy.deepcopy(self.new.state_dict())
        for noise in (0, -0.1, 1.1, float("nan"), float("inf"), True, "0.5"):
            with self.subTest(noise=noise), self.assertRaises(ValueError):
                warm_start_policy(self.new, self.path, noise_std=noise)
            self.assert_unchanged(before)

    def test_invalid_layer_shapes_names_and_iteration_are_transactional(self):
        mutations = [
            lambda item: item["model_state_dict"].update({"actor.2.weight": torch.zeros(29, 17)}),
            lambda item: item["model_state_dict"].update({"actor.0.weight": torch.zeros(17, 105)}),
            lambda item: item["model_state_dict"].update({"critic.0.weight": torch.zeros(16)}),
            lambda item: item["model_state_dict"].pop("critic.2.bias"),
            lambda item: item.update(iter="42"),
        ]
        before = copy.deepcopy(self.new.state_dict())
        for mutation in mutations:
            self.save(self.old, mutation)
            with self.assertRaises(ValueError):
                warm_start_policy(self.new, self.path)
            self.assert_unchanged(before)

    def test_unsupported_input_sizes_do_not_mutate_destination(self):
        before = copy.deepcopy(self.new.state_dict())
        for dimensions in ((104, 138), (105, 137), (106, 156)):
            self.save(SmallActorCritic(*dimensions))
            with self.assertRaises(ValueError):
                warm_start_policy(self.new, self.path)
            self.assert_unchanged(before)

    def test_nan_infinite_negative_noise_and_dtype_overflow_are_rejected(self):
        before = copy.deepcopy(self.new.state_dict())
        mutations = [
            lambda item: item["model_state_dict"]["actor.2.weight"].fill_(float("nan")),
            lambda item: item["model_state_dict"]["critic.0.bias"].fill_(float("inf")),
            lambda item: item["model_state_dict"]["std"].fill_(-0.2),
            lambda item: item["model_state_dict"].update({"actor.2.bias": torch.full((29,), 1e100, dtype=torch.float64)}),
        ]
        for mutation in mutations:
            self.save(self.old, mutation)
            with self.assertRaises(ValueError):
                warm_start_policy(self.new, self.path)
            self.assert_unchanged(before)


if __name__ == "__main__":
    unittest.main()
