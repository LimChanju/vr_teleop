"""Explicit, checked migration from sparse-position v1 to tracking-feedback v2."""
from __future__ import annotations

import copy
import math
from collections.abc import Mapping
from numbers import Real
import operator
import torch
from torch import nn


def warm_start_policy(policy, checkpoint_path, noise_std=None):
    """Preserve the old action/value functions while adding zero-weight inputs.

    Actor: old105 -> old105, base velocity3, marker error9, target velocity9.
    Critic: old105, velocity3, height1, qref29 -> new126, height1, qref29.
    The optimizer deliberately starts fresh; this is initialization, not resume.
    """
    if noise_std is not None:
        if isinstance(noise_std, bool) or not isinstance(noise_std, Real) or not math.isfinite(noise_std) or not 0 < noise_std <= 1:
            raise ValueError("Warm-start noise must be a finite number in (0,1]")
    saved = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    if not isinstance(saved, Mapping) or not isinstance(saved.get("model_state_dict"), Mapping):
        raise ValueError("Warm-start checkpoint requires model_state_dict")
    try:
        iteration = operator.index(saved["iter"])
    except (KeyError, TypeError) as exc:
        raise ValueError("Warm-start checkpoint requires an integer iteration") from exc
    source = saved["model_state_dict"]
    # state_dict tensors alias live parameters. Work on clones so any validation
    # or function-preservation failure leaves the existing policy untouched.
    destination = {name: value.detach().clone() for name, value in policy.state_dict().items()}
    if set(source) != set(destination) or not {"actor.0.weight", "critic.0.weight"}.issubset(source):
        raise ValueError("Warm start requires matching feed-forward ActorCritic parameter names")
    for name, value in source.items():
        if not isinstance(value, torch.Tensor) or not value.is_floating_point() or not torch.isfinite(value).all():
            raise ValueError(f"Warm-start parameter must be a finite floating tensor: {name}")
    for name in ("actor.0.weight", "critic.0.weight"):
        if source[name].ndim != 2 or destination[name].ndim != 2:
            raise ValueError(f"Warm-start first-layer weight must be a matrix: {name}")
    if "std" in source and torch.any(source["std"] <= 0):
        raise ValueError("Warm-start checkpoint has nonpositive action noise")
    if noise_std is not None and "std" not in destination:
        raise ValueError("Warm-start noise override requires scalar-standard-deviation ActorCritic")
    old_actor = source["actor.0.weight"].shape[1]
    new_actor = destination["actor.0.weight"].shape[1]
    old_critic = source["critic.0.weight"].shape[1]
    new_critic = destination["critic.0.weight"].shape[1]
    migrating = (old_actor, new_actor, old_critic, new_critic) == (105, 126, 138, 156)
    if not migrating and (old_actor != new_actor or old_critic != new_critic):
        raise ValueError(f"Unsupported input migration: {(old_actor, new_actor, old_critic, new_critic)}")
    for name, value in source.items():
        target = destination[name]
        if value.shape == target.shape:
            target.copy_(value)
        elif migrating and name == "actor.0.weight":
            if value.shape[0] != target.shape[0]:
                raise ValueError("Warm-start actor hidden width differs")
            target.zero_()
            target[:, :105].copy_(value)
        elif migrating and name == "critic.0.weight":
            if value.shape[0] != target.shape[0]:
                raise ValueError("Warm-start critic hidden width differs")
            target.zero_()
            target[:, :108].copy_(value[:, :108])
            target[:, 126:].copy_(value[:, 108:])
        else:
            raise ValueError(f"Incompatible warm-start parameter {name}: {value.shape} -> {target.shape}")
        if not torch.isfinite(target).all():
            raise ValueError(f"Warm-start parameter overflows the destination dtype: {name}")

    def compare(network, prefix, old_dim, new_dim, critic=False):
        new = copy.deepcopy(network).cpu().eval()
        new.load_state_dict({key[len(prefix):]: value for key, value in destination.items() if key.startswith(prefix)})
        old = copy.deepcopy(new)
        old[0] = nn.Linear(old_dim, new[0].out_features)
        old.load_state_dict({key[len(prefix):]: value for key, value in source.items() if key.startswith(prefix)})
        generator = torch.Generator().manual_seed(9101)
        new_obs = torch.randn((64, new_dim), generator=generator)
        if migrating and critic:
            old_obs = torch.cat((new_obs[:, :108], new_obs[:, 126:]), dim=1)
        else:
            old_obs = new_obs[:, :old_dim]
        with torch.inference_mode():
            return (new(new_obs) - old(old_obs)).abs().max().item()

    actor_error = compare(policy.actor, "actor.", old_actor, new_actor)
    critic_error = compare(policy.critic, "critic.", old_critic, new_critic, critic=True)
    if not math.isfinite(actor_error) or not math.isfinite(critic_error) or max(actor_error, critic_error) > 2e-5:
        raise RuntimeError(f"Warm-start function changed: actor={actor_error}, critic={critic_error}")
    if noise_std is not None:
        destination["std"].fill_(float(noise_std))
    policy.load_state_dict(destination)
    return {"source_checkpoint": str(checkpoint_path), "source_iteration": iteration,
            "old_actor_dim": old_actor, "new_actor_dim": new_actor,
            "old_critic_dim": old_critic, "new_critic_dim": new_critic,
            "actor_preservation_error": actor_error, "critic_preservation_error": critic_error,
            "optimizer_transferred": False, "noise_std_override": noise_std}
