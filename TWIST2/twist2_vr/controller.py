"""Simulation-only controller boundary and safety checks."""

from __future__ import annotations

import json
import os
import time
from typing import Any, Protocol

import numpy as np

from .models import RobotReference
from .retargeting import DEFAULT_MIMIC_OBS


class ReferenceSink(Protocol):
    def publish(self, reference: RobotReference) -> None: ...
    def fail_safe(self, reason: str) -> None: ...
    def close(self) -> None: ...


class SafetyGate:
    def __init__(self, max_age_s: float = 0.20, root_height_range_m: tuple[float, float] = (0.45, 1.20)) -> None:
        self.max_age_s = max_age_s
        self.root_height_range_m = root_height_range_m

    def validate(self, reference: RobotReference, now_s: float) -> tuple[bool, str]:
        if not reference.valid:
            return False, reference.reason or "invalid reference"
        age = now_s - reference.timestamp_s
        if age < -1e-6 or age > self.max_age_s:
            return False, f"reference timestamp stale ({age:.3f}s)"
        if reference.mimic_obs.shape != (35,) or not np.all(np.isfinite(reference.mimic_obs)):
            return False, "reference is not a finite 35-vector"
        root_height = float(reference.mimic_obs[2])
        low, high = self.root_height_range_m
        if not low <= root_height <= high:
            return False, f"root height {root_height:.3f}m outside [{low:.3f}, {high:.3f}]"
        return True, ""


class DryRunSink:
    """In-memory sink used by default and by all automated tests."""

    def __init__(self) -> None:
        self.references: list[RobotReference] = []
        self.failures: list[str] = []
        self.closed = False

    def publish(self, reference: RobotReference) -> None:
        if self.closed:
            raise RuntimeError("dry-run sink is closed")
        self.references.append(reference)

    def fail_safe(self, reason: str) -> None:
        self.failures.append(reason)

    def close(self) -> None:
        self.closed = True


class RedisSimulationSink:
    """Guarded publisher for the official local MuJoCo server.

    It refuses remote hosts and requires both a constructor flag and the
    ``TWIST2_ALLOW_SIM_REDIS=1`` environment guard.  No Unitree SDK is imported.
    """

    KEYS = {
        "body": "action_body_unitree_g1_with_hands",
        "left_hand": "action_hand_left_unitree_g1_with_hands",
        "right_hand": "action_hand_right_unitree_g1_with_hands",
        "neck": "action_neck_unitree_g1_with_hands",
    }

    def __init__(self, host: str = "localhost", port: int = 6379, *, allow_simulation_publish: bool = False,
                 client: Any | None = None) -> None:
        if host not in {"localhost", "127.0.0.1", "::1"}:
            raise ValueError("simulation publisher is localhost-only")
        if not allow_simulation_publish or os.environ.get("TWIST2_ALLOW_SIM_REDIS") != "1":
            raise PermissionError(
                "simulation Redis publishing is disarmed; pass the explicit flag and set TWIST2_ALLOW_SIM_REDIS=1"
            )
        if client is None:
            try:
                import redis
            except ImportError as error:
                raise RuntimeError("install the project with [redis] for simulation publishing") from error
            client = redis.Redis(host=host, port=port, db=0, socket_timeout=0.5)
        self.client = client
        self.pipeline = client.pipeline()
        self.client.ping()
        self.armed = True
        self._write(DEFAULT_MIMIC_OBS)

    def _write(self, mimic_obs: np.ndarray) -> None:
        self.pipeline.set(self.KEYS["body"], json.dumps(np.asarray(mimic_obs).tolist()))
        self.pipeline.set(self.KEYS["left_hand"], json.dumps(np.zeros(7).tolist()))
        self.pipeline.set(self.KEYS["right_hand"], json.dumps(np.zeros(7).tolist()))
        self.pipeline.set(self.KEYS["neck"], json.dumps(np.zeros(2).tolist()))
        self.pipeline.set("t_action", int(time.time() * 1000))
        self.pipeline.execute()

    def publish(self, reference: RobotReference) -> None:
        if not self.armed:
            raise RuntimeError("simulation sink is disarmed")
        self._write(reference.mimic_obs)

    def fail_safe(self, reason: str) -> None:
        if self.armed:
            self._write(DEFAULT_MIMIC_OBS)
            self.armed = False

    def close(self) -> None:
        if self.armed:
            self._write(DEFAULT_MIMIC_OBS)
        self.armed = False
