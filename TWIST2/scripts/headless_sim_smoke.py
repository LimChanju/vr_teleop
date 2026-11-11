#!/usr/bin/env python3
"""Run the unmodified official low-level controller for a finite headless smoke test.

The wrapper replaces only the MuJoCo viewer object and monitors physics state;
policy construction, observations, ONNX inference, action scaling, PD control,
and MuJoCo stepping remain in the official server implementation.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "deploy_real"))

import server_low_level_g1_sim as official  # noqa: E402
from twist2_vr.controller import RedisSimulationSink  # noqa: E402


class _Camera:
    def __init__(self) -> None:
        self.lookat = np.zeros(3)
        self.distance = 2.0


class _ViewerOptions:
    def __init__(self) -> None:
        self.flags = np.zeros(256, dtype=np.int32)


class HeadlessViewer:
    def __init__(self) -> None:
        self.cam = _Camera()
        self.opt = _ViewerOptions()

    def sync(self) -> None:
        pass

    def close(self) -> None:
        pass


def tilt_deg(q_wxyz: np.ndarray) -> float:
    w, x, y, z = q_wxyz
    up_z = 1.0 - 2.0 * (x * x + y * y)
    return float(np.degrees(np.arccos(np.clip(up_z, -1.0, 1.0))))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--policy-frequency", type=float, default=50.0)
    parser.add_argument("--redis-port", type=int, default=6379)
    parser.add_argument("--action-timeout-ms", type=int, default=300)
    parser.add_argument("--realtime", action="store_true")
    parser.add_argument("--output", type=Path, default=ROOT / "runtime/sim_smoke.json")
    args = parser.parse_args()
    if args.seconds <= 0:
        raise ValueError("seconds must be positive")

    sink = RedisSimulationSink(port=args.redis_port, allow_simulation_publish=True)
    original_launch = official.mjv.launch_passive
    original_step = official.mujoco.mj_step
    heights: list[float] = []
    tilts: list[float] = []
    torques: list[float] = []

    official.mjv.launch_passive = lambda *a, **k: HeadlessViewer()

    def monitored_step(model, data):
        original_step(model, data)
        heights.append(float(data.qpos[2]))
        tilts.append(tilt_deg(np.asarray(data.qpos[3:7])))
        torques.append(float(np.max(np.abs(data.ctrl))))

    official.mujoco.mj_step = monitored_step
    started = time.monotonic()
    try:
        controller = official.RealTimePolicyController(
            xml_file=str(ROOT / "assets/g1/g1_sim2sim_29dof.xml"),
            policy_path=str(ROOT / "assets/ckpts/twist2_1017_20k.onnx"),
            device="cpu",
            measure_fps=False,
            limit_fps=args.realtime,
            policy_frequency=args.policy_frequency,
            redis_port=args.redis_port,
            action_timeout_ms=args.action_timeout_ms,
        )
        controller.sim_duration = args.seconds
        controller.run()
    finally:
        official.mjv.launch_passive = original_launch
        official.mujoco.mj_step = original_step
        sink.close()
    report = {
        "simulated_seconds": args.seconds,
        "physics_steps": len(heights),
        "wall_seconds": time.monotonic() - started,
        "policy_frequency_hz": args.policy_frequency,
        "onnx_provider": "CPUExecutionProvider",
        "root_height_m": {"min": min(heights), "max": max(heights), "final": heights[-1]},
        "base_tilt_deg": {"max": max(tilts), "final": tilts[-1]},
        "max_abs_torque": max(torques),
        "finite": bool(np.all(np.isfinite(heights + tilts + torques))),
    }
    report["standing_smoke_passed"] = bool(
        report["finite"] and report["root_height_m"]["min"] > 0.45 and report["base_tilt_deg"]["max"] < 60.0
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["standing_smoke_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
