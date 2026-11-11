#!/usr/bin/env python3
"""Publish an official example motion to localhost simulation Redis only."""

from __future__ import annotations

import argparse
from pathlib import Path
import signal
import time

import numpy as np

from twist2_vr.controller import RedisSimulationSink
from twist2_vr.models import RobotReference
from twist2_vr.offline_motion import OfficialExampleMotionStream
from twist2_vr.retargeting import DEFAULT_MIMIC_OBS


ROOT = Path(__file__).resolve().parents[1]


def _reference(values: np.ndarray, source: str) -> RobotReference:
    return RobotReference(time.monotonic(), values, None, True, source)


def _hold(sink: RedisSimulationSink, reference: RobotReference, seconds: float, frequency: float) -> RobotReference:
    frames = max(0, round(seconds * frequency))
    deadline = time.monotonic()
    last = reference
    for _ in range(frames):
        deadline += 1.0 / frequency
        last = _reference(reference.mimic_obs, reference.source)
        sink.publish(last)
        time.sleep(max(0.0, deadline - time.monotonic()))
    return last


def _ramp(
    sink: RedisSimulationSink,
    start: RobotReference,
    end: RobotReference,
    seconds: float,
    frequency: float,
    source: str,
) -> RobotReference:
    frames = max(1, round(seconds * frequency))
    deadline = time.monotonic()
    last = start
    for index in range(1, frames + 1):
        deadline += 1.0 / frequency
        alpha = index / frames
        values = (1.0 - alpha) * start.mimic_obs + alpha * end.mimic_obs
        last = _reference(values, source)
        sink.publish(last)
        time.sleep(max(0.0, deadline - time.monotonic()))
    return last


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--motion",
        type=Path,
        default=ROOT / "assets/example_motions/0807_yanjie_walk_003.pkl",
    )
    parser.add_argument("--frequency", type=float, default=50.0)
    parser.add_argument("--prime-seconds", type=float, default=0.2)
    parser.add_argument("--transition-seconds", type=float, default=2.0)
    parser.add_argument("--wait-for-controller-seconds", type=float, default=15.0)
    args = parser.parse_args()
    if args.frequency <= 0:
        raise ValueError("frequency must be positive")

    stream = OfficialExampleMotionStream(args.motion, control_dt_s=1.0 / args.frequency)
    sink = RedisSimulationSink(allow_simulation_publish=True)
    first = stream.reference(0, time.monotonic())
    default = _reference(DEFAULT_MIMIC_OBS, "default_stand")
    last = default

    def _stop_on_sigterm(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _stop_on_sigterm)
    try:
        print("phase=WAIT_LOW_LEVEL", flush=True)
        deadline = time.monotonic() + args.wait_for_controller_seconds
        previous_state_time = None
        state_updates = 0
        while state_updates < 2:
            state_time = sink.client.get("t_state")
            if state_time is not None and state_time != previous_state_time:
                previous_state_time = state_time
                state_updates += 1
            sink.publish(default)
            if time.monotonic() >= deadline:
                raise TimeoutError("official simulation controller did not appear on localhost Redis")
            time.sleep(1.0 / args.frequency)

        print("phase=PRIME_DEFAULT", flush=True)
        last = _hold(sink, default, args.prime_seconds, args.frequency)
        print("phase=RAMP_IN", flush=True)
        last = _ramp(sink, last, first, args.transition_seconds, args.frequency, "ramp_in")

        print("phase=STREAM_EXACT", flush=True)
        deadline = time.monotonic()
        for step in range(stream.num_steps):
            deadline += 1.0 / args.frequency
            reference = stream.reference(step, time.monotonic())
            sink.publish(reference)
            last = reference
            if step == 0 or (step + 1) % 100 == 0:
                print(
                    f"motion={args.motion.name} step={step + 1}/{stream.num_steps} "
                    f"root_z={reference.mimic_obs[2]:.3f}",
                    flush=True,
                )
            time.sleep(max(0.0, deadline - time.monotonic()))
    except KeyboardInterrupt:
        print("Example motion publisher stopped.")
    finally:
        print("phase=RAMP_OUT", flush=True)
        try:
            _ramp(sink, last, default, args.transition_seconds, args.frequency, "ramp_out")
        except KeyboardInterrupt:
            sink.fail_safe("second interrupt during ramp-out")
        sink.close()
        print("phase=EXIT", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
