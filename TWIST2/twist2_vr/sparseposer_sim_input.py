"""SparsePoser/GMR references for a dedicated LOCAL simulation Redis only.

Runs in the SparsePoser interpreter, separately from the policy/VR renderer.
No pose scaling, clipping or smoothing is added. Invalid/stale input selects
the existing controller's neutral reference, never a fabricated tracked pose.
"""
from __future__ import annotations

import json
import signal
import threading
import time

import numpy as np

from .controller import RedisSimulationSink
from .models import RobotReference
from .retargeting import DEFAULT_MIMIC_OBS
from .sparseposer_g1_preview import (
    XML, estimation_worker, joint_signature, parser, validate_args,
)
from .sparseposer_adapter import file_sha256


def select_reference(event, now, *, replay=False, max_age=0.20):
    """Return an exact fresh valid reference, or a neutral fallback + reason."""
    reason = event.get("kind", "missing event")
    if reason == "g1_kinematic_pose":
        generated = float(event.get("generated_at_s", float("nan")))
        stamp = float(event.get("timestamp_s", float("nan")))
        age = now - generated
        source_age = now - stamp
        if not np.isfinite(age) or not -0.01 <= age <= max_age:
            reason = "stale generated reference"
        elif not replay and (not np.isfinite(source_age) or not -0.01 <= source_age <= max_age):
            reason = "stale sensor input"
        elif not event.get("fit_accepted", False):
            reason = event.get("reason") or "invalid human/GMR fit"
        else:
            return RobotReference(now, np.asarray(event["mimic_obs"]),
                                  np.asarray(event["qpos"]), True, "sparseposer_sim"), "tracking"
    return RobotReference(now, DEFAULT_MIMIC_OBS.copy(), None, True,
                          "sim_neutral_fallback", str(reason)), str(reason)


class PublisherQueue:
    """Queue-compatible adapter; estimation_worker remains unchanged."""
    def __init__(self, sink, stream, replay):
        self.sink, self.stream, self.replay = sink, stream, replay
        self.counts = {"tracking": 0, "neutral": 0}
        self.last_reason = None

    def put_nowait(self, event):
        now = time.monotonic()
        reference, reason = select_reference(event, now, replay=self.replay)
        self.sink.publish(reference)
        self.counts["tracking" if reason == "tracking" else "neutral"] += 1
        self.stream.write(json.dumps({"monotonic_s": now, "state": reason,
                                      "source_timestamp_s": event.get("timestamp_s"),
                                      "mimic_obs": reference.mimic_obs.tolist()}) + "\n")
        if reason != self.last_reason:
            self.sink.client.set("sparseposer_input_state", reason)
            print(f"SIM INPUT: {reason}", flush=True)
            self.stream.flush()
            self.last_reason = reason

    def cancel_join_thread(self):
        pass


def main(argv=None):
    p = parser()
    p.description = __doc__
    p.add_argument("--redis-port", type=int, required=True)
    args = p.parse_args(argv)
    if args.output is None or args.fast or not 1024 <= args.redis_port <= 65535:
        p.error("requires NEW --output and dedicated --redis-port; --fast is not supported")
    validate_args(args)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "run.json").write_text(json.dumps({
        "source": "replay" if args.recording else "openvr",
        "calibration": str(args.calibration), "calibration_sha256": file_sha256(args.calibration),
        "recording": str(args.recording) if args.recording else None,
        "transport": "isolated_loopback_redis", "redis_port": args.redis_port,
        "robot_output": False,
    }, indent=2) + "\n")
    stop = threading.Event()
    def stopping(*_):
        stop.set()
    signal.signal(signal.SIGTERM, stopping)
    signal.signal(signal.SIGINT, stopping)
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(XML))
    sink = RedisSimulationSink(host="127.0.0.1", port=args.redis_port, allow_simulation_publish=True)
    try:
        print("Waiting for local simulation state (then 2 seconds neutral settling)...", flush=True)
        deadline = time.monotonic() + 90
        while not stop.is_set():
            stamp = sink.client.get("t_state")
            if stamp and 0 <= time.time()*1000 - int(stamp) < 1000:
                break
            if time.monotonic() > deadline:
                raise TimeoutError("local simulation did not start")
            stop.wait(.05)
        if stop.wait(2):
            return 0
        with (args.output / "published.jsonl").open("x") as stream:
            queue = PublisherQueue(sink, stream, bool(args.recording))
            estimation_worker(args, args.output, queue, stop, joint_signature(model))
            (args.output / "publish_summary.json").write_text(json.dumps(queue.counts, indent=2)+"\n")
    finally:
        sink.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
