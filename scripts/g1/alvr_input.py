#!/usr/bin/env python3
"""Publish Quest/ALVR targets to the local G1 simulation receiver."""

from __future__ import annotations

import argparse
import dataclasses
import json
import os
from pathlib import Path
import select
import signal
import socket
import sys
import termios
import time
import tty

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import numpy as np

from g1_teleop.vr.calibration import TargetMapper, validate_target_bounds
from g1_teleop.vr.openvr_source import OpenVRSource, SyntheticSource
from g1_teleop.vr.protocol import DEFAULT_PORT, encode_target
from g1_teleop.vr.replay import ReplaySource


class Keyboard:
    """One terminal key; restores original terminal settings even on exceptions."""

    def __init__(self):
        self.saved = None

    def __enter__(self):
        if sys.stdin.isatty():
            self.saved = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())
        return self

    def read(self):
        if self.saved and select.select([sys.stdin], [], [], 0)[0]:
            key = os.read(sys.stdin.fileno(), 1).decode(errors="ignore")
            return {"c": "calibrate", "\n": "arm", "\r": "arm", " ": "stop", "r": "reset", "q": "quit"}.get(key)
        return None

    def __exit__(self, *_):
        if self.saved:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.saved)


def load_target_bounds(path):
    value = json.loads(Path(path).read_text())
    if not isinstance(value, dict) or value.get("units") != "m":
        raise ValueError("target-bounds JSON must be an object with units='m'")
    for key, expected in (("schema", "g1.teleop.target_bounds.v1"),
                          ("coordinate_version", "g1-yaw-floor-relative-v1"),
                          ("point_order", ["head", "left_hand", "right_hand"]),
                          ("axis_order", ["x", "y", "z"])):
        if key in value and value[key] != expected:
            raise ValueError(f"target-bounds JSON has incompatible {key}")
    if "delta_min" not in value or "delta_max" not in value:
        raise ValueError("target-bounds JSON needs delta_min and delta_max")
    lower, upper = validate_target_bounds(value["delta_min"], value["delta_max"])
    return dict(delta_min=lower, delta_max=upper)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("openvr", "synthetic", "replay"), default="openvr")
    parser.add_argument("--host", default="127.0.0.1", help="G1 receiver IPv4 host; default stays on this PC")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--hz", type=float, default=60.0)
    parser.add_argument("--duration", type=float, default=0, help="seconds; 0 runs until Ctrl+C")
    parser.add_argument("--nominal", type=Path, help="JSON 3x3 reference positions or object with nominal_targets/nominal_keypoints_root_m")
    parser.add_argument("--scale", type=float, default=0.65, help="human movement to robot displacement scale")
    parser.add_argument("--target-bounds", type=Path,
                        help="JSON metre delta_min/delta_max [head,left_hand,right_hand] x [X,Y,Z]; optional small-motion envelope")
    parser.add_argument("--velocity-limits", nargs=3, type=float, metavar=("VX", "VY", "YAW"),
                        help="command limits in m/s,m/s,rad/s; nonnegative, at most 0.5 0.25 0.6; zero disables an axis")
    parser.add_argument("--manifest", type=Path, help="custom SteamVR action manifest for other emulated controller types")
    parser.add_argument("--record", type=Path, help="new JSONL file: raw OpenVR poses/buttons plus final targets")
    parser.add_argument("--replay-file", type=Path)
    parser.add_argument("--enable-synthetic", action="store_true", help="allow synthetic targets to enable simulation control")
    parser.add_argument("--enable-replay", action="store_true", help="honor recorded enable state during replay")
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535 or not 10 <= args.hz <= 120 or not np.isfinite(args.duration) or args.duration < 0:
        parser.error("require valid port, 10 <= hz <= 120, duration >= 0")
    if not 0 < args.scale <= 2:
        parser.error("scale must be between 0 and 2")
    if args.backend == "replay" and args.replay_file is None:
        parser.error("--backend replay requires --replay-file")
    if args.backend != "replay" and (args.replay_file or args.enable_replay):
        parser.error("replay options require --backend replay")
    if args.enable_synthetic and args.backend != "synthetic":
        parser.error("--enable-synthetic requires --backend synthetic")
    if args.target_bounds and args.backend == "replay":
        parser.error("--target-bounds applies to live/synthetic mapping; replay preserves already-mapped recorded targets")
    if args.velocity_limits is not None:
        limits = np.asarray(args.velocity_limits)
        if not np.isfinite(limits).all() or np.any(limits < 0) or np.any(limits > [0.5, 0.25, 0.6]):
            parser.error("velocity-limits must be finite, nonnegative and no greater than 0.5 0.25 0.6")
        if args.backend == "replay":
            parser.error("--velocity-limits applies to live/synthetic mapping; replay preserves recorded velocity targets")
    return args


def main():
    args = parse_args()
    nominal = None
    if args.nominal:
        nominal = json.loads(args.nominal.read_text())
        if isinstance(nominal, dict):
            nominal = nominal.get("nominal_targets", nominal.get("nominal_keypoints_root_m"))
            if nominal is None:
                raise ValueError("nominal JSON lacks nominal_targets or nominal_keypoints_root_m")
    try:
        bounds = load_target_bounds(args.target_bounds) if args.target_bounds else {}
        if args.velocity_limits is not None:
            bounds["velocity_limits"] = args.velocity_limits
        mapper = TargetMapper(nominal=nominal, scale=args.scale, **bounds)
    except (OSError, ValueError, TypeError) as exc:
        print(f"[VR] ERROR: {exc}", file=sys.stderr, flush=True)
        return 1
    destination = (socket.gethostbyname(args.host), args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    source = None
    record = None
    last = None
    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    old_signals = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    started = time.monotonic()
    try:
        if args.record:
            args.record.parent.mkdir(parents=True, exist_ok=True)
            record = args.record.open("x", encoding="utf-8")
        if args.backend == "openvr":
            source = OpenVRSource(manifest=args.manifest)
        elif args.backend == "synthetic":
            source = SyntheticSource(enabled=args.enable_synthetic)
        else:
            source = ReplaySource(args.replay_file, enabled=args.enable_replay)
        print(f"[VR] source={args.backend} -> UDP {destination[0]}:{destination[1]}", flush=True)
        if args.target_bounds:
            print(f"[VR] provisional target envelope={args.target_bounds}; "
                  f"delta_min_m={mapper.delta_min.tolist()}, delta_max_m={mapper.delta_max.tolist()}", flush=True)
        if args.velocity_limits is not None:
            print(f"[VR] velocity limits [m/s,m/s,rad/s]={mapper.velocity_limits.tolist()}", flush=True)
        print("[VR] X/c calibrate, A/Enter arm, B/Space stop, Y/r reset, q quit. Hold BOTH grips to move.", flush=True)
        if args.backend != "openvr":
            print(f"[VR] {source.last_error}", flush=True)
        next_tick = started
        next_report = started
        with Keyboard() as keys:
            while not stopped and (args.duration == 0 or time.monotonic() - started < args.duration):
                command = keys.read()
                if command == "quit":
                    break
                raw = None
                if args.backend == "replay":
                    try:
                        frame = source.sample_target()
                    except StopIteration:
                        break
                    if command == "stop":
                        # Remains stopped for the rest of this replay process.
                        source.enabled = False
                        frame.enabled = False
                else:
                    raw = source.sample()
                    frame = mapper.update(raw, command=command)
                payload = encode_target(frame)
                sock.sendto(payload, destination)
                last = frame
                if record:
                    raw_dict = dataclasses.asdict(raw) if raw else None
                    if raw_dict:
                        raw_dict["poses"] = raw.poses.tolist()
                    record.write(json.dumps({"elapsed_s": time.monotonic() - started, "raw": raw_dict,
                                             "target": json.loads(payload)}, allow_nan=False) + "\n")
                now = time.monotonic()
                if now >= next_report:
                    print(f"[VR] seq={frame.seq} status={mapper.last_status if raw else 'REPLAY'} "
                          f"enabled={frame.enabled} tracking={frame.tracking_valid} "
                          f"velocity={np.round(frame.velocity, 3).tolist()} {source.last_error}", flush=True)
                    next_report = now + 1
                    if record:
                        record.flush()
                next_tick = max(next_tick + 1.0 / args.hz, now)
                time.sleep(max(0.0, next_tick - time.monotonic()))
    except (OSError, RuntimeError, ValueError, KeyError) as exc:
        print(f"[VR] ERROR: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        # Send several explicit stop packets, then receiver timeout remains the fallback.
        if last is not None:
            for offset in range(1, 4):
                stopped_frame = dataclasses.replace(last, seq=last.seq + offset, enabled=False,
                                                     velocity=np.zeros(3), timestamp_unix_ns=time.time_ns())
                try:
                    sock.sendto(encode_target(stopped_frame), destination)
                except OSError:
                    break
        if source:
            source.close()
        if record:
            record.close()
        sock.close()
        for sig, handler in old_signals.items():
            signal.signal(sig, handler)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
