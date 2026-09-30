#!/usr/bin/env python3
"""Finite SYNTHETIC UDP scenario; receiver/simulator results must be checked separately."""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
from pathlib import Path
import signal
import socket
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import numpy as np

from g1_teleop.vr.calibration import RawFrame, TargetMapper
from g1_teleop.vr.protocol import DEFAULT_PORT, encode_target

# name, duration multiplier, expected producer enabled state (None = no producer).
PHASES = (
    ("idle", 0.25, False),
    ("calibrate", 0.25, False),
    ("armed_motion", 2.0, True),
    ("deadman_release", 1.0, False),
    ("rearm_motion", 1.0, True),
    ("producer_pause", 0.5, None),
    ("held_arm_after_pause", 1.0, False),
    ("arm_release", 0.5, False),
    ("rearm_after_pause", 1.0, True),
    ("reset", 0.5, False),
    ("post_reset_rearm", 1.0, True),
    ("final_stop", 0.5, False),
)
ACTIVE_PHASES = {name for name, _, enabled in PHASES if enabled}


def make_raw(phase, fraction, scale):
    """Smooth, reachable 6cm robot-target excursions; no walking command."""
    poses = np.tile(np.eye(4)[:3], (3, 1, 1))
    poses[:, :, 3] = [[0, 1.65, 0], [-0.28, 1.2, -0.25], [0.28, 1.2, -0.25]]
    if phase in ACTIVE_PHASES:
        offset = 0.06 / scale * np.sin(2 * np.pi * fraction)
        poses[1, 1, 3] += offset  # OpenVR Y -> robot Z, left hand.
        poses[2, 2, 3] -= offset  # OpenVR -Z -> robot X, right hand.
    arm = phase in ACTIVE_PHASES or phase == "held_arm_after_pause"
    grips = (0.85, 0.85) if arm or phase == "arm_release" else (0.0, 0.0)
    return RawFrame(poses=poses, tracking_valid=True, inputs_valid=True,
                    calibrate=phase == "calibrate", arm=arm, grips=grips,
                    reset=phase == "reset", stop=phase in ("final_stop", "cleanup_stop"),
                    sampled_monotonic=time.monotonic(), source="synthetic")


def load_nominal(path):
    value = json.loads(path.read_text())
    if isinstance(value, dict):
        value = value.get("nominal_targets", value.get("nominal_keypoints_root_m"))
    if value is None:
        raise ValueError("nominal JSON needs nominal_targets or nominal_keypoints_root_m")
    return value


def analyze_scenario(events_path, run_dir, grace_seconds=0.15):
    """Check actual simulator trace against transmitted phases; no synthetic verdicts."""
    records = [json.loads(line) for line in Path(events_path).read_text().splitlines() if line.strip()]
    summaries = [r for r in records if r.get("kind") == "summary"]
    begins = {r["phase"]: r for r in records if r.get("kind") == "phase_begin"}
    ends = {r["phase"]: r for r in records if r.get("kind") == "phase_end"}
    sent = {r["target"]["seq"]: r for r in records if r.get("kind") == "frame"}
    run_dir = Path(run_dir)
    result = json.loads((run_dir / "result.json").read_text())
    with np.load(run_dir / "trace.npz", allow_pickle=False) as data:
        required = ("wall_time", "input_sequence", "input_fresh", "manual_reset", "enabled",
                    "done", "q", "action", "target", "actual")
        missing = set(required) - set(data.files)
        if missing:
            raise ValueError(f"Trace lacks required evidence: {sorted(missing)}")
        trace = {key: np.asarray(data[key]) for key in required}
    wall = trace["wall_time"].astype(float)
    n = len(wall)
    if n == 0 or any(len(value) != n for value in trace.values()):
        raise ValueError("Empty or inconsistent simulator trace")
    if not np.isfinite(wall).all() or np.any(np.diff(wall) <= 0):
        raise ValueError("Trace wall_time must be finite and increasing")
    if any(trace[key].shape != (n,) for key in required[:6]):
        raise ValueError("Trace time/input/enable/reset arrays must have shape [T]")
    if trace["q"].shape != (n, 29) or trace["action"].shape != (n, 29):
        raise ValueError("Trace q/action must have shape [T,29]")
    if trace["target"].shape != (n, 3, 3) or trace["actual"].shape != (n, 3, 3):
        raise ValueError("Trace target/actual must have shape [T,3,3]")
    checks = {}
    def check(name, passed, **details):
        checks[name] = dict(passed=bool(passed), **details)
    check("producer_completed", len(summaries) == 1 and summaries[0].get("producer_completed", False))
    check("synthetic_source", result.get("input_sources") == ["synthetic"]
          and not result.get("physical_quest_verified", False), observed=result.get("input_sources"))
    check("teleop_result", result.get("mode") == "teleop" and result.get("status") == "teleop_stopped",
          status=result.get("status"))
    check("no_falls", result.get("falls") == 0, falls=result.get("falls"))
    finite = {key: bool(np.isfinite(trace[key]).all()) for key in ("q", "action", "target", "actual")}
    check("finite_state_action_targets", all(finite.values()), fields=finite)
    valid_metrics = np.logical_and.reduce([
        np.isfinite(trace[key]).reshape(n, -1).all(-1) for key in ("q", "action", "target", "actual")])
    for index in np.flatnonzero(trace["done"].astype(bool) | trace["manual_reset"].astype(bool)):
        valid_metrics[max(0, index - 1):min(n, index + 2)] = False
    phases = {}
    for name, _, expected in PHASES:
        if name not in begins or name not in ends:
            check(f"phase_{name}", False, reason="missing phase begin/end event")
            continue
        start = begins[name]["timestamp_unix_ns"] / 1e9
        end = ends[name]["timestamp_unix_ns"] / 1e9
        # Outage keeps the last frame fresh for receiver timeout (default0.25s).
        lower = start + grace_seconds + (0.25 if expected is None else 0.0)
        upper = end - grace_seconds
        mask = (wall >= lower) & (wall < upper)
        count = int(mask.sum())
        coverage = count >= 3 and wall[0] <= lower and wall[-1] >= upper and upper > lower
        wanted = False if expected is None else expected
        enabled_ok = count > 0 and bool(np.all(trace["enabled"][mask] == wanted))
        fresh_ok = count > 0 and bool(np.all(trace["input_fresh"][mask] == (expected is not None)))
        check(f"phase_{name}", coverage and enabled_ok and fresh_ok,
              samples=count, coverage=coverage, expected_enabled=wanted,
              enabled_samples=int(trace["enabled"][mask].sum()), fresh_samples=int(trace["input_fresh"][mask].sum()),
              analyzed_wall_interval=[lower, upper])
        item = dict(samples=count, expected_enabled=wanted)
        if count:
            valid = mask & valid_metrics
            errors = np.linalg.norm(trace["target"][valid] - trace["actual"][valid], axis=-1)
            if len(errors):
                hands = errors[:, 1:].mean(axis=1)
                item.update(hand_error_mean_m=float(hands.mean()),
                            hand_error_p95_m=float(np.percentile(hands, 95)),
                            head_error_mean_m=float(errors[:, 0].mean()))
        phases[name] = item
    actual_resets = np.flatnonzero(trace["manual_reset"].astype(bool))
    reset_interval = None
    if "reset" in begins and "reset" in ends:
        reset_interval = (begins["reset"]["timestamp_unix_ns"] / 1e9,
                          ends["reset"]["timestamp_unix_ns"] / 1e9 + grace_seconds)
    check("one_manual_reset", len(actual_resets) == 1 and result.get("manual_resets") == 1
          and reset_interval is not None and reset_interval[0] <= wall[actual_resets[0]] <= reset_interval[1],
          trace_count=len(actual_resets), result_count=result.get("manual_resets"))
    active = np.flatnonzero(trace["enabled"].astype(bool))
    unmatched, wrong_phase, target_mismatch = [], [], []
    errors = []
    for index in active:
        seq = int(trace["input_sequence"][index])
        packet = sent.get(seq)
        if packet is None:
            unmatched.append(int(index))
            continue
        if packet["phase"] not in ACTIVE_PHASES or not packet["target"]["enabled"]:
            wrong_phase.append(int(index))
        delta = float(np.max(np.abs(trace["target"][index] - packet["target"]["target_positions_m"])))
        if np.isfinite(delta):
            errors.append(delta)
        if not np.isfinite(delta) or delta > 5e-5:
            target_mismatch.append(int(index))
    check("active_inputs_match_sent_targets", len(active) > 0 and not unmatched and not wrong_phase and not target_mismatch,
          active_steps=len(active), unknown_sequence_rows=unmatched[:20],
          invalid_phase_rows=wrong_phase[:20], target_mismatch_rows=target_mismatch[:20],
          max_abs_target_error_m=max(errors) if errors else None)
    # Descriptive movement evidence; this is not a tracking-quality threshold.
    response = {}
    valid_active = active[valid_metrics[active]]
    if len(valid_active):
        response = dict(actual_keypoint_range_m=np.ptp(trace["actual"][valid_active], axis=0).tolist(),
                        target_keypoint_range_m=np.ptp(trace["target"][valid_active], axis=0).tolist(),
                        max_joint_range_rad=float(np.ptp(trace["q"][valid_active], axis=0).max()))
    return dict(schema="g1.runtime.scenario.analysis.v1", events=str(events_path), run=str(run_dir),
                grace_seconds=grace_seconds, receiver_timeout_assumed_s=0.25,
                simulator_checks_passed=all(item["passed"] for item in checks.values()),
                physical_quest_verified=False, checks=checks, phases=phases, response=response,
                metric_reset_guard_frames=1,
                tracking_quality_status="Reported metrics only; no task success threshold applied")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nominal", type=Path)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--output", type=Path, required=True, help="new JSONL events/raw/target file; never overwritten")
    parser.add_argument("--phase-seconds", type=float, default=3.0,
                        help="default scenario about 28.5 seconds; producer pause at least 0.6s")
    parser.add_argument("--hz", type=float, default=60.0)
    parser.add_argument("--scale", type=float, default=0.65)
    parser.add_argument("--analyze-run", type=Path, help="analysis only: compare --output event JSONL with this simulator run")
    parser.add_argument("--analysis-output", type=Path, help="new report JSON (default RUN/scenario_analysis.json)")
    parser.add_argument("--grace-seconds", type=float, default=0.15,
                        help="explicit phase-boundary latency allowance for trace analysis")
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535 or not 10 <= args.hz <= 120:
        parser.error("require valid port and 10 <= hz <= 120")
    if not np.isfinite(args.phase_seconds) or not 0.3 <= args.phase_seconds <= 30:
        parser.error("phase-seconds must be finite and between 0.3 and 30")
    if not np.isfinite(args.scale) or not 0 < args.scale <= 2:
        parser.error("scale must be finite and between 0 and 2")
    if not args.analyze_run and args.nominal is None:
        parser.error("sending requires --nominal")
    if args.analysis_output and not args.analyze_run:
        parser.error("--analysis-output requires --analyze-run")
    if not np.isfinite(args.grace_seconds) or not 0 <= args.grace_seconds <= 0.5:
        parser.error("grace-seconds must be between 0 and 0.5")
    return args


def main(argv=None):
    args = parse_args(argv)
    if args.analyze_run:
        destination = args.analysis_output or args.analyze_run / "scenario_analysis.json"
        destination.parent.mkdir(parents=True, exist_ok=True)
        with destination.open("x", encoding="utf-8") as record:
            try:
                report = analyze_scenario(args.output, args.analyze_run, args.grace_seconds)
            except (OSError, ValueError, KeyError, TypeError) as exc:
                report = dict(simulator_checks_passed=False, error=f"{type(exc).__name__}: {exc}",
                              physical_quest_verified=False)
            json.dump(report, record, indent=2, allow_nan=False)
            record.write("\n")
        print(json.dumps(report, indent=2, allow_nan=False))
        return 0 if report["simulator_checks_passed"] else 1
    mapper = TargetMapper(nominal=load_nominal(args.nominal), scale=args.scale)
    destination = (socket.gethostbyname(args.host), args.port)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Open exclusively before creating a sender or installing signal handlers.
    with args.output.open("x", encoding="utf-8") as record, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        started = time.monotonic()
        stopped = False
        counters = {}
        error = None
        def stop(*_):
            nonlocal stopped
            stopped = True
        handlers = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}

        def log(kind, **values):
            row = dict(kind=kind, elapsed_s=time.monotonic() - started,
                       timestamp_unix_ns=time.time_ns(), source="synthetic", **values)
            record.write(json.dumps(row, allow_nan=False) + "\n")
            if kind != "frame":
                record.flush()

        def send(phase, raw, expected=None):
            frame = mapper.update(raw)
            if expected is not None and frame.enabled != expected:
                raise RuntimeError(f"Producer state mismatch in {phase}: expected enabled={expected}, "
                                   f"actual={frame.enabled}, mapper={mapper.last_status}")
            payload = encode_target(frame)
            if sock.sendto(payload, destination) != len(payload):
                raise RuntimeError("UDP send did not accept the entire target packet")
            raw_data = dataclasses.asdict(raw)
            raw_data["poses"] = raw.poses.tolist()
            log("frame", phase=phase, raw=raw_data, target=json.loads(payload),
                mapper_status=mapper.last_status, expected_enabled=expected)
            counters[phase] = counters.get(phase, 0) + 1

        print(f"[SCENARIO] SYNTHETIC ONLY -> UDP {destination[0]}:{destination[1]}; "
              "sender has no simulator acknowledgement", flush=True)
        try:
            log("start", schema="g1.runtime.scenario.v1", session=mapper.session,
                destination=list(destination), phase_seconds=args.phase_seconds, hz=args.hz,
                scale=args.scale, robot_hand_amplitude_m=0.06,
                nominal_file=str(args.nominal.resolve()),
                nominal_sha256=hashlib.sha256(args.nominal.read_bytes()).hexdigest(),
                physical_quest_verified=False, receiver_verified=False)
            for phase, multiplier, expected in PHASES:
                if stopped:
                    break
                duration = multiplier * args.phase_seconds
                if phase == "producer_pause":
                    duration = max(0.6, duration)
                phase_start = time.monotonic()
                next_tick = phase_start
                log("phase_begin", phase=phase, duration_s=duration, expected_enabled=expected,
                    sends_packets=expected is not None)
                print(f"[SCENARIO] {phase}: {duration:.2f}s", flush=True)
                while not stopped and time.monotonic() - phase_start < duration:
                    now = time.monotonic()
                    if expected is not None:
                        fraction = np.clip((now - phase_start) / duration, 0.0, 1.0)
                        send(phase, make_raw(phase, fraction, args.scale), expected)
                    # During the outage neither sendto nor TargetMapper.update runs.
                    next_tick = max(next_tick + 1.0 / args.hz, time.monotonic())
                    time.sleep(max(0.0, next_tick - time.monotonic()))
                actual_duration = time.monotonic() - phase_start
                if not stopped and expected is not None and not counters.get(phase):
                    raise RuntimeError(f"No frame was sent for required phase {phase}")
                log("phase_end", phase=phase, actual_duration_s=actual_duration,
                    sent_frames=counters.get(phase, 0), reset_id=mapper.reset_id)
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
            log("error", error=error)
            print(f"[SCENARIO] ERROR: {error}", file=sys.stderr, flush=True)
        finally:
            # Explicit stop complements, rather than replaces, receiver timeout.
            for _ in range(3):
                try:
                    send("cleanup_stop", make_raw("cleanup_stop", 0, args.scale), False)
                    time.sleep(1.0 / args.hz)
                except Exception as exc:
                    error = error or f"Final stop failed: {type(exc).__name__}: {exc}"
                    break
            for sig, handler in handlers.items():
                signal.signal(sig, handler)
            log("summary", producer_completed=not stopped and error is None,
                interrupted=stopped, error=error, sent_frames=counters, reset_id=mapper.reset_id,
                physical_quest_verified=False, receiver_verified=False, simulator_verified=False,
                interpretation="Producer assertions only; correlate simulator trace and result separately")
    return 1 if error or stopped else 0


if __name__ == "__main__":
    raise SystemExit(main())
