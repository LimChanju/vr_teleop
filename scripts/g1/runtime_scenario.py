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

from g1_teleop.vr.calibration import OPENVR_TO_ROBOT, RawFrame, TargetMapper
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
POINT_NAMES = ("head", "left_hand", "right_hand")
AXIS_NAMES = ("x", "y", "z")
# Deterministic evaluation frequencies, recorded in the transmitted evidence.
SWEEP_FREQUENCIES_HZ = np.array([[0.31, 0.37, 0.23], [0.43, 0.29, 0.47], [0.41, 0.33, 0.53]])
SWEEP_AMPLITUDES_M = np.array([[0.025, 0.025, 0.04], [0.06] * 3, [0.06] * 3])
SWEEP_AXES = {f"sweep_{point}_{axis}": (point_index, axis_index)
              for point_index, point in enumerate(POINT_NAMES) for axis_index, axis in enumerate(AXIS_NAMES)}
SWEEP_PHASES = (("idle", 0.5, False), ("calibrate", 0.5, False), ("tracking_warmup", 2.0, True),
                *((name, 5.0, True) for name in SWEEP_AXES),
                ("tracking_combined", 10.0, True), ("final_stop", 1.0, False))


def phase_plan(phase_seconds=3.0, tracking_sweep=False):
    if tracking_sweep:
        return list(SWEEP_PHASES)
    return [(name, max(0.6, multiplier * phase_seconds) if name == "producer_pause"
             else multiplier * phase_seconds, expected) for name, multiplier, expected in PHASES]


def make_sweep_raw(phase, fraction, scale, duration):
    """Independent robot-frame axes, with smooth neutral endpoints and held grips."""
    active = phase == "tracking_warmup" or phase == "tracking_combined" or phase in SWEEP_AXES
    raw = make_raw("armed_motion" if active else phase, 0.0, scale)
    delta = np.zeros((3, 3))
    elapsed = float(np.clip(fraction, 0, 1)) * duration
    # 0.5 s cosine taper at both ends: position and velocity return to zero.
    ramp = min(1.0, max(0.0, elapsed / 0.5), max(0.0, (duration - elapsed) / 0.5))
    envelope = np.sin(np.pi * ramp / 2) ** 2
    axes = [SWEEP_AXES[phase]] if phase in SWEEP_AXES else (
        list(SWEEP_AXES.values()) if phase == "tracking_combined" else [])
    for point, axis in axes:
        amplitude = SWEEP_AMPLITUDES_M[point, axis] * (0.5 if phase == "tracking_combined" else 1.0)
        angle = 2 * np.pi * SWEEP_FREQUENCIES_HZ[point, axis] * elapsed
        wave = -0.5 * (1 - np.cos(angle)) if (point, axis) == (0, 2) else np.sin(angle)
        delta[point, axis] = amplitude * envelope * wave
    raw.poses[:, :, 3] += delta @ OPENVR_TO_ROBOT / scale
    return raw


def axis_tracking_metrics(target, actual, min_gain=0.5, min_correlation=0.6, min_excitation_m=0.005):
    """Zero-lag LS response, preserving offsets and errors as separate diagnostics."""
    target, actual = np.asarray(target, dtype=float), np.asarray(actual, dtype=float)
    report = {}
    for point, name in enumerate(POINT_NAMES):
        for axis, axis_name in enumerate(AXIS_NAMES):
            wanted, measured = target[:, point, axis], actual[:, point, axis]
            valid = np.isfinite(wanted) & np.isfinite(measured)
            wanted, measured = wanted[valid], measured[valid]
            if len(wanted) < 3 or np.ptp(wanted) < min_excitation_m:
                continue
            centered_target, centered_actual = wanted - wanted.mean(), measured - measured.mean()
            target_sum = float(centered_target @ centered_target)
            actual_sum = float(centered_actual @ centered_actual)
            cross = float(centered_target @ centered_actual)
            gain = cross / target_sum
            correlation = float(np.clip(cross / np.sqrt(target_sum * actual_sum), -1, 1)) if actual_sum > 1e-20 else 0.0
            difference = measured - wanted
            reasons = []
            if gain < min_gain:
                reasons.append("gain_below_provisional_minimum")
            if correlation < min_correlation:
                reasons.append("correlation_below_provisional_minimum")
            report[f"{name}.{axis_name}"] = dict(samples=len(wanted), ls_gain=gain,
                ls_intercept_m=float(measured.mean() - gain * wanted.mean()), correlation=correlation,
                target_range_m=float(np.ptp(wanted)), actual_range_m=float(np.ptp(measured)),
                range_ratio=float(np.ptp(measured) / np.ptp(wanted)),
                mean_abs_error_m=float(np.abs(difference).mean()), rmse_m=float(np.sqrt(np.mean(difference ** 2))),
                error_bias_m=float(difference.mean()), p95_abs_error_m=float(np.percentile(np.abs(difference), 95)),
                provisional_quality_passed=not reasons, flags=reasons)
    return report


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


def analyze_scenario(events_path, run_dir, grace_seconds=0.15, min_gain=0.5, min_correlation=0.6):
    """Check actual simulator trace against transmitted phases; no synthetic verdicts."""
    records = [json.loads(line) for line in Path(events_path).read_text().splitlines() if line.strip()]
    summaries = [r for r in records if r.get("kind") == "summary"]
    starts = [r for r in records if r.get("kind") == "start"]
    declared_plan = starts[0].get("phase_plan") if len(starts) == 1 else None
    if declared_plan is not None:
        if not isinstance(declared_plan, list) or not declared_plan:
            raise ValueError("Invalid declared phase plan")
        plan = []
        for item in declared_plan:
            if (not isinstance(item, dict) or not isinstance(item.get("phase"), str)
                    or item.get("expected_enabled") not in (True, False, None)):
                raise ValueError("Invalid declared phase state")
            plan.append((item["phase"], item.get("duration_s"), item.get("expected_enabled")))
        if len({name for name, _, _ in plan}) != len(plan):
            raise ValueError("Repeated phases are not supported")
    else:
        plan = PHASES  # Backward-compatible analysis of the original state scenario.
    scenario_mode = starts[0].get("scenario_mode", "control_state") if len(starts) == 1 else "control_state"
    active_phases = {name for name, _, enabled in plan if enabled}
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
    for name, _, declared_expected in plan:
        if name not in begins or name not in ends:
            check(f"phase_{name}", False, reason="missing phase begin/end event")
            continue
        start = begins[name]["timestamp_unix_ns"] / 1e9
        end = ends[name]["timestamp_unix_ns"] / 1e9
        expected = begins[name].get("expected_enabled", declared_expected)
        state_matches_plan = expected == declared_expected
        # Outage keeps the last frame fresh for receiver timeout (default0.25s).
        lower = start + grace_seconds + (0.25 if expected is None else 0.0)
        upper = end - grace_seconds
        mask = (wall >= lower) & (wall < upper)
        count = int(mask.sum())
        coverage = count >= 3 and wall[0] <= lower and wall[-1] >= upper and upper > lower
        wanted = False if expected is None else expected
        enabled_ok = count > 0 and bool(np.all(trace["enabled"][mask] == wanted))
        fresh_ok = count > 0 and bool(np.all(trace["input_fresh"][mask] == (expected is not None)))
        check(f"phase_{name}", coverage and enabled_ok and fresh_ok and state_matches_plan,
              samples=count, coverage=coverage, expected_enabled=wanted,
              declared_state_matches_event=state_matches_plan,
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
            if expected is True:
                tracking_mask = valid & trace["enabled"].astype(bool) & trace["input_fresh"].astype(bool)
                item["axis_tracking"] = axis_tracking_metrics(trace["target"][tracking_mask],
                    trace["actual"][tracking_mask], min_gain, min_correlation)
        phases[name] = item
    actual_resets = np.flatnonzero(trace["manual_reset"].astype(bool))
    reset_interval = None
    if "reset" in begins and "reset" in ends:
        reset_interval = (begins["reset"]["timestamp_unix_ns"] / 1e9,
                          ends["reset"]["timestamp_unix_ns"] / 1e9 + grace_seconds)
    expected_resets = int(any(name == "reset" for name, _, _ in plan))
    reset_timing_ok = expected_resets == 0 or (len(actual_resets) == 1 and reset_interval is not None
        and reset_interval[0] <= wall[actual_resets[0]] <= reset_interval[1])
    check("one_manual_reset" if expected_resets else "no_manual_reset",
          len(actual_resets) == expected_resets and result.get("manual_resets") == expected_resets and reset_timing_ok,
          expected=expected_resets, trace_count=len(actual_resets), result_count=result.get("manual_resets"))
    active = np.flatnonzero(trace["enabled"].astype(bool))
    unmatched, wrong_phase, target_mismatch = [], [], []
    errors = []
    for index in active:
        seq = int(trace["input_sequence"][index])
        packet = sent.get(seq)
        if packet is None:
            unmatched.append(int(index))
            continue
        if packet["phase"] not in active_phases or not packet["target"]["enabled"]:
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
    # Separate control-state correctness from provisional tracking response.
    response = {}
    valid_active = active[valid_metrics[active] & trace["input_fresh"][active].astype(bool)]
    if len(valid_active):
        response = dict(actual_keypoint_range_m=np.ptp(trace["actual"][valid_active], axis=0).tolist(),
                        target_keypoint_range_m=np.ptp(trace["target"][valid_active], axis=0).tolist(),
                        max_joint_range_rad=float(np.ptp(trace["q"][valid_active], axis=0).max()))
    axis_tracking = axis_tracking_metrics(trace["target"][valid_active], trace["actual"][valid_active],
                                         min_gain, min_correlation)
    quality_items = axis_tracking
    if scenario_mode == "tracking_sweep":
        # Each independent axis must respond during its own excitation block;
        # pooled combined movement cannot mask a failed isolated axis.
        quality_items = {}
        for phase, (point, axis) in SWEEP_AXES.items():
            key = f"{POINT_NAMES[point]}.{AXIS_NAMES[axis]}"
            item = phases.get(phase, {}).get("axis_tracking", {}).get(key)
            quality_items[key] = item or dict(provisional_quality_passed=False,
                                               flags=["missing_independent_axis_evidence"])
    quality_passed = bool(quality_items) and all(item["provisional_quality_passed"] for item in quality_items.values())
    control_passed = all(item["passed"] for item in checks.values())
    return dict(schema="g1.runtime.scenario.analysis.v1", events=str(events_path), run=str(run_dir),
                grace_seconds=grace_seconds, receiver_timeout_assumed_s=0.25,
                scenario_mode=scenario_mode, simulator_checks_passed=control_passed,
                control_state_passed=control_passed, tracking_quality_passed=quality_passed,
                control_and_tracking_passed=control_passed and quality_passed,
                physical_quest_verified=False, checks=checks, phases=phases, response=response,
                metric_reset_guard_frames=1, axis_tracking=axis_tracking,
                tracking_quality=dict(provisional=True, min_ls_gain=min_gain, min_correlation=min_correlation,
                    min_excitation_range_m=0.005, comparison="zero-lag actual versus desired; LS fit includes intercept",
                    axis_results=quality_items, flagged_axes=[key for key, item in quality_items.items()
                                                             if not item["provisional_quality_passed"]]),
                tracking_quality_status=("Provisional response thresholds met on excited axes only; not hardware/task validation"
                    if quality_passed else "Provisional tracking response failed or insufficient excitation; inspect axis metrics"))


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
    parser.add_argument("--tracking-sweep", action="store_true",
                        help="59s independent head/hand XYZ tracking scenario instead of state-transition scenario")
    parser.add_argument("--analyze-run", type=Path, help="analysis only: compare --output event JSONL with this simulator run")
    parser.add_argument("--analysis-output", type=Path, help="new report JSON (default RUN/scenario_analysis.json)")
    parser.add_argument("--grace-seconds", type=float, default=0.15,
                        help="explicit phase-boundary latency allowance for trace analysis")
    parser.add_argument("--min-gain", type=float, default=0.5, help="provisional per-axis LS gain minimum")
    parser.add_argument("--min-correlation", type=float, default=0.6, help="provisional zero-lag correlation minimum")
    parser.add_argument("--require-tracking-quality", action="store_true",
                        help="analysis exits nonzero if provisional tracking quality fails (state checks always required)")
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
    if not np.isfinite(args.min_gain) or not 0 <= args.min_gain <= 2:
        parser.error("min-gain must be finite and between 0 and 2")
    if not np.isfinite(args.min_correlation) or not 0 <= args.min_correlation <= 1:
        parser.error("min-correlation must be finite and between 0 and 1")
    if args.require_tracking_quality and not args.analyze_run:
        parser.error("--require-tracking-quality requires --analyze-run")
    return args


def main(argv=None):
    args = parse_args(argv)
    if args.analyze_run:
        destination = args.analysis_output or args.analyze_run / "scenario_analysis.json"
        destination.parent.mkdir(parents=True, exist_ok=True)
        with destination.open("x", encoding="utf-8") as record:
            try:
                report = analyze_scenario(args.output, args.analyze_run, args.grace_seconds,
                                          args.min_gain, args.min_correlation)
            except (OSError, ValueError, KeyError, TypeError) as exc:
                report = dict(simulator_checks_passed=False, error=f"{type(exc).__name__}: {exc}",
                              physical_quest_verified=False)
            json.dump(report, record, indent=2, allow_nan=False)
            record.write("\n")
        print(json.dumps(report, indent=2, allow_nan=False))
        passed = report["simulator_checks_passed"] and (
            not args.require_tracking_quality or report.get("tracking_quality_passed", False))
        return 0 if passed else 1
    mapper = TargetMapper(nominal=load_nominal(args.nominal), scale=args.scale)
    destination = (socket.gethostbyname(args.host), args.port)
    plan = phase_plan(args.phase_seconds, args.tracking_sweep)
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
                scenario_mode="tracking_sweep" if args.tracking_sweep else "control_state",
                phase_plan=[dict(phase=name, duration_s=duration, expected_enabled=enabled)
                            for name, duration, enabled in plan],
                destination=list(destination), phase_seconds=args.phase_seconds, hz=args.hz,
                scale=args.scale, robot_hand_amplitude_m=0.06,
                sweep_frequencies_hz=SWEEP_FREQUENCIES_HZ.tolist() if args.tracking_sweep else None,
                sweep_amplitudes_m=SWEEP_AMPLITUDES_M.tolist() if args.tracking_sweep else None,
                nominal_file=str(args.nominal.resolve()),
                nominal_sha256=hashlib.sha256(args.nominal.read_bytes()).hexdigest(),
                physical_quest_verified=False, receiver_verified=False)
            for phase, duration, expected in plan:
                if stopped:
                    break
                phase_start = time.monotonic()
                next_tick = phase_start
                log("phase_begin", phase=phase, duration_s=duration, expected_enabled=expected,
                    sends_packets=expected is not None)
                print(f"[SCENARIO] {phase}: {duration:.2f}s", flush=True)
                while not stopped and time.monotonic() - phase_start < duration:
                    now = time.monotonic()
                    if expected is not None:
                        fraction = np.clip((now - phase_start) / duration, 0.0, 1.0)
                        raw = (make_sweep_raw(phase, fraction, args.scale, duration) if args.tracking_sweep
                               else make_raw(phase, fraction, args.scale))
                        send(phase, raw, expected)
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
