"""Capture calibration + held-out validation without installing either result.

Only the first phase estimates A and B in Q = A V B. The subsequent phase
tests those exact matrices, including the fixture offset B, without refitting.
No upstream source, active calibration, Redis stream or robot is modified.
"""

from __future__ import annotations

import argparse
from collections import Counter
from contextlib import ExitStack
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
import traceback

import numpy as np

from .models import PoseSample
from .providers.openvr import OpenVRProvider
from .providers.vive_linux import ViveUltimateLinuxProvider
from .spatial_alignment import pose_to_matrix, solve_hand_eye, validate_transform
from .spatial_check import announce


ROOT = Path(__file__).resolve().parents[1]
TRACKER_ROLE = "Calibration_Tracker"
PHASES = ("calibration", "validation")


def pose_record(pose):
    if pose is None:
        return None
    return {
        "device_id": pose.device_id, "role": pose.role,
        "position_m": pose.position_m.tolist(),
        "orientation_xyzw": pose.orientation_xyzw.tolist(),
        "timestamp_s": pose.timestamp_s, "valid": pose.valid,
        "tracking_state": pose.tracking_state, "metadata": dict(pose.metadata),
    }


def rejection_reasons(record, *, tracker_serial, quest_role, max_stale, max_skew):
    reasons = []
    if (record["quest_coordinate_frame"] != "openvr_standing"
            or record["vive_coordinate_frame"] != "vive_hub_world"):
        reasons.append("unexpected_coordinate_frame")
    for source, role in (("quest", quest_role), ("vive", TRACKER_ROLE)):
        pose = record[source]
        if pose is None:
            reasons.append(source + "_missing")
            continue
        if not pose["valid"]:
            reasons.append(source + "_invalid_tracking")
        if pose["role"] != role:
            reasons.append(source + "_wrong_role")
        if source == "vive" and pose["device_id"] != tracker_serial:
            reasons.append("wrong_tracker_serial")
        age = record["timestamp_s"] - pose["timestamp_s"]
        if not math.isfinite(age) or not -0.01 <= age <= max_stale:
            reasons.append(source + "_stale_or_future")
    if record["quest"] is not None and record["vive"] is not None:
        skew = abs(record["quest"]["timestamp_s"] - record["vive"]["timestamp_s"])
        if not math.isfinite(skew) or skew > max_skew:
            reasons.append("timestamp_skew")
    return reasons


def select_pairs(records, **quality):
    """Reject stale, unsynchronized, repeated and wrong-identity observations."""
    identities = {source: set() for source in ("quest", "vive")}
    states = {source: Counter() for source in identities}
    discarded = Counter()
    quest, vive, times, skews = [], [], [], []
    previous = (-math.inf, -math.inf)
    for record in records:
        for source in identities:
            pose = record[source]
            states[source][pose["tracking_state"] if pose else "missing"] += 1
            if pose:
                identities[source].add((pose["device_id"],
                                        pose["metadata"].get("connection_generation", 0)))
        reasons = rejection_reasons(record, **quality)
        if reasons:
            discarded.update(reasons)
            continue
        timestamps = tuple(record[s]["timestamp_s"] for s in ("quest", "vive"))
        if any(now <= before for now, before in zip(timestamps, previous)):
            discarded["duplicate_or_out_of_order_pose"] += 1
            continue
        try:
            matrices = [pose_to_matrix(PoseSample(**record[s])) for s in ("quest", "vive")]
        except (ValueError, TypeError):
            discarded["malformed_pose"] += 1
            continue
        previous = timestamps
        quest.append(matrices[0])
        vive.append(matrices[1])
        times.append(timestamps)
        skews.append(abs(timestamps[0] - timestamps[1]))
    return quest, vive, {
        "frame_count": len(records), "valid_pairs": len(quest),
        "valid_fraction": len(quest) / max(1, len(records)),
        "tracking_states": {s: dict(c) for s, c in states.items()},
        "device_identities": {s: [list(x) for x in sorted(v)] for s, v in identities.items()},
        "discarded_reasons": dict(discarded),
        "max_pair_skew_s": max(skews, default=None),
        "mean_pair_skew_s": float(np.mean(skews)) if skews else None,
        "first_pose_timestamps_s": list(times[0]) if times else None,
        "last_pose_timestamps_s": list(times[-1]) if times else None,
    }


def residual_metrics(quest, vive, world, fixture):
    """Evaluate fixed transforms on ALL pairs; never optimize on validation."""
    if not quest or len(quest) != len(vive):
        raise ValueError("residuals need nonempty, equally sized pose lists")
    world = validate_transform(world)
    fixture = validate_transform(fixture)
    position, orientation = [], []
    for q, v in zip(quest, vive):
        predicted = world @ v @ fixture
        position.append(float(np.linalg.norm(q[:3, 3] - predicted[:3, 3])))
        relative = q[:3, :3].T @ predicted[:3, :3]
        orientation.append(float(np.degrees(np.arccos(np.clip((np.trace(relative) - 1) / 2, -1, 1)))))
    return {
        "sample_count": len(quest),
        "position_rmse_m": float(np.sqrt(np.mean(np.square(position)))),
        "position_p95_m": float(np.percentile(position, 95)),
        "position_max_m": max(position),
        "orientation_rmse_deg": float(np.sqrt(np.mean(np.square(orientation)))),
        "orientation_p95_deg": float(np.percentile(orientation, 95)),
        "orientation_max_deg": max(orientation),
    }


def motion_metrics(poses):
    if not poses:
        return {"position_bbox_diagonal_m": 0.0, "max_rotation_from_start_deg": 0.0}
    points = np.asarray([p[:3, 3] for p in poses])
    angles = [np.degrees(np.arccos(np.clip((np.trace(poses[0][:3, :3].T @ p[:3, :3]) - 1) / 2,
                                         -1, 1))) for p in poses]
    return {"position_bbox_diagonal_m": float(np.linalg.norm(np.ptp(points, axis=0))),
            "max_rotation_from_start_deg": float(max(angles))}


def analyse_session(stages, args):
    quality = dict(tracker_serial=args.tracker_serial,
                   quest_role="Right_Wrist" if args.quest_hand == "right" else "Left_Wrist",
                   max_stale=args.max_stale, max_skew=args.max_skew)
    selected = {stage: select_pairs(stages.get(stage, []), **quality) for stage in PHASES}
    issues = []
    report = {
        "schema_version": 1, "status": "needs_review", "held_out_validation_passed": False,
        "candidate_written": False, "active_calibration_changed": False,
        "simulation_publish_enabled": False, "mounts_calibrated": False,
        "scope": "One rigid fixture and its sampled trajectories only; not full-body validation.",
        "thresholds": {
            "max_position_rmse_m": args.max_position_rmse,
            "max_orientation_rmse_deg": args.max_orientation_rmse,
            "min_valid_fraction": args.min_valid_fraction,
            "max_pose_age_s": args.max_stale, "max_pair_skew_s": args.max_skew,
            "min_validation_translation_m": args.min_validation_translation,
            "min_validation_rotation_deg": args.min_validation_rotation,
        },
        "phases": {stage: data[2] for stage, data in selected.items()}, "issues": issues,
    }
    sessions = {r.get("vive_session_id") for rows in stages.values() for r in rows}
    report["vive_session_ids"] = sorted(sessions, key=str)
    if len(sessions) != 1 or None in sessions:
        issues.append("missing_or_changed_vive_server_session")
    for stage, (q, v, summary) in selected.items():
        if len(q) < 8 or summary["valid_fraction"] < args.min_valid_fraction:
            issues.append(stage + ": insufficient_fresh_unique_pairs")
        if any(len(values) != 1 for values in summary["device_identities"].values()):
            issues.append(stage + ": missing_or_changed_identity_or_generation")
        summary["motion"] = {"quest": motion_metrics(q), "vive": motion_metrics(v)}
    for source in ("quest", "vive"):
        identities = {tuple(x) for data in selected.values()
                      for x in data[2]["device_identities"][source]}
        if len(identities) != 1:
            issues.append(source + ": identity_or_generation_changed_across_phases")
    fit_q, fit_v, fit_summary = selected["calibration"]
    val_q, val_v, val_summary = selected["validation"]
    if fit_q and val_q and any(b <= a for a, b in zip(
            fit_summary["last_pose_timestamps_s"], val_summary["first_pose_timestamps_s"])):
        issues.append("validation_reuses_or_precedes_calibration_samples")
    # A stationary hold-out cannot validate spatial tracking. These are documented
    # diagnostic motion-coverage checks, not robot motion or joint limits.
    for source in ("quest", "vive"):
        motion = val_summary["motion"][source]
        if (motion["position_bbox_diagonal_m"] < args.min_validation_translation
                or motion["max_rotation_from_start_deg"] < args.min_validation_rotation):
            issues.append("validation: " + source + "_motion_coverage_too_small")
    if len(fit_q) < 8:
        return report, None
    # Evenly span calibration only. The existing solver enumerates O(N^2) pairs;
    # capping solver inputs avoids a long pause. Score ALL accepted samples below.
    indices = np.linspace(0, len(fit_q) - 1, min(args.fit_max_samples, len(fit_q)), dtype=int)
    try:
        result = solve_hand_eye([fit_q[i] for i in indices], [fit_v[i] for i in indices])
    except (ValueError, np.linalg.LinAlgError) as error:
        issues.append("calibration_solver: " + str(error))
        return report, None
    report["solver"] = {"input_samples": len(indices), "pair_count": result.pair_count,
                        "rotation_system_rank": result.rotation_system_rank,
                        "translation_system_rank": result.translation_system_rank,
                        "used_validation_samples": False}
    report["estimated_transform_for_diagnostics"] = {
        "source_to_target_matrix": result.vive_to_quest.tolist(),
        "tracker_from_controller_matrix": result.tracker_from_controller.tolist(),
    }
    for stage, (q, v, summary) in selected.items():
        if not q:
            continue
        metrics = residual_metrics(q, v, result.vive_to_quest, result.tracker_from_controller)
        summary["residuals"] = metrics
        if metrics["position_rmse_m"] > args.max_position_rmse:
            issues.append(stage + ": position_rmse_exceeds_threshold")
        if metrics["orientation_rmse_deg"] > args.max_orientation_rmse:
            issues.append(stage + ": orientation_rmse_exceeds_threshold")
    if issues:
        return report, None
    report["held_out_validation_passed"] = True
    report["status"] = "passed_for_sampled_fixture"
    now = datetime.now(timezone.utc)
    candidate = {
        "schema_version": 1, "calibrated": True,
        "calibration_id": "quest-vive-" + now.strftime("%Y%m%dT%H%M%S_%fZ"),
        "calibrated_at": now.isoformat(),
        "source_frame": "vive_hub_world", "target_frame": "openvr_standing",
        "source_to_target_matrix": result.vive_to_quest.tolist(),
        "calibration_fixture": {
            "tracker_serial": args.tracker_serial, "quest_role": quality["quest_role"],
            "tracker_from_controller_matrix": result.tracker_from_controller.tolist(),
        },
        "fit": {**fit_summary["residuals"], **report["solver"]},
        "held_out_validation": val_summary["residuals"],
        "installed": False,
        "warning": "Review before installation. Recheck if tracking origins/maps change. "
                   "This does not calibrate body mounts or certify all five trackers.",
    }
    return report, candidate


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tracker-serial", required=True)
    parser.add_argument("--quest-hand", choices=("left", "right"), default="right")
    parser.add_argument("--countdown", type=int, default=15)
    parser.add_argument("--fit-seconds", type=float, default=20.0)
    parser.add_argument("--validation-seconds", type=float, default=12.0)
    parser.add_argument("--frequency", type=float, default=60.0)
    parser.add_argument("--ready-timeout", type=float, default=30.0)
    parser.add_argument("--fit-max-samples", type=int, default=240)
    parser.add_argument("--max-stale", type=float, default=0.20)
    parser.add_argument("--max-skew", type=float, default=0.05)
    parser.add_argument("--min-valid-fraction", type=float, default=0.80)
    parser.add_argument("--max-position-rmse", type=float, default=0.030)
    parser.add_argument("--max-orientation-rmse", type=float, default=5.0)
    parser.add_argument("--min-validation-translation", type=float, default=0.10)
    parser.add_argument("--min-validation-rotation", type=float, default=15.0)
    parser.add_argument("--speech", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--action-manifest", type=Path, default=ROOT / "config/openvr_actions.json")
    parser.add_argument("--vive-socket", type=Path, default=Path("/tmp/vut.sock"))
    parser.add_argument("--output-root", type=Path, default=ROOT / "runtime/alignment_check")
    return parser


def validate_args(parser, args):
    args.tracker_serial = args.tracker_serial.strip()
    if not args.tracker_serial or args.tracker_serial.startswith("REPLACE_WITH_"):
        parser.error("use the real tracker serial")
    for name, lower, upper in (
        ("fit_seconds", 1, 120), ("validation_seconds", 1, 120), ("frequency", 10, 120),
        ("ready_timeout", 1, 120), ("min_valid_fraction", 0.01, 1),
        ("max_stale", 0.001, 1), ("max_skew", 0.001, 1),
        ("max_position_rmse", 0.0001, 1), ("max_orientation_rmse", 0.001, 180),
        ("min_validation_translation", 0.001, 2), ("min_validation_rotation", 0.1, 180),
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or not lower <= value <= upper:
            parser.error(f"--{name.replace('_', '-')} must be finite and in [{lower}, {upper}]")
    if not 0 <= args.countdown <= 60 or not 8 <= args.fit_max_samples <= 400:
        parser.error("countdown must be 0..60; fit-max-samples must be 8..400")


def write_json(path, value):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, allow_nan=False, default=str)
        stream.write("\n")


def capture(quest, vive, args, output, log):
    """One connection, continuous fixture, raw frames flushed even on failure."""
    quest_role = "Right_Wrist" if args.quest_hand == "right" else "Left_Wrist"
    quality = dict(tracker_serial=args.tracker_serial, quest_role=quest_role,
                   max_stale=args.max_stale, max_skew=args.max_skew)
    stages = {stage: [] for stage in PHASES}
    with (output / "poses.jsonl").open("x", encoding="utf-8") as stream:
        def sample(stage):
            q, v = quest.read(), vive.read()
            record = {
                "schema_version": 1, "stage": stage, "timestamp_s": time.monotonic(),
                "quest_coordinate_frame": q.coordinate_frame, "vive_coordinate_frame": v.coordinate_frame,
                "quest_frame_timestamp_s": q.timestamp_s, "vive_frame_timestamp_s": v.timestamp_s,
                "vive_session_id": vive.client.session_id,
                "quest": pose_record(q.poses.get(quest_role)),
                "vive": pose_record(v.poses.get(TRACKER_ROLE)),
            }
            record["rejected_reasons"] = rejection_reasons(record, **quality)
            stream.write(json.dumps(record, allow_nan=False) + "\n")
            stream.flush()
            return record

        log(f"Waiting for {args.tracker_serial} and {quest_role} tracking (up to {args.ready_timeout:g}s).")
        deadline = time.monotonic() + args.ready_timeout
        ready_since = None
        while time.monotonic() < deadline:
            record = sample("waiting_for_tracking")
            if record["rejected_reasons"]:
                ready_since = None
            elif ready_since is None:
                ready_since = time.monotonic()
            elif time.monotonic() - ready_since >= 0.5:
                break
            time.sleep(1 / args.frequency)
        else:
            raise RuntimeError("both devices did not maintain fresh tracking; inspect poses.jsonl")

        announce("Prepare. Keep the tracker and controller firmly together. Do not change your grip.",
                 speech=args.speech)
        deadline = time.monotonic() + args.countdown
        previous = None
        while time.monotonic() < deadline:
            remaining = math.ceil(deadline - time.monotonic())
            if remaining != previous:
                log(f"Starting in {remaining}...")
                previous = remaining
            sample("countdown")
            time.sleep(1 / args.frequency)

        for stage, seconds, instruction in (
            ("calibration", args.fit_seconds,
             "Calibration. Move slowly in a figure eight and tilt in several directions."),
            ("validation", args.validation_seconds,
             "Validation. Keep the same grip. Continue moving to different positions and angles."),
        ):
            log(f"[{stage.upper()}] {seconds:g}s: {instruction}")
            announce(instruction, speech=args.speech)
            deadline = time.monotonic() + seconds
            previous = None
            while time.monotonic() < deadline:
                started = time.monotonic()
                record = sample(stage)
                stages[stage].append(record)
                remaining = math.ceil(deadline - time.monotonic())
                if remaining != previous and remaining % 5 == 0:
                    log(f"{stage}: {remaining}s left; states=" + ", ".join(
                        f"{s}:{record[s]['tracking_state'] if record[s] else 'missing'}"
                        for s in ("quest", "vive")))
                    previous = remaining
                time.sleep(max(0.0, 1 / args.frequency - (time.monotonic() - started)))
    log("CAPTURE FINISHED. You can put the devices down; calculating now.")
    announce("Capture finished. You can put the devices down.", speech=args.speech)
    return stages


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)
    now = datetime.now(timezone.utc)
    output = args.output_root / now.strftime("%Y%m%dT%H%M%S_%fZ")
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / "settings.json", vars(args))
    # Preserve the exact active configuration for later comparison; never modify it.
    active = ROOT / "config/local/calibration.quest_vive.json"
    if active.exists():
        with (output / "active_calibration_snapshot.json").open("x", encoding="utf-8") as stream:
            stream.write(active.read_text(encoding="utf-8"))
    report = {
        "schema_version": 1, "status": "failed", "held_out_validation_passed": False,
        "candidate_written": False, "active_calibration_changed": False,
        "simulation_publish_enabled": False, "mounts_calibrated": False,
    }
    exit_code = 2
    with (output / "session.log").open("x", encoding="utf-8") as log_stream:
        def log(message):
            print(message, flush=True)
            log_stream.write(message + "\n")
            log_stream.flush()

        log(f"Output: {output}")
        log("DIAGNOSTIC ONLY: existing calibrations unchanged; no robot or Redis output.")
        log("Both stages use ONE unchanged rigid grip. Keep all five trackers on. No Enter between stages.")
        log("Speech uses PC audio and may not be audible in Quest; use a helper or --no-speech.")
        log(f"After tracking is ready: {args.countdown}s preparation, {args.fit_seconds:g}s calibration, "
            f"then {args.validation_seconds:g}s held-out validation. Keep moving gently through BOTH stages.")
        try:
            with ExitStack() as stack:
                quest = stack.enter_context(OpenVRProvider({}, action_manifest=args.action_manifest))
                vive = stack.enter_context(ViveUltimateLinuxProvider(
                    {args.tracker_serial: TRACKER_ROLE}, socket_path=args.vive_socket,
                    max_pose_age_s=args.max_stale))
                stages = capture(quest, vive, args, output, log)
            report, candidate = analyse_session(stages, args)
            for stage, data in report["phases"].items():
                log(f"{stage}: fresh unique pairs {data['valid_pairs']}/{data['frame_count']}")
                if "residuals" in data:
                    metrics = data["residuals"]
                    log(f"{stage}: position RMSE={1000 * metrics['position_rmse_m']:.1f}mm, "
                        f"orientation RMSE={metrics['orientation_rmse_deg']:.2f}deg")
            if candidate is not None:
                write_json(output / "candidate.json", candidate)
                report["candidate_written"] = True
                log("PASS for this fixture's sampled motion. Candidate saved here, NOT installed.")
                exit_code = 0
            else:
                log("NEEDS REVIEW: " + "; ".join(report["issues"]))
                log("No candidate written for this run. Raw poses and diagnostics are retained.")
        except KeyboardInterrupt:
            report["status"] = "interrupted"
            report["error"] = "operator interrupted capture/analysis"
            log("Interrupted. Raw records already written are preserved.")
            exit_code = 130
        except Exception as error:
            report["status"] = "failed"
            report["error"] = f"{type(error).__name__}: {error}"
            log(traceback.format_exc())
        report["started_at"] = now.isoformat()
        report["finished_at"] = datetime.now(timezone.utc).isoformat()
        report["tracker_serial"] = args.tracker_serial
        report["quest_hand"] = args.quest_hand
        write_json(output / "summary.json", report)
        log(f"Saved {output / 'summary.json'}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
