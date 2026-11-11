"""Read-only Quest/VIVE proximity check; never install calibration or drive a robot.

The operator holds a Touch controller close to an attached VIVE tracker.
Distance between their reported origins can expose gross space misalignment.
One nearby pair cannot validate orientation or recover a full rigid transform.
"""

from __future__ import annotations

import argparse
from collections import Counter
from contextlib import ExitStack
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import shutil
import subprocess
import time

import numpy as np

from .cli import CONFIG, ROOT, _vive_linux_mapping
from .config import load_json
from .models import TrackerFrame
from .providers.hybrid import QuestViveHybridProvider
from .providers.openvr import OpenVRProvider
from .providers.replay import frame_to_dict
from .providers.vive_linux import ViveUltimateLinuxProvider


VIVE_ROLES = ("Pelvis", "Left_Knee", "Right_Knee", "Left_Foot", "Right_Foot")
QUEST_ROLES = ("Left_Wrist", "Right_Wrist")
BATCH_ROLES = ("Pelvis", "Right_Knee", "Right_Foot", "Left_Foot", "Left_Knee")
ROLE_LABELS = {
    "Pelvis": ("배", "belly"),
    "Right_Knee": ("오른쪽 정강이", "right shin"),
    "Right_Foot": ("오른쪽 발등", "right foot"),
    "Left_Foot": ("왼쪽 발등", "left foot"),
    "Left_Knee": ("왼쪽 정강이", "left shin"),
}


def summarize_proximity(
    frames: list[TrackerFrame],
    *,
    vive_role: str = "Pelvis",
    quest_role: str = "Right_Wrist",
    max_separation: float = 0.15,
    max_stale: float = 0.20,
    max_skew: float = 0.05,
    max_position_std: float = 0.015,
) -> dict:
    if not frames:
        raise ValueError("no frames captured")
    if vive_role not in VIVE_ROLES or quest_role not in QUEST_ROLES:
        raise ValueError("select one VIVE body role and one Quest wrist")
    if any(not math.isfinite(v) or v <= 0 for v in
           (max_separation, max_stale, max_skew, max_position_std)):
        raise ValueError("diagnostic thresholds must be finite and positive")
    if any(f.coordinate_frame != "openvr_standing" for f in frames):
        raise ValueError("proximity check requires aligned openvr_standing frames")

    roles = (quest_role, vive_role)
    identities = {r: set() for r in roles}
    states = {r: Counter() for r in roles}
    pairs = []
    for frame in frames:
        poses = [frame.poses.get(r) for r in roles]
        for role, pose in zip(roles, poses):
            states[role][pose.tracking_state if pose else "missing"] += 1
            if pose is not None:
                identities[role].add((pose.device_id,
                                     pose.metadata.get("connection_generation", 0),
                                     pose.metadata.get("spatial_calibration_id", "")))
        if not all(p is not None and p.valid
                   and -0.01 <= frame.timestamp_s - p.timestamp_s <= max_stale
                   for p in poses):
            continue
        if abs(poses[0].timestamp_s - poses[1].timestamp_s) > max_skew:
            continue
        pairs.append(poses)

    result = {
        "schema_version": 1,
        "purpose": "operator_confirmed_nearby_devices_position_check_only",
        "coordinate_frame": "openvr_standing",
        "quest_role": quest_role,
        "vive_role": vive_role,
        "frame_count": len(frames),
        "valid_pairs": len(pairs),
        "valid_fraction": len(pairs) / len(frames),
        "tracking_states": {r: dict(s) for r, s in states.items()},
        "device_identities": {r: [list(x) for x in sorted(v)] for r, v in identities.items()},
        "thresholds": {"max_separation_m": max_separation, "max_stale_s": max_stale,
                       "max_pair_skew_s": max_skew, "max_position_std_m": max_position_std,
                       "minimum_valid_fraction": 0.8},
        "proximity_consistent": False,
        "mounts_calibrated": False,
        "full_spatial_alignment_verified": False,
        "issues": [],
        "warning": (
            "Interpret only if the controller was physically held next to the selected tracker. "
            "Reported device origins are not housing contact points or anatomical joints. "
            "This is a coarse position check, not an orientation or mount calibration. "
            "No transform was changed and no robot command was sent."
        ),
    }
    if len(pairs) < 2 or result["valid_fraction"] < 0.8:
        result["issues"].append("insufficient_fresh_synchronized_tracking")
    if any(len(v) != 1 for v in identities.values()):
        result["issues"].append("missing_or_changed_device_identity_or_generation")
    if pairs:
        points = np.array([[p.position_m for p in pair] for pair in pairs])
        delta = points[:, 0] - points[:, 1]
        distances = np.linalg.norm(delta, axis=1)
        std = points.std(axis=0)
        result.update({
            "mean_quest_minus_vive_m": delta.mean(axis=0).tolist(),
            "mean_separation_m": float(distances.mean()),
            "max_separation_m": float(distances.max()),
            "position_std_m": {r: std[i].tolist() for i, r in enumerate(roles)},
        })
        if float(std.max()) > max_position_std:
            result["issues"].append("devices_not_held_still")
        if float(distances.max()) > max_separation:
            result["issues"].append("reported_positions_too_far_apart_for_nearby_devices")
    result["proximity_consistent"] = not result["issues"]
    return result


def summarize_batch(stages: list[dict], expected_roles: tuple[str, ...]) -> dict:
    """Aggregate all stages, including failures, without granting calibration."""
    issues = []
    if [s["vive_role"] for s in stages] != list(expected_roles):
        issues.append("missing_or_unexpected_stage")
    if any(not s["proximity_consistent"] for s in stages):
        issues.append("one_or_more_stages_need_review")
    identities: dict[str, set] = {}
    for stage in stages:
        for role, values in stage["device_identities"].items():
            identities.setdefault(role, set()).update(tuple(v) for v in values)
    if any(len(values) != 1 for values in identities.values()):
        issues.append("device_identity_or_generation_changed_between_stages")
    return {
        "schema_version": 1,
        "purpose": "sequential_nearby_device_checks_only",
        "expected_roles": list(expected_roles),
        "stages": stages,
        "issues": issues,
        "all_proximity_checks_consistent": not issues,
        "mounts_calibrated": False,
        "full_spatial_alignment_verified": False,
        "simulation_publish_enabled": False,
        "warning": "Nearby-device checks do not measure tracker mounts or validate full-body robot control.",
    }


def announce(message: str, *, speech: bool) -> None:
    """Optional OS speech; no installation, global audio changes, or VR writes."""
    if not speech:
        return
    program = shutil.which("spd-say")
    if program is None:
        return
    try:
        # Queue speech without waiting for playback; keep draining pose queues.
        subprocess.run([program, "-l", "en", "-N", "twist2-position-check", message],
                       check=True, timeout=2, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except (OSError, subprocess.SubprocessError):
        print("Speech unavailable; follow the terminal stage/countdown messages.", flush=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mapping", type=Path,
                        default=CONFIG / "local/tracker_mapping.vive_linux.json")
    parser.add_argument("--cross-calibration", type=Path,
                        default=CONFIG / "local/calibration.quest_vive.json")
    parser.add_argument("--action-manifest", type=Path, default=CONFIG / "openvr_actions.json")
    parser.add_argument("--vive-socket", type=Path, default=Path("/tmp/vut.sock"))
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--vive-role", choices=VIVE_ROLES,
                           help="single tracker to check (default: Pelvis)")
    selection.add_argument("--all-trackers", action="store_true",
                           help="check all five attached trackers in one guided session")
    parser.add_argument("--quest-hand", choices=("left", "right"), default="right")
    parser.add_argument("--speech", action=argparse.BooleanOptionalAction, default=False,
                        help="optional English stage announcements on the PC audio output")
    parser.add_argument("--countdown", type=int, default=10)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--frequency", type=float, default=50.0)
    parser.add_argument("--max-stale", type=float, default=0.20)
    parser.add_argument("--max-skew", type=float, default=0.05)
    parser.add_argument("--max-separation", type=float, default=0.15,
                        help="coarse diagnostic distance in meters, NOT a robot motion limit")
    parser.add_argument("--output-root", type=Path, default=ROOT / "runtime/spatial_check")
    return parser


def capture_stage(provider, args, *, role: str, serial: str, index: int, total: int):
    label, spoken_label = ROLE_LABELS[role]
    print(f"\n[{index}/{total}] {label}: {role} ({serial})", flush=True)
    print(f"Hold the {args.quest_hand} Touch controller BODY next to this tracker.", flush=True)
    announce(f"Step {index}. {spoken_label}. Move your {args.quest_hand} controller next to the tracker.",
             speech=args.speech)
    deadline = time.monotonic() + args.countdown
    previous = None
    while time.monotonic() < deadline:
        remaining = math.ceil(deadline - time.monotonic())
        if remaining != previous:
            print(f"Starting in {remaining}...", flush=True)
            previous = remaining
        provider.read()
        time.sleep(1.0 / args.frequency)
    print(f"Hold still for {args.seconds:g} seconds...", flush=True)
    announce("Hold still.", speech=args.speech)
    frames = []
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        started = time.monotonic()
        frames.append(provider.read())
        time.sleep(max(0.0, 1.0 / args.frequency - (time.monotonic() - started)))
    return frames


def save_stage(output: Path, frames: list[TrackerFrame], result: dict) -> None:
    output.mkdir(parents=True, exist_ok=False)
    with (output / "poses.jsonl").open("x", encoding="utf-8") as stream:
        for frame in frames:
            record = frame_to_dict(frame)
            for role, pose in frame.poses.items():
                record["poses"][role]["timestamp_s"] = pose.timestamp_s
            stream.write(json.dumps(record) + "\n")
    with (output / "summary.json").open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2)
        stream.write("\n")


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if (not 0 <= args.countdown <= 60 or not 0 < args.seconds <= 60
            or not 1 <= args.frequency <= 120
            or any(not math.isfinite(v) or v <= 0 for v in
                   (args.max_stale, args.max_skew, args.max_separation))):
        parser.error("use countdown in [0,60], seconds in (0,60], frequency in [1,120], positive finite thresholds")
    mapping = _vive_linux_mapping(args.mapping)
    alignment = load_json(args.cross_calibration)
    quest_role = "Left_Wrist" if args.quest_hand == "left" else "Right_Wrist"
    roles = BATCH_ROLES if args.all_trackers else (args.vive_role or "Pelvis",)
    now = datetime.now(timezone.utc)
    output = args.output_root / now.strftime("%Y%m%dT%H%M%S_%fZ")
    stages = []
    with ExitStack() as stack:
        quest = stack.enter_context(OpenVRProvider({}, action_manifest=args.action_manifest))
        vive = stack.enter_context(ViveUltimateLinuxProvider(
            mapping, socket_path=args.vive_socket, max_pose_age_s=args.max_stale))
        provider = QuestViveHybridProvider(quest, vive, alignment)
        print("POSITION CHECK ONLY: no robot output and no calibration changes.", flush=True)
        print("Keep all VIVE trackers attached. Do not cover cameras or recenter menus.\n"
              "Place the controller body nearby; pointing its ray is not sufficient.\n"
              "Follow each step automatically. Ctrl+C stops the check; completed stages stay saved.", flush=True)
        print(f"Planned sequence: {' -> '.join(ROLE_LABELS[r][0] for r in roles)}\n"
              f"Per step: {args.countdown}s preparation + {args.seconds:g}s hold.", flush=True)
        if args.speech:
            print("Speech uses the PC audio output, not guaranteed headset audio.\n"
                  "Confirm you can hear the first instruction before relying on voice cues.", flush=True)
            if shutil.which("spd-say") is None:
                print("spd-say not installed: text prompts only. Use --no-speech to suppress this notice.")
        for index, role in enumerate(roles, 1):
            serial = next(s for s, r in mapping.items() if r == role)
            frames = capture_stage(provider, args, role=role, serial=serial, index=index, total=len(roles))
            result = summarize_proximity(frames, vive_role=role, quest_role=quest_role,
                                         max_separation=args.max_separation,
                                         max_stale=args.max_stale, max_skew=args.max_skew)
            result.update({"captured_at": datetime.now(timezone.utc).isoformat(),
                           "serial_to_role": mapping, "spatial_alignment": alignment,
                           "vive_session_id": vive.client.session_id})
            stage_output = output / f"{index:02d}_{role}" if args.all_trackers else output
            save_stage(stage_output, frames, result)
            stages.append(result)
            print(f"Fresh synchronized pairs: {result['valid_pairs']}/{len(frames)}")
            if "mean_separation_m" in result:
                print(f"Separation: mean={result['mean_separation_m'] * 100:.1f}cm, "
                      f"max={result['max_separation_m'] * 100:.1f}cm")
            print("NEARBY POSITION CHECK OK (not calibrated)." if result["proximity_consistent"]
                  else "CHECK NEEDS REVIEW: " + ", ".join(result["issues"]))
            # Failed stages are kept; they do not prevent collecting the other locations.

    if args.all_trackers:
        result = summarize_batch(stages, roles)
        result["started_at"] = now.isoformat()
        with (output / "summary.json").open("x", encoding="utf-8") as stream:
            json.dump(result, stream, indent=2)
            stream.write("\n")
        passed = result["all_proximity_checks_consistent"]
    else:
        passed = result["proximity_consistent"]
    announce("Checks finished. You can relax.", speech=args.speech)
    print("ALL REQUESTED CHECKS CONSISTENT (NOT calibrated)." if passed else "Review recorded checks before calibration.")
    print(f"Saved {output / 'summary.json'}", flush=True)
    return 0 if passed else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
