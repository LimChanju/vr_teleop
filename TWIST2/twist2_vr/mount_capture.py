"""Read-only neutral-stance diagnostics; never estimate or install mount offsets."""

from __future__ import annotations

import argparse
from collections import Counter
from contextlib import ExitStack
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np

from .cli import CONFIG, ROOT, _vive_linux_mapping, _vive_linux_source_args
from .config import load_json
from .models import TrackerFrame
from .providers.hybrid import QuestViveHybridProvider
from .providers.openvr import OpenVRProvider
from .providers.replay import frame_to_dict
from .providers.vive_linux import ViveUltimateLinuxProvider


ROLES = ("Head", "Pelvis", "Left_Wrist", "Right_Wrist",
         "Left_Knee", "Right_Knee", "Left_Foot", "Right_Foot")
LOWER_BODY_HEAD_ROLES = ("Head", "Pelvis", "Left_Knee", "Right_Knee", "Left_Foot", "Right_Foot")


def _required_roles(exclude_roles, lower_body_head):
    excluded = set(exclude_roles)
    if lower_body_head and excluded:
        raise ValueError("lower-body-head capture cannot be combined with role exclusions")
    if excluded - {"Pelvis"}:
        raise ValueError("only Pelvis may be excluded from diagnostics")
    return list(LOWER_BODY_HEAD_ROLES) if lower_body_head else [role for role in ROLES if role not in excluded]


def summarize(frames: list[TrackerFrame], *, max_stale: float = 0.20,
              exclude_roles=(), lower_body_head: bool = False) -> dict:
    """Report sample quality without labeling sensor poses as anatomical joints."""
    if not frames:
        raise ValueError("no frames captured")
    required_roles = _required_roles(exclude_roles, lower_body_head)
    excluded = set(ROLES) - set(required_roles)

    def usable(frame, role):
        pose = frame.poses.get(role)
        return (pose is not None and pose.valid
                and -0.01 <= frame.timestamp_s - pose.timestamp_s <= max_stale)

    result = {
        "schema_version": 1,
        "purpose": "neutral_mount_diagnostics_only",
        "mounts_calibrated": False,
        "coordinate_frame": "openvr_standing",
        "frame_count": len(frames),
        "capture_mode": ("lower_body_head" if lower_body_head else
                         "pelvis_excluded" if excluded else "full_body"),
        "partial_capture": bool(excluded),
        "excluded_roles": [role for role in ROLES if role in excluded],
        "required_roles": required_roles,
        "all_eight_valid_frames": sum(all(usable(f, r) for r in ROLES) for f in frames),
        "all_required_valid_frames": sum(all(usable(f, r) for r in required_roles) for f in frames),
        "roles": {},
        "warning": "Means describe sensors, not joint centers. No mount transforms or robot commands were generated.",
    }
    if excluded:
        result["warning"] += " Partial diagnostics only: excluded roles are not checked for stability; NOT a full-body capture."
    if any(f.coordinate_frame != result["coordinate_frame"] for f in frames):
        raise ValueError("mount capture requires aligned openvr_standing frames")
    for role in ROLES:
        poses = [f.poses[role] for f in frames if usable(f, role)]
        states = Counter(f.poses[role].tracking_state if role in f.poses else "missing"
                         for f in frames)
        identities = {(p.device_id, p.metadata.get("connection_generation", 0)) for p in poses}
        row = {"valid_samples": len(poses), "valid_fraction": len(poses) / len(frames),
               "tracking_states": dict(states), "stable_capture": False,
               "included_in_stability_check": role not in excluded}
        if poses:
            positions = np.stack([p.position_m for p in poses])
            quaternions = np.stack([p.orientation_xyzw for p in poses])
            # Quaternion signs are equivalent; averaging outer products handles q/-q.
            _, vectors = np.linalg.eigh(quaternions.T @ quaternions)
            mean_quat = vectors[:, -1]
            if mean_quat[3] < 0:
                mean_quat = -mean_quat
            angles = np.degrees(2 * np.arccos(np.clip(np.abs(quaternions @ mean_quat), 0, 1)))
            std = positions.std(axis=0)
            row.update({
                "device_id": poses[-1].device_id,
                "mean_position_m": positions.mean(axis=0).tolist(),
                "position_std_m": std.tolist(),
                "mean_orientation_xyzw": mean_quat.tolist(),
                "max_orientation_deviation_deg": float(angles.max()),
                "identity_or_connection_changed": len(identities) != 1,
                "stable_capture": bool(len(poses) >= 2 and row["valid_fraction"] >= 0.8
                                       and len(identities) == 1 and std.max() <= 0.015
                                       and angles.max() <= 5.0),
            })
        result["roles"][role] = row
    result["stable_capture"] = bool(
        result["all_required_valid_frames"] / len(frames) >= 0.8
        and all(result["roles"][role]["stable_capture"] for role in required_roles)
    )
    result["full_body_stable_capture"] = result["stable_capture"] and not result["partial_capture"]
    return result


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    _vive_linux_source_args(parser)
    parser.set_defaults(mapping=CONFIG / "local/tracker_mapping.vive_linux.json",
                        cross_calibration=CONFIG / "local/calibration.quest_vive.json")
    parser.add_argument("--source", choices=("vive-linux", "openvr"), default="vive-linux",
                        help="openvr reads SteamVR poses directly; it never applies SDK alignment")
    parser.add_argument("--spacecal-profile", type=Path,
                        default=Path.home() / ".config/space-calibrator/profile.json",
                        help="saved xi-ve profile to snapshot for OpenVR provenance only")
    parser.add_argument("--coordinates", type=Path,
                        default=CONFIG / "local/coordinates.vive_linux_fullbody.json")
    parser.add_argument("--countdown", type=int, default=10)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--frequency", type=float, default=50.0)
    parser.add_argument("--max-stale", type=float, default=0.20)
    scope = parser.add_mutually_exclusive_group()
    scope.add_argument("--exclude-role", choices=("Pelvis",), action="append", default=[],
                       help="exclude Pelvis from diagnostic stability checks only; no full-body calibration or synthetic pose")
    scope.add_argument("--lower-body-head", action="store_true",
                       help="check HMD, pelvis, both shin and both foot trackers (6 roles); controllers are diagnostic-only, no robot output")
    parser.add_argument("--output-root", type=Path, default=ROOT / "runtime/mount_capture")
    return parser


def _spacecal_profile_snapshot(path: Path) -> dict:
    """Copy saved profile evidence without inferring that the runtime applied it."""
    result = {"path": str(path), "captured_at": datetime.now(timezone.utc).isoformat()}
    try:
        contents = path.read_bytes()
    except OSError as error:
        return {**result, "status": "unavailable", "error": f"{type(error).__name__}: {error}"}
    result["sha256"] = hashlib.sha256(contents).hexdigest()
    try:
        result["profile"] = json.loads(contents)
    except (ValueError, UnicodeError) as error:
        return {**result, "status": "invalid_json", "error": f"{type(error).__name__}: {error}"}
    return {**result, "status": "read"}


def main(argv=None):
    args = build_parser().parse_args(argv)
    if (args.countdown < 0 or not 0 < args.seconds <= 60 or not 1 <= args.frequency <= 120
            or not math.isfinite(args.max_stale) or args.max_stale <= 0):
        raise ValueError("use countdown >= 0, seconds in (0, 60], frequency in [1, 120], max-stale > 0")
    mapping = _vive_linux_mapping(args.mapping)
    coordinates = load_json(args.coordinates)
    if args.source == "openvr":
        alignment = {
            "expected_runtime_alignment": "xi-ve SpaceCalibrator",
            "application_alignment_applied": False,
            "runtime_alignment_verified": False,
            "warning": "A saved profile does not prove that SteamVR applied it or that alignment is correct.",
            "profile_before": _spacecal_profile_snapshot(args.spacecal_profile),
        }
    else:
        alignment = load_json(args.cross_calibration)
    frames = []
    # No Ready override: invalid/unknown states are retained for diagnosis.
    with ExitStack() as stack:
        if args.source == "openvr":
            provider = stack.enter_context(OpenVRProvider(mapping, action_manifest=args.action_manifest))
            print("Input: OpenVR standing poses; SpaceCal correction must already be active in SteamVR.",
                  flush=True)
        else:
            quest = stack.enter_context(OpenVRProvider({}, action_manifest=args.action_manifest))
            vive = stack.enter_context(ViveUltimateLinuxProvider(
                mapping, socket_path=args.vive_socket, max_pose_age_s=args.max_stale))
            provider = QuestViveHybridProvider(quest, vive, alignment)
        print("Capture only: no robot output and no calibration changes.", flush=True)
        if args.lower_body_head:
            print("Lower-body + head partial capture: checking Head, Pelvis, both knees/shins and both feet "
                  "(6 roles). NOT full-body. Controller samples, if present, remain in the recording.",
                  flush=True)
        elif args.exclude_role:
            print("Partial capture: Pelvis excluded; checking the other 7 roles. "
                  "NOT full-body. Raw Pelvis samples, if present, remain in the recording.", flush=True)
        if args.lower_body_head:
            print("Keep the HMD on and tracking session running. Stand facing forward, feet parallel, "
                  "arms relaxed. Controller tracking is not required for this diagnostic capture.", flush=True)
        else:
            print("Stand facing forward, feet parallel, arms relaxed, controllers in both hands.", flush=True)
        deadline = time.monotonic() + args.countdown
        previous = None
        while time.monotonic() < deadline:
            remaining = math.ceil(deadline - time.monotonic())
            if remaining != previous:
                print(f"Starting in {remaining}...", flush=True)
                previous = remaining
            provider.read()  # Drain pose queues throughout the countdown.
            time.sleep(1.0 / args.frequency)
        print(f"Hold still for {args.seconds:g} seconds...", flush=True)
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            started = time.monotonic()
            frames.append(provider.read())
            time.sleep(max(0.0, 1.0 / args.frequency - (time.monotonic() - started)))

    if args.source == "openvr":
        alignment["profile_after"] = _spacecal_profile_snapshot(args.spacecal_profile)
        before = alignment["profile_before"].get("sha256")
        after = alignment["profile_after"].get("sha256")
        alignment["profile_changed_during_capture"] = before != after if before and after else None
    result = summarize(frames, max_stale=args.max_stale, exclude_roles=args.exclude_role,
                       lower_body_head=args.lower_body_head)
    now = datetime.now(timezone.utc)
    result.update({"captured_at": now.isoformat(), "serial_to_role": mapping,
                   "capture_source": args.source,
                   "spatial_alignment": alignment,
                   "mount_observations": coordinates.get("mount_observations", {})})
    output = args.output_root / now.strftime("%Y%m%dT%H%M%S_%fZ")
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
    for role, row in result["roles"].items():
        suffix = " [excluded from stability check]" if not row["included_in_stability_check"] else ""
        print(f"{role}: valid={row['valid_samples']}/{len(frames)} "
              f"stable={row['stable_capture']} states={row['tracking_states']}{suffix}")
    print(f"Saved {output / 'summary.json'}", flush=True)
    if result["partial_capture"]:
        print(f"Partial capture stable for {len(result['required_roles'])} checked roles "
              "(NOT full-body; NOT calibrated)."
              if result["stable_capture"] else
              "Partial capture needs review: tracking, motion, or identity issues in checked roles. "
              "NOT full-body; NOT calibrated.")
    else:
        print("Stable capture (NOT calibrated)." if result["stable_capture"]
              else "Capture needs review: missing tracking, motion, or changed device identity.")
    return 0 if result["stable_capture"] else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
