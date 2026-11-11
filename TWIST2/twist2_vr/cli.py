from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import signal
import sys
import time

from .calibration import NeutralCalibration
from .config import load_json
from .controller import DryRunSink, RedisSimulationSink, SafetyGate
from .filtering import PoseFilter
from .normalization import PoseNormalizer
from .pipeline import TeleopPipeline
from .providers.mock import FULL_TARGET_POSES, SPARSE_TARGET_POSES, MockVRProvider
from .providers.hybrid import QuestViveHybridProvider
from .providers.openvr import OpenVRProvider
from .providers.replay import JSONLReplayProvider, JSONLWriter
from .providers.vive_linux import ViveUltimateLinuxProvider
from .reconstruction import SparseKinematicReconstructor, StrictDirectReconstructor
from .retargeting import NeutralTestRetargeter, OfficialGMRRetargeter, QuestArmIKRetargeter
from .spatial_alignment import pose_to_matrix, solve_hand_eye
from .upperbody import QuestUpperBodyReconstructor


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config"


def _ik_summary(retargeter: QuestArmIKRetargeter) -> str:
    return " ".join(
        f"{side[0]}={retargeter.last_residual_m[side] * 1000:.1f}mm"
        for side in ("Left", "Right")
    )


def _mapping(path: Path) -> dict[str, str]:
    config = load_json(path)
    allowed = set(config.get("allowed_roles", []))
    mapping = dict(config.get("serial_to_role", {}))
    unknown = sorted(set(mapping.values()) - allowed) if allowed else []
    if unknown:
        raise ValueError(f"tracker mapping contains unsupported roles: {unknown}")
    roles = list(mapping.values())
    if len(roles) != len(set(roles)):
        raise ValueError("tracker mapping assigns more than one serial to the same role")
    return mapping


def _vive_linux_mapping(path: Path) -> dict[str, str]:
    mapping = _mapping(path)
    placeholders = sorted(serial for serial in mapping if serial.startswith("REPLACE_WITH_"))
    if placeholders:
        raise ValueError(
            "replace VIVE tracker serial placeholders after running vive-linux-inventory: "
            + ", ".join(placeholders)
        )
    forbidden = {"Head", "Left_Wrist", "Right_Wrist"} & set(mapping.values())
    if forbidden:
        raise ValueError(
            "Linux VIVE roles collide with Quest automatic roles: " + ", ".join(sorted(forbidden))
        )
    required = {"Pelvis", "Left_Knee", "Right_Knee", "Left_Foot", "Right_Foot"}
    missing = sorted(required - set(mapping.values()))
    if missing:
        raise ValueError("VIVE mapping is missing required roles: " + ", ".join(missing))
    return mapping


def _hybrid_provider(args: argparse.Namespace) -> QuestViveHybridProvider:
    mapping = _vive_linux_mapping(args.mapping)
    quest = OpenVRProvider({}, action_manifest=args.action_manifest)
    vive = None
    try:
        vive = ViveUltimateLinuxProvider(
            mapping,
            socket_path=args.vive_socket,
            max_pose_age_s=args.max_stale,
        )
        vive.wait_until_ready(args.startup_timeout)
        return QuestViveHybridProvider(quest, vive, load_json(args.cross_calibration))
    except Exception:
        try:
            if vive is not None:
                vive.close()
        finally:
            quest.close()
        raise


def _provider(args: argparse.Namespace):
    if args.command == "mock":
        targets = FULL_TARGET_POSES if args.mock_layout == "full" else SPARSE_TARGET_POSES
        return MockVRProvider(target_poses=targets, frequency_hz=args.frequency)
    if args.command == "replay":
        return JSONLReplayProvider(args.input, realtime=args.realtime, loop=args.loop)
    if args.command == "vive-linux-fullbody":
        return _hybrid_provider(args)
    return OpenVRProvider(
        _mapping(args.mapping),
        action_manifest=getattr(args, "action_manifest", None),
    )


def _retargeter(args: argparse.Namespace):
    if args.command == "quest-arms":
        return QuestArmIKRetargeter(args.arm_robot_xml, load_json(args.arm_config))
    if args.retargeter == "test":
        print("WARNING: using neutral test-only retargeter; this is not GMR validation.", file=sys.stderr)
        return NeutralTestRetargeter()
    return OfficialGMRRetargeter(args.gmr_checkout, actual_human_height=args.human_height)


def _sink(args: argparse.Namespace):
    if not args.publish_sim:
        return DryRunSink()
    if args.command in {"live", "vive-fullbody", "vive-linux-fullbody"}:
        raise ValueError(
            "full-body live Redis publishing remains disabled until tracker calibration and replay stability pass"
        )
    return RedisSimulationSink(port=args.redis_port, allow_simulation_publish=True)


def _run(args: argparse.Namespace) -> int:
    previous_sigterm = signal.getsignal(signal.SIGTERM)

    def _stop_on_sigterm(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _stop_on_sigterm)
    provider = _provider(args)
    normalizer = PoseNormalizer(load_json(args.coordinates))
    calibration = NeutralCalibration(load_json(args.calibration))
    pose_filter = PoseFilter(load_json(args.filtering))
    if args.command == "quest-arms":
        reconstructor = QuestUpperBodyReconstructor(load_json(args.arm_config))
    elif args.reconstruction == "strict":
        reconstructor = StrictDirectReconstructor()
    else:
        reconstructor = SparseKinematicReconstructor(load_json(args.reconstruction_config))
    sink = _sink(args)
    retargeter = _retargeter(args)
    pipeline = TeleopPipeline(
        provider,
        normalizer,
        calibration,
        pose_filter,
        reconstructor,
        retargeter,
        sink,
        SafetyGate(max_age_s=args.max_stale),
    )
    writer = JSONLWriter(args.record) if args.record else None
    published = 0
    last_profile_state = None
    started = time.monotonic()
    if args.command == "quest-arms":
        print("Quest arms SIMULATION ONLY: root/legs/waist are locked; physical G1 output is unavailable.")
        print("Release both grips and hold a neutral pose; after READY, hold both grips to move the arms.")
    try:
        for _ in range(args.frames if args.frames > 0 else sys.maxsize):
            step_started = time.monotonic()
            try:
                result = pipeline.step()
            except EOFError:
                break
            if writer:
                writer.write(result.raw_frame)
            if not result.published:
                print(f"SAFE STOP: {result.reason}", file=sys.stderr)
                return 2
            published += 1
            profile_status = getattr(reconstructor, "status", None)
            if profile_status is not None and profile_status.state != last_profile_state:
                label = "TRACKING_LOST" if profile_status.state == "WAITING_TRACKING" else profile_status.state
                print(f"[{label}] {profile_status.detail}", flush=True)
                last_profile_state = profile_status.state
            if published == 1 or published % args.status_every == 0:
                estimated = sum(v.startswith("estimated") for v in result.body_frame.provenance.values())
                ik_status = f" IK={_ik_summary(retargeter)}" if args.command == "quest-arms" else ""
                print(
                    f"frame={published} source={result.raw_frame.source} "
                    f"reference={result.reference.source} estimated_joints={estimated} "
                    f"root_z={result.reference.mimic_obs[2]:.3f} "
                    f"sink={'redis-sim' if args.publish_sim else 'dry-run'}{ik_status}"
                )
            if args.command == "mock" or (
                args.command in {"live", "quest-arms", "vive-fullbody", "vive-linux-fullbody"}
                and args.rate_limit
            ):
                elapsed = time.monotonic() - step_started
                time.sleep(max(0.0, 1.0 / args.frequency - elapsed))
    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        if writer:
            writer.close()
        pipeline.close()
        signal.signal(signal.SIGTERM, previous_sigterm)
    duration = max(time.monotonic() - started, 1e-9)
    print(f"Completed {published} valid frames in {duration:.2f}s ({published / duration:.1f} Hz).")
    return 0


def _inventory(args: argparse.Namespace) -> int:
    with OpenVRProvider(_mapping(args.mapping), action_manifest=getattr(args, "action_manifest", None)) as provider:
        print(json.dumps(provider.device_inventory(), indent=2))
    return 0


def _vive_linux_inventory(args: argparse.Namespace) -> int:
    with ViveUltimateLinuxProvider(
        {}, socket_path=args.vive_socket, max_pose_age_s=args.max_stale
    ) as provider:
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            provider.read()
            time.sleep(1.0 / args.frequency)
        print(json.dumps(provider.device_inventory(), indent=2))
    return 0


def _collect_neutral_calibration(
    provider,
    args: argparse.Namespace,
) -> int:
    coordinate_config = load_json(args.coordinates)
    calibration_config = load_json(args.calibration)
    normalizer = PoseNormalizer(coordinate_config)
    positions = []
    last_frame = None
    deadline = time.monotonic() + args.timeout
    while len(positions) < args.samples and time.monotonic() < deadline:
        frame = normalizer(provider.read())
        pose = frame.poses.get(args.anchor)
        if pose is not None and pose.valid:
            positions.append(pose.position_m.copy())
            last_frame = frame
        time.sleep(1.0 / args.frequency)
    if last_frame is None or len(positions) < args.samples:
        raise RuntimeError(f"collected only {len(positions)}/{args.samples} valid {args.anchor} samples")
    import numpy as np
    samples = np.stack(positions)
    if float(np.max(np.std(samples, axis=0))) > args.max_position_std:
        raise RuntimeError("calibration anchor moved too much; hold the neutral pose and retry")
    calibration = NeutralCalibration(calibration_config)
    matrix = calibration.calibrate_from_frame(last_frame, args.anchor)
    calibration_config["calibrated"] = True
    calibration_config["anchor_role"] = args.anchor
    calibration_config["tracking_to_neutral_matrix"] = matrix.tolist()
    calibration_config["calibrated_at_unix_s"] = time.time()
    output = Path(args.output)
    output.write_text(json.dumps(calibration_config, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote calibration to {output}. This calibrates room yaw/origin only; verify mount offsets separately.")
    return 0


def _calibrate_live(args: argparse.Namespace) -> int:
    with OpenVRProvider(_mapping(args.mapping)) as provider:
        return _collect_neutral_calibration(provider, args)


def _calibrate_vive_linux_neutral(args: argparse.Namespace) -> int:
    with _hybrid_provider(args) as provider:
        return _collect_neutral_calibration(provider, args)


def _align_vive_linux(args: argparse.Namespace) -> int:
    serial = args.tracker_serial.strip()
    if not serial or serial.startswith("REPLACE_WITH_"):
        raise ValueError("--tracker-serial must be the real serial from vive-linux-inventory")
    quest_role = "Left_Wrist" if args.quest_hand == "left" else "Right_Wrist"
    quest = OpenVRProvider({}, action_manifest=args.action_manifest)
    try:
        vive = ViveUltimateLinuxProvider(
            {serial: "Calibration_Tracker"},
            socket_path=args.vive_socket,
            max_pose_age_s=args.max_stale,
        )
        try:
            vive.wait_until_ready(args.startup_timeout)
            print(
                f"Rigidly fasten VIVE tracker {serial} to the {args.quest_hand} Quest controller.\n"
                "Do not let the tracker move relative to the controller. During collection, move through "
                "a wide figure-eight and rotate around pitch, yaw, and roll."
            )
            for remaining in range(args.countdown, 0, -1):
                print(f"Starting in {remaining}...", flush=True)
                time.sleep(1.0)

            quest_samples = []
            vive_samples = []
            generations: tuple[int, int] | None = None
            deadline = time.monotonic() + args.timeout
            while len(quest_samples) < args.samples and time.monotonic() < deadline:
                started = time.monotonic()
                quest_frame = quest.read()
                vive_frame = vive.read()
                quest_pose = quest_frame.poses.get(quest_role)
                vive_pose = vive_frame.poses.get("Calibration_Tracker")
                if (
                    quest_pose is not None
                    and vive_pose is not None
                    and quest_pose.valid
                    and vive_pose.valid
                    and abs(quest_pose.timestamp_s - vive_pose.timestamp_s) <= args.max_sample_skew
                ):
                    current_generations = (
                        int(quest_pose.metadata.get("connection_generation", 0)),
                        int(vive_pose.metadata.get("connection_generation", 0)),
                    )
                    if generations is None:
                        generations = current_generations
                    elif current_generations != generations:
                        raise RuntimeError("controller or tracker reconnected during alignment; retry")
                    quest_samples.append(pose_to_matrix(quest_pose))
                    vive_samples.append(pose_to_matrix(vive_pose))
                    if len(quest_samples) == 1 or len(quest_samples) % 25 == 0:
                        print(f"alignment samples: {len(quest_samples)}/{args.samples}", flush=True)
                elapsed = time.monotonic() - started
                time.sleep(max(0.0, 1.0 / args.frequency - elapsed))
        finally:
            vive.close()
    finally:
        quest.close()

    if len(quest_samples) < args.samples:
        raise RuntimeError(
            f"collected only {len(quest_samples)}/{args.samples} synchronized valid samples"
        )
    result = solve_hand_eye(quest_samples, vive_samples)
    if result.position_rmse_m > args.max_position_rmse:
        raise RuntimeError(
            f"alignment position RMSE {result.position_rmse_m:.4f}m exceeds "
            f"{args.max_position_rmse:.4f}m"
        )
    if result.orientation_rmse_deg > args.max_orientation_rmse:
        raise RuntimeError(
            f"alignment orientation RMSE {result.orientation_rmse_deg:.2f}deg exceeds "
            f"{args.max_orientation_rmse:.2f}deg"
        )

    now = datetime.now(timezone.utc)
    calibration = {
        "schema_version": 1,
        "calibrated": True,
        "calibration_id": "quest-vive-" + now.strftime("%Y%m%dT%H%M%SZ"),
        "calibrated_at": now.isoformat(),
        "source_frame": "vive_hub_world",
        "target_frame": "openvr_standing",
        "source_to_target_matrix": result.vive_to_quest.tolist(),
        "calibration_fixture": {
            "tracker_serial": serial,
            "quest_role": quest_role,
            "tracker_from_controller_matrix": result.tracker_from_controller.tolist(),
        },
        "fit": {
            "sample_count": result.sample_count,
            "pair_count": result.pair_count,
            "position_rmse_m": result.position_rmse_m,
            "orientation_rmse_deg": result.orientation_rmse_deg,
            "rotation_system_rank": result.rotation_system_rank,
            "translation_system_rank": result.translation_system_rank,
        },
        "warning": "Repeat this alignment if either tracking map/room origin is reset.",
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(calibration, indent=2) + "\n", encoding="utf-8")
    print(
        f"Wrote {output}: position RMSE={result.position_rmse_m * 1000:.1f}mm, "
        f"orientation RMSE={result.orientation_rmse_deg:.2f}deg"
    )
    return 0


def _common(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--coordinates", type=Path, default=CONFIG / "coordinates.openvr.json")
    parser.add_argument("--calibration", type=Path, default=CONFIG / "calibration.mock.json")
    parser.add_argument("--filtering", type=Path, default=CONFIG / "filtering.json")
    parser.add_argument("--reconstruction", choices=("sparse", "strict"), default="sparse")
    parser.add_argument("--reconstruction-config", type=Path, default=CONFIG / "reconstruction.sparse.json")
    parser.add_argument("--retargeter", choices=("gmr", "test"), default="gmr")
    parser.add_argument("--gmr-checkout", type=Path, default=ROOT / "external/GMR")
    parser.add_argument("--human-height", type=float, default=1.70)
    parser.add_argument("--frames", type=int, default=100)
    parser.add_argument("--frequency", type=float, default=50.0)
    parser.add_argument("--max-stale", type=float, default=0.20)
    parser.add_argument("--status-every", type=int, default=25)
    parser.add_argument("--record", type=Path)
    parser.add_argument("--publish-sim", action="store_true", help="Explicitly publish only to local TWIST2 MuJoCo Redis")
    parser.add_argument("--redis-port", type=int, default=6379, help="localhost simulation Redis port")


def _vive_linux_source_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--mapping", type=Path, default=CONFIG / "tracker_mapping.vive_linux.example.json"
    )
    parser.add_argument("--action-manifest", type=Path, default=CONFIG / "openvr_actions.json")
    parser.add_argument("--vive-socket", type=Path, default=Path("/tmp/vut.sock"))
    parser.add_argument(
        "--cross-calibration", type=Path, default=CONFIG / "calibration.quest_vive.example.json"
    )
    parser.add_argument("--startup-timeout", type=float, default=30.0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Simulation-first Quest/tracker adapter for TWIST2")
    sub = parser.add_subparsers(dest="command", required=True)
    mock = sub.add_parser("mock", help="run deterministic synthetic trackers")
    _common(mock)
    mock.add_argument(
        "--mock-layout", choices=("full", "sparse"), default="full",
        help="full is an FK-derived direct-tracker fixture; sparse exercises estimated joints and is not balance-safe",
    )
    replay = sub.add_parser("replay", help="replay a JSONL tracker recording")
    _common(replay)
    replay.add_argument("--input", type=Path, required=True)
    replay.add_argument("--realtime", action="store_true")
    replay.add_argument("--loop", action="store_true")
    live = sub.add_parser("live", help="read live OpenVR devices; dry-run output only")
    _common(live)
    live.set_defaults(calibration=CONFIG / "calibration.live.json")
    live.add_argument("--mapping", type=Path, default=CONFIG / "tracker_mapping.example.json")
    live.add_argument("--action-manifest", type=Path, default=CONFIG / "openvr_actions.json")
    live.add_argument("--rate-limit", action="store_true")
    quest = sub.add_parser(
        "quest-arms",
        help="simulation-only Quest HMD + two-controller arm teleoperation",
    )
    _common(quest)
    quest.set_defaults(
        calibration=CONFIG / "calibration.mock.json",
        filtering=CONFIG / "filtering.quest_arms.json",
        rate_limit=True,
    )
    quest.add_argument("--mapping", type=Path, default=CONFIG / "tracker_mapping.example.json")
    quest.add_argument("--action-manifest", type=Path, default=CONFIG / "openvr_actions.json")
    quest.add_argument("--arm-config", type=Path, default=CONFIG / "quest_arms.json")
    quest.add_argument("--arm-robot-xml", type=Path, default=ROOT / "assets/g1/g1_mocap_29dof.xml")
    quest.add_argument("--rate-limit", action=argparse.BooleanOptionalAction, default=True)
    vive = sub.add_parser(
        "vive-fullbody",
        help="Quest + serial-mapped Vive trackers; hardware-validation dry-run only",
    )
    _common(vive)
    vive.set_defaults(calibration=CONFIG / "calibration.live.json")
    vive.add_argument(
        "--mapping", type=Path, default=CONFIG / "tracker_mapping.vive_fullbody.example.json"
    )
    vive.add_argument("--action-manifest", type=Path, default=CONFIG / "openvr_actions.json")
    vive.add_argument("--rate-limit", action=argparse.BooleanOptionalAction, default=True)
    vive_linux = sub.add_parser(
        "vive-linux-fullbody",
        help="Quest + Linux VIVE Hub Ultimate Trackers; calibrated dry-run only",
    )
    _common(vive_linux)
    vive_linux.set_defaults(
        calibration=CONFIG / "calibration.live.json",
        coordinates=CONFIG / "coordinates.vive_linux_fullbody.example.json",
    )
    _vive_linux_source_args(vive_linux)
    vive_linux.add_argument("--rate-limit", action=argparse.BooleanOptionalAction, default=True)
    inventory = sub.add_parser("live-inventory", help="list OpenVR class/model/serial identifiers")
    inventory.add_argument("--mapping", type=Path, default=CONFIG / "tracker_mapping.example.json")
    inventory.add_argument("--action-manifest", type=Path, default=CONFIG / "openvr_actions.json")
    calibrate = sub.add_parser("live-calibrate", help="write room-yaw/origin calibration from a neutral pose")
    calibrate.add_argument("--coordinates", type=Path, default=CONFIG / "coordinates.openvr.json")
    calibrate.add_argument("--calibration", type=Path, default=CONFIG / "calibration.live.json")
    calibrate.add_argument("--mapping", type=Path, default=CONFIG / "tracker_mapping.example.json")
    calibrate.add_argument("--output", type=Path, default=CONFIG / "calibration.live.json")
    calibrate.add_argument("--anchor", default="Pelvis")
    calibrate.add_argument("--samples", type=int, default=50)
    calibrate.add_argument("--frequency", type=float, default=50.0)
    calibrate.add_argument("--timeout", type=float, default=15.0)
    calibrate.add_argument("--max-position-std", type=float, default=0.015)
    vive_inventory = sub.add_parser(
        "vive-linux-inventory", help="list Ultimate Trackers exposed by Linux VIVE Hub"
    )
    vive_inventory.add_argument("--vive-socket", type=Path, default=Path("/tmp/vut.sock"))
    vive_inventory.add_argument("--duration", type=float, default=2.0)
    vive_inventory.add_argument("--frequency", type=float, default=100.0)
    vive_inventory.add_argument("--max-stale", type=float, default=0.20)
    align = sub.add_parser(
        "vive-linux-align",
        help="solve the rigid transform from VIVE map space to Quest/OpenVR space",
    )
    align.add_argument("--tracker-serial", required=True)
    align.add_argument("--quest-hand", choices=("left", "right"), default="right")
    align.add_argument("--action-manifest", type=Path, default=CONFIG / "openvr_actions.json")
    align.add_argument("--vive-socket", type=Path, default=Path("/tmp/vut.sock"))
    align.add_argument(
        "--output", type=Path, default=CONFIG / "local/calibration.quest_vive.json"
    )
    align.add_argument("--samples", type=int, default=240)
    align.add_argument("--frequency", type=float, default=60.0)
    align.add_argument("--timeout", type=float, default=90.0)
    align.add_argument("--countdown", type=int, default=5)
    align.add_argument("--startup-timeout", type=float, default=30.0)
    align.add_argument("--max-stale", type=float, default=0.20)
    align.add_argument("--max-sample-skew", type=float, default=0.050)
    align.add_argument("--max-position-rmse", type=float, default=0.030)
    align.add_argument("--max-orientation-rmse", type=float, default=5.0)
    vive_neutral = sub.add_parser(
        "vive-linux-neutral-calibrate",
        help="write neutral yaw/origin calibration after Quest/VIVE spatial alignment",
    )
    vive_neutral.add_argument(
        "--coordinates", type=Path, default=CONFIG / "coordinates.vive_linux_fullbody.example.json"
    )
    vive_neutral.add_argument("--calibration", type=Path, default=CONFIG / "calibration.live.json")
    vive_neutral.add_argument(
        "--output", type=Path, default=CONFIG / "local/calibration.vive_linux.json"
    )
    vive_neutral.add_argument("--anchor", default="Pelvis")
    vive_neutral.add_argument("--samples", type=int, default=80)
    vive_neutral.add_argument("--frequency", type=float, default=50.0)
    vive_neutral.add_argument("--timeout", type=float, default=20.0)
    vive_neutral.add_argument("--max-position-std", type=float, default=0.015)
    vive_neutral.add_argument("--max-stale", type=float, default=0.20)
    _vive_linux_source_args(vive_neutral)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "live-inventory":
            return _inventory(args)
        if args.command == "vive-linux-inventory":
            return _vive_linux_inventory(args)
        if args.command == "vive-linux-align":
            return _align_vive_linux(args)
        if args.command == "live-calibrate":
            return _calibrate_live(args)
        if args.command == "vive-linux-neutral-calibrate":
            return _calibrate_vive_linux_neutral(args)
        return _run(args)
    except Exception as error:
        print(f"ERROR: {type(error).__name__}: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
