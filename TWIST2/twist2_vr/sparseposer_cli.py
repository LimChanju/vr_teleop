"""Calibrate and inspect full-body estimates; NO Redis, DDS or robot output."""
from __future__ import annotations

import argparse
from contextlib import nullcontext
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import numpy as np

from .sparseposer_adapter import (EightTrackerEstimator, ROLES, file_sha256,
                                  fit_tpose_calibration, read_recording)
from .sparseposer_backend import SparsePoserBackend, load_skeleton

ROOT = Path(__file__).resolve().parents[1]
CHECKOUT = ROOT / "external/SparsePoser"
REFERENCE = CHECKOUT / "SparsePoserUnity/Assets/Data/male_170_85.txt"
MODELS = CHECKOUT / "python/models/model_xsens"
CALIBRATION = ROOT / "config/local/calibration.sparseposer_tpose.json"


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("command", choices=("calibrate", "live", "replay", "doctor"))
    p.add_argument("--calibration", type=Path, default=CALIBRATION)
    p.add_argument("--recording", type=Path, help="raw schema-1 poses JSONL, not a summary or estimated skeleton")
    p.add_argument("--output", type=Path, help="NEW run directory; never overwrite earlier results")
    p.add_argument("--mapping", type=Path, default=ROOT / "config/local/tracker_mapping.vive_linux.json")
    p.add_argument("--spacecal-profile", type=Path, default=Path.home() / ".config/space-calibrator/profile.json")
    p.add_argument("--height", type=float, default=1.70, help="actual standing height in metres (calibration)")
    p.add_argument("--countdown", type=float, default=10.)
    p.add_argument("--seconds", type=float, default=0., help="live duration; 0=until Ctrl+C; calibration defaults to 4 seconds")
    p.add_argument("--hz", type=float, default=60., help="temporal resampling/poll rate; reference BVH nominally 60Hz")
    p.add_argument("--device", default="cpu")
    p.add_argument("--threads", type=int, default=2)
    p.add_argument("--preview", action="store_true", help="desktop estimated-human skeleton, NOT G1/Quest video")
    return p


def _provider(args):
    from .providers.openvr import OpenVRProvider
    mapping = json.loads(args.mapping.read_text())
    return OpenVRProvider(mapping["serial_to_role"])


def _frames(args, provider):
    if args.recording:
        yield from read_recording(args.recording)
        return
    start = time.monotonic()
    while args.seconds <= 0 or time.monotonic() - start < args.seconds:
        tick = time.monotonic()
        yield provider.read()
        time.sleep(max(0., 1 / args.hz - (time.monotonic() - tick)))


def _raw_record(frame):
    from .providers.replay import frame_to_dict
    row = frame_to_dict(frame)
    for role, pose in frame.poses.items():
        row["poses"][role]["timestamp_s"] = pose.timestamp_s
    return row


def _write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x") as stream:
        json.dump(data, stream, indent=2, allow_nan=False)
        stream.write("\n")


class SkeletonPreview:
    """Optional desktop display of all 22 estimated joints, plus sensor points."""
    def __init__(self):
        try:
            import matplotlib
            import matplotlib.pyplot as plt
        except ImportError as exc:
            raise RuntimeError("preview needs matplotlib; run bash scripts/setup_sparseposer.sh") from exc
        if matplotlib.get_backend().lower() in ("agg", "pdf", "ps", "svg", "template", "cairo"):
            raise RuntimeError("preview requires a desktop GUI backend (e.g. TkAgg); run in desktop terminal or omit --preview")
        self.plt = plt
        plt.ion()
        self.fig = plt.figure("SparsePoser + shin refinement — HUMAN estimate, no robot output")
        self.axes = self.fig.add_subplot(111, projection="3d")
        self.axes.set_title("Waiting for HMD + both controllers + 5 trackers")
        self.last_draw = 0.
        plt.show(block=False)
        plt.pause(0.001)

    def update(self, result, frame):
        if not self.plt.fignum_exists(self.fig.number):
            raise KeyboardInterrupt
        if time.monotonic() - self.last_draw < 0.10:
            # Pump GUI events even when the estimator is waiting for tracking.
            self.plt.pause(0.001)
            return
        self.last_draw = time.monotonic()
        ax = self.axes
        ax.clear()
        # Display OpenVR Y-up as plot Z-up. No transformation of output data.
        points = None
        if "positions_m" in result:
            points = np.asarray(result["positions_m"])[:, [0, 2, 1]]
            for child, parent in enumerate(result["parents"]):
                if child == 0 or parent < 0:
                    continue
                pair = points[[parent, child]]
                ax.plot(pair[:, 0], pair[:, 1], pair[:, 2], color="royalblue")
        available = [r for r in ROLES if r in frame.poses and frame.poses[r].valid
                     and frame.poses[r].tracking_state in ("200", "tracked", "ready")
                     and -0.01 <= frame.timestamp_s-frame.poses[r].timestamp_s <= 0.20]
        sensors = np.array([frame.poses[r].position_m for r in available]).reshape(-1, 3)[:, [0, 2, 1]]
        if len(sensors):
            ax.scatter(sensors[:, 0], sensors[:, 1], sensors[:, 2], color="orange", s=20)
        center = (points[0] if points is not None else
                  frame.poses["Pelvis"].position_m[[0, 2, 1]] if "Pelvis" in available else
                  np.array([sensors[:, 0].mean(), sensors[:, 1].mean(), 1.]) if len(sensors) else
                  np.array([0., 0., 1.]))
        ax.set(xlim=(center[0]-1, center[0]+1), ylim=(center[1]-1, center[1]+1),
               zlim=(center[2]-1, center[2]+1), xlabel="OpenVR X", ylabel="OpenVR Z", zlabel="Up (Y)")
        ax.set_box_aspect((1, 1, 1))
        if points is not None:
            status = "fit OK" if result["refinement"]["success"] else "fit NOT within tolerance"
            title = f"Estimated HUMAN: blue bones / orange sensors — {status}"
        elif result["kind"] == "warming_up":
            title = "Warming up motion history — keep all 8 sensors tracked"
        else:
            missing = [r for r in ROLES if r not in available]
            detail = ", ".join(missing) if missing else result.get("reason", "tracking unavailable")
            title = f"Waiting: {detail}\nSensors {len(available)}/8 — no full-body estimate"
        ax.set_title(title)
        self.plt.pause(0.001)


def main(argv=None):
    args = parser().parse_args(argv)
    if (not np.isfinite(args.hz) or args.hz <= 0 or args.threads < 1
            or not np.isfinite(args.seconds) or args.seconds < 0
            or not np.isfinite(args.countdown) or args.countdown < 0):
        raise ValueError("hz/threads must be positive; seconds/countdown must be finite and nonnegative")
    if args.command == "replay" and not args.recording:
        raise ValueError("replay requires --recording raw_poses.jsonl")
    if args.command == "live" and args.recording:
        raise ValueError("use replay for a recording")
    import torch
    torch.set_num_threads(args.threads)
    skeleton = load_skeleton(CHECKOUT, REFERENCE)
    if args.command == "doctor":
        backend = SparsePoserBackend(CHECKOUT, REFERENCE, MODELS, device=args.device)
        print(f"Official weights loaded: {len(backend.skeleton.names)} joints, device={args.device}")
        print("Six network inputs + two shin constraints. No VR connection/robot output started.")
        print(f"T-pose calibration present: {args.calibration.is_file()}")
        return 0

    if args.command == "calibrate" and args.calibration.exists():
        raise ValueError(f"preserving existing calibration: choose a NEW --calibration path: {args.calibration}")
    offline = args.recording is not None
    profile_hash = file_sha256(args.spacecal_profile) if args.spacecal_profile.is_file() else None
    if not offline and profile_hash is None:
        raise ValueError("SpaceCal profile missing: complete room alignment first (this program will not change it)")
    if args.command != "calibrate":
        calibration = json.loads(args.calibration.read_text())
        if not offline and calibration.get("spacecal_sha256") != profile_hash:
            raise ValueError("SpaceCal profile changed/missing since T-pose calibration; recalibrate mounts")
        backend = SparsePoserBackend(CHECKOUT, REFERENCE, MODELS, device=args.device)
        estimator = EightTrackerEstimator(backend, calibration, hz=args.hz)

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    out = args.output or ROOT / "runtime/sparseposer" / stamp
    out.mkdir(parents=True, exist_ok=False)
    metadata = {"command": args.command, "reference": str(REFERENCE), "reference_sha256": file_sha256(REFERENCE),
                "model_sha256": {p.name: file_sha256(p) for p in MODELS.glob("*.pt")},
                "calibration": str(args.calibration), "hz": args.hz, "source": str(args.recording) if offline else "openvr",
                "spacecal_sha256": profile_hash, "robot_output": False,
                "note": "8 sensor adapter around unchanged 6-input SparsePoser. Mounts/template proportions approximate."}
    if args.command != "calibrate":
        metadata["calibration_sha256"] = file_sha256(args.calibration)
    _write_json(out / "run.json", metadata)
    print(f"Output: {out}\nRead/estimate only. No robot command or VR runtime changes.", flush=True)
    captured, valid, blocked, warmup = [], 0, 0, 0
    preview = SkeletonPreview() if args.preview and args.command != "calibrate" else None
    context = nullcontext(None) if offline else _provider(args)
    started, last_report = time.monotonic(), 0.
    latencies = []
    try:
        with context as provider, (out / "raw_poses.jsonl").open("x") as raw, (out / "body_estimates.jsonl").open("x") as estimates:
            if args.command == "calibrate":
                if not offline:
                    print("T-POSE: face forward, feet parallel, arms straight out horizontally, controllers in hands. Keep all 5 trackers mounted.", flush=True)
                    remaining = args.countdown
                    while remaining > 0:
                        print(f"Starting in {remaining:.0f}...", flush=True)
                        delay = min(1., remaining)
                        time.sleep(delay)
                        remaining -= delay
                args.seconds = args.seconds or 4.
            for frame in _frames(args, provider):
                raw.write(json.dumps(_raw_record(frame), allow_nan=False) + "\n")
                if args.command == "calibrate":
                    captured.append(frame)
                    continue
                tick = time.perf_counter()
                try:
                    # Saved profile integrity only; runtime recenter/remount still requires operator check.
                    if not offline and time.monotonic() - last_report > 1.:
                        if file_sha256(args.spacecal_profile) != profile_hash:
                            raise RuntimeError("SpaceCal changed while estimating; stop and recalibrate")
                    result = estimator.process(frame)
                    if result is None:
                        warmup += 1
                        result = {"kind": "warming_up", "timestamp_s": frame.timestamp_s}
                    else:
                        valid += 1
                        latencies.append((time.perf_counter() - tick) * 1000)
                except ValueError as exc:
                    blocked += 1
                    result = {"kind": "tracking_blocked", "timestamp_s": frame.timestamp_s, "reason": str(exc)}
                # A missing controller must not leave a blank, unresponsive window.
                # Rendering errors must not be misreported as tracking failures.
                if preview:
                    preview.update(result, frame)
                estimates.write(json.dumps(result, allow_nan=False) + "\n")
                if time.monotonic() - last_report > 1.:
                    print(f"estimates={valid} warmup={warmup} blocked={blocked} last={result.get('reason', result['kind'])}", flush=True)
                    raw.flush()
                    estimates.flush()
                    last_report = time.monotonic()
    except KeyboardInterrupt:
        print("Stopped. Existing VR session left unchanged.", flush=True)
        if args.command == "calibrate":
            print("Calibration cancelled; no mount profile written.")
            return 130
    if args.command == "calibrate":
        calibration = fit_tpose_calibration(captured, skeleton, height_m=args.height, spacecal_sha256=profile_hash)
        _write_json(args.calibration, calibration)
        print(f"T-pose mount estimates saved: {args.calibration}")
        for role, quality in calibration["quality"].items():
            print(f"  {role}: estimated sensor→joint offset {quality['estimated_mount_distance_m']*100:.1f} cm")
        print("Approximate template calibration, not measured anatomy. Check the HUMAN skeleton preview before retargeting.")
    _write_json(out / "summary.json", {"estimates": valid, "blocked": blocked, "warmup": warmup,
                                     "elapsed_s": time.monotonic()-started,
                                     "median_estimation_ms": float(np.median(latencies)) if latencies else None,
                                     "robot_output": False})
    return 0 if args.command == "calibrate" or valid else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, RuntimeError, OSError, KeyError) as error:
        raise SystemExit(f"SparsePoser adapter: {error}") from error
