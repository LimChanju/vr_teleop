"""One-terminal G1 kinematic preview, with optional Quest view and MP4 recording.

No TWIST2 policy, physics stepping, Redis, DDS or robot command transport.
Tracker acquisition runs in a CHILD PROCESS: OpenVR Background and Scene
applications must not share the Python OpenVR singleton in the same process.
"""
from __future__ import annotations

import argparse
from contextlib import nullcontext
from datetime import datetime, timezone
import json
import multiprocessing as mp
from pathlib import Path
from queue import Empty, Full
import time
import traceback

import numpy as np

from .sparseposer_cli import (ROOT, CHECKOUT, REFERENCE, MODELS, CALIBRATION,
                             _provider, _raw_record, _write_json)
from .sparseposer_adapter import file_sha256, read_recording


XML = ROOT / "assets/g1/g1_sim2sim_29dof.xml"


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--viewer", choices=("vr", "desktop", "headless"), default="vr")
    p.add_argument("--calibration", type=Path, default=CALIBRATION)
    p.add_argument("--mapping", type=Path, default=ROOT / "config/local/tracker_mapping.vive_linux.json")
    p.add_argument("--spacecal-profile", type=Path, default=Path.home() / ".config/space-calibrator/profile.json")
    p.add_argument("--recording", type=Path, help="replay raw_poses.jsonl instead of connecting tracker input")
    p.add_argument("--output", type=Path, help="new run directory, never overwrite")
    p.add_argument("--seconds", type=float, default=0., help="wall-clock duration after viewer startup; 0=until stopped")
    p.add_argument("--frames", type=int, default=0, help="stop after this many G1 poses; 0=unlimited")
    p.add_argument("--hz", type=float, default=60.)
    p.add_argument("--threads", type=int, default=2)
    p.add_argument("--device", default="cpu")
    recording = p.add_mutually_exclusive_group()
    recording.add_argument("--record", action="store_true", help="also save a front-camera MP4; off by default")
    recording.add_argument("--no-record", dest="record", action="store_false", help=argparse.SUPPRESS)
    p.set_defaults(record=False)
    p.add_argument("--video-fps", type=int, default=30)
    p.add_argument("--fast", action="store_true", help="offline headless replay only; ignore capture-time pacing")
    return p


def joint_signature(model):
    """Validate q29 layout instead of assuming two G1 assets share joint order."""
    return [(str(model.joint(j).name), int(model.jnt_qposadr[j]))
            for j in range(model.njnt) if int(model.jnt_qposadr[j]) >= 7]


def put_latest(queue, value):
    """Bound IPC latency; disk logs retain every generated reference."""
    try:
        queue.put_nowait(value)
    except Full:
        try:
            queue.get_nowait()
        except Empty:
            pass
        try:
            queue.put_nowait(value)
        except Full:
            pass


def _source_frames(args, provider, stop):
    started = time.monotonic()
    if args.recording:
        first_stamp = None
        for frame in read_recording(args.recording):
            if first_stamp is None:
                first_stamp = frame.timestamp_s
            if not args.fast:
                delay = started + frame.timestamp_s - first_stamp - time.monotonic()
                if delay > 0 and stop.wait(delay):
                    return
            if stop.is_set():
                return
            yield frame
    else:
        while not stop.is_set():
            tick = time.monotonic()
            yield provider.read()
            stop.wait(max(0., 1/args.hz - (time.monotonic()-tick)))


def estimation_worker(args, output, queue, stop, expected_joint_order):
    """Background-only input and IK; never creates a VR scene or robot client."""
    poses, blocked, fit_warnings = 0, 0, 0
    try:
        import torch
        from .sparseposer_backend import SparsePoserBackend
        from .sparseposer_adapter import EightTrackerEstimator
        from .sparseposer_gmr import SparsePoserG1Retargeter
        torch.set_num_threads(args.threads)
        calibration = json.loads(args.calibration.read_text())
        backend = SparsePoserBackend(CHECKOUT, REFERENCE, MODELS, device=args.device)
        estimator = EightTrackerEstimator(backend, calibration, hz=args.hz)
        retargeter = SparsePoserG1Retargeter(ROOT / "external/GMR", backend.skeleton, calibration)
        if joint_signature(retargeter.model) != expected_joint_order:
            raise ValueError("GMR and viewer G1 joint orders differ; cannot display qpos safely")
        expected_profile = calibration.get("spacecal_sha256")
        profile_checked = 0.
        context = nullcontext(None) if args.recording else _provider(args)
        with context as provider, (output / "raw_poses.jsonl").open("x") as raw, \
                (output / "body_estimates.jsonl").open("x") as bodies, \
                (output / "g1_reference.jsonl").open("x") as references:
            for frame in _source_frames(args, provider, stop):
                raw.write(json.dumps(_raw_record(frame), allow_nan=False) + "\n")
                if not args.recording and time.monotonic() - profile_checked > 1.:
                    if file_sha256(args.spacecal_profile) != expected_profile:
                        raise RuntimeError("SpaceCal profile changed: stop and recalibrate")
                    profile_checked = time.monotonic()
                try:
                    estimate = estimator.process(frame)
                except ValueError as error:
                    blocked += 1
                    event = {"kind": "tracking_blocked", "reason": str(error),
                             "timestamp_s": frame.timestamp_s}
                    bodies.write(json.dumps(event) + "\n")
                    put_latest(queue, event)
                    continue
                if estimate is None:
                    put_latest(queue, {"kind": "warming_up", "timestamp_s": frame.timestamp_s})
                    continue
                bodies.write(json.dumps(estimate, allow_nan=False) + "\n")
                reference, diagnostics = retargeter.retarget(estimate)
                # In this kinematic preview a finite low-quality fit is shown
                # with warnings, NOT silently promoted into a robot command.
                poses += 1
                fit_warnings += int(not reference.valid)
                event = {"kind": "g1_kinematic_pose", "timestamp_s": frame.timestamp_s,
                         "qpos": reference.qpos.tolist(), "mimic_obs": reference.mimic_obs.tolist(),
                         "fit_accepted": reference.valid, "reason": reference.reason,
                         "diagnostics": diagnostics, "pose_number": poses,
                         "refinement": estimate["refinement"], "generated_at_s": time.monotonic(),
                         "robot_output": False, "physics_stepped": False}
                references.write(json.dumps(event, allow_nan=False) + "\n")
                put_latest(queue, event)
                if poses % 30 == 0:
                    raw.flush(); bodies.flush(); references.flush()
                if args.frames and poses >= args.frames:
                    break
        _write_json(output / "worker_summary.json", {"g1_poses": poses, "tracking_blocked": blocked,
                                                     "fit_warnings": fit_warnings, "robot_output": False})
        put_latest(queue, {"kind": "complete", "g1_poses": poses})
    except KeyboardInterrupt:
        pass
    except BaseException as error:
        details = {"kind": "fatal_error", "reason": f"{type(error).__name__}: {error}",
                   "traceback": traceback.format_exc()}
        _write_json(output / "worker_error.json", details)
        put_latest(queue, details)
        raise
    finally:
        # Parent may be shutting down after its window closed, leaving queued
        # snapshots unread. Do not keep a child alive waiting for that pipe.
        queue.cancel_join_thread()


def validate_args(args):
    if (not np.isfinite(args.seconds) or args.seconds < 0 or args.frames < 0
            or not np.isfinite(args.hz) or args.hz <= 0 or args.threads < 1
            or not 1 <= args.video_fps <= 120):
        raise ValueError("invalid duration/frame count/rate/thread count")
    if args.fast and (not args.recording or args.viewer != "headless"):
        raise ValueError("--fast is only for offline --recording with --viewer headless")
    calibration = json.loads(args.calibration.read_text())
    if not args.recording:
        if not args.spacecal_profile.is_file() or calibration.get("spacecal_sha256") != file_sha256(args.spacecal_profile):
            raise ValueError("SpaceCal changed/missing since T-pose calibration")
    else:
        if not args.recording.is_file():
            raise ValueError("raw pose recording does not exist")
        run_meta = args.recording.parent / "run.json"
        if run_meta.is_file():
            expected = json.loads(run_meta.read_text()).get("calibration_sha256")
            if expected and expected != file_sha256(args.calibration):
                raise ValueError("recording used a different calibration; supply its original profile")
    return calibration


def apply_preview_pose(model, data, event):
    """Kinematic assignment only; deliberately never calls mj_step."""
    import mujoco
    qpos = np.asarray(event["qpos"], dtype=float)
    if qpos.shape != (model.nq,) or not np.isfinite(qpos).all():
        raise ValueError("preview requires finite qpos matching the model")
    if not np.isclose(np.linalg.norm(qpos[3:7]), 1., atol=1e-4):
        raise ValueError("preview root quaternion is not unit length")
    data.qpos[:] = qpos
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)


def main(argv=None):
    args = parser().parse_args(argv)
    calibration = validate_args(args)
    # Recorder EGL runs in its own process; leave the viewer's GLFW/OpenGL
    # platform unchanged. Mixing EGL and GLFW in one process is unnecessary.
    import mujoco
    from .retargeting import DEFAULT_G1_Q29
    model = mujoco.MjModel.from_xml_path(str(XML))
    data = mujoco.MjData(model)
    apply_preview_pose(model, data, {"qpos": np.r_[[0.,0.,.793,1.,0.,0.,0.], DEFAULT_G1_Q29]})
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    output = args.output or ROOT / "runtime/sparseposer_g1" / stamp
    output.mkdir(parents=True, exist_ok=False)
    _write_json(output / "run.json", {"mode": "g1_kinematic_preview", "viewer": args.viewer,
        "record_video": args.record, "calibration": str(args.calibration),
        "calibration_sha256": file_sha256(args.calibration), "height_m": calibration["height_m"],
        "spacecal_sha256": calibration.get("spacecal_sha256"), "source": str(args.recording) if args.recording else "openvr",
        "xml": str(XML), "xml_sha256": file_sha256(XML),
        "gmr_config_sha256": file_sha256(ROOT / "external/GMR/general_motion_retargeting/ik_configs/bvh_xsens_to_g1.json"),
        "source_provenance": str(ROOT / "SOURCE_PROVENANCE.json"),
        "robot_output": False, "physics_stepped": False, "twist2_policy_loaded": False,
        "note": "Finite GMR poses shown with fit warnings. This is NOT a dynamic balance or real robot test."})
    print(f"G1 KINEMATIC PREVIEW — NOT dynamics/policy, NO robot output.\nOutput: {output}", flush=True)
    if not args.recording:
        print("Keep HMD + both controllers + 5 trackers active. Input loss holds the last displayed pose.", flush=True)
    if args.viewer == "vr":
        print("Quest view: front; right thumbstick click recenters view. PC mirror is also shown.", flush=True)
    elif args.viewer == "desktop":
        print("PC G1 front view; no Quest scene submission.", flush=True)
    print("Ctrl+C stops. " + ("MP4 recording enabled." if args.record else "Automatic video recording OFF; use your screen recorder."), flush=True)
    context = mp.get_context("spawn")
    queue, stop = context.Queue(maxsize=8), context.Event()
    worker = context.Process(target=estimation_worker, args=(args, output, queue, stop, joint_signature(model)),
                             name="g1-sparseposer-input")
    viewer, recorder = None, None
    displayed, last_report, last_update = 0, 0., time.monotonic()
    status = {"kind": "starting", "reason": "waiting for tracking/inference"}
    completed, failure, interrupted = False, None, False
    try:
        if args.record:
            from .g1_preview_recording import AsyncG1PreviewRecorder
            recorder = AsyncG1PreviewRecorder(XML, output, fps=args.video_fps)
        if args.viewer == "vr":
            from .mujoco_openvr import OpenVRMuJoCoViewer, OpenVRViewerConfig
            viewer = OpenVRMuJoCoViewer(model, data, OpenVRViewerConfig(view="front", distance_m=2.2))
        elif args.viewer == "desktop":
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(model, data)
            viewer.cam.azimuth = 180.; viewer.cam.elevation = -12.; viewer.cam.distance = 2.7
            viewer.cam.lookat[:] = [0,0,.95]
        worker.start()
        started = time.monotonic()
        while True:
            pose_event = None
            while True:
                try:
                    event = queue.get_nowait()
                except Empty:
                    break
                if event["kind"] == "complete":
                    completed = True
                elif event["kind"] == "fatal_error":
                    failure = event["reason"]
                else:
                    status = event
                    if event["kind"] == "g1_kinematic_pose":
                        pose_event = event
            if pose_event:
                if args.viewer == "desktop":
                    with viewer.lock():
                        apply_preview_pose(model, data, pose_event)
                else:
                    apply_preview_pose(model, data, pose_event)
                displayed += 1
                last_update = time.monotonic()
            if time.monotonic()-last_update > 0.5 and status.get("kind") == "g1_kinematic_pose":
                status = {"kind": "stale_preview", "reason": "no fresh G1 pose; last displayed pose held"}
            elapsed = time.monotonic() - started
            if recorder:
                recorder.capture(data, elapsed, status)
            if viewer:
                if args.viewer == "vr":
                    viewer.update_state(data)
                    viewer.raise_if_failed()
                else:
                    with viewer.lock():
                        viewer.cam.lookat[:] = data.qpos[:3] + np.array([0.,0.,.15])
                    viewer.sync()
                if not viewer.is_running():
                    break
            if time.monotonic()-last_report >= 1.:
                print(f"displayed={displayed} {status['kind']} {status.get('reason', '')}", flush=True)
                last_report = time.monotonic()
            if failure or completed or (args.seconds > 0 and elapsed >= args.seconds):
                break
            if not worker.is_alive():
                # Terminal queue snapshots may be dropped during child exit;
                # its exclusive on-disk report is authoritative.
                error_file = output / "worker_error.json"
                summary_file = output / "worker_summary.json"
                if error_file.is_file():
                    failure = json.loads(error_file.read_text())["reason"]
                elif worker.exitcode:
                    failure = f"input/IK worker exited {worker.exitcode}; see worker_error.json"
                elif summary_file.is_file():
                    completed = True
                else:
                    failure = "input/IK worker ended without a completion report"
                break
            time.sleep(0.002)
    except KeyboardInterrupt:
        interrupted = True
        print("Stopping preview; " + ("finalizing video..." if recorder else "saving pose logs..."), flush=True)
    except Exception as error:
        failure = f"{type(error).__name__}: {error}"
        _write_json(output / "main_error.json", {"reason": failure, "traceback": traceback.format_exc()})
    finally:
        stop.set()
        if worker.pid is not None:
            worker.join(timeout=5.)
            if worker.is_alive():
                worker.terminate()  # only the child this preview created
                worker.join(timeout=3.)
        cleanup_errors = []
        if recorder:
            try:
                recorder.close()
            except Exception as error:
                cleanup_errors.append(f"video close: {error}")
        if viewer:
            try:
                viewer.close()
            except Exception as error:
                cleanup_errors.append(f"viewer close: {error}")
        if cleanup_errors:
            failure = "; ".join(([failure] if failure else []) + cleanup_errors)
        _write_json(output / "summary.json", {"display_updates": displayed,
            "interrupted": interrupted, "error": failure,
            "recording": recorder.stats if recorder else None,
            "robot_output": False, "physics_stepped": False, "twist2_policy_loaded": False})
        queue.close()
    print(f"Saved preview run: {output}", flush=True)
    if failure:
        raise RuntimeError(failure)
    return 0 if displayed else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, RuntimeError, OSError, KeyError) as error:
        raise SystemExit(f"G1 preview: {error}") from error
