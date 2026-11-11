"""One-command SparsePoser -> official TWIST2 ONNX -> MuJoCo dynamics.

Only the local simulation controller is imported. The existing policy,
observations, PD gains, torque limits, and physics loop are not modified.
The wrapper provides isolated IPC, input supervision and measurements.
"""
from __future__ import annotations

import argparse
import errno
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--calibration", type=Path, required=True, help="explicit current T-pose profile")
    p.add_argument("--recording", type=Path, help="optional raw_poses.jsonl replay, instead of live VR input")
    p.add_argument("--viewer", choices=("vr", "desktop", "headless"), default="vr")
    p.add_argument("--seconds", type=float, default=0, help="simulated seconds; 0 until Ctrl+C")
    p.add_argument("--redis-port", type=int, default=16479, help="dedicated localhost port; refuses occupied ports")
    p.add_argument("--output", type=Path, help="new run directory")
    p.add_argument("--neutral-only", action="store_true", help="policy baseline; do not read tracking or send motion")
    return p


def tilt_deg(q):
    return float(np.degrees(np.arccos(np.clip(1-2*(q[1]**2+q[2]**2), -1, 1))))


def stop_owned(process):
    if process is not None and process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def check_simulation_port(port):
    """Match Redis's restart semantics without allowing an active listener.

    Closed server connections can leave the local port in TIME_WAIT. Redis
    uses SO_REUSEADDR to restart immediately; the preflight must do the same.
    This does NOT use SO_REUSEPORT or attach to an existing Redis. The later
    startup/PID check still protects against a race after this socket closes.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind(("127.0.0.1", port))
        except OSError as exc:
            if exc.errno == errno.EADDRINUSE:
                raise RuntimeError(
                    f"Simulation port 127.0.0.1:{port} is already in use. "
                    "Stop the previous simulation, or choose a different --redis-port. "
                    "No existing server was stopped or reused."
                ) from exc
            raise


def main(argv=None):
    args = parser().parse_args(argv)
    if not np.isfinite(args.seconds) or args.seconds < 0 or not 1024 <= args.redis_port <= 65535:
        raise ValueError("invalid duration/port")
    if args.viewer == "headless" and args.seconds <= 0:
        raise ValueError("headless tests require a finite --seconds")
    args.calibration = args.calibration.resolve(strict=True)
    if args.recording:
        args.recording = args.recording.resolve(strict=True)
    # Do not attach to an existing Redis (including real-robot infrastructure).
    check_simulation_port(args.redis_port)
    output = args.output or ROOT / "runtime/sparseposer_physics" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    sys.path.insert(0, str(ROOT / "deploy_real"))
    import server_low_level_g1_sim as controller_module
    import redis
    from .controller import RedisSimulationSink
    from .models import RobotReference
    from .retargeting import DEFAULT_MIMIC_OBS
    from .sparseposer_adapter import file_sha256
    redis_proc = input_proc = sink = None
    original_step = controller_module.mujoco.mj_step
    original_viewer = controller_module.mjv.launch_passive
    original_sigterm = signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    steps, min_height, max_tilt, first_fall = 0, float("inf"), 0., None
    error = None
    last_input_state = None
    previous_publish_guard = os.environ.get("TWIST2_ALLOW_SIM_REDIS")
    started = time.monotonic()
    policy_path = ROOT / "assets/ckpts/twist2_1017_20k.onnx"
    xml = ROOT / "assets/g1/g1_sim2sim_29dof.xml"
    metadata = {"mode": "twist2_policy_mujoco_dynamics", "viewer": args.viewer,
                "calibration": str(args.calibration), "calibration_sha256": file_sha256(args.calibration),
                "recording": str(args.recording) if args.recording else None,
                "policy": str(policy_path), "policy_sha256": file_sha256(policy_path),
                "xml_sha256": file_sha256(xml), "neutral_only": args.neutral_only,
                "physics_stepped": True, "robot_output": False, "redis_port": args.redis_port,
                "policy_frequency_hz": 50, "physics_frequency_hz": 1000,
                "input_behavior": "exact valid references; neutral fallback on invalid/stale/lost input; 2s initial neutral"}
    (output / "run.json").write_text(json.dumps(metadata, indent=2)+"\n")
    print(f"SIMULATION ONLY: gravity/contact + TWIST2 policy. Robot output disabled.\nLogs: {output}", flush=True)
    print("No added motion clamps/smoothing. Falls are possible. Ctrl+C to stop; no automatic reset.", flush=True)
    try:
        with (output / "redis.log").open("x") as redis_log, (output / "input.log").open("x") as input_log, \
                (output / "physics.jsonl").open("x") as physics_log:
            redis_proc = subprocess.Popen([str(ROOT / ".conda-vr/bin/redis-server"), "--bind", "127.0.0.1",
                "--port", str(args.redis_port), "--protected-mode", "yes", "--save", "", "--appendonly", "no",
                "--dir", str(output)], stdout=redis_log, stderr=subprocess.STDOUT)
            client = redis.Redis(host="127.0.0.1", port=args.redis_port, socket_timeout=.5)
            for _ in range(100):
                if redis_proc.poll() is not None:
                    raise RuntimeError("isolated Redis failed to start")
                try:
                    if client.ping():
                        if client.info("server")["process_id"] != redis_proc.pid:
                            raise RuntimeError("port belongs to a different Redis; refusing connection")
                        break
                except redis.exceptions.ConnectionError:
                    time.sleep(.05)
            else:
                raise TimeoutError("isolated Redis startup timed out")
            os.environ["TWIST2_ALLOW_SIM_REDIS"] = "1"
            sink = RedisSimulationSink(host="127.0.0.1", port=args.redis_port, allow_simulation_publish=True)
            if not args.neutral_only:
                command = [str(ROOT / ".venv-sparseposer/bin/python"), "-u", "-m", "twist2_vr.sparseposer_sim_input",
                           "--calibration", str(args.calibration), "--redis-port", str(args.redis_port),
                           "--output", str(output / "input")]
                if args.recording:
                    command += ["--recording", str(args.recording)]
                input_proc = subprocess.Popen(command, cwd=ROOT, stdout=input_log, stderr=subprocess.STDOUT)
            if args.viewer == "headless":
                from scripts.headless_sim_smoke import HeadlessViewer
                controller_module.mjv.launch_passive = lambda *a, **k: HeadlessViewer()
            controller = controller_module.RealTimePolicyController(
                xml_file=str(xml), policy_path=str(policy_path), device="cpu", policy_frequency=50,
                viewer_mode="desktop" if args.viewer == "headless" else args.viewer,
                vr_view="front", redis_host="127.0.0.1", redis_port=args.redis_port,
                action_timeout_ms=300, limit_fps=True)
            if args.seconds:
                controller.sim_duration = args.seconds
            def measured_step(model, data):
                nonlocal steps, min_height, max_tilt, first_fall, last_input_state
                original_step(model, data)
                steps += 1
                z, tilt = float(data.qpos[2]), tilt_deg(data.qpos[3:7])
                if not np.all(np.isfinite(data.qpos)):
                    raise RuntimeError("nonfinite physics state")
                min_height, max_tilt = min(min_height, z), max(max_tilt, tilt)
                if first_fall is None and (z < .45 or tilt > 60):
                    first_fall = float(data.time)
                    print(f"FALL INDICATOR at sim {data.time:.2f}s: pelvis_z={z:.3f}m tilt={tilt:.1f}deg", flush=True)
                if steps % 50 == 0:
                    if args.neutral_only:
                        sink.publish(RobotReference(time.monotonic(), DEFAULT_MIMIC_OBS, None, True, "neutral_baseline"))
                    elif input_proc.poll() is not None:
                        if input_proc.returncode != 0:
                            raise RuntimeError(f"input failed; see {output / 'input.log'}")
                        print("Input replay completed; stopping simulation.", flush=True)
                        raise KeyboardInterrupt
                    physics_log.write(json.dumps({"sim_s": float(data.time), "monotonic_s": time.monotonic(),
                        "root_height_m": z, "tilt_deg": tilt, "qpos": data.qpos.tolist()})+"\n")
                if steps % 1000 == 0:
                    physics_log.flush()
                    state = client.get("sparseposer_input_state")
                    if state != last_input_state:
                        print(f"SIM INPUT: {state.decode() if state else 'waiting for input model'}", flush=True)
                        last_input_state = state
            controller_module.mujoco.mj_step = measured_step
            controller.run()
    except KeyboardInterrupt:
        print("Stopped.", flush=True)
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        controller_module.mujoco.mj_step = original_step
        controller_module.mjv.launch_passive = original_viewer
        stop_owned(input_proc)
        try:
            if sink is not None:
                sink.close()
        except Exception as exc:
            print(f"Neutral shutdown write failed: {exc}", flush=True)
        finally:
            stop_owned(redis_proc)
        if previous_publish_guard is None:
            os.environ.pop("TWIST2_ALLOW_SIM_REDIS", None)
        else:
            os.environ["TWIST2_ALLOW_SIM_REDIS"] = previous_publish_guard
        signal.signal(signal.SIGTERM, original_sigterm)
        report = {"physics_steps": steps, "simulated_seconds": steps*.001,
                  "wall_seconds": time.monotonic()-started, "root_height_min_m": min_height if steps else None,
                  "base_tilt_max_deg": max_tilt if steps else None, "first_fall_indicator_sim_s": first_fall,
                  "error": error, "robot_output": False, "physics_stepped": steps > 0}
        (output / "summary.json").write_text(json.dumps(report, indent=2)+"\n")
        print(json.dumps(report, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
