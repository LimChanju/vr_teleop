#!/usr/bin/env python3
"""Run a finite synthetic UDP -> exported policy -> Isaac physics validation."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]
SCENARIOS = {
    "state": (2000, []),
    "tracking": (4500, ["--tracking-sweep"]),
    "whole_body": (5500, ["--whole-body-sweep"]),
}


def terminate_owned(process):
    if process is not None and process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=10)


def validate_completion(output, expected_steps, python, environment):
    """Require the full finite run, beyond coverage of the producer's phases.

    This orchestrator stays stdlib-only. The selected simulator interpreter
    reads NumPy traces, so system Python need not have NumPy installed.
    """
    output = Path(output)
    result = json.loads((output / "result.json").read_text())
    if result.get("status") != "teleop_stopped" or result.get("mode") != "teleop":
        raise RuntimeError(f"Simulator did not finish teleop: {result.get('status')}")
    if type(result.get("steps")) is not int or result["steps"] != expected_steps:
        raise RuntimeError(f"Simulator completed {result.get('steps')} steps; expected {expected_steps}")
    code = """
import json, sys
import numpy as np
expected = int(sys.argv[2])
with np.load(sys.argv[1], allow_pickle=False) as trace:
    if 'wall_time' not in trace or trace['wall_time'].shape != (expected,):
        raise ValueError('wall_time does not cover every requested step')
    for name in trace.files:
        if name == 'dt':
            continue
        if trace[name].ndim == 0 or trace[name].shape[0] != expected:
            raise ValueError(f'{name} has an incomplete or invalid trace row count')
print(json.dumps({'trace_rows': expected, 'trace_fields': trace.files}))
"""
    checked = subprocess.run([str(python), "-I", "-c", code, str(output / "trace.npz"), str(expected_steps)],
        env=environment, capture_output=True, text=True, timeout=30)
    if checked.returncode != 0:
        raise RuntimeError(f"Simulator trace completion check failed: {checked.stderr.strip()}")
    trace = json.loads(checked.stdout)
    return {"status": result["status"], "expected_steps": expected_steps,
            "completed_steps": result["steps"], **trace}


def save_receipt(output, receipt):
    """Keep startup failures even if run.py never created its result folder."""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    with (output / "scenario_analysis_harness.json").open("x") as stream:
        json.dump(receipt, stream, indent=2)
        stream.write("\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scenario", choices=SCENARIOS, default="state")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--python", type=Path, default=Path(os.environ.get(
        "G1_PYTHON", str(Path.home() / "anaconda3/envs/g1_teleop/bin/python"))))
    args = parser.parse_args()
    checkpoint = args.checkpoint.expanduser().resolve(strict=True)
    python = args.python.expanduser().resolve(strict=True)
    nominal = checkpoint.parent / "nominal_targets.json"
    if not nominal.is_file():
        parser.error(f"Missing nominal target file: {nominal}")
    if not os.access(python, os.X_OK):
        parser.error(f"Python is not executable: {python}")
    output = args.output.expanduser().absolute()
    if output.exists():
        parser.error("Choose a new --output; existing results are preserved")
    output.parent.mkdir(parents=True, exist_ok=True)
    # Keep startup logs outside output: run.py requires a new/empty output.
    logs = {kind: output.with_name(output.name + "." + kind + ".log")
            for kind in ("sim", "sender", "analysis")}
    if any(path.exists() for path in logs.values()):
        parser.error("A sibling log already exists; choose a new --output")
    steps, options = SCENARIOS[args.scenario]
    environment = dict(os.environ, G1_PYTHON=str(python), PYTHONNOUSERSITE="1")
    sim_command = ["bash", str(ROOT / "scripts/g1/run.sh"), "teleop", "--headless",
        "--device", args.device, "--num-envs", "1", "--steps", str(steps),
        "--checkpoint", str(checkpoint), "--use-exported-policy", "--record-trace",
        "--output", str(output)]
    sender_command = [str(python), str(ROOT / "scripts/g1/runtime_scenario.py"),
        *options, "--nominal", str(nominal), "--output", str(output / "scenario.jsonl")]
    analysis_command = [str(python), str(ROOT / "scripts/g1/runtime_scenario.py"),
        "--output", str(output / "scenario.jsonl"), "--analyze-run", str(output),
        "--require-tracking-quality"]
    processes = []
    started = time.monotonic()
    receipt = {"schema": "g1.synthetic_runtime_harness.v1", "scenario": args.scenario,
        "physical_quest_verified": False, "remote_server_verified": False,
        "commands": [sim_command, sender_command, analysis_command],
        "logs": {key: str(path) for key, path in logs.items()}, "status": "starting"}

    def interrupted(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        with logs["sim"].open("x") as log:
            sim = subprocess.Popen(sim_command, cwd=ROOT, env=environment,
                                   stdout=log, stderr=subprocess.STDOUT)
            processes.append(sim)
            deadline = time.monotonic() + 180
            while "[G1] Waiting for ALVR targets at" not in logs["sim"].read_text(errors="replace"):
                if sim.poll() is not None:
                    raise RuntimeError(f"Simulator exited before readiness: {sim.returncode}; {logs['sim']}")
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"Simulator readiness timeout; {logs['sim']}")
                time.sleep(.2)
            print(f"Simulator ready; sending synthetic {args.scenario} input", flush=True)
            with logs["sender"].open("x") as sender_log:
                sender = subprocess.Popen(sender_command, cwd=ROOT, env=environment,
                    stdout=sender_log, stderr=subprocess.STDOUT)
                processes.append(sender)
                if sender.wait(timeout=100) != 0:
                    raise RuntimeError(f"Synthetic sender failed; {logs['sender']}")
            if sim.wait(timeout=160) != 0:
                raise RuntimeError(f"Simulator failed; {logs['sim']}")
        receipt["completion"] = validate_completion(output, steps, python, environment)
        with logs["analysis"].open("x") as log:
            result = subprocess.run(analysis_command, cwd=ROOT, env=environment,
                                    stdout=log, stderr=subprocess.STDOUT, timeout=120)
        receipt["analysis_exit_code"] = result.returncode
        receipt["status"] = "passed" if result.returncode == 0 else "criteria_failed_inspect_analysis"
        print(f"Synthetic validation: {receipt['status']}; {output}", flush=True)
        return result.returncode
    except BaseException as error:
        receipt.update(status="interrupted" if isinstance(error, KeyboardInterrupt) else "execution_failed",
                       error=f"{type(error).__name__}: {error}")
        raise
    finally:
        for process in reversed(processes):
            terminate_owned(process)
        receipt["elapsed_seconds"] = time.monotonic() - started
        # Preserve successful physics output even if a quality criterion fails.
        save_receipt(output, receipt)


if __name__ == "__main__":
    raise SystemExit(main())
