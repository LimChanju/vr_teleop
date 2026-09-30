"""Run the finite sender against actual localhost UDP, without any simulator."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from g1_teleop.runtime import TeleopGate
from g1_teleop.vr.protocol import DEFAULT_NOMINAL, UDPReceiver
from scripts.g1.runtime_scenario import analyze_scenario


class RuntimeScenarioTests(unittest.TestCase):
    def test_real_udp_outage_held_arm_reset_and_stop(self):
        with tempfile.TemporaryDirectory() as directory, UDPReceiver(port=0) as receiver:
            directory = Path(directory)
            nominal = directory / "nominal.json"
            nominal.write_text(json.dumps({"nominal_targets": DEFAULT_NOMINAL.tolist()}))
            output = directory / "scenario.jsonl"
            command = [sys.executable, str(ROOT / "scripts/g1/runtime_scenario.py"),
                       "--nominal", str(nominal), "--port", str(receiver.address[1]),
                       "--output", str(output), "--phase-seconds", "0.3"]
            gate = TeleopGate()
            samples = []
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            deadline = time.monotonic() + 12
            try:
                while process.poll() is None and time.monotonic() < deadline:
                    frame = receiver.poll()
                    enabled = gate.update(frame)
                    samples.append((frame.seq, frame.fresh, enabled, frame.reset, gate.reason))
                    time.sleep(0.005)
                if process.poll() is None:
                    process.kill()
                    self.fail("scenario exceeded bounded test runtime")
                stdout, stderr = process.communicate(timeout=1)
                self.assertEqual(process.returncode, 0, stdout + stderr)
                final = receiver.poll()
                self.assertFalse(gate.update(final))
                self.assertFalse(final.enabled)
            finally:
                if process.poll() is None:
                    process.kill()
                process.communicate()

            records = [json.loads(line) for line in output.read_text().splitlines()]
            summary = records[-1]
            self.assertTrue(summary["producer_completed"])
            self.assertFalse(summary["physical_quest_verified"])
            self.assertFalse(summary["receiver_verified"])
            self.assertFalse(summary["simulator_verified"])
            self.assertEqual(summary["reset_id"], 1)
            frames = {r["target"]["seq"]: r for r in records if r["kind"] == "frame"}
            observed = {}
            for seq, fresh, active, reset, reason in samples:
                if seq in frames and fresh:
                    observed.setdefault(frames[seq]["phase"], []).append(active)
            for phase in ("armed_motion", "rearm_motion", "rearm_after_pause", "post_reset_rearm"):
                self.assertTrue(any(observed.get(phase, [])), phase)
            for phase in ("idle", "calibrate", "deadman_release", "held_arm_after_pause",
                          "arm_release", "reset", "final_stop"):
                self.assertTrue(observed.get(phase), f"no UDP observation of {phase}")
                self.assertFalse(any(observed[phase]), phase)
            self.assertTrue(any(not fresh and reason == "input_timeout" for _, fresh, _, _, reason in samples))
            self.assertEqual(sum(reset for _, _, _, reset, _ in samples), 1)
            self.assertNotIn("producer_pause", summary["sent_frames"])
            pause = next(r for r in records if r["kind"] == "phase_end" and r["phase"] == "producer_pause")
            self.assertGreaterEqual(pause["actual_duration_s"], 0.6)
            held = [r for r in frames.values() if r["phase"] == "held_arm_after_pause"]
            self.assertTrue(all(r["raw"]["arm"] and not r["target"]["enabled"] for r in held))
            motion = np.array([r["target"]["target_positions_m"] for r in frames.values()
                               if r["phase"] == "armed_motion"])
            self.assertGreater(np.max(np.abs(motion[:, 1, 2] - DEFAULT_NOMINAL[1, 2])), 0.055)
            self.assertLessEqual(np.max(np.abs(motion[:, 1, 2] - DEFAULT_NOMINAL[1, 2])), 0.060001)
            # Existing evidence must survive a second invocation unchanged.
            before = output.read_bytes()
            duplicate = subprocess.run(command, capture_output=True, text=True, timeout=5)
            self.assertNotEqual(duplicate.returncode, 0)
            self.assertEqual(output.read_bytes(), before)

    def test_analysis_rejects_missing_phases_wrong_enable_and_nonfinite_trace(self):
        # Deliberately invalid trace fixture, not claimed as simulator evidence.
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            events = directory / "events.jsonl"
            rows = [
                dict(kind="phase_begin", phase="held_arm_after_pause", timestamp_unix_ns=10_000_000_000),
                dict(kind="phase_end", phase="held_arm_after_pause", timestamp_unix_ns=12_000_000_000),
                dict(kind="frame", phase="held_arm_after_pause",
                     target=dict(seq=0, enabled=False, target_positions_m=DEFAULT_NOMINAL.tolist())),
                dict(kind="summary", producer_completed=True),
            ]
            events.write_text("\n".join(json.dumps(row) for row in rows))
            (directory / "result.json").write_text(json.dumps(dict(
                mode="teleop", status="teleop_stopped", input_sources=["synthetic"],
                physical_quest_verified=False, falls=0, manual_resets=0)))
            n = 101
            trace = dict(wall_time=np.linspace(10, 12, n), input_sequence=np.zeros(n, dtype=int),
                         input_fresh=np.ones(n, dtype=bool), manual_reset=np.zeros(n, dtype=bool),
                         enabled=np.zeros(n, dtype=bool), done=np.zeros(n, dtype=bool),
                         q=np.zeros((n, 29)), action=np.zeros((n, 29)),
                         target=np.tile(DEFAULT_NOMINAL, (n, 1, 1)), actual=np.tile(DEFAULT_NOMINAL, (n, 1, 1)))
            trace["enabled"][50] = True
            trace["actual"][30, 1, 2] = np.nan
            np.savez(directory / "trace.npz", **trace)
            report = analyze_scenario(events, directory)
            self.assertFalse(report["simulator_checks_passed"])
            self.assertFalse(report["checks"]["phase_held_arm_after_pause"]["passed"])
            self.assertFalse(report["checks"]["phase_armed_motion"]["passed"])
            self.assertFalse(report["checks"]["finite_state_action_targets"]["passed"])
            self.assertFalse(report["checks"]["active_inputs_match_sent_targets"]["passed"])
            self.assertFalse(report["checks"]["one_manual_reset"]["passed"])
            json.dumps(report, allow_nan=False)  # malformed state still yields a readable failure report
            trace.pop("wall_time")
            np.savez(directory / "trace.npz", **trace)
            with self.assertRaisesRegex(ValueError, "lacks required evidence"):
                analyze_scenario(events, directory)


if __name__ == "__main__":
    unittest.main()
