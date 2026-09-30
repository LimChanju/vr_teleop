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
from g1_teleop.vr.calibration import TargetMapper
from scripts.g1.runtime_scenario import (SWEEP_AXES, SWEEP_PHASES, analyze_scenario,
                                         axis_tracking_metrics, make_sweep_raw, phase_plan)


class RuntimeScenarioTests(unittest.TestCase):
    def test_axis_tracking_detects_small_response_despite_high_correlation(self):
        target = np.tile(DEFAULT_NOMINAL, (101, 1, 1))
        wave = 0.06 * np.sin(np.linspace(0, 2 * np.pi, 101))
        target[:, 1, 2] += wave
        target[:, 2, 0] += wave
        actual = np.tile(DEFAULT_NOMINAL, (101, 1, 1))
        actual[:, 1, 2] += 0.07 * wave
        actual[:, 2, 0] += 0.9 * wave + 0.01
        report = axis_tracking_metrics(target, actual)
        self.assertEqual(set(report), {"left_hand.z", "right_hand.x"})
        self.assertAlmostEqual(report["left_hand.z"]["ls_gain"], 0.07)
        self.assertAlmostEqual(report["left_hand.z"]["correlation"], 1.0)
        self.assertFalse(report["left_hand.z"]["provisional_quality_passed"])
        self.assertAlmostEqual(report["right_hand.x"]["range_ratio"], 0.9)
        self.assertTrue(report["right_hand.x"]["provisional_quality_passed"])

    def test_sweep_mapper_excites_only_declared_axis_and_returns_neutral(self):
        self.assertAlmostEqual(sum(duration for _, duration, _ in phase_plan(tracking_sweep=True)), 59.0)
        mapper = TargetMapper(nominal=DEFAULT_NOMINAL)
        mapper.update(make_sweep_raw("calibrate", 0, 0.65, 0.5))
        mapper.update(make_sweep_raw("tracking_warmup", 0, 0.65, 2))
        for phase, (point, axis) in SWEEP_AXES.items():
            targets = []
            for fraction in np.linspace(0, 1, 151):
                raw = make_sweep_raw(phase, fraction, 0.65, 5)
                frame = mapper.update(raw)
                self.assertTrue(frame.enabled)
                self.assertGreaterEqual(min(raw.grips), 0.7)
                targets.append(frame.positions - DEFAULT_NOMINAL)
            targets = np.asarray(targets)
            np.testing.assert_allclose(targets[[0, -1]], 0, atol=1e-12)
            wanted = targets[:, point, axis].copy()
            targets[:, point, axis] = 0
            np.testing.assert_allclose(targets, 0, atol=1e-12)
            if (point, axis) == (0, 2):
                self.assertLessEqual(wanted.max(), 1e-12)
                self.assertLess(wanted.min(), -0.039)
            else:
                amplitude = 0.025 if point == 0 else 0.06
                self.assertGreater(wanted.max(), 0.9 * amplitude)
                self.assertLess(wanted.min(), -0.9 * amplitude)
                self.assertLessEqual(np.abs(wanted).max(), amplitude + 1e-12)

    def test_sweep_analysis_separates_control_states_from_tracking_quality(self):
        # Mathematical trace fixture only; not simulator or hardware evidence.
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            rows = [dict(kind="start", scenario_mode="tracking_sweep", phase_plan=[
                dict(phase=name, duration_s=duration, expected_enabled=enabled)
                for name, duration, enabled in SWEEP_PHASES])]
            mapper = TargetMapper(nominal=DEFAULT_NOMINAL)
            wall, targets, enabled = [], [], []
            start = 10.0
            for phase, duration, expected in SWEEP_PHASES:
                rows.append(dict(kind="phase_begin", phase=phase, expected_enabled=expected,
                                 timestamp_unix_ns=round(start * 1e9)))
                count = round(duration * 50)
                for offset in np.arange(count) / 50:
                    frame = mapper.update(make_sweep_raw(phase, offset / duration, 0.65, duration))
                    sequence = len(wall)
                    rows.append(dict(kind="frame", phase=phase, target=dict(seq=sequence,
                        enabled=frame.enabled, target_positions_m=frame.positions.tolist())))
                    wall.append(start + offset)
                    targets.append(frame.positions)
                    enabled.append(frame.enabled)
                start += duration
                rows.append(dict(kind="phase_end", phase=phase, timestamp_unix_ns=round(start * 1e9)))
            rows.append(dict(kind="summary", producer_completed=True))
            events = directory / "events.jsonl"
            events.write_text("\n".join(json.dumps(row) for row in rows))
            (directory / "result.json").write_text(json.dumps(dict(mode="teleop", status="teleop_stopped",
                input_sources=["synthetic"], falls=0, manual_resets=0, physical_quest_verified=False)))
            n = len(wall)
            actual = np.asarray(targets).copy()
            actual[:, 1, 2] = DEFAULT_NOMINAL[1, 2]
            np.savez(directory / "trace.npz", wall_time=wall, input_sequence=np.arange(n),
                input_fresh=np.ones(n, dtype=bool), manual_reset=np.zeros(n, dtype=bool), enabled=enabled,
                done=np.zeros(n, dtype=bool), q=np.zeros((n, 29)), action=np.zeros((n, 29)),
                target=targets, actual=actual)
            report = analyze_scenario(events, directory)
            self.assertTrue(report["control_state_passed"], report["checks"])
            self.assertTrue(report["simulator_checks_passed"])
            self.assertFalse(report["tracking_quality_passed"])
            self.assertEqual(report["tracking_quality"]["flagged_axes"], ["left_hand.z"])
            self.assertEqual(len(report["tracking_quality"]["axis_results"]), 9)
            self.assertTrue(report["checks"]["no_manual_reset"]["passed"])
            self.assertFalse(report["physical_quest_verified"])
            json.dumps(report, allow_nan=False)

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
