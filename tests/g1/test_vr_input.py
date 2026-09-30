"""Hardware-free tests for geometry, fail-stop controls and UDP/replay transport."""

from __future__ import annotations

import dataclasses
import json
import math
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock
import uuid

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from g1_teleop.vr.calibration import RawFrame, TargetMapper, valid_poses
from g1_teleop.vr.openvr_source import SyntheticSource
from g1_teleop.vr.protocol import TargetFrame, UDPReceiver, decode_target, encode_target
from g1_teleop.vr.replay import ReplaySource
from scripts.g1.alvr_input import load_target_bounds, parse_args
from scripts.g1 import alvr_input


def raw_frame():
    return SyntheticSource(enabled=True).sample()


def target(**kwargs):
    return TargetFrame(session=str(uuid.uuid4()), seq=0, timestamp_unix_ns=time.time_ns(),
                       tracking_valid=True, calibrated=True, **kwargs)


class MappingTests(unittest.TestCase):
    def test_cli_velocity_limits_validate_caps_zero_axes_and_replay(self):
        self.assertIsNone(parse_args([]).velocity_limits)
        self.assertEqual(parse_args(["--velocity-limits", ".15", ".08", "0"]).velocity_limits, [0.15, 0.08, 0])
        self.assertEqual(parse_args(["--velocity-limits", "0", "0", "0"]).velocity_limits, [0, 0, 0])
        for values in [("nan", "0", "0"), ("0", "inf", "0"), ("-.1", "0", "0"),
                       (".51", "0", "0"), ("0", ".26", "0"), ("0", "0", ".61")]:
            with self.subTest(values=values), mock.patch("sys.stderr"), self.assertRaises(SystemExit):
                parse_args(["--velocity-limits", *values])
        with mock.patch("sys.stderr"), self.assertRaises(SystemExit):
            parse_args(["--backend", "replay", "--replay-file", "input.jsonl", "--velocity-limits", ".15", ".08", "0"])

    def test_asymmetric_small_bounds_apply_after_robot_transform_and_scale(self):
        bounds = load_target_bounds(ROOT / "config/g1/teleop_small_motion.json")
        mapper = TargetMapper(scale=0.4, **bounds)
        raw = raw_frame()
        mapper.update(raw)
        neutral = raw.poses.copy()
        raw.calibrate, raw.arm = False, True
        displacement = np.ones((3, 3)) @ mapper.basis / mapper.scale
        raw.poses[:, :, 3] += displacement
        frame = mapper.update(raw)
        self.assertTrue(frame.enabled)
        np.testing.assert_allclose(frame.positions - mapper.nominal, bounds["delta_max"], atol=1e-10)
        self.assertAlmostEqual(frame.positions[0, 2], mapper.nominal[0, 2])  # No head motion above neutral.
        raw.poses = neutral.copy()
        raw.poses[:, :, 3] -= displacement
        frame = mapper.update(raw)
        np.testing.assert_allclose(frame.positions - mapper.nominal, bounds["delta_min"], atol=1e-10)
        raw.calibrate, raw.arm = True, False
        frame = mapper.update(raw)
        np.testing.assert_allclose(frame.positions, mapper.nominal)  # Recalibration changes the zero point.
        self.assertFalse(frame.enabled)
        bounds["delta_min"][:] = -1  # Caller mutations cannot silently change the live envelope.
        self.assertAlmostEqual(mapper.delta_min[0, 2], -0.04)

    def test_bounds_validate_shapes_finiteness_neutral_and_nonzero_span(self):
        lower, upper = -np.ones((3, 3)) * 0.05, np.ones((3, 3)) * 0.05
        for bad_lower, bad_upper in [(None, upper), (lower, None),
                (lower[:2], upper), (np.full((3, 3), np.nan), upper),
                (lower, np.full((3, 3), np.inf)), (np.ones((3, 3)), upper),
                (lower, -np.ones((3, 3))), (np.zeros((3, 3)), np.zeros((3, 3))),
                (lower.astype(str), upper), (np.zeros((3, 3), dtype=bool), upper)]:
            with self.subTest(lower=bad_lower, upper=bad_upper), self.assertRaises(ValueError):
                TargetMapper(delta_min=bad_lower, delta_max=bad_upper)
        with self.assertRaisesRegex(ValueError, "not both"):
            TargetMapper(max_delta=upper, delta_min=lower, delta_max=upper)
        mapper = TargetMapper(max_delta=upper)
        np.testing.assert_allclose(mapper.delta_min, lower)
        np.testing.assert_allclose(mapper.delta_max, upper)

    def test_bounds_json_units_axes_and_replay_option_are_explicit(self):
        value = json.loads((ROOT / "config/g1/teleop_small_motion.json").read_text())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bounds.json"
            for key, invalid in [("units", "cm"), ("coordinate_version", "OpenVR"),
                                 ("point_order", ["left_hand", "right_hand", "head"]),
                                 ("delta_max", [[0, 0, 0]]), ("delta_min", None)]:
                path.write_text(json.dumps({**value, key: invalid}))
                with self.subTest(key=key), self.assertRaises(ValueError):
                    load_target_bounds(path)
        args = parse_args(["--backend", "synthetic", "--target-bounds", "bounds.json"])
        self.assertEqual(args.target_bounds, Path("bounds.json"))
        with mock.patch("sys.stderr"), self.assertRaises(SystemExit):
            parse_args(["--backend", "replay", "--replay-file", "record.jsonl", "--target-bounds", "bounds.json"])

    def test_neutral_frame_does_not_use_human_height_as_robot_height(self):
        raw = raw_frame()
        mapper = TargetMapper()
        frame = mapper.update(raw)
        np.testing.assert_allclose(frame.positions, mapper.nominal)
        self.assertFalse(frame.enabled)
        raw.calibrate = False
        raw.arm = True
        raw.poses[1, :, 3] += [0.02, 0.04, -0.1]
        frame = mapper.update(raw)
        np.testing.assert_allclose(frame.positions[1] - mapper.nominal[1], [0.065, -0.013, 0.026])
        self.assertTrue(frame.enabled)

    def test_arbitrary_neutral_yaw_is_removed(self):
        raw = raw_frame()
        theta = 1.1
        c, s = math.cos(theta), math.sin(theta)
        rotation = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        raw.poses = np.einsum("ij,bjk->bik", rotation, raw.poses)
        mapper = TargetMapper(scale=1)
        mapper.update(raw)
        raw.calibrate = False
        raw.poses[1, :, 3] += rotation @ [0, 0, -0.1]
        frame = mapper.update(raw)
        np.testing.assert_allclose(frame.positions[1] - mapper.nominal[1], [0.1, 0, 0], atol=1e-10)

    def test_tracking_loss_stops_and_requires_explicit_rearm(self):
        mapper = TargetMapper()
        raw = raw_frame()
        mapper.update(raw)
        raw.calibrate, raw.arm = False, True
        self.assertTrue(mapper.update(raw).enabled)
        raw.tracking_valid = False
        self.assertFalse(mapper.update(raw).enabled)
        raw.tracking_valid = True
        self.assertFalse(mapper.update(raw).enabled)  # held A must not rearm
        raw.arm = False
        mapper.update(raw)
        raw.arm = True
        self.assertTrue(mapper.update(raw).enabled)
        raw.grips = (0, 0.8)
        self.assertFalse(mapper.update(raw).enabled)

    def test_reset_stop_and_tracking_origin_change(self):
        mapper = TargetMapper()
        raw = raw_frame()
        mapper.update(raw)
        raw.calibrate, raw.arm = False, True
        self.assertFalse(mapper.update(raw, "stop").enabled)
        self.assertTrue(mapper.update(raw, "arm").enabled)
        raw.reset = True
        frame = mapper.update(raw, "arm")
        self.assertFalse(frame.enabled)
        self.assertEqual(frame.reset_id, 1)
        self.assertEqual(mapper.update(raw).reset_id, 1)
        raw.reference_changed = True
        self.assertFalse(mapper.update(raw).calibrated)

    def test_inactive_actions_or_sender_pause_cannot_resume_with_held_arm(self):
        mapper = TargetMapper()
        raw = raw_frame()
        mapper.update(raw)
        raw.calibrate, raw.arm = False, True
        self.assertTrue(mapper.update(raw).enabled)
        # An inactive action API zeros buttons; a held A may reappear on recovery.
        raw.inputs_valid, raw.arm = False, False
        self.assertFalse(mapper.update(raw).enabled)
        raw.inputs_valid, raw.arm = True, True
        self.assertFalse(mapper.update(raw).enabled)
        raw.arm = False
        mapper.update(raw)
        raw.arm = True
        self.assertTrue(mapper.update(raw).enabled)
        mapper._last_update -= 1.0
        self.assertFalse(mapper.update(raw).enabled)

    def test_bad_or_stale_pose_and_bad_inputs_fail_stopped(self):
        mapper = TargetMapper()
        raw = raw_frame()
        mapper.update(raw)
        raw.calibrate, raw.arm = False, True
        raw.sampled_monotonic -= 1
        self.assertFalse(mapper.update(raw).tracking_valid)
        raw.sampled_monotonic = time.monotonic()
        raw.poses[0, 0, 0] = float("nan")
        self.assertFalse(mapper.update(raw).enabled)
        raw = raw_frame()
        raw.left_stick = (float("nan"), 0)
        self.assertFalse(mapper.update(raw).tracking_valid)
        raw = raw_frame()
        raw.inputs_valid = False
        self.assertFalse(mapper.update(raw).tracking_valid)
        reflected = raw.poses.copy()
        reflected[:, :, 0] *= -1
        self.assertFalse(valid_poses(reflected))

    def test_workspace_clamp_and_velocity_signs(self):
        mapper = TargetMapper()
        raw = raw_frame()
        mapper.update(raw)
        raw.calibrate, raw.arm = False, True
        raw.poses[1, 2, 3] -= 1
        raw.left_stick, raw.right_stick = (1, 1), (1, 0)
        frame = mapper.update(raw)
        np.testing.assert_allclose(frame.positions[1] - mapper.nominal[1], [0.3, 0, 0])
        np.testing.assert_allclose(frame.velocity, [0.5, -0.25, -0.6])


class ProtocolTests(unittest.TestCase):
    def test_cli_velocity_limits_reach_real_mapper_and_disable_yaw(self):
        with tempfile.TemporaryDirectory() as directory, UDPReceiver(port=0) as receiver:
            record = Path(directory) / "velocity.jsonl"
            args = parse_args(["--backend", "synthetic", "--enable-synthetic", "--duration", ".1",
                "--port", str(receiver.address[1]), "--record", str(record), "--velocity-limits", ".15", ".08", "0"])
            source = SyntheticSource(enabled=True)
            original_sample = source.sample
            def stick_sample():
                raw = original_sample()
                raw.left_stick, raw.right_stick = (1, 0.6), (-1, 0)
                return raw
            with mock.patch.object(source, "sample", side_effect=stick_sample), \
                 mock.patch.object(alvr_input, "SyntheticSource", return_value=source), \
                 mock.patch.object(alvr_input, "parse_args", return_value=args), mock.patch("sys.stdout"):
                self.assertEqual(alvr_input.main(), 0)
            active = [json.loads(line)["target"] for line in record.read_text().splitlines()
                      if json.loads(line)["target"]["enabled"]]
            self.assertGreater(len(active), 0)
            # Forward 0.6 becomes 0.5 after the existing 0.2 deadzone; rightward
            # left stick is negative robot Y; a full yaw stick remains disabled.
            for frame in active:
                np.testing.assert_allclose(frame["command_velocity"], [0.075, -0.08, 0])

    def test_sender_process_applies_selected_asymmetric_bounds(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            bounds_path, record = directory / "bounds.json", directory / "record.jsonl"
            lower = np.full((3, 3), -0.001)
            upper = np.full((3, 3), 0.001)
            upper[0, 2] = 0
            bounds_path.write_text(json.dumps(dict(units="m", delta_min=lower.tolist(), delta_max=upper.tolist())))
            with UDPReceiver(port=0) as receiver:
                command = [sys.executable, str(ROOT / "scripts/g1/alvr_input.py"),
                    "--backend", "synthetic", "--enable-synthetic", "--duration", "0.4",
                    "--port", str(receiver.address[1]), "--target-bounds", str(bounds_path), "--record", str(record)]
                result = subprocess.run(command, capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("provisional target envelope", result.stdout)
            entries = [json.loads(line) for line in record.read_text().splitlines()]
            positions = np.asarray([entry["target"]["target_positions_m"] for entry in entries])
            delta = positions - positions[0]
            self.assertTrue(np.all(delta >= lower - 1e-10))
            self.assertTrue(np.all(delta <= upper + 1e-10))
            self.assertAlmostEqual(delta[:, 1, 2].max(), 0.001)

    def test_schema_values_and_wallclock_are_validated(self):
        frame = target(enabled=True)
        data = encode_target(frame)
        parsed = decode_target(data, now_unix_ns=time.time_ns())
        np.testing.assert_allclose(parsed.positions, frame.positions)
        self.assertTrue(parsed.enabled)
        cases = [("schema", "other"), ("frame", "openvr"), ("coordinate_version", "root-relative"), ("seq", True),
                 ("enabled", 1), ("session", "bad"), ("command_velocity", [99, 0, 0]),
                 ("target_positions_m", [[0, 0, "1"]] * 3), ("tracking_valid", False)]
        for key, value in cases:
            with self.subTest(key=key):
                packet = json.loads(data)
                packet[key] = value
                with self.assertRaises(ValueError):
                    decode_target(json.dumps(packet).encode())
        with self.assertRaises(ValueError):
            decode_target(data, now_unix_ns=time.time_ns() + 5_000_000_000)
        with self.assertRaises(ValueError):
            decode_target(b"x" * 8193)

    def test_udp_stale_reorder_session_and_reset_deduplication(self):
        with UDPReceiver(port=0, timeout_s=0.03) as receiver, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            self.assertFalse(receiver.poll().fresh)
            first = target(enabled=True, reset_id=1)
            sender.sendto(encode_target(first), receiver.address)
            self.assertTrue(receiver.poll().reset)
            self.assertFalse(receiver.poll().reset)
            sender.sendto(encode_target(first), receiver.address)
            self.assertTrue(receiver.poll().enabled)
            self.assertEqual(receiver.rejected, 1)
            second = target(enabled=True)
            sender.sendto(encode_target(second), receiver.address)
            self.assertEqual(receiver.poll().session, first.session)
            time.sleep(0.04)
            self.assertFalse(receiver.poll().enabled)
            second.timestamp_unix_ns = time.time_ns()
            sender.sendto(encode_target(second), receiver.address)
            self.assertEqual(receiver.poll().session, second.session)
            time.sleep(0.04)
            first.seq = 2
            first.timestamp_unix_ns = time.time_ns()
            sender.sendto(encode_target(first), receiver.address)
            self.assertFalse(receiver.poll().enabled)
            self.assertEqual(receiver.poll().session, second.session)

    def test_invalid_tracking_stops_immediately_and_duplicates_do_not_refresh(self):
        with UDPReceiver(port=0, timeout_s=0.03) as receiver, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            frame = target(enabled=True)
            sender.sendto(encode_target(frame), receiver.address)
            self.assertTrue(receiver.poll().enabled)
            time.sleep(0.04)
            sender.sendto(encode_target(frame), receiver.address)
            self.assertFalse(receiver.poll().fresh)
            frame.seq = 1
            frame.timestamp_unix_ns = time.time_ns()
            frame.enabled = False
            frame.tracking_valid = False
            sender.sendto(encode_target(frame), receiver.address)
            got = receiver.poll()
            self.assertFalse(got.enabled)
            self.assertFalse(got.fresh)

    def test_replay_has_new_identity_and_explicit_enable_requirement(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "record.jsonl"
            original = target(enabled=True)
            path.write_text(json.dumps({"elapsed_s": 1, "target": json.loads(encode_target(original))}) + "\n")
            source = ReplaySource(path)
            try:
                frame = source.sample_target()
                self.assertEqual(frame.source, "replay")
                self.assertNotEqual(frame.session, original.session)
                self.assertFalse(frame.enabled)
            finally:
                source.close()

    def test_sender_process_reaches_receiver_then_sends_stop(self):
        with tempfile.TemporaryDirectory() as folder, UDPReceiver(port=0) as receiver:
            record = Path(folder) / "raw.jsonl"
            command = [sys.executable, str(ROOT / "scripts/g1/alvr_input.py"), "--backend", "synthetic",
                       "--enable-synthetic", "--duration", "0.6", "--port", str(receiver.address[1]),
                       "--record", str(record)]
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            observed = []
            deadline = time.monotonic() + 8
            while process.poll() is None and time.monotonic() < deadline:
                observed.append(receiver.poll())
                time.sleep(0.01)
            if process.poll() is None:
                process.kill()
            stdout, stderr = process.communicate(timeout=2)
            self.assertEqual(process.returncode, 0, stderr)
            self.assertIn("SYNTHETIC", stdout)
            self.assertTrue(any(frame.enabled and frame.source == "synthetic" for frame in observed))
            self.assertFalse(receiver.poll().enabled)
            entries = [json.loads(line) for line in record.read_text().splitlines()]
            self.assertGreater(len(entries), 5)
            self.assertEqual(entries[0]["raw"]["source"], "synthetic")


if __name__ == "__main__":
    unittest.main()
