"""Calibration generations synchronize XR recenter without granting control."""

import dataclasses
import json
from pathlib import Path
import socket
import sys
import time
import unittest
import uuid

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from g1_teleop.vr.calibration import RawFrame, TargetMapper
from g1_teleop.vr.protocol import TargetFrame, UDPReceiver, decode_target, encode_target


def raw_frame():
    poses = np.tile(np.eye(4)[:3], (3, 1, 1))
    poses[:, :, 3] = [[0, 1.65, 0], [-0.28, 1.2, -0.25], [0.28, 1.2, -0.25]]
    return RawFrame(poses=poses, tracking_valid=True, inputs_valid=True, source="synthetic")


class CalibrationGenerationTests(unittest.TestCase):
    def test_x_edges_increment_and_calibration_keeps_control_stopped(self):
        mapper = TargetMapper()
        raw = raw_frame()
        self.assertEqual(mapper.update(raw).calibration_id, 0)
        raw.calibrate = True
        first = mapper.update(raw)
        self.assertEqual(first.calibration_id, 1)
        self.assertTrue(first.calibrated)
        self.assertFalse(first.enabled)
        self.assertEqual(mapper.update(raw).calibration_id, 1)  # held X
        raw.calibrate = False
        raw.arm, raw.grips = True, (1.0, 1.0)
        self.assertTrue(mapper.update(raw).enabled)
        raw.calibrate = True
        second = mapper.update(raw)
        self.assertEqual(second.calibration_id, 2)
        self.assertFalse(second.enabled)
        self.assertEqual(second.reset_id, 0)

    def test_failed_calibration_does_not_increment(self):
        mapper = TargetMapper()
        raw = raw_frame()
        mapper.update(raw, command="calibrate")
        raw.poses[0, :, :3] = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]  # looking vertically
        failed = mapper.update(raw, command="calibrate")
        self.assertEqual(failed.calibration_id, 1)
        self.assertFalse(failed.enabled)
        raw.tracking_valid = False
        self.assertEqual(mapper.update(raw, command="calibrate").calibration_id, 1)

    def test_reference_change_reset_and_calibration_have_distinct_generations(self):
        mapper = TargetMapper()
        raw = raw_frame()
        mapper.update(raw, command="calibrate")
        raw.reference_changed = True
        invalidated = mapper.update(raw)
        self.assertFalse(invalidated.calibrated)
        self.assertEqual(invalidated.calibration_id, 1)
        raw.reference_changed = False
        calibrated = mapper.update(raw, command="calibrate")
        self.assertEqual(calibrated.calibration_id, 2)
        reset = mapper.update(raw, command="reset")
        self.assertEqual(reset.reset_id, 1)
        self.assertEqual(reset.calibration_id, 2)
        self.assertFalse(reset.enabled)

    def test_old_packet_default_and_invalid_generation_rejection(self):
        frame = TargetFrame(session=str(uuid.uuid4()), seq=0, timestamp_unix_ns=time.time_ns(),
                            calibration_id=5, tracking_valid=True, calibrated=True)
        packet = json.loads(encode_target(frame))
        self.assertEqual(decode_target(json.dumps(packet).encode()).calibration_id, 5)
        packet.pop("calibration_id")
        self.assertEqual(decode_target(json.dumps(packet).encode()).calibration_id, 0)
        for value in (-1, True, 1.2, "1", None, 2**63):
            with self.subTest(value=value):
                packet["calibration_id"] = value
                with self.assertRaisesRegex(ValueError, "calibration_id"):
                    decode_target(json.dumps(packet).encode())

    def test_receiver_preserves_monotonic_generation_per_session(self):
        with UDPReceiver(port=0, timeout_s=0.04) as receiver, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            frame = TargetFrame(session=str(uuid.uuid4()), seq=0, timestamp_unix_ns=time.time_ns(),
                                tracking_valid=True, calibrated=True, calibration_id=2)
            def send(value):
                sock.sendto(encode_target(value), receiver.address)
                return receiver.poll()
            self.assertEqual(send(frame).calibration_id, 2)
            rejected = send(dataclasses.replace(frame, seq=1, calibration_id=1))
            self.assertEqual(rejected.seq, 0)
            self.assertEqual(receiver.rejected, 1)
            changed = send(dataclasses.replace(frame, seq=2, calibration_id=3))
            self.assertEqual(changed.calibration_id, 3)
            self.assertFalse(changed.reset)
            reset = send(dataclasses.replace(frame, seq=3, calibration_id=3, reset_id=1))
            self.assertTrue(reset.reset)
            self.assertEqual(reset.calibration_id, 3)
            time.sleep(0.06)
            fresh = send(dataclasses.replace(frame, session=str(uuid.uuid4()), seq=0,
                                             timestamp_unix_ns=time.time_ns(), calibration_id=0))
            self.assertTrue(fresh.fresh)
            self.assertEqual(fresh.calibration_id, 0)
            self.assertEqual(fresh.reset_id, 0)


if __name__ == "__main__":
    unittest.main()
