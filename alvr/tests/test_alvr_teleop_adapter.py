"""Pure unit tests for ALVR-to-Unitree pose, input, and safety mapping."""

from __future__ import annotations

from contextlib import redirect_stderr
import io
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np


ALVR_ROOT = Path(__file__).resolve().parents[1]
if str(ALVR_ROOT) not in sys.path:
    sys.path.insert(0, str(ALVR_ROOT))

import alvr_teleop_adapter as adapter


ALL_ACTIONS = {
    "trigger": True,
    "grip": True,
    "thumbstick": True,
    "primary": True,
    "secondary": True,
    "thumbstick_click": True,
}


def matrix_rows(position=(0.0, 0.0, 0.0), rotation=None):
    matrix = np.eye(4, dtype=float)
    matrix[:3, 3] = position
    if rotation is not None:
        matrix[:3, :3] = rotation
    return matrix[:3, :].tolist()


def device(position=(0.0, 0.0, 0.0), *, aim_position=None):
    value = {
        "connected": True,
        "pose_valid": True,
        "matrix_3x4": matrix_rows(position),
    }
    if aim_position is not None:
        value["input"] = {
            "aim_pose": {
                "active": True,
                "pose_valid": True,
                "matrix_3x4": matrix_rows(aim_position),
            }
        }
    return value


def controls(
    *,
    trigger=0.0,
    grip=0.0,
    thumbstick=(0.0, 0.0),
    primary=False,
    secondary=False,
    thumbstick_click=False,
    available=None,
    active=True,
):
    return {
        "trigger": trigger,
        "grip": grip,
        "thumbstick": list(thumbstick),
        "primary": primary,
        "secondary": secondary,
        "thumbstick_click": thumbstick_click,
        "available": dict(ALL_ACTIONS if available is None else available),
        "active": active,
    }


def input_snapshot(left=None, right=None):
    return {
        "devices": {
            "head": {},
            "left": {"input": controls() if left is None else left},
            "right": {"input": controls() if right is None else right},
        }
    }


def tracking_snapshot(*, left_position=(-0.15, 1.13, -0.30), right_position=(0.15, 1.13, -0.30), aim=True):
    left = device(left_position, aim_position=left_position if aim else None)
    right = device(right_position, aim_position=right_position if aim else None)
    return {
        "devices": {
            "head": device((0.0, 1.50, -0.20)),
            "left": left,
            "right": right,
        }
    }


class CoordinateTransformTests(unittest.TestCase):
    def test_openvr_translation_axes_map_to_robot_axes(self):
        cases = (
            ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
            ((0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            ((0.0, 0.0, -1.0), (1.0, 0.0, 0.0)),
        )
        for openvr_position, robot_position in cases:
            with self.subTest(openvr=openvr_position):
                transform = adapter.matrix34_to_homogeneous(matrix_rows(openvr_position))
                converted = adapter.change_basis(transform)
                np.testing.assert_allclose(converted[:3, 3], robot_position, atol=1e-9)

    def test_official_controller_neutral_poses_match_unitree_targets(self):
        head = adapter.matrix34_to_homogeneous(matrix_rows((0.0, 1.50, -0.20)))
        left = adapter.matrix34_to_homogeneous(matrix_rows((-0.15, 1.13, -0.30)))
        right = adapter.matrix34_to_homogeneous(matrix_rows((0.15, 1.13, -0.30)))
        np.testing.assert_allclose(adapter.official_wrist_pose(head, left), adapter.LEFT_NEUTRAL, atol=1e-9)
        np.testing.assert_allclose(adapter.official_wrist_pose(head, right), adapter.RIGHT_NEUTRAL, atol=1e-9)

    def test_matrix_conversion_rejects_non_finite_or_wrong_shapes(self):
        with self.assertRaisesRegex(ValueError, "finite 3x4"):
            adapter.matrix34_to_homogeneous(np.eye(4))
        bad = np.zeros((3, 4))
        bad[0, 0] = np.nan
        with self.assertRaisesRegex(ValueError, "finite 3x4"):
            adapter.matrix34_to_homogeneous(bad)

    def test_aim_pose_mapper_matches_official_controller_mapping(self):
        with patch.dict(os.environ, {"ALVR_POSE_MODE": "aim"}, clear=True), redirect_stderr(io.StringIO()):
            mapper = adapter.PoseMapper()
            left, right, ready = mapper.map(tracking_snapshot(aim=True))
        self.assertTrue(ready)
        self.assertEqual(mapper.source, "aim")
        np.testing.assert_allclose(left, adapter.LEFT_NEUTRAL, atol=1e-9)
        np.testing.assert_allclose(right, adapter.RIGHT_NEUTRAL, atol=1e-9)

    def test_raw_pose_calibration_maps_relative_forward_motion(self):
        with patch.dict(
            os.environ,
            {
                "ALVR_POSE_MODE": "calibrated",
                "ALVR_CALIBRATION_FRAMES": "2",
                "ALVR_CALIBRATION_MIN_DURATION": "0",
            },
            clear=True,
        ), redirect_stderr(io.StringIO()):
            mapper = adapter.PoseMapper()
            _, _, first_ready = mapper.map(tracking_snapshot(aim=False))
            left, right, second_ready = mapper.map(tracking_snapshot(aim=False))
            moved_left, moved_right, moved_ready = mapper.map(
                tracking_snapshot(left_position=(-0.15, 1.13, -0.40), aim=False)
            )

        self.assertFalse(first_ready)
        self.assertTrue(second_ready)
        self.assertTrue(moved_ready)
        np.testing.assert_allclose(left, adapter.LEFT_NEUTRAL, atol=1e-9)
        np.testing.assert_allclose(right, adapter.RIGHT_NEUTRAL, atol=1e-9)
        np.testing.assert_allclose(moved_left[:3, 3], (0.35, 0.15, 0.08), atol=1e-9)
        np.testing.assert_allclose(moved_right, adapter.RIGHT_NEUTRAL, atol=1e-9)

    def test_auto_mapper_waits_for_and_prefers_late_aim_actions(self):
        with patch.dict(
            os.environ,
            {"ALVR_POSE_MODE": "auto", "ALVR_AIM_WAIT_FRAMES": "5"},
            clear=True,
        ), redirect_stderr(io.StringIO()):
            mapper = adapter.PoseMapper()
            _, _, first_ready = mapper.map(tracking_snapshot(aim=False))
            left, right, second_ready = mapper.map(tracking_snapshot(aim=True))
        self.assertFalse(first_ready)
        self.assertTrue(second_ready)
        self.assertEqual(mapper.source, "aim")
        np.testing.assert_allclose(left, adapter.LEFT_NEUTRAL, atol=1e-9)
        np.testing.assert_allclose(right, adapter.RIGHT_NEUTRAL, atol=1e-9)

    def test_device_connection_generation_forces_recalibration(self):
        with patch.dict(os.environ, {"ALVR_POSE_MODE": "aim"}, clear=True), redirect_stderr(
            io.StringIO()
        ):
            mapper = adapter.PoseMapper()
            snapshot = tracking_snapshot(aim=True)
            _, _, initially_ready = mapper.map(snapshot)
            snapshot["devices"]["left"]["connection_generation"] = 1
            _, _, after_reconnect_ready = mapper.map(snapshot)
        self.assertTrue(initially_ready)
        self.assertFalse(after_reconnect_ready)
        self.assertFalse(mapper.calibrated)

    def test_raw_calibration_requires_minimum_stationary_time(self):
        environment = {
            "ALVR_POSE_MODE": "calibrated",
            "ALVR_CALIBRATION_FRAMES": "2",
            "ALVR_CALIBRATION_MIN_DURATION": "0.4",
        }
        with patch.dict(os.environ, environment, clear=True), patch.object(
            adapter.time, "monotonic", side_effect=(0.0, 0.1, 0.6)
        ), redirect_stderr(io.StringIO()):
            mapper = adapter.PoseMapper()
            _, _, first = mapper.map(tracking_snapshot(aim=False))
            _, _, too_soon = mapper.map(tracking_snapshot(aim=False))
            _, _, ready = mapper.map(tracking_snapshot(aim=False))
        self.assertFalse(first)
        self.assertFalse(too_soon)
        self.assertTrue(ready)

    def test_raw_calibration_rejects_crossed_controllers(self):
        with patch.dict(
            os.environ,
            {
                "ALVR_POSE_MODE": "calibrated",
                "ALVR_CALIBRATION_FRAMES": "2",
                "ALVR_CALIBRATION_MIN_DURATION": "0",
            },
            clear=True,
        ):
            mapper = adapter.PoseMapper()
            crossed = tracking_snapshot(
                left_position=(0.15, 1.13, -0.30),
                right_position=(-0.15, 1.13, -0.30),
                aim=False,
            )
            mapper.map(crossed)
            _, _, ready = mapper.map(crossed)
        self.assertFalse(ready)
        self.assertFalse(mapper.calibrated)

    def test_pose_slew_holds_at_zero_dt_and_steps_exact_pi_rotation(self):
        previous = np.eye(4)
        desired = np.eye(4)
        desired[:3, 3] = (0.5, 0.0, 0.0)
        desired[:3, :3] = np.diag((1.0, -1.0, -1.0))
        np.testing.assert_allclose(
            adapter._slew_pose(previous, desired, 0.0, 1.0, 1.0),
            previous,
        )
        stepped = adapter._slew_pose(previous, desired, 0.1, 1.0, 1.0)
        angle = math.acos(float(np.clip((np.trace(stepped[:3, :3]) - 1.0) * 0.5, -1.0, 1.0)))
        self.assertAlmostEqual(angle, 0.1, places=7)
        self.assertAlmostEqual(stepped[0, 3], 0.1, places=7)

    def test_steamvr_stick_y_is_converted_to_webxr_convention(self):
        np.testing.assert_allclose(adapter._stick({"thumbstick": [0.25, 0.75]}), (0.25, -0.75))
        np.testing.assert_allclose(adapter._stick({"thumbstick": [float("nan"), 0.2]}), (0.0, 0.0))


class SafetyGateTests(unittest.TestCase):
    def make_gate(self):
        environment = {"ALVR_DEADMAN": "both_grips", "ALVR_DEADMAN_THRESHOLD": "0.55"}
        with patch.dict(os.environ, environment, clear=True):
            return adapter.SafetyGate()

    def test_deadman_requires_release_before_first_arm_and_after_tracking_loss(self):
        gate = self.make_gate()
        pressed = input_snapshot(
            controls(grip=0.9),
            controls(grip=0.9),
        )
        released = input_snapshot(controls(grip=0.0), controls(grip=0.0))
        with redirect_stderr(io.StringIO()):
            self.assertFalse(gate.update(pressed, ready=True))
            self.assertFalse(gate.update(released, ready=True))
            self.assertTrue(gate.update(pressed, ready=True))
            self.assertFalse(gate.update(pressed, ready=False))
            self.assertFalse(gate.update(pressed, ready=True))
            self.assertFalse(gate.update(released, ready=True))
            self.assertTrue(gate.update(pressed, ready=True))

    def test_both_thumbstick_clicks_latch_estop_until_explicit_reset(self):
        gate = self.make_gate()
        released = input_snapshot(controls(), controls())
        armed = input_snapshot(controls(grip=0.9), controls(grip=0.9))
        estop = input_snapshot(
            controls(grip=0.9, thumbstick_click=True),
            controls(grip=0.9, thumbstick_click=True),
        )
        reset = input_snapshot(
            controls(secondary=True),
            controls(secondary=True),
        )
        with redirect_stderr(io.StringIO()):
            gate.update(released, ready=True)
            self.assertTrue(gate.update(armed, ready=True))
            self.assertFalse(gate.update(estop, ready=True))
            self.assertTrue(gate.estop_latched)
            self.assertFalse(gate.update(released, ready=True))
            self.assertTrue(gate.estop_latched)
            self.assertFalse(gate.update(reset, ready=True))
            self.assertFalse(gate.estop_latched)
            self.assertTrue(gate.update(armed, ready=True))

    def test_required_actions_are_task_specific_and_must_exist_on_both_hands(self):
        fixed_available = {
            name: name in {"trigger", "grip", "secondary", "thumbstick_click"}
            for name in ALL_ACTIONS
        }
        snapshot = input_snapshot(
            controls(available=fixed_available),
            controls(available=fixed_available),
        )
        self.assertTrue(adapter.required_actions_available(snapshot, wholebody=False))
        self.assertFalse(adapter.required_actions_available(snapshot, wholebody=True))

        wholebody_available = dict(fixed_available, thumbstick=True, thumbstick_click=True)
        snapshot = input_snapshot(
            controls(available=wholebody_available),
            controls(available=wholebody_available),
        )
        self.assertTrue(adapter.required_actions_available(snapshot, wholebody=True))
        snapshot["devices"]["right"]["input"]["active"] = False
        self.assertFalse(adapter.required_actions_available(snapshot, wholebody=False))


class HandSynergyTests(unittest.TestCase):
    def test_synergy_returns_finite_webxr_order_joint_positions(self):
        for side in ("left", "right"):
            for curl in (0.0, 0.5, 1.0):
                with self.subTest(side=side, curl=curl):
                    hand = adapter.synthetic_hand_positions(curl, side)
                    self.assertEqual(hand.shape, (25, 3))
                    self.assertTrue(np.all(np.isfinite(hand)))
                    np.testing.assert_allclose(hand[0], (0.0, 0.0, 0.0))

    def test_left_and_right_synergies_are_mirrored_and_curl_is_clamped(self):
        left_open = adapter.synthetic_hand_positions(0.0, "left")
        right_open = adapter.synthetic_hand_positions(0.0, "right")
        np.testing.assert_allclose(left_open[:, :2], right_open[:, :2], atol=1e-12)
        np.testing.assert_allclose(left_open[:, 2], -right_open[:, 2], atol=1e-12)

        left_closed = adapter.synthetic_hand_positions(1.0, "left")
        self.assertGreater(np.linalg.norm(left_closed[9] - left_open[9]), 0.01)
        np.testing.assert_allclose(adapter.synthetic_hand_positions(-2.0, "left"), left_open)
        np.testing.assert_allclose(adapter.synthetic_hand_positions(2.0, "left"), left_closed)
        with self.assertRaisesRegex(ValueError, "side must be"):
            adapter.synthetic_hand_positions(0.5, "middle")


class FakeProbe:
    def __init__(self, snapshots):
        self.snapshots = list(snapshots)

    def sample(self):
        return self.snapshots.pop(0)


class FakePoseMapper:
    def map(self, _snapshot):
        return adapter.LEFT_NEUTRAL.copy(), adapter.RIGHT_NEUTRAL.copy(), True


class FakeBase:
    def __init__(self):
        self.calls = []

    def publish(self, left_stick, right_stick, active):
        self.calls.append((np.array(left_stick), np.array(right_stick), active))


class AdapterGateIntegrationTests(unittest.TestCase):
    def make_wrapper(self, snapshots):
        with patch.dict(os.environ, {"ALVR_DEADMAN": "both_grips"}, clear=True):
            safety = adapter.SafetyGate()
        wrapper = object.__new__(adapter.TeleVuerWrapper)
        wrapper.use_hand_tracking = True
        wrapper.wholebody = False
        wrapper.pose_mapper = FakePoseMapper()
        wrapper.safety = safety
        wrapper.left_target = adapter.LEFT_NEUTRAL.copy()
        wrapper.right_target = adapter.RIGHT_NEUTRAL.copy()
        wrapper.head_target = np.eye(4, dtype=float)
        wrapper.left_curl = 0.0
        wrapper.right_curl = 0.0
        wrapper.left_grip_value = 0.0
        wrapper.right_grip_value = 0.0
        wrapper.linear_speed = 1.0
        wrapper.angular_speed = 4.0
        wrapper._last_update = time.monotonic()
        wrapper.probe = FakeProbe(snapshots)
        wrapper.base = FakeBase()
        return wrapper

    def test_motion_and_hand_synergy_update_only_while_safety_gate_is_armed(self):
        release = input_snapshot(
            controls(trigger=0.7, grip=0.0),
            controls(trigger=0.4, grip=0.0),
        )
        press = input_snapshot(
            controls(trigger=0.8, grip=0.9, thumbstick=(0.2, 0.3)),
            controls(trigger=0.25, grip=0.9, thumbstick=(-0.1, -0.4)),
        )
        unavailable = input_snapshot(
            controls(trigger=1.0, grip=0.9, available=dict(ALL_ACTIONS, trigger=False)),
            controls(trigger=1.0, grip=0.9),
        )
        wrapper = self.make_wrapper([release, press, unavailable])

        with redirect_stderr(io.StringIO()):
            before_arm = wrapper.get_tele_data()
            armed = wrapper.get_tele_data()
            lost_input = wrapper.get_tele_data()

        self.assertFalse(before_arm.motion_data_ready)
        self.assertTrue(armed.motion_data_ready)
        self.assertAlmostEqual(armed.left_ctrl_triggerValue, 2.0)
        self.assertAlmostEqual(armed.right_ctrl_triggerValue, 7.5)
        np.testing.assert_allclose(armed.left_hand_pos, adapter.synthetic_hand_positions(0.8, "left"))
        np.testing.assert_allclose(armed.right_hand_pos, adapter.synthetic_hand_positions(0.25, "right"))

        self.assertFalse(lost_input.motion_data_ready)
        # Tracking/input failure holds the last commanded grasp instead of
        # opening the hand or accepting the new trigger value.
        np.testing.assert_allclose(lost_input.left_hand_pos, armed.left_hand_pos)
        np.testing.assert_allclose(lost_input.right_hand_pos, armed.right_hand_pos)
        self.assertAlmostEqual(lost_input.left_ctrl_triggerValue, armed.left_ctrl_triggerValue)
        self.assertAlmostEqual(lost_input.right_ctrl_triggerValue, armed.right_ctrl_triggerValue)
        np.testing.assert_allclose(lost_input.left_ctrl_thumbstickValue, (0.0, 0.0))
        np.testing.assert_allclose(lost_input.right_ctrl_thumbstickValue, (0.0, 0.0))
        self.assertEqual([call[2] for call in wrapper.base.calls], [False, True, False])


class FakeMessage:
    def __init__(self, data):
        self.data = data


class FakeWriter:
    def __init__(self):
        self.messages = []

    def Write(self, message):
        self.messages.append(message)


class BlockingWriter(FakeWriter):
    def __init__(self):
        super().__init__()
        self.entered = threading.Event()
        self.release = threading.Event()

    def Write(self, message):
        self.messages.append(message)
        self.entered.set()
        self.release.wait(timeout=2.0)


class WholebodyPublisherTests(unittest.TestCase):
    def test_inactive_gate_always_publishes_zero_velocity(self):
        publisher = object.__new__(adapter.WholebodyCommandPublisher)
        publisher.enabled = True
        publisher.publisher = FakeWriter()
        publisher.message_type = FakeMessage
        publisher.speed = 0.3
        publisher.yaw_speed = 0.3
        publisher.deadzone = 0.1
        publisher.height = 0.8

        publisher.publish(np.array([1.0, -1.0]), np.array([1.0, 0.0]), active=True)
        publisher.publish(np.array([1.0, -1.0]), np.array([1.0, 0.0]), active=False)
        active_payload = json.loads(publisher.publisher.messages[0].data)
        inactive_payload = json.loads(publisher.publisher.messages[1].data)
        np.testing.assert_allclose(active_payload, (0.3, -0.3, -0.3, 0.8), atol=1e-12)
        np.testing.assert_allclose(inactive_payload, (0.0, 0.0, 0.0, 0.8), atol=1e-12)

    def test_heartbeat_timeout_latches_zero_until_deadman_release(self):
        publisher = object.__new__(adapter.WholebodyCommandPublisher)
        publisher.enabled = True
        publisher.publisher = FakeWriter()
        publisher.message_type = FakeMessage
        publisher.speed = 0.3
        publisher.yaw_speed = 0.3
        publisher.deadzone = 0.1
        publisher.height = 0.8
        publisher.publish_rate = 200.0
        publisher.command_timeout = 0.05
        publisher._state_lock = threading.Lock()
        publisher._write_lock = threading.Lock()
        publisher._stop = threading.Event()
        publisher._state_revision = 0
        publisher._latest_payload = [0.3, 0.0, 0.0, 0.8]
        publisher._last_command_at = time.monotonic() - 1.0
        publisher._last_active = True
        publisher._timeout_latched = False
        with redirect_stderr(io.StringIO()):
            thread = threading.Thread(target=publisher._repeat_latest)
            thread.start()
            time.sleep(0.02)
            publisher._stop.set()
            thread.join(timeout=1.0)
        self.assertTrue(publisher._timeout_latched)
        self.assertTrue(publisher.publisher.messages)
        np.testing.assert_allclose(
            json.loads(publisher.publisher.messages[-1].data),
            (0.0, 0.0, 0.0, 0.8),
        )

        publisher.publish(np.array([0.0, -1.0]), np.zeros(2), active=True)
        np.testing.assert_allclose(json.loads(publisher.publisher.messages[-1].data)[:3], (0.0, 0.0, 0.0))
        publisher.publish(np.zeros(2), np.zeros(2), active=False)
        publisher.publish(np.array([0.0, -1.0]), np.zeros(2), active=True)
        self.assertGreater(json.loads(publisher.publisher.messages[-1].data)[0], 0.0)

    def test_blocked_dds_write_cannot_block_watchdog_or_deadman_release(self):
        publisher = object.__new__(adapter.WholebodyCommandPublisher)
        publisher.enabled = True
        publisher.publisher = BlockingWriter()
        publisher.message_type = FakeMessage
        publisher.speed = 0.3
        publisher.yaw_speed = 0.3
        publisher.deadzone = 0.1
        publisher.height = 0.8
        publisher.publish_rate = 200.0
        publisher.command_timeout = 0.05
        publisher._state_lock = threading.Lock()
        publisher._write_lock = threading.Lock()
        publisher._stop = threading.Event()
        publisher._writer_stop = threading.Event()
        publisher._pending_lock = threading.Lock()
        publisher._pending_event = threading.Event()
        publisher._latest_payload = [0.0, 0.0, 0.0, 0.8]
        publisher._pending_payload = list(publisher._latest_payload)
        publisher._state_revision = 0
        publisher._pending_revision = 0
        publisher._last_command_at = time.monotonic()
        publisher._last_active = False
        publisher._timeout_latched = False
        publisher._writer_error = None
        publisher._writer_busy = False
        publisher._write_started_at = 0.0
        publisher._last_write_completed_at = time.monotonic()
        publisher._writer_thread = threading.Thread(target=publisher._writer_loop, daemon=True)
        publisher._thread = threading.Thread(target=publisher._repeat_latest, daemon=True)
        publisher._writer_thread.start()
        publisher._thread.start()

        publisher.publish(np.array([0.0, -1.0]), np.zeros(2), active=True)
        self.assertTrue(publisher.publisher.entered.wait(timeout=1.0))
        deadline = time.monotonic() + 1.0
        while not publisher._timeout_latched and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(publisher._timeout_latched)
        self.assertFalse(publisher.healthy())

        started = time.monotonic()
        publisher.publish(np.zeros(2), np.zeros(2), active=False)
        self.assertLess(time.monotonic() - started, 0.05)
        publisher.publisher.release.set()
        publisher.close()
        self.assertFalse(publisher._writer_thread.is_alive())
        np.testing.assert_allclose(
            json.loads(publisher.publisher.messages[-1].data),
            (0.0, 0.0, 0.0, 0.8),
        )


if __name__ == "__main__":
    unittest.main()
