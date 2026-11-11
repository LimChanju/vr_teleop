from __future__ import annotations

import numpy as np
import pytest

from twist2_vr.mujoco_openvr import (
    ControllerButtonSample,
    OpenVRViewerConfig,
    PendingSignal,
    ReleaseArmedButton,
    camera_vectors,
    front_view_head_pose,
    look_at_head_pose,
    projection_raw_to_mujoco,
    poll_controller_button,
    relative_hmd_pose,
    rigid_inverse,
)


class _FakePose:
    def __init__(self, connected: bool) -> None:
        self.bDeviceIsConnected = connected


class _FakeControllerState:
    def __init__(self, pressed: int) -> None:
        self.ulButtonPressed = pressed


class _FakeOpenVR:
    TrackedControllerRole_LeftHand = 1
    TrackedControllerRole_RightHand = 2
    TrackedDeviceClass_Controller = 2
    k_unTrackedDeviceIndexInvalid = 0xFFFFFFFF
    Prop_SerialNumber_String = 1000
    Prop_ModelNumber_String = 1001
    Prop_ControllerType_String = 1002
    k_EButton_Axis0 = 32


class _FakeVRSystem:
    def __init__(self) -> None:
        self.classes = {1: 2, 2: 2, 3: 2}
        # SteamVR may assign the right-hand role to a synthetic hand device.
        self.roles = {1: 1, 2: 2, 3: 0}
        self.properties = {
            (1, 1000): "Touch_Controller_Left",
            (1, 1001): "Meta Quest 3 (Left Controller)",
            (1, 1002): "oculus_touch",
            (2, 1000): "VRLINKQ_Hand_Right",
            (2, 1001): "VRLink Hand Tracker (Right Hand)",
            (2, 1002): "svl_hand_interaction_augmented",
            (3, 1000): "Touch_Controller_Right",
            (3, 1001): "Meta Quest 3 (Right Controller)",
            (3, 1002): "oculus_touch",
        }
        self.pressed = {
            1: 1 << _FakeOpenVR.k_EButton_Axis0,
            2: 0,
            3: 1 << _FakeOpenVR.k_EButton_Axis0,
        }

    def getTrackedDeviceIndexForControllerRole(self, role: int) -> int:
        return 2 if role == _FakeOpenVR.TrackedControllerRole_RightHand else 1

    def getTrackedDeviceClass(self, index: int) -> int:
        return self.classes.get(index, 0)

    def getControllerRoleForTrackedDeviceIndex(self, index: int) -> int:
        return self.roles.get(index, 0)

    def getStringTrackedDeviceProperty(self, index: int, prop: int) -> str:
        return self.properties.get((index, prop), "")

    def getControllerState(self, index: int) -> tuple[bool, _FakeControllerState]:
        return True, _FakeControllerState(self.pressed[index])


def test_front_view_starts_in_front_and_faces_g1() -> None:
    robot = np.array([0.0, 0.0, 0.793])
    pose = front_view_head_pose(
        robot,
        np.array([1.0, 0.0, 0.0]),
        distance_m=2.2,
        eye_height_m=1.35,
        target_height_m=0.95,
    )
    position, forward, up = camera_vectors(pose)

    np.testing.assert_allclose(position, [2.2, 0.0, 1.35])
    expected_forward = np.array([0.0, 0.0, 0.95]) - position
    expected_forward /= np.linalg.norm(expected_forward)
    np.testing.assert_allclose(forward, expected_forward)
    assert up[2] > 0.98
    assert np.linalg.det(pose[:3, :3]) == pytest.approx(1.0)


def test_hmd_motion_is_relative_to_startup_pose() -> None:
    neutral = np.eye(4)
    neutral[:3, 3] = [4.0, -2.0, 1.7]
    current = neutral.copy()
    current[:3, 3] += [0.25, 0.0, -0.1]

    delta = relative_hmd_pose(neutral, current)
    np.testing.assert_allclose(delta[:3, 3], [0.25, 0.0, -0.1])
    np.testing.assert_allclose(rigid_inverse(neutral) @ neutral, np.eye(4))


def test_openvr_local_right_maps_to_virtual_camera_right() -> None:
    anchor = look_at_head_pose(np.array([2.0, 0.0, 1.3]), np.array([0.0, 0.0, 1.0]))
    local_motion = np.eye(4)
    local_motion[0, 3] = 0.2
    moved = anchor @ local_motion

    # In a front view of a +X-facing robot, the user's right is world +Y.
    np.testing.assert_allclose(moved[:3, 3] - anchor[:3, 3], [0.0, 0.2, 0.0], atol=1e-8)


def test_projection_conversion_preserves_openvr_vertical_edges() -> None:
    result = projection_raw_to_mujoco(
        (-1.3, 0.8, -1.4, 0.9), near_m=0.05, far_m=50.0
    )
    assert result["frustum_bottom"] == pytest.approx(-0.07)
    assert result["frustum_top"] == pytest.approx(0.045)
    assert result["frustum_center"] == pytest.approx(-0.0125)
    assert result["frustum_width"] == pytest.approx(0.0525)
    assert result["frustum_near"] == pytest.approx(0.05)
    assert result["frustum_far"] == pytest.approx(50.0)

    # These are the asymmetric vertical terms in OpenGL's projection matrix.
    vertical_span = result["frustum_top"] - result["frustum_bottom"]
    assert 2.0 * result["frustum_near"] / vertical_span == pytest.approx(
        0.8695652174
    )
    assert (
        result["frustum_top"] + result["frustum_bottom"]
    ) / vertical_span == pytest.approx(-0.2173913043)


def test_viewer_config_rejects_unsafe_or_unsupported_values() -> None:
    OpenVRViewerConfig().validate()
    with pytest.raises(ValueError, match="unsupported VR view"):
        OpenVRViewerConfig(view="robot-pov").validate()
    with pytest.raises(ValueError, match="render scale"):
        OpenVRViewerConfig(render_scale=0.1).validate()
    with pytest.raises(ValueError, match="recenter button"):
        OpenVRViewerConfig(recenter_button="left-trigger").validate()


def test_release_armed_button_requires_release_and_emits_one_edge() -> None:
    detector = ReleaseArmedButton()
    held = ControllerButtonSample("right-touch", True, True)
    released = ControllerButtonSample("right-touch", True, False)

    assert detector.update(held) is False  # already held at startup
    assert detector.update(held) is False
    assert detector.update(released) is False  # arms only after a release
    assert detector.update(held) is True
    assert detector.update(held) is False
    assert detector.update(released) is False
    assert detector.update(held) is True

    # A disconnect or identity change disarms it again.
    assert detector.update(ControllerButtonSample(None, False, False)) is False
    assert detector.update(held) is False


def test_controller_poll_prefers_right_touch_over_hand_and_left() -> None:
    system = _FakeVRSystem()
    poses = [_FakePose(False), _FakePose(True), _FakePose(True), _FakePose(True)]

    sample = poll_controller_button(
        system,
        _FakeOpenVR,
        poses,
        side="right",
        button_id=_FakeOpenVR.k_EButton_Axis0,
    )

    assert sample == ControllerButtonSample("Touch_Controller_Right", True, True)


def test_pending_signal_coalesces_without_losing_future_requests() -> None:
    signal = PendingSignal()
    assert signal.take() is False
    signal.request()
    signal.request()
    assert signal.take() is True
    assert signal.take() is False
    signal.request()
    assert signal.take() is True
