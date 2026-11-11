from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from twist2_vr.openvr_actions import (
    ANALOG_ACTIONS,
    DIGITAL_ACTIONS,
    choose_primary_devices,
    empty_controller_input,
    resolve_controller_input,
)
from twist2_vr.providers.openvr import OpenVRProvider


ROOT = Path(__file__).resolve().parents[2]


class FakeVR:
    TrackedDeviceClass_Invalid = 0
    TrackedDeviceClass_HMD = 1
    TrackedDeviceClass_Controller = 2
    TrackedDeviceClass_GenericTracker = 3
    TrackedDeviceClass_TrackingReference = 4
    TrackedControllerRole_LeftHand = 1
    TrackedControllerRole_RightHand = 2
    TrackingResult_Running_OK = 200
    TrackingUniverseStanding = 1
    k_unMaxTrackedDeviceCount = 7
    k_unTrackedDeviceIndexInvalid = 2**32 - 1
    k_ulInvalidInputValueHandle = 0
    k_EButton_A = 7
    k_EButton_ApplicationMenu = 1
    k_EButton_Axis0 = 32
    Prop_SerialNumber_String = "serial"
    Prop_ModelNumber_String = "model"
    Prop_ControllerType_String = "controller_type"
    Prop_TrackingSystemName_String = "tracking_system"


class Matrix:
    def __init__(self, position=(0.0, 0.0, 0.0)):
        self.m = [
            [1.0, 0.0, 0.0, float(position[0])],
            [0.0, 1.0, 0.0, float(position[1])],
            [0.0, 0.0, 1.0, float(position[2])],
        ]


class Pose:
    def __init__(self, position=(0.0, 0.0, 0.0), *, connected=True, valid=True):
        self.bDeviceIsConnected = connected
        self.bPoseIsValid = valid
        self.eTrackingResult = FakeVR.TrackingResult_Running_OK if valid else 1
        self.mDeviceToAbsoluteTracking = Matrix(position)


class Axis:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y


class ControllerState:
    def __init__(self):
        self.rAxis = [Axis(0.25, -0.5), Axis(0.7), Axis(0.8)]
        self.ulButtonPressed = (1 << FakeVR.k_EButton_A) | (1 << FakeVR.k_EButton_Axis0)


class FakeSystem:
    def __init__(self):
        self.poses = [
            Pose((0.0, 1.7, 0.0)),
            Pose((-0.2, 1.2, -0.3)),
            Pose((0.2, 1.2, -0.3)),
            Pose((0.3, 1.2, -0.3)),
            Pose((0.4, 1.2, -0.3)),
            Pose((0.0, 1.0, 0.0)),
            Pose((0.0, 0.5, 0.0)),
        ]
        self.classes = [
            FakeVR.TrackedDeviceClass_HMD,
            FakeVR.TrackedDeviceClass_Controller,
            FakeVR.TrackedDeviceClass_Controller,
            FakeVR.TrackedDeviceClass_Controller,
            FakeVR.TrackedDeviceClass_Controller,
            FakeVR.TrackedDeviceClass_GenericTracker,
            FakeVR.TrackedDeviceClass_GenericTracker,
        ]
        self.roles = {
            1: FakeVR.TrackedControllerRole_LeftHand,
            2: FakeVR.TrackedControllerRole_RightHand,
            3: FakeVR.TrackedControllerRole_RightHand,
            4: FakeVR.TrackedControllerRole_RightHand,
        }
        self.properties = {
            0: ("hmd", "Quest 3", "hmd", "alvr"),
            1: ("left-touch", "Quest Touch Left", "oculus_touch", "alvr"),
            2: ("wrong-space", "Other Right", "controller", "other_driver"),
            3: ("right-touch", "Quest Touch Right", "oculus_touch", "alvr"),
            4: ("right-hand", "ALVR Hand Tracker Right", "hand_tracker", "alvr"),
            5: ("pelvis-serial", "VIVE Ultimate Tracker", "tracker", "vive"),
            6: ("unmapped-tracker", "VIVE Ultimate Tracker", "tracker", "vive"),
        }
        self.states = {1: ControllerState(), 3: ControllerState()}

    def getTrackedDeviceClass(self, index):
        return self.classes[index]

    def getControllerRoleForTrackedDeviceIndex(self, index):
        return self.roles.get(index, 0)

    def getStringTrackedDeviceProperty(self, index, prop):
        field = {
            FakeVR.Prop_SerialNumber_String: 0,
            FakeVR.Prop_ModelNumber_String: 1,
            FakeVR.Prop_ControllerType_String: 2,
            FakeVR.Prop_TrackingSystemName_String: 3,
        }[prop]
        return self.properties[index][field]

    def getControllerState(self, index):
        state = self.states.get(index)
        return state is not None, state

    def getDeviceToAbsoluteTrackingPose(self, _origin, _seconds, _count):
        return self.poses


def action_input(device_index, *, grip, aim_position):
    result = {
        "trigger": 0.3,
        "grip": grip,
        "thumbstick": [0.1, -0.2],
        "primary": False,
        "secondary": False,
        "thumbstick_click": False,
        "active": True,
        "available": {name: True for name in (*ANALOG_ACTIONS, *DIGITAL_ACTIONS)},
        "source": "actions",
        "device_index": device_index,
        "aim_pose": {
            "active": True,
            "pose_valid": True,
            "tracking_result": FakeVR.TrackingResult_Running_OK,
            "device_index": device_index,
            "matrix_3x4": Matrix(aim_position).m,
        },
    }
    return result


class FakeActions:
    def read(self):
        return {
            "left": action_input(1, grip=0.6, aim_position=(8.0, 8.0, 8.0)),
            "right": action_input(3, grip=0.9, aim_position=(9.0, 9.0, 9.0)),
        }


def make_provider(system):
    provider = object.__new__(OpenVRProvider)
    provider.vr = FakeVR
    provider.system = system
    provider.serial_to_role = {"pelvis-serial": "Pelvis"}
    provider.sequence = 0
    provider._closed = False
    provider.applications = None
    provider.actions = FakeActions()
    provider.action_error = None
    provider._application_registered = False
    provider._properties = {}
    provider._connected = {}
    provider._generations = {}
    return provider


def test_primary_selection_requires_same_tracking_system_and_physical_touch():
    system = FakeSystem()
    selected = choose_primary_devices(FakeVR, system, system.poses)
    assert selected == {"head": 0, "left": 1, "right": 3}


def test_actions_merge_only_missing_channels_from_matching_legacy_controller():
    system = FakeSystem()
    actions = action_input(1, grip=0.1, aim_position=(1.0, 2.0, 3.0))
    actions["available"] = {
        name: name == "trigger" for name in (*ANALOG_ACTIONS, *DIGITAL_ACTIONS)
    }
    actions["trigger"] = 0.2
    merged = resolve_controller_input(FakeVR, system, 1, actions)
    assert merged["source"] == "actions+legacy"
    assert merged["trigger"] == 0.2
    assert merged["grip"] == 0.8
    assert merged["thumbstick"] == [0.25, -0.5]
    assert merged["primary"] is True
    assert merged["aim_pose"]["pose_valid"] is True


def test_mismatched_action_origin_rejects_aim_and_falls_back_to_selected_device():
    system = FakeSystem()
    actions = action_input(3, grip=0.99, aim_position=(9.0, 9.0, 9.0))
    merged = resolve_controller_input(FakeVR, system, 1, actions)
    assert merged["source"] == "legacy"
    assert merged["device_index"] == 1
    assert merged["grip"] == 0.8
    assert merged["aim_pose"]["pose_valid"] is False


def test_provider_uses_raw_controller_pose_and_explicit_generic_tracker_mapping():
    system = FakeSystem()
    provider = make_provider(system)
    frame = provider.read()

    assert set(frame.poses) == {"Head", "Left_Wrist", "Right_Wrist", "Pelvis"}
    np.testing.assert_allclose(frame.poses["Left_Wrist"].position_m, (-0.2, 1.2, -0.3))
    np.testing.assert_allclose(frame.poses["Right_Wrist"].position_m, (0.3, 1.2, -0.3))
    # The action aim/tip poses have huge offsets. They validate identity but
    # must never replace the anatomical wrist's raw controller origin.
    assert frame.poses["Left_Wrist"].metadata["pose_source"] == "device"
    assert frame.poses["Left_Wrist"].metadata["aim_identity_validated"] is True
    np.testing.assert_allclose(frame.poses["Pelvis"].position_m, (0.0, 1.0, 0.0))
    assert "unmapped-tracker" not in {pose.device_id for pose in frame.poses.values()}
    assert frame.controls["left"]["grip"] == 0.6
    assert frame.controls["right"]["grip"] == 0.9
    assert frame.poses["Pelvis"].metadata["serial"] == "pelvis-serial"
    assert frame.poses["Pelvis"].metadata["model"] == "VIVE Ultimate Tracker"
    assert frame.poses["Pelvis"].metadata["connection_generation"] == 0


def test_reconnect_increments_generation_and_requires_fresh_device_selection():
    system = FakeSystem()
    provider = make_provider(system)
    provider.read()
    system.poses[1].bDeviceIsConnected = False
    system.poses[1].bPoseIsValid = False
    missing = provider.read()
    assert "Left_Wrist" not in missing.poses
    system.poses[1].bDeviceIsConnected = True
    system.poses[1].bPoseIsValid = True
    reconnected = provider.read()
    assert reconnected.poses["Left_Wrist"].metadata["connection_generation"] == 2


def test_action_manifests_cover_every_declared_output():
    manifest = json.loads((ROOT / "config/openvr_actions.json").read_text())
    action_names = {entry["name"] for entry in manifest["actions"]}
    binding = json.loads((ROOT / "config/openvr_bindings_oculus_touch.json").read_text())
    action_set = binding["bindings"]["/actions/teleop"]
    bound_outputs = {
        item["output"]
        for source in action_set["sources"]
        for item in source["inputs"].values()
    }
    bound_outputs.update(item["output"] for item in action_set["poses"])
    assert bound_outputs == action_names
    app = json.loads((ROOT / "config/openvr_app.vrmanifest").read_text())
    assert app["applications"][0]["action_manifest_path"] == "openvr_actions.json"


def test_close_is_idempotent_and_releases_manifest_before_runtime():
    calls = []

    class Applications:
        def removeApplicationManifest(self, path):
            calls.append(("remove", path))

    class Runtime:
        @staticmethod
        def shutdown():
            calls.append(("shutdown",))

    provider = object.__new__(OpenVRProvider)
    provider._closed = False
    provider._application_registered = True
    provider.applications = Applications()
    provider.application_manifest = Path("/tmp/twist2.vrmanifest")
    provider.actions = object()
    provider.system = object()
    provider.vr = Runtime
    provider.close()
    provider.close()
    assert calls == [
        ("remove", "/tmp/twist2.vrmanifest"),
        ("shutdown",),
    ]
    assert provider.system is None
    assert provider.actions is None


def test_empty_controls_are_explicitly_unavailable():
    controls = empty_controller_input()
    assert controls["active"] is False
    assert not any(controls["available"].values())
