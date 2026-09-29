"""Host-side unit tests; no headset or running SteamVR is required."""

import importlib.util
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("alvr_probe", ROOT / "alvr_probe.py")
probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)


class FakeMatrix:
    def __init__(self, rows):
        self.m = rows


class FakePose:
    def __init__(self, connected, valid, tracking_result=200):
        self.bDeviceIsConnected = connected
        self.bPoseIsValid = valid
        self.eTrackingResult = tracking_result


class FakeVR:
    TrackingResult_Running_OK = 200
    TrackedDeviceClass_HMD = 1
    TrackedDeviceClass_Controller = 2
    TrackedControllerRole_LeftHand = 1
    TrackedControllerRole_RightHand = 2
    Prop_ModelNumber_String = 1001
    Prop_SerialNumber_String = 1002
    Prop_TrackingSystemName_String = 1003


class FakeSystem:
    def __init__(self):
        self.classes = [1, 2, 2]
        self.roles = [0, 1, 1]
        self.properties = {
            (0, 1003): "alvr",
            (1, 1003): "alvr",
            (2, 1003): "other-driver",
        }

    def getTrackedDeviceClass(self, index):
        return self.classes[index]

    def getControllerRoleForTrackedDeviceIndex(self, index):
        return self.roles[index]

    def getStringTrackedDeviceProperty(self, index, property_id):
        return self.properties[(index, property_id)]


class TransformTests(unittest.TestCase):
    def test_identity_transform(self):
        value = probe.matrix34_to_transform(FakeMatrix([[1, 0, 0, 0.2], [0, 1, 0, 1.5], [0, 0, 1, -0.4]]))
        self.assertEqual(value["position_m"], [0.2, 1.5, -0.4])
        self.assertEqual(value["orientation_xyzw"], [0.0, 0.0, 0.0, 1.0])

    def test_quaternion_for_quarter_turn_about_z(self):
        value = probe.rotation_matrix_to_quaternion([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        expected = math.sqrt(0.5)
        self.assertAlmostEqual(value[2], expected)
        self.assertAlmostEqual(value[3], expected)
        self.assertAlmostEqual(sum(component * component for component in value), 1.0)

    def test_device_selection_does_not_mix_tracking_drivers(self):
        poses = [FakePose(True, True), FakePose(False, False), FakePose(True, True)]
        indexes = probe.choose_device_indexes(FakeVR, FakeSystem(), poses)
        self.assertEqual(indexes["head"], 0)
        self.assertEqual(indexes["left"], 1)
        self.assertIsNone(indexes["right"])

    def test_legacy_buttons_preserve_a_valid_action_aim_pose(self):
        action = probe.empty_controller_input("actions")
        action["aim_pose"] = {
            "active": True,
            "pose_valid": True,
            "device_index": 7,
            "position_m": [1.0, 2.0, 3.0],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "matrix_3x4": [[1, 0, 0, 1], [0, 1, 0, 2], [0, 0, 1, 3]],
        }
        legacy = probe.empty_controller_input("legacy")
        legacy.update({"active": True, "device_index": 7, "trigger": 0.5})
        with patch.object(probe, "legacy_controller_input", return_value=legacy):
            merged = probe.resolve_controller_input(FakeVR, object(), 7, action)
        self.assertEqual(merged["source"], "legacy")
        self.assertEqual(merged["trigger"], 0.5)
        self.assertTrue(merged["aim_pose"]["pose_valid"])
        self.assertEqual(merged["aim_pose"]["position_m"], [1.0, 2.0, 3.0])

    def test_mismatched_aim_pose_is_rejected(self):
        action = probe.empty_controller_input("actions")
        action["aim_pose"].update({"active": True, "pose_valid": True, "device_index": 99})
        with patch.object(
            probe,
            "legacy_controller_input",
            return_value=probe.empty_controller_input("legacy"),
        ):
            merged = probe.resolve_controller_input(FakeVR, object(), 7, action)
        self.assertFalse(merged["aim_pose"]["pose_valid"])

    def test_only_running_ok_tracking_is_accepted(self):
        self.assertTrue(probe.pose_is_running_ok(FakeVR, FakePose(True, True, 200)))
        self.assertFalse(probe.pose_is_running_ok(FakeVR, FakePose(True, True, 201)))
        self.assertFalse(probe.pose_is_running_ok(FakeVR, FakePose(False, True, 200)))


class ManifestTests(unittest.TestCase):
    def test_actions_and_bindings_are_consistent(self):
        manifest = json.loads((ROOT / "actions.json").read_text())
        action_names = {entry["name"] for entry in manifest["actions"]}
        expected = {
            f"/actions/teleop/in/{hand}_{name}"
            for hand in ("left", "right")
            for name in (*probe.ANALOG_ACTIONS, *probe.DIGITAL_ACTIONS)
        }
        expected.update(
            f"/actions/teleop/in/{hand}_aim_pose"
            for hand in ("left", "right")
        )
        self.assertEqual(action_names, expected)
        for default in manifest["default_bindings"]:
            binding_path = ROOT / default["binding_url"]
            binding = json.loads(binding_path.read_text())
            self.assertEqual(binding["controller_type"], default["controller_type"])
            for source in binding["bindings"]["/actions/teleop"]["sources"]:
                for output in source["inputs"].values():
                    self.assertIn(output["output"], action_names)
            for pose_binding in binding["bindings"]["/actions/teleop"]["poses"]:
                self.assertIn(pose_binding["output"], action_names)
                self.assertTrue(pose_binding["path"].endswith("/pose/openxr_aim"))


if __name__ == "__main__":
    unittest.main()
