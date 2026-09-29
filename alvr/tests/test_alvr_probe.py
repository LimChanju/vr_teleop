"""Host-side unit tests; no headset or running SteamVR is required."""

import importlib.util
import json
import math
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("alvr_probe", ROOT / "alvr_probe.py")
probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)


class FakeMatrix:
    def __init__(self, rows):
        self.m = rows


class FakePose:
    def __init__(self, connected, valid):
        self.bDeviceIsConnected = connected
        self.bPoseIsValid = valid


class FakeVR:
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


class ManifestTests(unittest.TestCase):
    def test_actions_and_bindings_are_consistent(self):
        manifest = json.loads((ROOT / "actions.json").read_text())
        action_names = {entry["name"] for entry in manifest["actions"]}
        expected = {
            f"/actions/teleop/in/{hand}_{name}"
            for hand in ("left", "right")
            for name in (*probe.ANALOG_ACTIONS, *probe.DIGITAL_ACTIONS)
        }
        self.assertEqual(action_names, expected)
        for default in manifest["default_bindings"]:
            binding_path = ROOT / default["binding_url"]
            binding = json.loads(binding_path.read_text())
            self.assertEqual(binding["controller_type"], default["controller_type"])
            for source in binding["bindings"]["/actions/teleop"]["sources"]:
                for output in source["inputs"].values():
                    self.assertIn(output["output"], action_names)


if __name__ == "__main__":
    unittest.main()
