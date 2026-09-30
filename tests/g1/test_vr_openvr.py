"""Exercise real pyopenvr ctypes contracts through a fake runtime, not a headset."""

import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

try:
    import openvr
except ImportError:
    openvr = None

from g1_teleop.vr.openvr_source import ANALOG_ACTIONS, APP_KEY, DIGITAL_ACTIONS, OpenVRSource


@unittest.skipIf(openvr is None, "optional openvr dependency is absent")
class OpenVRTests(unittest.TestCase):
    def make_runtime(self, failure=None):
        state = SimpleNamespace(missing=False, inactive=False, closed=False, event=None, app_type=None,
                                input_available=True, pause=False, calls=[], installed=False,
                                identified_pid=0, existing_pid=0, manifest_path=None, failure=failure)
        def record(name):
            state.calls.append(name)
            if state.failure == name:
                raise RuntimeError(f"mock {name} failure")

        class Applications:
            def isApplicationInstalled(self, key):
                assert key == APP_KEY
                return state.installed or bool(state.existing_pid)

            def getApplicationProcessId(self, key):
                assert key == APP_KEY
                return state.existing_pid or state.identified_pid

            def addApplicationManifest(self, path, temporary=False):
                assert temporary is True
                state.manifest_path = Path(path)
                state.manifest = json.loads(state.manifest_path.read_text())
                state.installed = state.failure != "unrecognized"
                record("add")

            def identifyApplication(self, pid, key):
                assert key == APP_KEY and state.installed
                assert pid == os.getpid()
                record("identify")
                state.identified_pid = 0 if state.failure == "wrong_pid" else pid

            def removeApplicationManifest(self, path):
                assert Path(path) == state.manifest_path
                assert state.manifest_path.is_file()
                record("remove")
                state.installed = False

        class System:
            def isInputAvailable(self):
                return state.input_available

            def shouldApplicationPause(self):
                return state.pause

            def pollNextEvent(self, event):
                if state.event is None:
                    return False
                event.eventType = state.event
                state.event = None
                return True

            def getDeviceToAbsoluteTrackingPose(self, origin, prediction, poses):
                self_origin = openvr.TrackingUniverseStanding
                assert origin == self_origin and prediction == 0
                for index in (0, 4, 7):
                    poses[index].bDeviceIsConnected = True
                    poses[index].bPoseIsValid = True
                    poses[index].eTrackingResult = openvr.TrackingResult_Running_OK
                    for row in range(3):
                        for col in range(4):
                            poses[index].mDeviceToAbsoluteTracking.m[row][col] = float(row == col)
                    poses[index].mDeviceToAbsoluteTracking.m[1][3] = 1.5

            def getTrackedDeviceIndexForControllerRole(self, role):
                if state.missing and role == openvr.TrackedControllerRole_LeftHand:
                    return openvr.k_unTrackedDeviceIndexInvalid
                return 4 if role == openvr.TrackedControllerRole_LeftHand else 7

        class Inputs:
            def setActionManifestPath(self, path):
                assert Path(path).is_file()
                assert state.identified_pid == os.getpid()
                assert state.manifest["applications"][0]["action_manifest_path"] == path
                record("actions")

            def getActionHandle(self, path):
                return path.rsplit("/", 1)[-1]

            def getActionSetHandle(self, path):
                return 42

            def updateActionState(self, sets):
                assert sets[0].ulActionSet == 42
                assert sets[0].nPriority == 0
                record("update")

            def getAnalogActionData(self, action, device):
                assert action in ANALOG_ACTIONS
                return SimpleNamespace(bActive=not state.inactive, x=0.8 if "grip" in action else 0.1, y=0.2)

            def getDigitalActionData(self, action, device):
                assert action in DIGITAL_ACTIONS
                return SimpleNamespace(bActive=not state.inactive, bState=action == "calibrate")

        def init(app_type):
            state.app_type = app_type
            record("init")
            return System()
        def shutdown():
            state.closed = True
            record("shutdown")
        values = {name: getattr(openvr, name) for name in (
            "VRApplication_Background", "VRActiveActionSet_t", "k_ulInvalidInputValueHandle",
            "TrackedDevicePose_t", "k_unMaxTrackedDeviceCount", "VREvent_t", "TrackingUniverseStanding",
            "k_unTrackedDeviceIndex_Hmd", "TrackedControllerRole_LeftHand", "TrackedControllerRole_RightHand",
            "TrackingResult_Running_OK", "VREvent_Quit", "VREvent_ProcessQuit",
            "VREvent_ChaperoneUniverseHasChanged", "VREvent_SeatedZeroPoseReset")}
        runtime = SimpleNamespace(**values, init=init, shutdown=shutdown, VRInput=Inputs,
                                  VRApplications=Applications)
        return runtime, state

    def test_role_lookup_and_background_action_bindings(self):
        runtime, state = self.make_runtime()
        source = OpenVRSource(vr_module=runtime)
        try:
            raw = source.sample()
            self.assertEqual(state.app_type, openvr.VRApplication_Background)
            self.assertEqual(source.device_ids, {"head": 0, "left": 4, "right": 7})
            self.assertTrue(raw.tracking_valid)
            self.assertTrue(raw.inputs_valid)
            self.assertTrue(raw.calibrate)
            self.assertEqual(raw.grips, (0.8, 0.8))
            app = state.manifest["applications"][0]
            self.assertEqual(app["app_key"], APP_KEY)
            self.assertEqual(app["launch_type"], "binary")
            self.assertFalse(app["is_dashboard_overlay"])
            binary_key = {"linux": "binary_path_linux", "win32": "binary_path_windows",
                          "darwin": "binary_path_osx"}[sys.platform]
            self.assertEqual(app[binary_key], str(Path(sys.executable).absolute()))
            self.assertEqual(state.calls[:5], ["init", "add", "identify", "actions", "update"])
            state.missing = True
            self.assertFalse(source.sample().tracking_valid)
            state.inactive = True
            inactive = source.sample()
            self.assertFalse(inactive.inputs_valid)
            self.assertFalse(inactive.calibrate)
            self.assertEqual(inactive.grips, (0, 0))
            self.assertIn("left_grip", source.last_error)
        finally:
            source.close()
        self.assertTrue(state.closed)
        self.assertEqual(state.calls[-2:], ["remove", "shutdown"])
        self.assertFalse(state.manifest_path.parent.exists())
        calls = list(state.calls)
        source.close()
        self.assertEqual(state.calls, calls)

    def test_tracking_origin_change_and_runtime_quit(self):
        runtime, state = self.make_runtime()
        source = OpenVRSource(vr_module=runtime)
        try:
            for event in (801, 804, 807, 808):
                state.event = event
                self.assertTrue(source.sample().reference_changed)
            state.event = openvr.VREvent_Quit
            with self.assertRaises(RuntimeError):
                source.sample()
        finally:
            source.close()

    def test_dashboard_and_input_focus_fail_closed(self):
        runtime, state = self.make_runtime()
        source = OpenVRSource(vr_module=runtime)
        try:
            for field in ("input_available", "pause"):
                state.input_available, state.pause = True, False
                setattr(state, field, field == "pause")
                before = state.calls.count("update")
                frame = source.sample()
                self.assertTrue(frame.tracking_valid)
                self.assertFalse(frame.inputs_valid)
                self.assertFalse(frame.calibrate)
                self.assertEqual(frame.grips, (0, 0))
                self.assertEqual(state.calls.count("update"), before)
                self.assertIn("input unavailable", source.last_error)
        finally:
            source.close()

    def test_register_identify_and_action_failures_cleanup(self):
        for failure in ("add", "identify", "actions"):
            with self.subTest(failure=failure):
                runtime, state = self.make_runtime(failure)
                with self.assertRaisesRegex(RuntimeError, "mock .* failure") as caught:
                    OpenVRSource(vr_module=runtime)
                if failure != "actions":
                    self.assertIn("Input remains disabled", str(caught.exception))
                self.assertEqual(state.calls[-2:], ["remove", "shutdown"])
                self.assertTrue(state.closed)
                self.assertFalse(state.manifest_path.parent.exists())

    def test_shutdown_even_if_runtime_init_fails(self):
        runtime, state = self.make_runtime("init")
        with self.assertRaisesRegex(RuntimeError, "mock init failure"):
            OpenVRSource(vr_module=runtime)
        self.assertEqual(state.calls, ["init", "shutdown"])

    def test_success_return_with_missing_registration_or_identity_is_rejected(self):
        for failure in ("unrecognized", "wrong_pid"):
            with self.subTest(failure=failure):
                runtime, state = self.make_runtime(failure)
                with self.assertRaisesRegex(RuntimeError, "Input remains disabled"):
                    OpenVRSource(vr_module=runtime)
                self.assertNotIn("actions", state.calls)
                self.assertEqual(state.calls[-2:], ["remove", "shutdown"])
                self.assertFalse(state.manifest_path.parent.exists())

    def test_reject_duplicate_without_removing_existing_manifest(self):
        runtime, state = self.make_runtime()
        state.existing_pid = 12345
        with self.assertRaisesRegex(RuntimeError, "close that reader first"):
            OpenVRSource(vr_module=runtime)
        self.assertEqual(state.calls, ["init", "shutdown"])
        self.assertIsNone(state.manifest_path)

    def test_cleanup_continues_when_runtime_unregister_or_shutdown_fails(self):
        for failure in ("remove", "shutdown"):
            with self.subTest(failure=failure):
                runtime, state = self.make_runtime()
                source = OpenVRSource(vr_module=runtime)
                state.failure = failure
                with self.assertLogs("g1_teleop.vr.openvr_source", level="WARNING"):
                    source.close()
                self.assertEqual(state.calls[-2:], ["remove", "shutdown"])
                self.assertFalse(state.manifest_path.parent.exists())
                self.assertTrue(source.cleanup_errors)

    def test_update_failure_never_reuses_previous_input(self):
        runtime, state = self.make_runtime()
        source = OpenVRSource(vr_module=runtime)
        try:
            self.assertTrue(source.sample().inputs_valid)
            state.failure = "update"
            frame = source.sample()
            self.assertFalse(frame.inputs_valid)
            self.assertFalse(frame.calibrate)
            self.assertEqual(frame.grips, (0, 0))
        finally:
            state.failure = None
            source.close()


if __name__ == "__main__":
    unittest.main()
