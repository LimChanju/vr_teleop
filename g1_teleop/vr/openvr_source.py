"""SteamVR background reader using IVRInput actions and standing-space poses.

This module never starts SteamVR, claims its scene compositor, changes a user's
ALVR configuration, or initializes a robot SDK. openvr is imported only when a
physical reader is explicitly constructed.
"""

from __future__ import annotations

import json
import logging
import os
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

from .calibration import RawFrame, valid_poses

ANALOG_ACTIONS = ("left_stick", "right_stick", "left_grip", "right_grip", "left_trigger", "right_trigger")
DIGITAL_ACTIONS = ("calibrate", "arm", "stop", "reset")
APP_KEY = "org.limchanju.vr_teleop.g1_input"


class OpenVRSource:
    def __init__(self, manifest=None, vr_module=None):
        if vr_module is None:
            try:
                import openvr as vr_module
            except ImportError as exc:
                raise RuntimeError("Install g1_teleop/vr/requirements.txt in a separate VR venv first.") from exc
        self.vr = vr_module
        self.system = None
        self.applications = None
        self.app_key = APP_KEY
        self.app_manifest_path = None
        self._manifest_dir = None
        self._registration_attempted = False
        self._shutdown_needed = False
        self.cleanup_errors = []
        self.last_error = ""
        self.device_ids = {}
        try:
            path = Path(manifest) if manifest else Path(__file__).with_name("actions.json")
            path = path.resolve(strict=True)
            self._shutdown_needed = True
            self.system = self.vr.init(self.vr.VRApplication_Background)
            self._register_application(path)
            self.inputs = self.vr.VRInput()
            self.inputs.setActionManifestPath(str(path))
            self.actions = {name: self.inputs.getActionHandle(f"/actions/g1/in/{name}")
                            for name in ANALOG_ACTIONS + DIGITAL_ACTIONS}
            self.action_sets = (self.vr.VRActiveActionSet_t * 1)()
            self.action_sets[0].ulActionSet = self.inputs.getActionSetHandle("/actions/g1")
            self.action_sets[0].ulRestrictedToDevice = self.vr.k_ulInvalidInputValueHandle
            self.action_sets[0].nPriority = 0
            self.poses = (self.vr.TrackedDevicePose_t * self.vr.k_unMaxTrackedDeviceCount)()
        except BaseException:
            self.close()
            raise

    def _register_application(self, action_manifest):
        """Give this process its own bindings identity without claiming the scene.

        Valve's SteamVR Unity plugin similarly registers a temporary manifest
        then identifies the current PID. This descriptor is registration metadata;
        we never call launchApplication or configure SteamVR autostart.
        """
        try:
            self.applications = self.vr.VRApplications()
            if self.applications.isApplicationInstalled(self.app_key):
                pid = self.applications.getApplicationProcessId(self.app_key)
                if pid:
                    raise RuntimeError(f"G1 input reader already registered with PID {pid}; close that reader first")
            binary_key = {"linux": "binary_path_linux", "win32": "binary_path_windows",
                          "darwin": "binary_path_osx"}.get(sys.platform)
            if binary_key is None:
                raise RuntimeError(f"Unsupported OpenVR application manifest platform: {sys.platform}")
            application = {
                "app_key": self.app_key,
                "launch_type": "binary",
                binary_key: str(Path(sys.executable).absolute()),
                "is_dashboard_overlay": False,
                "action_manifest_path": str(action_manifest),
                "strings": {"en_us": {"name": "G1 Teleop Input",
                                       "description": "ALVR controller input for simulated G1"}},
            }
            self._manifest_dir = tempfile.TemporaryDirectory(prefix="g1-openvr-")
            self.app_manifest_path = Path(self._manifest_dir.name) / "g1_input.vrmanifest"
            self.app_manifest_path.write_text(json.dumps({"applications": [application]}, indent=2), encoding="utf-8")
            # Also remove our own path if the runtime partially registers then raises.
            self._registration_attempted = True
            self.applications.addApplicationManifest(str(self.app_manifest_path), True)
            if not self.applications.isApplicationInstalled(self.app_key):
                raise RuntimeError("SteamVR did not recognize the temporary G1 application manifest")
            self.applications.identifyApplication(os.getpid(), self.app_key)
            if self.applications.getApplicationProcessId(self.app_key) != os.getpid():
                raise RuntimeError("SteamVR did not associate the G1 app key with this process")
        except Exception as exc:
            raise RuntimeError(
                f"SteamVR G1 application registration/identification failed ({self.app_key}): {exc}. "
                "Start SteamVR in this user's graphical session, close duplicate G1 input readers, "
                "and check the SteamVR application log. Input remains disabled."
            ) from exc

    def sample(self):
        vr = self.vr
        sampled = time.monotonic()
        changed = False
        event = vr.VREvent_t()
        for _ in range(128):
            if not self.system.pollNextEvent(event):
                break
            if event.eventType in (getattr(vr, "VREvent_Quit", 700), getattr(vr, "VREvent_ProcessQuit", 701)):
                raise RuntimeError("SteamVR is shutting down; target stream stopped")
            if event.eventType in (getattr(vr, "VREvent_ChaperoneUniverseHasChanged", 801),
                                   getattr(vr, "VREvent_SeatedZeroPoseReset", 804),
                                   getattr(vr, "VREvent_ChaperoneRoomSetupCommitted", 807),
                                   getattr(vr, "VREvent_StandingZeroPoseReset", 808)):
                changed = True
        self.system.getDeviceToAbsoluteTrackingPose(vr.TrackingUniverseStanding, 0.0, self.poses)
        ids = [vr.k_unTrackedDeviceIndex_Hmd,
               self.system.getTrackedDeviceIndexForControllerRole(vr.TrackedControllerRole_LeftHand),
               self.system.getTrackedDeviceIndexForControllerRole(vr.TrackedControllerRole_RightHand)]
        if self.device_ids and list(self.device_ids.values()) != ids:
            changed = True
        self.device_ids = dict(zip(("head", "left", "right"), ids))
        matrices = np.tile(np.eye(4)[:3], (3, 1, 1))
        valid = len(set(ids)) == 3
        for i, device_id in enumerate(ids):
            if not 0 <= device_id < len(self.poses):
                valid = False
                continue
            pose = self.poses[device_id]
            valid = valid and bool(pose.bDeviceIsConnected and pose.bPoseIsValid
                                    and pose.eTrackingResult == vr.TrackingResult_Running_OK)
            matrices[i] = [[pose.mDeviceToAbsoluteTracking.m[r][c] for c in range(4)] for r in range(3)]
        raw = RawFrame(poses=matrices, tracking_valid=bool(valid and valid_poses(matrices)),
                       sampled_monotonic=sampled, reference_changed=changed)
        try:
            if not self.system.isInputAvailable() or self.system.shouldApplicationPause():
                self.last_error = f"SteamVR input unavailable ({self.app_key}): close dashboard and return to the scene."
                return raw
            self.inputs.updateActionState(self.action_sets)
            analog = {name: self.inputs.getAnalogActionData(self.actions[name], vr.k_ulInvalidInputValueHandle)
                      for name in ANALOG_ACTIONS}
            digital = {name: self.inputs.getDigitalActionData(self.actions[name], vr.k_ulInvalidInputValueHandle)
                       for name in DIGITAL_ACTIONS}
            inactive = [name for name, action in {**analog, **digital}.items() if not action.bActive]
            if inactive:
                self.last_error = (f"Actions inactive ({self.app_key}): {', '.join(inactive)}. "
                                   "Check SteamVR G1 Teleop Input bindings/controller profile and close dashboard.")
                return raw
            raw.inputs_valid = True
            raw.left_stick = (analog["left_stick"].x, analog["left_stick"].y)
            raw.right_stick = (analog["right_stick"].x, analog["right_stick"].y)
            raw.grips = (analog["left_grip"].x, analog["right_grip"].x)
            raw.triggers = (analog["left_trigger"].x, analog["right_trigger"].x)
            for name, action in digital.items():
                setattr(raw, name, bool(action.bActive and action.bState))
            self.last_error = ""
        except Exception as exc:
            # Never reuse the last button state on an input API error.
            raw.inputs_valid = False
            self.last_error = f"SteamVR action read failed: {type(exc).__name__}: {exc}"
        return raw

    def close(self):
        # Every cleanup step runs even if SteamVR has already exited. Do not mask
        # an initialization/input exception with a secondary teardown failure.
        if self._registration_attempted:
            self._registration_attempted = False
            try:
                self.applications.removeApplicationManifest(str(self.app_manifest_path))
            except Exception as exc:
                self.cleanup_errors.append(f"removeApplicationManifest: {exc}")
        self.system = None
        if self._shutdown_needed:
            self._shutdown_needed = False
            try:
                self.vr.shutdown()
            except Exception as exc:
                self.cleanup_errors.append(f"shutdown: {exc}")
        if self._manifest_dir is not None:
            directory, self._manifest_dir = self._manifest_dir, None
            try:
                directory.cleanup()
            except Exception as exc:
                self.cleanup_errors.append(f"temporary manifest cleanup: {exc}")
        if self.cleanup_errors:
            logging.getLogger(__name__).warning("SteamVR cleanup: %s", "; ".join(self.cleanup_errors))


class SyntheticSource:
    """Deterministic simulated input for IPC tests; never reported as hardware."""

    def __init__(self, enabled=False):
        self.started = time.monotonic()
        self.index = 0
        self.enabled = enabled
        self.last_error = "SYNTHETIC INPUT: no physical headset verification"

    def sample(self):
        t = time.monotonic() - self.started
        poses = np.tile(np.eye(4)[:3], (3, 1, 1))
        poses[:, :, 3] = [[0, 1.65, 0], [-0.28, 1.2, -0.25], [0.28, 1.2, -0.25]]
        if self.index > 1:
            poses[1, 1, 3] += 0.06 * np.sin(t)
            poses[2, 2, 3] -= 0.06 * np.sin(t * 0.7)
        raw = RawFrame(poses=poses, tracking_valid=True, inputs_valid=True,
                       calibrate=self.index == 0, arm=self.enabled and self.index == 1,
                       grips=(0.85, 0.85) if self.enabled else (0, 0), source="synthetic")
        self.index += 1
        return raw

    def close(self):
        pass
