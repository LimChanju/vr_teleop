"""Live SteamVR/OpenVR provider for Quest/ALVR and generic 6DoF trackers.

Importing this module does not initialize a VR runtime. A provider owns one
background OpenVR connection and closes exactly the resources it initialized.
The MuJoCo VR viewer remains a separate scene application/process.
"""

from __future__ import annotations

import os
from pathlib import Path
import time
from typing import Any, Mapping

import numpy as np

from ..math3d import matrix_to_quaternion
from ..models import PoseSample, TrackerFrame
from ..openvr_actions import (
    ActionReader,
    choose_primary_devices,
    pose_is_running_ok,
    resolve_controller_input,
    safe_property,
)
from .base import VRInputProvider


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_ACTION_MANIFEST = ROOT / "config/openvr_actions.json"
DEFAULT_APPLICATION_MANIFEST = ROOT / "config/openvr_app.vrmanifest"
APPLICATION_KEY = "io.github.amazon-far.twist2.vr_input"


def _matrix34(matrix: Any) -> tuple[np.ndarray, np.ndarray]:
    rows_source = matrix.m if hasattr(matrix, "m") else matrix
    rows = np.array(
        [[float(rows_source[r][c]) for c in range(4)] for r in range(3)],
        dtype=np.float64,
    )
    return rows[:, 3], matrix_to_quaternion(rows[:, :3])


class OpenVRProvider(VRInputProvider):
    """Read tracking and Quest Touch controls without submitting VR frames.

    ``serial_to_role`` remains the public mapping API. HMD and physical Touch
    controllers are assigned automatically, while GenericTracker devices are
    accepted only through an explicit stable-serial mapping.
    """

    def __init__(
        self,
        serial_to_role: Mapping[str, str] | None = None,
        *,
        action_manifest: str | Path = DEFAULT_ACTION_MANIFEST,
        application_manifest: str | Path = DEFAULT_APPLICATION_MANIFEST,
    ) -> None:
        try:
            import openvr
        except ImportError as error:
            raise RuntimeError("OpenVR support is optional; install the project with [openvr]") from error

        self.vr = openvr
        self.serial_to_role = dict(serial_to_role or {})
        self.action_manifest = Path(action_manifest).expanduser().resolve()
        self.application_manifest = Path(application_manifest).expanduser().resolve()
        self.sequence = 0
        self._closed = False
        self.system: Any = None
        self.applications: Any = None
        self.actions: ActionReader | None = None
        self.action_error: str | None = None
        self._application_registered = False
        self._properties: dict[int, dict[str, Any]] = {}
        self._connected: dict[int, bool] = {}
        self._generations: dict[int, int] = {}
        try:
            self.system = openvr.init(openvr.VRApplication_Background)
            self._initialize_actions()
        except Exception:
            self.close()
            raise

    def _initialize_actions(self) -> None:
        """Register a self-identified action app, falling back to legacy input."""

        try:
            self.applications = self.vr.VRApplications()
            self.applications.addApplicationManifest(str(self.application_manifest), True)
            self._application_registered = True
            self.applications.identifyApplication(os.getpid(), APPLICATION_KEY)
            self.actions = ActionReader(
                self.vr,
                self.action_manifest,
                self.vr.TrackingUniverseStanding,
            )
        except Exception as error:
            self.actions = None
            self.action_error = f"{type(error).__name__}: {error}"

    def _property(self, index: int, property_name: str) -> str:
        return safe_property(self.vr, self.system, index, property_name)

    def _device_properties(self, index: int, device_class: int) -> dict[str, Any]:
        current = {
            "serial": self._property(index, "Prop_SerialNumber_String"),
            "model": self._property(index, "Prop_ModelNumber_String"),
            "controller_type": self._property(index, "Prop_ControllerType_String"),
            "tracking_system": self._property(index, "Prop_TrackingSystemName_String"),
            "device_class": int(device_class),
        }
        previous = self._properties.get(index, {})
        # A sleeping/disconnected driver may temporarily reject property reads.
        # Preserve the last non-empty identity until the index is reused.
        for name in ("serial", "model", "controller_type", "tracking_system"):
            if not current[name] and previous.get(name):
                current[name] = previous[name]
        return current

    def _refresh_generations(self, poses: Any) -> None:
        """Track disconnects and device-index reuse for calibration invalidation."""

        for index, pose in enumerate(poses):
            try:
                device_class = int(self.system.getTrackedDeviceClass(index))
            except Exception:
                device_class = int(self.vr.TrackedDeviceClass_Invalid)
            connected = bool(pose.bDeviceIsConnected)
            if (
                device_class == int(self.vr.TrackedDeviceClass_Invalid)
                and index not in self._properties
                and not connected
            ):
                continue
            properties = self._device_properties(index, device_class)
            previous = self._properties.get(index)
            previous_connected = self._connected.get(index)
            generation = self._generations.setdefault(index, 0)
            identity = (
                properties["serial"],
                properties["model"],
                properties["tracking_system"],
                properties["device_class"],
            )
            previous_identity = None if previous is None else (
                previous.get("serial", ""),
                previous.get("model", ""),
                previous.get("tracking_system", ""),
                previous.get("device_class", int(self.vr.TrackedDeviceClass_Invalid)),
            )
            if previous_connected is not None and previous_connected != connected:
                generation += 1
            elif connected and previous_identity is not None and identity != previous_identity:
                generation += 1
            self._generations[index] = generation
            self._connected[index] = connected
            self._properties[index] = properties

    def device_inventory(self) -> list[dict[str, Any]]:
        poses_raw = self.system.getDeviceToAbsoluteTrackingPose(
            self.vr.TrackingUniverseStanding,
            0.0,
            self.vr.k_unMaxTrackedDeviceCount,
        )
        self._refresh_generations(poses_raw)
        selected = choose_primary_devices(self.vr, self.system, poses_raw)
        automatic_roles = {
            selected["head"]: "Head",
            selected["left"]: "Left_Wrist",
            selected["right"]: "Right_Wrist",
        }
        result = []
        for index in range(self.vr.k_unMaxTrackedDeviceCount):
            device_class = int(self.system.getTrackedDeviceClass(index))
            if device_class == int(self.vr.TrackedDeviceClass_Invalid):
                continue
            properties = self._device_properties(index, device_class)
            serial = str(properties["serial"])
            result.append(
                {
                    "index": index,
                    "class": device_class,
                    "class_name": {
                        int(self.vr.TrackedDeviceClass_HMD): "hmd",
                        int(self.vr.TrackedDeviceClass_Controller): "controller",
                        int(self.vr.TrackedDeviceClass_GenericTracker): "generic_tracker",
                        int(self.vr.TrackedDeviceClass_TrackingReference): "tracking_reference",
                    }.get(device_class, "other"),
                    **properties,
                    "connected": bool(poses_raw[index].bDeviceIsConnected),
                    "pose_valid": pose_is_running_ok(self.vr, poses_raw[index]),
                    "tracking_result": int(poses_raw[index].eTrackingResult),
                    "connection_generation": self._generations.get(index, 0),
                    "automatic_role": automatic_roles.get(index),
                    "mapped_role": self.serial_to_role.get(serial),
                }
            )
        return result

    def _read_actions(self) -> dict[str, dict[str, Any]]:
        if self.actions is None:
            return {}
        try:
            result = self.actions.read()
            self.action_error = None
            return result
        except Exception as error:
            self.action_error = f"{type(error).__name__}: {error}"
            return {}

    def _sample_for_role(
        self,
        role: str,
        index: int,
        tracked: Any,
        timestamp_s: float,
        controller_input: Mapping[str, Any] | None = None,
    ) -> PoseSample:
        properties = self._properties[index]
        valid = pose_is_running_ok(self.vr, tracked)
        pose_matrix: Any = tracked.mDeviceToAbsoluteTracking
        tracking_result = int(tracked.eTrackingResult)
        aim_identity_validated = False
        if controller_input is not None:
            aim = controller_input.get("aim_pose", {})
            aim_identity_validated = bool(
                aim.get("active")
                and aim.get("pose_valid")
                and aim.get("device_index") == index
            )

        # Keep the tracked controller origin for the anatomical wrist pose.
        # SteamVR's /pose/tip has a controller-local translation offset, so
        # using it here makes a stationary wrist orbit when the user merely
        # rotates the controller. The action pose still verifies that action
        # channels originate from this exact physical device.

        if valid:
            position, orientation = _matrix34(pose_matrix)
        else:
            position = np.zeros(3, dtype=np.float64)
            orientation = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        serial = str(properties["serial"])
        return PoseSample(
            device_id=serial or f"openvr-index-{index}",
            role=role,
            position_m=position,
            orientation_xyzw=orientation,
            timestamp_s=timestamp_s,
            valid=valid,
            tracking_state=str(tracking_result),
            metadata={
                "openvr_index": index,
                "device_class": int(properties["device_class"]),
                "serial": serial,
                "model": str(properties["model"]),
                "controller_type": str(properties["controller_type"]),
                "tracking_system": str(properties["tracking_system"]),
                "connection_generation": self._generations[index],
                "pose_source": "device",
                "aim_identity_validated": aim_identity_validated,
            },
        )

    def read(self) -> TrackerFrame:
        now = time.monotonic()
        poses_raw = self.system.getDeviceToAbsoluteTrackingPose(
            self.vr.TrackingUniverseStanding,
            0.0,
            self.vr.k_unMaxTrackedDeviceCount,
        )
        self._refresh_generations(poses_raw)
        selected = choose_primary_devices(self.vr, self.system, poses_raw)
        action_inputs = self._read_actions()
        controls = {
            hand: resolve_controller_input(
                self.vr,
                self.system,
                selected[hand],
                action_inputs.get(hand),
            )
            for hand in ("left", "right")
        }

        # Explicit stable-serial mappings are the only automatic path for
        # GenericTracker devices. An explicit mapping also overrides an
        # automatic HMD/controller role for the same device.
        explicit_by_index: dict[int, str] = {}
        for index, properties in self._properties.items():
            if not self._connected.get(index, False):
                continue
            role = self.serial_to_role.get(str(properties.get("serial", "")))
            if role:
                explicit_by_index[index] = role

        candidates: list[tuple[int, str, int, Mapping[str, Any] | None]] = []
        automatic = {
            "Head": selected["head"],
            "Left_Wrist": selected["left"],
            "Right_Wrist": selected["right"],
        }
        for role, index in automatic.items():
            if index is None or index in explicit_by_index:
                continue
            hand = "left" if role == "Left_Wrist" else "right" if role == "Right_Wrist" else None
            candidates.append((1, role, index, controls[hand] if hand else None))
        for index, role in explicit_by_index.items():
            hand = "left" if index == selected["left"] else "right" if index == selected["right"] else None
            candidates.append((2, role, index, controls[hand] if hand else None))

        poses: dict[str, PoseSample] = {}
        priorities: dict[str, int] = {}
        for priority, role, index, controller_input in candidates:
            previous_priority = priorities.get(role, -1)
            if priority < previous_priority:
                continue
            poses[role] = self._sample_for_role(
                role,
                index,
                poses_raw[index],
                now,
                controller_input,
            )
            priorities[role] = priority

        frame = TrackerFrame(
            timestamp_s=now,
            poses=poses,
            source="openvr",
            coordinate_frame="openvr_standing",
            controls=controls,
            sequence=self.sequence,
        )
        self.sequence += 1
        return frame

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            if self._application_registered and self.applications is not None:
                try:
                    self.applications.removeApplicationManifest(str(self.application_manifest))
                except Exception:
                    pass
        finally:
            self._application_registered = False
            self.actions = None
            self.applications = None
            if self.system is not None:
                try:
                    self.vr.shutdown()
                finally:
                    self.system = None


__all__ = [
    "APPLICATION_KEY",
    "DEFAULT_ACTION_MANIFEST",
    "DEFAULT_APPLICATION_MANIFEST",
    "OpenVRProvider",
    "_matrix34",
]
