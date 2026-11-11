"""SteamVR action input helpers shared by the live OpenVR provider.

The helpers in this module do not initialize or shut down OpenVR.  Runtime
ownership remains with :class:`twist2_vr.providers.openvr.OpenVRProvider`, so a
provider cannot accidentally tear down the MuJoCo scene application's OpenVR
connection. Action input identifies the physical device and supplies buttons;
the provider retains the raw controller origin for translation-only arm IK.
The legacy controller API is a per-channel fallback for incomplete bindings.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any


ANALOG_ACTIONS = {
    "trigger": "vector1",
    "grip": "vector1",
    "thumbstick": "vector2",
}
DIGITAL_ACTIONS = ("primary", "secondary", "thumbstick_click")


def safe_property(vr: Any, system: Any, device_index: int, property_name: str) -> str:
    """Return one tracked-device string property without leaking runtime errors."""

    try:
        return str(system.getStringTrackedDeviceProperty(device_index, getattr(vr, property_name)))
    except Exception:
        return ""


def pose_is_running_ok(vr: Any, pose: Any) -> bool:
    """Accept only connected, valid, fully-running tracking poses."""

    return bool(
        pose.bDeviceIsConnected
        and pose.bPoseIsValid
        and int(pose.eTrackingResult) == int(vr.TrackingResult_Running_OK)
    )


def _device_identity(vr: Any, system: Any, index: int) -> tuple[str, str, str]:
    return (
        safe_property(vr, system, index, "Prop_SerialNumber_String"),
        safe_property(vr, system, index, "Prop_ModelNumber_String"),
        safe_property(vr, system, index, "Prop_ControllerType_String"),
    )


def choose_primary_devices(vr: Any, system: Any, poses: Any) -> dict[str, int | None]:
    """Resolve the ALVR HMD and physical left/right controllers every frame.

    SteamVR device indices can change after a controller sleeps or reconnects.
    Controller candidates must share the selected HMD's tracking-system name;
    synthetic hand-tracking controllers are deliberately excluded.
    """

    head_candidates: list[tuple[int, int]] = []
    for index, pose in enumerate(poses):
        try:
            device_class = int(system.getTrackedDeviceClass(index))
        except Exception:
            continue
        if device_class != int(vr.TrackedDeviceClass_HMD) or not pose.bDeviceIsConnected:
            continue
        score = 4 * bool(pose.bPoseIsValid) + 2 * pose_is_running_ok(vr, pose)
        head_candidates.append((score, index))

    head_index = max(head_candidates, default=(-1, None), key=lambda item: (item[0], -item[1]))[1]
    head_tracking_system = (
        safe_property(vr, system, head_index, "Prop_TrackingSystemName_String")
        if head_index is not None
        else ""
    )

    candidates: dict[str, list[tuple[int, int]]] = {"left": [], "right": []}
    for index, pose in enumerate(poses):
        if not pose.bDeviceIsConnected:
            continue
        try:
            device_class = int(system.getTrackedDeviceClass(index))
        except Exception:
            continue
        if device_class != int(vr.TrackedDeviceClass_Controller):
            continue
        tracking_system = safe_property(vr, system, index, "Prop_TrackingSystemName_String")
        if head_tracking_system and tracking_system != head_tracking_system:
            continue

        serial, model, controller_type = _device_identity(vr, system, index)
        identity = f"{serial} {model} {controller_type}".lower()
        controller_type_lower = controller_type.lower()
        if "hand" in controller_type_lower or "hand tracker" in identity:
            continue
        try:
            role = int(system.getControllerRoleForTrackedDeviceIndex(index))
        except Exception:
            role = 0
        if role == int(vr.TrackedControllerRole_LeftHand):
            side = "left"
        elif role == int(vr.TrackedControllerRole_RightHand):
            side = "right"
        elif "left" in identity:
            side = "left"
        elif "right" in identity:
            side = "right"
        else:
            continue

        score = (
            8 * pose_is_running_ok(vr, pose)
            + 4 * bool(pose.bPoseIsValid)
            + 2 * bool(role)
            + 2 * ("touch" in controller_type_lower or "touch" in identity)
        )
        candidates[side].append((score, index))

    return {
        "head": head_index,
        "left": max(candidates["left"], default=(-1, None), key=lambda item: (item[0], -item[1]))[1],
        "right": max(candidates["right"], default=(-1, None), key=lambda item: (item[0], -item[1]))[1],
    }


def empty_controller_input(source: str = "none") -> dict[str, Any]:
    return {
        "trigger": 0.0,
        "grip": 0.0,
        "thumbstick": [0.0, 0.0],
        "primary": False,
        "secondary": False,
        "thumbstick_click": False,
        "active": False,
        "available": {name: False for name in (*ANALOG_ACTIONS, *DIGITAL_ACTIONS)},
        "source": source,
        "device_index": None,
        "aim_pose": {
            "active": False,
            "pose_valid": False,
            "tracking_result": None,
            "device_index": None,
            "matrix_3x4": None,
        },
    }


def _finite_axis(value: Any, minimum: float = -1.0, maximum: float = 1.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(number):
        return 0.0
    return min(maximum, max(minimum, number))


def legacy_controller_input(
    vr: Any,
    system: Any,
    device_index: int | None,
) -> dict[str, Any]:
    """Read the Quest Touch legacy layout when one or more actions are absent."""

    if device_index is None:
        return empty_controller_input()
    try:
        valid, state = system.getControllerState(device_index)
    except Exception:
        valid = False
    if not valid:
        return empty_controller_input()

    def pressed(button: int) -> bool:
        return bool(int(state.ulButtonPressed) & (1 << int(button)))

    return {
        "trigger": _finite_axis(state.rAxis[1].x, 0.0, 1.0),
        "grip": _finite_axis(state.rAxis[2].x, 0.0, 1.0),
        "thumbstick": [
            _finite_axis(state.rAxis[0].x),
            _finite_axis(state.rAxis[0].y),
        ],
        "primary": pressed(vr.k_EButton_A),
        "secondary": pressed(vr.k_EButton_ApplicationMenu),
        "thumbstick_click": pressed(vr.k_EButton_Axis0),
        "active": True,
        "available": {name: True for name in (*ANALOG_ACTIONS, *DIGITAL_ACTIONS)},
        "source": "legacy",
        "device_index": device_index,
        "aim_pose": empty_controller_input()["aim_pose"],
    }


def resolve_controller_input(
    vr: Any,
    system: Any,
    device_index: int | None,
    action_input: dict[str, Any] | None,
) -> dict[str, Any]:
    """Merge action data with matching-device legacy channels safely."""

    controller = dict(action_input or empty_controller_input())
    aim_pose = dict(controller.get("aim_pose", empty_controller_input()["aim_pose"]))
    if controller.get("active") and controller.get("device_index") != device_index:
        controller = empty_controller_input("mismatched_action_device")
    if aim_pose.get("device_index") != device_index:
        aim_pose = dict(empty_controller_input()["aim_pose"])
    controller["aim_pose"] = aim_pose

    names = (*ANALOG_ACTIONS, *DIGITAL_ACTIONS)
    action_available = controller.get("available", {})
    needs_legacy = not controller.get("active") or not all(
        action_available.get(name, False) for name in names
    )
    if needs_legacy:
        legacy = legacy_controller_input(vr, system, device_index)
        if legacy["active"]:
            used_action = False
            used_legacy = False
            merged_available: dict[str, bool] = {}
            for name in names:
                if controller.get("active") and action_available.get(name, False):
                    used_action = True
                    merged_available[name] = True
                else:
                    controller[name] = legacy[name]
                    merged_available[name] = bool(legacy["available"].get(name, False))
                    used_legacy = used_legacy or merged_available[name]
            controller["available"] = merged_available
            controller["active"] = any(merged_available.values())
            controller["device_index"] = device_index
            controller["source"] = (
                "actions+legacy" if used_action and used_legacy else "legacy" if used_legacy else "actions"
            )
        elif controller.get("source") == "none":
            legacy["aim_pose"] = aim_pose
            controller = legacy
    controller["aim_pose"] = aim_pose
    return controller


def _matrix34_rows(matrix: Any) -> list[list[float]]:
    rows = matrix.m if hasattr(matrix, "m") else matrix
    return [[float(rows[row][column]) for column in range(4)] for row in range(3)]


class ActionReader:
    """Read Quest Touch controls and aim poses from one SteamVR action set."""

    def __init__(self, vr: Any, manifest: str | Path, origin: int) -> None:
        self.vr = vr
        self.origin = origin
        self.input = vr.VRInput()
        self.input.setActionManifestPath(str(Path(manifest).resolve()))
        action_set_handle = self.input.getActionSetHandle("/actions/teleop")
        self.active_sets = (vr.VRActiveActionSet_t * 1)()
        self.active_sets[0].ulActionSet = action_set_handle
        self.active_sets[0].ulRestrictedToDevice = vr.k_ulInvalidInputValueHandle
        names = (*ANALOG_ACTIONS, *DIGITAL_ACTIONS)
        self.handles = {
            hand: {
                name: self.input.getActionHandle(f"/actions/teleop/in/{hand}_{name}")
                for name in names
            }
            for hand in ("left", "right")
        }
        self.pose_handles = {
            hand: self.input.getActionHandle(f"/actions/teleop/in/{hand}_aim_pose")
            for hand in ("left", "right")
        }

    def _origin_index(self, active_origin: int) -> int | None:
        if not active_origin:
            return None
        info = self.input.getOriginTrackedDeviceInfo(active_origin)
        index = int(info.trackedDeviceIndex)
        return None if index == int(self.vr.k_unTrackedDeviceIndexInvalid) else index

    def read(self) -> dict[str, dict[str, Any]]:
        self.input.updateActionState(self.active_sets)
        result: dict[str, dict[str, Any]] = {}
        for hand, handles in self.handles.items():
            available: dict[str, bool] = {}
            values: dict[str, Any] = {}
            device_indexes: set[int] = set()
            for name, kind in ANALOG_ACTIONS.items():
                data = self.input.getAnalogActionData(
                    handles[name], self.vr.k_ulInvalidInputValueHandle
                )
                available[name] = bool(data.bActive)
                index = self._origin_index(data.activeOrigin) if data.bActive else None
                if index is not None:
                    device_indexes.add(index)
                if kind == "vector2":
                    values[name] = (
                        [_finite_axis(data.x), _finite_axis(data.y)]
                        if data.bActive
                        else [0.0, 0.0]
                    )
                else:
                    values[name] = _finite_axis(data.x, 0.0, 1.0) if data.bActive else 0.0
            for name in DIGITAL_ACTIONS:
                data = self.input.getDigitalActionData(
                    handles[name], self.vr.k_ulInvalidInputValueHandle
                )
                available[name] = bool(data.bActive)
                index = self._origin_index(data.activeOrigin) if data.bActive else None
                if index is not None:
                    device_indexes.add(index)
                values[name] = bool(data.bState) if data.bActive else False

            values.update(
                {
                    "active": any(available.values()) and len(device_indexes) == 1,
                    "available": available,
                    "source": "actions",
                    "device_index": next(iter(device_indexes)) if len(device_indexes) == 1 else None,
                }
            )
            try:
                pose_data = self.input.getPoseActionDataRelativeToNow(
                    self.pose_handles[hand],
                    self.origin,
                    0.0,
                    self.vr.k_ulInvalidInputValueHandle,
                )
                pose_index = self._origin_index(pose_data.activeOrigin) if pose_data.bActive else None
                pose_valid = bool(pose_data.bActive and pose_is_running_ok(self.vr, pose_data.pose))
                values["aim_pose"] = {
                    "active": bool(pose_data.bActive),
                    "pose_valid": pose_valid,
                    "tracking_result": int(pose_data.pose.eTrackingResult),
                    "device_index": pose_index,
                    "matrix_3x4": (
                        _matrix34_rows(pose_data.pose.mDeviceToAbsoluteTracking)
                        if pose_valid
                        else None
                    ),
                }
            except Exception as error:
                values["aim_pose"] = empty_controller_input()["aim_pose"]
                values["aim_pose_error"] = f"{type(error).__name__}: {error}"
            result[hand] = values
        return result


__all__ = [
    "ANALOG_ACTIONS",
    "DIGITAL_ACTIONS",
    "ActionReader",
    "choose_primary_devices",
    "empty_controller_input",
    "legacy_controller_input",
    "pose_is_running_ok",
    "resolve_controller_input",
    "safe_property",
]
