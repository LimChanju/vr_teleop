#!/usr/bin/env python3
"""Read Quest/ALVR tracking and controller input from a running SteamVR session.

This is a diagnostic receiver only. It never sends commands to Unitree G1.
OpenVR standing-space coordinates are right-handed: +X right, +Y up, -Z forward.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any


ROOT = Path(__file__).resolve().parent
DEFAULT_MANIFEST = ROOT / "actions.json"
APPLICATION_MANIFEST = ROOT / "app.vrmanifest"
APPLICATION_KEY = "io.github.limchanju.vr_teleop.alvr_probe"

ANALOG_ACTIONS = {
    "trigger": "vector1",
    "grip": "vector1",
    "thumbstick": "vector2",
}
DIGITAL_ACTIONS = ("primary", "secondary", "thumbstick_click")


def matrix34_to_transform(matrix: Any) -> dict[str, list[float] | list[list[float]]]:
    """Convert OpenVR's row-major 3x4 device transform to JSON-safe values."""

    rows = [[float(matrix.m[row][column]) for column in range(4)] for row in range(3)]
    rotation = [row[:3] for row in rows]
    quaternion = rotation_matrix_to_quaternion(rotation)
    return {
        "position_m": [rows[0][3], rows[1][3], rows[2][3]],
        "orientation_xyzw": quaternion,
        "matrix_3x4": rows,
    }


def rotation_matrix_to_quaternion(rotation: list[list[float]]) -> list[float]:
    """Return a normalized [x, y, z, w] quaternion."""

    m00, m01, m02 = rotation[0]
    m10, m11, m12 = rotation[1]
    m20, m21, m22 = rotation[2]
    trace = m00 + m11 + m22
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2
        x, y, z, w = (m21 - m12) / scale, (m02 - m20) / scale, (m10 - m01) / scale, 0.25 * scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(max(0.0, 1.0 + m00 - m11 - m22)) * 2
        x, y, z, w = 0.25 * scale, (m01 + m10) / scale, (m02 + m20) / scale, (m21 - m12) / scale
    elif m11 > m22:
        scale = math.sqrt(max(0.0, 1.0 + m11 - m00 - m22)) * 2
        x, y, z, w = (m01 + m10) / scale, 0.25 * scale, (m12 + m21) / scale, (m02 - m20) / scale
    else:
        scale = math.sqrt(max(0.0, 1.0 + m22 - m00 - m11)) * 2
        x, y, z, w = (m02 + m20) / scale, (m12 + m21) / scale, 0.25 * scale, (m10 - m01) / scale
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-8 or not math.isfinite(norm):
        raise ValueError("OpenVR returned an invalid rotation matrix")
    return [x / norm, y / norm, z / norm, w / norm]


def vector3(vector: Any) -> list[float]:
    return [float(vector.v[index]) for index in range(3)]


def safe_property(vr: Any, system: Any, device_index: int, property_name: str) -> str | None:
    try:
        return system.getStringTrackedDeviceProperty(device_index, getattr(vr, property_name))
    except Exception:
        return None


def tracking_result_names(vr: Any) -> dict[int, str]:
    return {
        int(getattr(vr, name)): name.removeprefix("TrackingResult_")
        for name in dir(vr)
        if name.startswith("TrackingResult_") and isinstance(getattr(vr, name), int)
    }


def choose_device_indexes(vr: Any, system: Any, poses: Any) -> dict[str, int | None]:
    """Find the best connected HMD and controller for each role every frame."""

    head_candidates: list[tuple[int, int]] = []
    for index, pose in enumerate(poses):
        device_class = system.getTrackedDeviceClass(index)
        if device_class == vr.TrackedDeviceClass_HMD:
            score = 4 * bool(pose.bDeviceIsConnected) + 2 * bool(pose.bPoseIsValid)
            head_candidates.append((score, index))

    head_index = max(head_candidates, default=(-1, None))[1]
    head_tracking_system = (
        safe_property(vr, system, head_index, "Prop_TrackingSystemName_String")
        if head_index is not None
        else None
    )
    candidates: dict[str, list[tuple[int, int]]] = {"left": [], "right": []}
    for index, pose in enumerate(poses):
        device_class = system.getTrackedDeviceClass(index)
        if device_class != vr.TrackedDeviceClass_Controller:
            continue
        # Mixing poses from unrelated drivers can silently combine incompatible
        # tracking origins. Quest controllers must match the ALVR HMD driver.
        controller_tracking_system = safe_property(vr, system, index, "Prop_TrackingSystemName_String")
        if head_tracking_system and controller_tracking_system != head_tracking_system:
            continue
        role = system.getControllerRoleForTrackedDeviceIndex(index)
        side = None
        if role == vr.TrackedControllerRole_LeftHand:
            side = "left"
        elif role == vr.TrackedControllerRole_RightHand:
            side = "right"
        else:
            identity = " ".join(
                filter(
                    None,
                    (
                        safe_property(vr, system, index, "Prop_ModelNumber_String"),
                        safe_property(vr, system, index, "Prop_SerialNumber_String"),
                    ),
                )
            ).lower()
            if "left" in identity:
                side = "left"
            elif "right" in identity:
                side = "right"
        if side:
            score = 4 * bool(pose.bDeviceIsConnected) + 2 * bool(pose.bPoseIsValid) + int(role != 0)
            candidates[side].append((score, index))

    return {
        "head": head_index,
        "left": max(candidates["left"], default=(-1, None))[1],
        "right": max(candidates["right"], default=(-1, None))[1],
    }


class ActionReader:
    """SteamVR Input action reader with Quest Touch and hand-tracking bindings."""

    def __init__(self, vr: Any, manifest: Path):
        self.vr = vr
        self.input = vr.VRInput()
        self.input.setActionManifestPath(str(manifest.resolve()))
        action_set_handle = self.input.getActionSetHandle("/actions/teleop")
        self.active_sets = (vr.VRActiveActionSet_t * 1)()
        self.active_sets[0].ulActionSet = action_set_handle
        self.active_sets[0].ulRestrictedToDevice = vr.k_ulInvalidInputValueHandle
        self.handles: dict[str, dict[str, int]] = {}
        for hand in ("left", "right"):
            names = list(ANALOG_ACTIONS) + list(DIGITAL_ACTIONS)
            self.handles[hand] = {
                name: self.input.getActionHandle(f"/actions/teleop/in/{hand}_{name}")
                for name in names
            }

    def read(self) -> dict[str, dict[str, Any]]:
        self.input.updateActionState(self.active_sets)
        result: dict[str, dict[str, Any]] = {}
        for hand, handles in self.handles.items():
            available: dict[str, bool] = {}
            values: dict[str, Any] = {}
            device_indexes: set[int] = set()
            for name, kind in ANALOG_ACTIONS.items():
                data = self.input.getAnalogActionData(handles[name], self.vr.k_ulInvalidInputValueHandle)
                available[name] = bool(data.bActive)
                if data.bActive and data.activeOrigin:
                    info = self.input.getOriginTrackedDeviceInfo(data.activeOrigin)
                    if info.trackedDeviceIndex != self.vr.k_unTrackedDeviceIndexInvalid:
                        device_indexes.add(int(info.trackedDeviceIndex))
                if kind == "vector2":
                    values[name] = [float(data.x), float(data.y)] if data.bActive else [0.0, 0.0]
                else:
                    values[name] = float(data.x) if data.bActive else 0.0
            for name in DIGITAL_ACTIONS:
                data = self.input.getDigitalActionData(handles[name], self.vr.k_ulInvalidInputValueHandle)
                available[name] = bool(data.bActive)
                if data.bActive and data.activeOrigin:
                    info = self.input.getOriginTrackedDeviceInfo(data.activeOrigin)
                    if info.trackedDeviceIndex != self.vr.k_unTrackedDeviceIndexInvalid:
                        device_indexes.add(int(info.trackedDeviceIndex))
                values[name] = bool(data.bState) if data.bActive else False
            values.update(
                {
                    "active": any(available.values()) and len(device_indexes) == 1,
                    "available": available,
                    "source": "actions",
                    "device_index": next(iter(device_indexes)) if len(device_indexes) == 1 else None,
                }
            )
            result[hand] = values
        return result


def legacy_controller_input(vr: Any, system: Any, device_index: int | None) -> dict[str, Any]:
    """Fallback for runtimes that expose only the deprecated controller state."""

    if device_index is None:
        return empty_controller_input("none")
    try:
        valid, state = system.getControllerState(device_index)
    except Exception:
        valid = False
    if not valid:
        return empty_controller_input("none")

    def pressed(button: int) -> bool:
        return bool(int(state.ulButtonPressed) & (1 << button))

    return {
        "trigger": float(state.rAxis[1].x),
        "grip": float(state.rAxis[2].x),
        "thumbstick": [float(state.rAxis[0].x), float(state.rAxis[0].y)],
        "primary": pressed(vr.k_EButton_A),
        "secondary": pressed(vr.k_EButton_ApplicationMenu),
        "thumbstick_click": pressed(vr.k_EButton_Axis0),
        "active": True,
        "available": {name: True for name in (*ANALOG_ACTIONS, *DIGITAL_ACTIONS)},
        "source": "legacy",
        "device_index": device_index,
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
    }


class OpenVRProbe:
    def __init__(self, vr: Any, origin: int, origin_name: str, manifest: Path):
        self.vr = vr
        self.origin = origin
        self.origin_name = origin_name
        self.manifest = manifest
        self.system: Any = None
        self.applications: Any = None
        self.application_registered = False
        self.actions: ActionReader | None = None
        self.action_error: str | None = None
        self.tracking_names = tracking_result_names(vr)
        self.device_properties: dict[int, dict[str, str | None]] = {}

    def start(self) -> None:
        self.system = self.vr.init(self.vr.VRApplication_Background)
        # Python executables otherwise share a generated SteamVR app key. A
        # temporary, self-identified app keeps these bindings separate from
        # Isaac Sim and other Python OpenVR clients.
        self.applications = self.vr.VRApplications()
        self.applications.addApplicationManifest(str(APPLICATION_MANIFEST.resolve()), True)
        self.application_registered = True
        self.applications.identifyApplication(os.getpid(), APPLICATION_KEY)
        try:
            self.actions = ActionReader(self.vr, self.manifest)
        except Exception as error:
            self.action_error = f"{type(error).__name__}: {error}"

    def close(self) -> None:
        if self.system is not None:
            if self.application_registered:
                try:
                    self.applications.removeApplicationManifest(str(APPLICATION_MANIFEST.resolve()))
                except Exception:
                    pass
            self.vr.shutdown()
            self.system = None

    def _device(self, index: int | None, pose: Any | None) -> dict[str, Any]:
        if index is None or pose is None:
            return {
                "device_index": None,
                "connected": False,
                "pose_valid": False,
                "tracking_result": "Unavailable",
                "position_m": None,
                "orientation_xyzw": None,
                "matrix_3x4": None,
                "linear_velocity_m_s": None,
                "angular_velocity_rad_s": None,
                "model": None,
                "serial": None,
                "controller_type": None,
                "tracking_system": None,
            }
        valid = bool(pose.bPoseIsValid)
        transform = matrix34_to_transform(pose.mDeviceToAbsoluteTracking) if valid else {
            "position_m": None,
            "orientation_xyzw": None,
            "matrix_3x4": None,
        }
        if index not in self.device_properties:
            self.device_properties[index] = {
                "model": safe_property(self.vr, self.system, index, "Prop_ModelNumber_String"),
                "serial": safe_property(self.vr, self.system, index, "Prop_SerialNumber_String"),
                "controller_type": safe_property(self.vr, self.system, index, "Prop_ControllerType_String"),
                "tracking_system": safe_property(self.vr, self.system, index, "Prop_TrackingSystemName_String"),
            }
        return {
            "device_index": index,
            "connected": bool(pose.bDeviceIsConnected),
            "pose_valid": valid,
            "tracking_result": self.tracking_names.get(int(pose.eTrackingResult), str(int(pose.eTrackingResult))),
            **transform,
            "linear_velocity_m_s": vector3(pose.vVelocity) if valid else None,
            "angular_velocity_rad_s": vector3(pose.vAngularVelocity) if valid else None,
            **self.device_properties[index],
        }

    def sample(self) -> dict[str, Any]:
        poses = (self.vr.TrackedDevicePose_t * self.vr.k_unMaxTrackedDeviceCount)()
        self.system.getDeviceToAbsoluteTrackingPose(self.origin, 0.0, poses)
        indexes = choose_device_indexes(self.vr, self.system, poses)
        devices = {
            name: self._device(index, poses[index] if index is not None else None)
            for name, index in indexes.items()
        }
        action_inputs: dict[str, dict[str, Any]] = {}
        if self.actions is not None:
            try:
                action_inputs = self.actions.read()
                self.action_error = None
            except Exception as error:
                self.action_error = f"{type(error).__name__}: {error}"
        for hand in ("left", "right"):
            controller_input = action_inputs.get(hand, empty_controller_input())
            if controller_input["active"] and controller_input["device_index"] != indexes[hand]:
                controller_input = empty_controller_input("mismatched_action_device")
            if not controller_input["active"]:
                legacy_input = legacy_controller_input(self.vr, self.system, indexes[hand])
                if legacy_input["active"] or controller_input["source"] == "none":
                    controller_input = legacy_input
            devices[hand]["input"] = controller_input

        valid_channels = sum(bool(devices[name]["pose_valid"]) for name in ("head", "left", "right"))
        tracking_status = "RECEIVING" if valid_channels == 3 else ("PARTIAL" if valid_channels else "WAITING")
        active_inputs = sum(bool(devices[hand]["input"]["active"]) for hand in ("left", "right"))
        input_status = "RECEIVING" if active_inputs == 2 else ("PARTIAL" if active_inputs else "WAITING")
        ready = tracking_status == "RECEIVING" and input_status == "RECEIVING"
        status = "READY" if ready else ("TRACKING_ONLY" if tracking_status == "RECEIVING" else tracking_status)
        return {
            "status": status,
            "tracking_status": tracking_status,
            "input_status": input_status,
            "ready_for_teleop": ready,
            "utc": datetime.now(timezone.utc).isoformat(),
            "frame": f"OpenVR {self.origin_name}: +X right, +Y up, -Z forward",
            "devices": devices,
            "action_error": self.action_error,
        }


def compact_device(name: str, device: dict[str, Any]) -> str:
    if not device["pose_valid"]:
        return f"{name}=--"
    x, y, z = device["position_m"]
    text = f"{name}=({x:+.2f},{y:+.2f},{z:+.2f})"
    if "input" in device:
        value = device["input"]
        sx, sy = value["thumbstick"]
        text += f" t={value['trigger']:.2f} g={value['grip']:.2f} s=({sx:+.2f},{sy:+.2f})"
        text += f" p={int(value['primary'])} q={int(value['secondary'])} [{value['source']}]"
    return text


def compact_snapshot(snapshot: dict[str, Any]) -> str:
    devices = snapshot["devices"]
    return " ".join(
        [
            f"[{snapshot['status']}]",
            compact_device("HMD", devices["head"]),
            compact_device("L", devices["left"]),
            compact_device("R", devices["right"]),
        ]
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate", type=float, default=10.0, help="samples per second (default: 10)")
    parser.add_argument("--duration", type=float, default=0.0, help="stop after N seconds; 0 runs until Ctrl+C")
    parser.add_argument("--format", choices=("compact", "jsonl"), default="compact")
    parser.add_argument("--output", type=Path, help="also append full samples to this JSONL file")
    parser.add_argument("--origin", choices=("standing", "seated", "raw"), default="standing")
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if not math.isfinite(args.rate) or not 0.1 <= args.rate <= 250:
        parser.error("--rate must be between 0.1 and 250")
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("--duration must be finite and non-negative")
    if not args.manifest.is_file():
        parser.error(f"action manifest not found: {args.manifest}")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        import openvr
    except ImportError:
        print("openvr is not installed. Run: bash scripts/setup.sh", file=sys.stderr)
        return 2

    origins = {
        "standing": openvr.TrackingUniverseStanding,
        "seated": openvr.TrackingUniverseSeated,
        "raw": openvr.TrackingUniverseRawAndUncalibrated,
    }
    probe = OpenVRProbe(openvr, origins[args.origin], args.origin, args.manifest)
    output_file = None
    input_warning_printed = False
    try:
        probe.start()
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            output_file = args.output.open("a", encoding="utf-8")
        print("OpenVR connected. This probe does not send robot commands. Ctrl+C to stop.", file=sys.stderr)
        interval = 1.0 / args.rate
        started = time.monotonic()
        deadline = started
        while not args.duration or time.monotonic() - started < args.duration:
            snapshot = probe.sample()
            if (
                not input_warning_printed
                and snapshot["tracking_status"] == "RECEIVING"
                and snapshot["input_status"] != "RECEIVING"
                and time.monotonic() - started >= 2.0
            ):
                print(
                    "Controller poses are visible, but button actions are inactive. "
                    "Wake both Touch controllers and disable Quest hand tracking for this test.",
                    file=sys.stderr,
                )
                input_warning_printed = True
            encoded = json.dumps(snapshot, separators=(",", ":"), allow_nan=False)
            print(encoded if args.format == "jsonl" else compact_snapshot(snapshot), flush=True)
            if output_file:
                output_file.write(encoded + "\n")
                output_file.flush()
            deadline += interval
            time.sleep(max(0.0, deadline - time.monotonic()))
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"ALVR/OpenVR probe failed: {type(error).__name__}: {error}", file=sys.stderr)
        print("Check that SteamVR is running and ALVR shows the headset as connected.", file=sys.stderr)
        return 2
    finally:
        if output_file:
            output_file.close()
        probe.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
