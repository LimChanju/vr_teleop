"""Linux VIVE Hub provider for HTC VIVE Ultimate Tracker.

This module implements the public VUT protocol documented by the official
VIVE Hub for Linux beta.  It talks only to the local ``tracker_server`` Unix
socket; USB pairing and tracking-map setup remain owned by VIVE Hub.

The protocol boundary is deliberately small and dependency-free so the
official VIVE binaries can stay unmodified under ``external/``.
"""

from __future__ import annotations

from dataclasses import dataclass
import os
import socket
import struct
import time
from pathlib import Path
from typing import Callable, Mapping

import numpy as np

from ..models import PoseSample, TrackerFrame
from .base import VRInputProvider


DEFAULT_VUT_SOCKET = Path("/tmp/vut.sock")
PROTOCOL_VERSION = 1
MAX_PAYLOAD = 64 * 1024

MSG_HELLO = 0x01
MSG_HELLO_ACK = 0x02
MSG_LIST_DEVICES = 0x10
MSG_LIST_DEVICES_RESP = 0x11
MSG_EVT_POSE = 0x30
MSG_EVT_STATUS = 0x31

SUB_POSE = 1 << 0
SUB_STATUS = 1 << 1
SUB_ALL = SUB_POSE | SUB_STATUS

DEV_NOT_CONNECTED = 0
DEV_LOADING = 1
DEV_READY = 2
DEV_LOST_TRACKING = 3
DEV_ROTATION_ONLY = 4
DEV_PREPARING = 5
DEV_SYNCING = 6
DEV_SETUP_REQUIRED = 7
DEV_OFF = 8


def decode_device_state(raw_state: int) -> tuple[int, int]:
    """Separate the known still-pose bit leaked by VIVE Hub Linux v1.0.3.

    HTC's ultimate_tracker.h defines PoseStable=0x10 and PoseStateMask=0x07.
    The running daemon logs 0x12 as Running_good plus flags=0x10, but the
    VUT state snapshot/events may expose the packed 0x12 instead of 2.
    Decode only this documented flag. Unknown bits and DEV_OFF=8 must not
    be masked into a usable state. PoseEvent.flags is a separate wire field.
    """
    raw_state = int(raw_state)
    if 0x10 <= raw_state <= 0x17:
        return raw_state & 0x07, 0x10
    return raw_state, 0

DEVICE_STATE_NAMES = {
    DEV_NOT_CONNECTED: "not_connected",
    DEV_LOADING: "loading",
    DEV_READY: "ready",
    DEV_LOST_TRACKING: "lost_tracking",
    DEV_ROTATION_ONLY: "rotation_only",
    DEV_PREPARING: "preparing",
    DEV_SYNCING: "syncing",
    DEV_SETUP_REQUIRED: "setup_required",
    DEV_OFF: "off",
}

HEADER = struct.Struct("<BBHII")
HELLO = struct.Struct("<HHI32s")
HELLO_ACK = struct.Struct("<HHIQ")
POSE_EVENT = struct.Struct("<IIQ3f4f3f3f32s")
STATUS_EVENT = struct.Struct("<IIIIQ")
DEVICE_INFO = struct.Struct("<II32s")


class VUTProtocolError(RuntimeError):
    """The local VIVE Hub daemon sent an incompatible or malformed packet."""


@dataclass(frozen=True)
class VUTMessage:
    message_type: int
    request_id: int
    payload: bytes


class VUTSocketClient:
    """Minimal client for VIVE Hub Linux's public ``/tmp/vut.sock`` API."""

    def __init__(
        self,
        socket_path: str | Path = DEFAULT_VUT_SOCKET,
        *,
        client_name: str = "twist2-vr",
    ) -> None:
        self.socket_path = str(socket_path)
        self.client_name = client_name
        self.sock: socket.socket | None = None
        self.server_time_ns: int | None = None
        self.session_id: int | None = None
        self._pending: list[VUTMessage] = []
        self._request_id = 2

    @property
    def connected(self) -> bool:
        return self.sock is not None

    @staticmethod
    def _packet(message_type: int, request_id: int, payload: bytes) -> bytes:
        return HEADER.pack(
            message_type,
            PROTOCOL_VERSION,
            0,
            request_id,
            len(payload),
        ) + payload

    @staticmethod
    def _send_packet(stream: socket.socket, packet: bytes) -> None:
        """Send exactly one ``SOCK_SEQPACKET`` record."""

        sent = stream.send(packet)
        if sent != len(packet):
            raise ConnectionError(f"short VUT send: {sent}/{len(packet)} bytes")

    @staticmethod
    def _receive_packet(stream: socket.socket) -> bytes:
        """Receive one record and reject kernel-level truncation."""

        packet, _ancillary, flags, _address = stream.recvmsg(HEADER.size + MAX_PAYLOAD)
        if flags & socket.MSG_TRUNC:
            raise VUTProtocolError("VUT packet exceeded the maximum supported size")
        return packet

    def connect(self, timeout_s: float = 2.0) -> None:
        if self.sock is not None:
            return
        # VUT uses SOCK_SEQPACKET: one send/recv is exactly one framed message.
        # It is not wire-compatible with the separate SDK's stream socket at
        # /tmp/vive_tracker.sock.
        stream = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        try:
            stream.set_inheritable(False)
            stream.settimeout(timeout_s)
            stream.connect(self.socket_path)
            if hasattr(socket, "SO_PEERCRED"):
                credentials = stream.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12)
                _pid, peer_uid, _gid = struct.unpack("3i", credentials)
                if peer_uid != os.getuid():
                    raise PermissionError(
                        f"VIVE Hub socket owner uid {peer_uid} does not match current uid {os.getuid()}"
                    )
            encoded_name = self.client_name.encode("utf-8")[:31]
            name = encoded_name + b"\0" * (32 - len(encoded_name))
            hello = HELLO.pack(PROTOCOL_VERSION, 0, SUB_ALL, name)
            self._send_packet(stream, self._packet(MSG_HELLO, 1, hello))

            packet = self._receive_packet(stream)
            if not packet:
                raise ConnectionError("VIVE Hub closed the socket during handshake")
            if len(packet) < HEADER.size:
                raise VUTProtocolError(f"truncated HELLO_ACK header: {len(packet)} bytes")
            message_type, version, _flags, request_id, payload_size = HEADER.unpack_from(packet)
            if payload_size > MAX_PAYLOAD:
                raise VUTProtocolError(f"HELLO_ACK payload is too large: {payload_size}")
            payload = packet[HEADER.size:]
            if len(payload) != payload_size:
                raise VUTProtocolError(
                    f"HELLO_ACK size mismatch: header={payload_size}, packet={len(payload)}"
                )
            if message_type != MSG_HELLO_ACK or request_id != 1:
                raise VUTProtocolError(
                    f"expected HELLO_ACK request 1, got type=0x{message_type:02x} request={request_id}"
                )
            if version != PROTOCOL_VERSION or len(payload) != HELLO_ACK.size:
                raise VUTProtocolError(
                    f"incompatible VUT handshake version={version} payload={len(payload)}"
                )
            server_version, _reserved, session_id, server_time_ns = HELLO_ACK.unpack(payload)
            if server_version != PROTOCOL_VERSION:
                raise VUTProtocolError(
                    f"VUT protocol mismatch: client={PROTOCOL_VERSION}, server={server_version}"
                )
            stream.setblocking(False)
            self.sock = stream
            self.session_id = int(session_id)
            self.server_time_ns = int(server_time_ns)
            self._pending.clear()
        except Exception:
            stream.close()
            raise

    @staticmethod
    def _decode_packet(packet: bytes) -> VUTMessage:
        if len(packet) < HEADER.size:
            raise VUTProtocolError(f"truncated VUT header: {len(packet)} bytes")
        message_type, version, _flags, request_id, payload_size = HEADER.unpack_from(packet)
        if version != PROTOCOL_VERSION:
            raise VUTProtocolError(f"unsupported VUT packet version: {version}")
        if payload_size > MAX_PAYLOAD:
            raise VUTProtocolError(f"VUT payload is too large: {payload_size}")
        payload = packet[HEADER.size:]
        if len(payload) != payload_size:
            raise VUTProtocolError(
                f"VUT size mismatch: header={payload_size}, packet={len(payload)}"
            )
        return VUTMessage(message_type, request_id, payload)

    def list_devices(self, timeout_s: float = 1.0) -> list[tuple[int, int, str]]:
        """Fetch the initial state snapshot without treating fallback names as serials."""

        if self.sock is None:
            raise ConnectionError("VIVE Hub socket is not connected")
        request_id = self._request_id
        self._request_id += 1
        self.sock.settimeout(timeout_s)
        try:
            self._send_packet(
                self.sock,
                self._packet(MSG_LIST_DEVICES, request_id, b""),
            )
            while True:
                packet = self._receive_packet(self.sock)
                if not packet:
                    raise ConnectionError("VIVE Hub closed the socket during LIST_DEVICES")
                message = self._decode_packet(packet)
                if (
                    message.message_type == MSG_LIST_DEVICES_RESP
                    and message.request_id == request_id
                ):
                    if len(message.payload) < 4:
                        raise VUTProtocolError("truncated LIST_DEVICES response")
                    count = struct.unpack_from("<I", message.payload)[0]
                    expected = 4 + count * DEVICE_INFO.size
                    if len(message.payload) != expected or count > 5:
                        raise VUTProtocolError(
                            f"invalid LIST_DEVICES response count={count} size={len(message.payload)}"
                        )
                    result = []
                    offset = 4
                    for _ in range(count):
                        device_id, state, raw_name = DEVICE_INFO.unpack_from(message.payload, offset)
                        offset += DEVICE_INFO.size
                        name = raw_name.split(b"\0", 1)[0].decode("utf-8", "replace").strip()
                        result.append((int(device_id), int(state), name))
                    return result
                self._pending.append(message)
        finally:
            if self.sock is not None:
                self.sock.setblocking(False)

    def receive_available(self) -> list[VUTMessage]:
        if self.sock is None:
            raise ConnectionError("VIVE Hub socket is not connected")
        messages, self._pending = self._pending, []
        while True:
            try:
                packet = self._receive_packet(self.sock)
            except BlockingIOError:
                break
            if not packet:
                self.close()
                raise ConnectionError("VIVE Hub closed the socket")
            try:
                messages.append(self._decode_packet(packet))
            except VUTProtocolError:
                self.close()
                raise
        return messages

    def close(self) -> None:
        stream, self.sock = self.sock, None
        self._pending.clear()
        if stream is not None:
            try:
                stream.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            stream.close()


@dataclass
class _LatestPose:
    device_id: int
    serial: str
    timestamp_ns: int
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    velocity: tuple[float, float, float]
    angular_velocity: tuple[float, float, float]
    flags: int
    received_s: float
    connection_generation: int


class ViveUltimateLinuxProvider(VRInputProvider):
    """Read serial-mapped Ultimate Trackers directly from Linux VIVE Hub.

    Only ``DEV_READY`` samples become valid poses.  Transitional, lost,
    rotation-only, setup-required, stale, or disconnected devices remain
    visible in inventory but fail closed in the teleoperation pipeline.
    """

    coordinate_frame = "vive_hub_world"

    def __init__(
        self,
        serial_to_role: Mapping[str, str] | None = None,
        *,
        socket_path: str | Path = DEFAULT_VUT_SOCKET,
        connect_timeout_s: float = 2.0,
        max_pose_age_s: float = 0.20,
        clock: Callable[[], float] = time.monotonic,
        client: VUTSocketClient | None = None,
    ) -> None:
        self.serial_to_role = dict(serial_to_role or {})
        roles = list(self.serial_to_role.values())
        if len(roles) != len(set(roles)):
            raise ValueError("VIVE mapping assigns more than one serial to the same role")
        if max_pose_age_s <= 0:
            raise ValueError("max_pose_age_s must be positive")
        self.max_pose_age_s = float(max_pose_age_s)
        self.clock = clock
        self.client = client or VUTSocketClient(socket_path)
        self.sequence = 0
        self._closed = False
        self._latest: dict[int, _LatestPose] = {}
        self._states: dict[int, int] = {}
        self._raw_states: dict[int, int] = {}
        self._errors: dict[int, int] = {}
        self._serials: dict[int, str] = {}
        self._display_names: dict[int, str] = {}
        self._generations: dict[int, int] = {}
        self._connection_up = False
        try:
            self.client.connect(connect_timeout_s)
            for device_id, state, name in self.client.list_devices(connect_timeout_s):
                self._validate_device_id(device_id)
                self._raw_states[device_id] = int(state)
                self._states[device_id], _ = decode_device_state(state)
                self._display_names[device_id] = name
            self._connection_up = True
        except (OSError, ConnectionError) as error:
            self.client.close()
            raise RuntimeError(
                f"cannot connect to VIVE Hub Linux socket {self.client.socket_path}; "
                "start VIVE Hub with run-vivehub.sh first"
            ) from error
        except Exception:
            self.client.close()
            raise

    @staticmethod
    def _decode_serial(raw: bytes) -> str:
        return raw.split(b"\0", 1)[0].decode("utf-8", "replace").strip()

    @staticmethod
    def _validate_device_id(device_id: int) -> None:
        if not 1 <= int(device_id) <= 5:
            raise VUTProtocolError(f"invalid VIVE tracker device_id: {device_id}")

    def _handle_pose(self, payload: bytes, received_s: float) -> None:
        if len(payload) != POSE_EVENT.size:
            raise VUTProtocolError(
                f"invalid VUT pose payload: expected {POSE_EVENT.size}, got {len(payload)}"
            )
        values = POSE_EVENT.unpack(payload)
        device_id = int(values[0])
        self._validate_device_id(device_id)
        # Never inherit identity from a dongle slot. HTC documents device_id as
        # a recyclable 1..5 slot and Pose.serial may legitimately be empty
        # while a newly connected tracker is being identified.
        serial = self._decode_serial(values[16])
        previous_serial = self._serials.get(device_id)
        if serial and previous_serial and serial != previous_serial:
            self._generations[device_id] = self._generations.get(device_id, 0) + 1
        if serial:
            self._serials[device_id] = serial
        self._latest[device_id] = _LatestPose(
            device_id=device_id,
            serial=serial,
            timestamp_ns=int(values[2]),
            position=tuple(float(v) for v in values[3:6]),
            orientation=tuple(float(v) for v in values[6:10]),
            velocity=tuple(float(v) for v in values[10:13]),
            angular_velocity=tuple(float(v) for v in values[13:16]),
            flags=int(values[1]),
            received_s=received_s,
            connection_generation=self._generations.get(device_id, 0),
        )

    def _handle_status(self, payload: bytes) -> None:
        if len(payload) != STATUS_EVENT.size:
            raise VUTProtocolError(
                f"invalid VUT status payload: expected {STATUS_EVENT.size}, got {len(payload)}"
            )
        device_id, state, error_code, _reserved, _timestamp_ns = STATUS_EVENT.unpack(payload)
        self._validate_device_id(int(device_id))
        device_id = int(device_id)
        raw_state = int(state)
        state, _ = decode_device_state(raw_state)
        previous = self._states.get(device_id)
        if previous in {DEV_NOT_CONNECTED, DEV_OFF} and state not in {DEV_NOT_CONNECTED, DEV_OFF}:
            self._generations[device_id] = self._generations.get(device_id, 0) + 1
        if state != DEV_READY:
            # A pose from an earlier tracking state must never become valid
            # again merely because the same slot later reports Ready.
            self._latest.pop(device_id, None)
        if state in {DEV_NOT_CONNECTED, DEV_OFF}:
            self._serials.pop(device_id, None)
        self._states[device_id] = state
        self._raw_states[device_id] = raw_state
        self._errors[device_id] = int(error_code)

    def _poll(self) -> None:
        received_s = self.clock()
        try:
            messages = self.client.receive_available()
            for message in messages:
                if message.message_type == MSG_EVT_POSE:
                    self._handle_pose(message.payload, received_s)
                elif message.message_type == MSG_EVT_STATUS:
                    self._handle_status(message.payload)
        except (ConnectionError, OSError, VUTProtocolError):
            self.client.close()
            if self._connection_up:
                for device_id in set(self._states) | set(self._latest):
                    self._states[device_id] = DEV_NOT_CONNECTED
                    self._raw_states[device_id] = DEV_NOT_CONNECTED
                    self._generations[device_id] = self._generations.get(device_id, 0) + 1
                self._connection_up = False
            return

    def _pose_sample(self, latest: _LatestPose, now_s: float, role: str) -> PoseSample:
        state = self._states.get(latest.device_id, DEV_NOT_CONNECTED)
        state_name = DEVICE_STATE_NAMES.get(state, f"unknown_{state}")
        timestamp_s = latest.timestamp_ns * 1e-9
        age_s = now_s - timestamp_s
        position = np.asarray(latest.position, dtype=np.float64)
        orientation = np.asarray(latest.orientation, dtype=np.float64)
        finite = bool(np.all(np.isfinite(position)) and np.all(np.isfinite(orientation)))
        quaternion_ok = finite and float(np.linalg.norm(orientation)) > 1e-8
        current_generation = self._generations.get(latest.device_id, 0)
        generation_ok = latest.connection_generation == current_generation
        valid = bool(
            self.client.connected
            and state == DEV_READY
            and quaternion_ok
            and generation_ok
            and -0.050 <= age_s <= self.max_pose_age_s
        )
        if not quaternion_ok:
            position = np.zeros(3, dtype=np.float64)
            orientation = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
            tracking_state = "invalid_pose"
        elif age_s < -0.050 or age_s > self.max_pose_age_s:
            tracking_state = "stale"
        elif not generation_ok:
            tracking_state = "reconnected"
        else:
            tracking_state = state_name
        return PoseSample(
            device_id=latest.serial,
            role=role,
            position_m=position,
            orientation_xyzw=orientation,
            timestamp_s=timestamp_s,
            valid=valid,
            tracking_state=tracking_state,
            metadata={
                "provider": "vive_hub_linux",
                "device_id": latest.device_id,
                "serial": latest.serial,
                "state": state,
                "raw_state": self._raw_states.get(latest.device_id, state),
                "state_flags": decode_device_state(self._raw_states.get(latest.device_id, state))[1],
                "state_name": state_name,
                "error_code": self._errors.get(latest.device_id, 0),
                "flags": latest.flags,
                "source_timestamp_ns": latest.timestamp_ns,
                "received_monotonic_s": latest.received_s,
                "pose_age_s": age_s,
                "velocity_m_s": list(latest.velocity),
                "angular_velocity_rad_s": list(latest.angular_velocity),
                "connection_generation": current_generation,
                "pose_connection_generation": latest.connection_generation,
            },
        )

    def read(self) -> TrackerFrame:
        if self._closed:
            raise RuntimeError("VIVE provider is closed")
        self._poll()
        now_s = self.clock()
        poses: dict[str, PoseSample] = {}
        for latest in self._latest.values():
            serial = latest.serial
            role = self.serial_to_role.get(serial)
            if not serial or role is None:
                continue
            if role in poses:
                raise RuntimeError(f"multiple VIVE trackers resolved to role {role!r}")
            poses[role] = self._pose_sample(latest, now_s, role)
        frame = TrackerFrame(
            timestamp_s=now_s,
            poses=poses,
            source="vive_hub_linux",
            coordinate_frame=self.coordinate_frame,
            sequence=self.sequence,
        )
        self.sequence += 1
        return frame

    def wait_until_ready(self, timeout_s: float = 20.0, poll_hz: float = 100.0) -> None:
        required = set(self.serial_to_role)
        if not required:
            return
        deadline = self.clock() + timeout_s
        latest_frame: TrackerFrame | None = None
        while self.clock() < deadline:
            latest_frame = self.read()
            valid_serials = {pose.device_id for pose in latest_frame.poses.values() if pose.valid}
            if required <= valid_serials:
                return
            time.sleep(1.0 / poll_hz)
        details = self.device_inventory(poll=False)
        raise RuntimeError(
            f"VIVE trackers did not all reach Ready within {timeout_s:.1f}s; inventory={details}"
        )

    def device_inventory(self, *, poll: bool = True) -> list[dict[str, object]]:
        if poll and not self._closed:
            self._poll()
        now_s = self.clock()
        rows = []
        for device_id in sorted(set(self._states) | set(self._latest)):
            latest = self._latest.get(device_id)
            serial = self._serials.get(device_id, latest.serial if latest else "")
            state = self._states.get(device_id, DEV_NOT_CONNECTED)
            timestamp_s = latest.timestamp_ns * 1e-9 if latest else None
            age_s = now_s - timestamp_s if timestamp_s is not None else None
            rows.append({
                "device_id": device_id,
                "serial": serial,
                "display_name": self._display_names.get(device_id, ""),
                "mapped_role": self.serial_to_role.get(serial),
                "state": state,
                "raw_state": self._raw_states.get(device_id, state),
                "state_flags": decode_device_state(self._raw_states.get(device_id, state))[1],
                "state_name": DEVICE_STATE_NAMES.get(state, f"unknown_{state}"),
                "ready": bool(state == DEV_READY and age_s is not None and age_s <= self.max_pose_age_s),
                "pose_age_s": age_s,
                "flags": latest.flags if latest else None,
                "error_code": self._errors.get(device_id, 0),
                "connection_generation": self._generations.get(device_id, 0),
            })
        return rows

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.client.close()


__all__ = [
    "DEFAULT_VUT_SOCKET",
    "DEVICE_STATE_NAMES",
    "DEV_READY",
    "HELLO",
    "HELLO_ACK",
    "HEADER",
    "MSG_EVT_POSE",
    "MSG_EVT_STATUS",
    "MSG_HELLO",
    "MSG_HELLO_ACK",
    "POSE_EVENT",
    "PROTOCOL_VERSION",
    "STATUS_EVENT",
    "SUB_ALL",
    "VUTMessage",
    "VUTProtocolError",
    "VUTSocketClient",
    "ViveUltimateLinuxProvider",
]
