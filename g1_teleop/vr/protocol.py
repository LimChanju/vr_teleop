"""Bounded, versioned UDP targets. No robot SDK or hardware command path.

Positions are metres in a pelvis-yaw frame: +X forward, +Y left, +Z up.
The origin uses pelvis world XY and ground + nominal standing pelvis height Z;
it follows yaw, not measured pelvis roll/pitch or the current crouched height.
Ordered landmarks: head, left end effector, right end effector. A local
monotonic receive timer controls freshness; a wall clock age check also
rejects queued packets. Across hosts keep clocks synchronized.
"""

from __future__ import annotations

import dataclasses
import json
import math
import socket
import time
import uuid
from collections import deque

import numpy as np

SCHEMA = "g1.vr.target.v1"
FRAME = "robot_yaw_nominal_height"
COORDINATE_VERSION = "g1-yaw-floor-relative-v1"
MAX_PACKET_BYTES = 8192
DEFAULT_PORT = 8765
LANDMARKS = ("head", "left", "right")
# Override with the exact reference positions in the trained policy metadata.
DEFAULT_NOMINAL = np.array([[0.0, 0.0, 0.55], [0.25, 0.25, 0.25], [0.25, -0.25, 0.25]])


@dataclasses.dataclass
class TargetFrame:
    positions: np.ndarray = dataclasses.field(default_factory=lambda: DEFAULT_NOMINAL.copy())
    velocity: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros(3))
    enabled: bool = False
    reset: bool = False
    fresh: bool = False
    tracking_valid: bool = False
    calibrated: bool = False
    source: str = "synthetic"
    session: str = ""
    seq: int = -1
    reset_id: int = 0
    calibration_id: int = 0
    timestamp_unix_ns: int = 0
    age_s: float = math.inf


def _integer(value, name, low=0, high=2**63 - 1):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"invalid {name}")
    return value


def _boolean(value, name):
    if type(value) is not bool:
        raise ValueError(f"invalid {name}")
    return value


def _array(value, shape, limit, name):
    # np.asarray alone would silently accept strings and booleans as numbers.
    if not isinstance(value, list):
        raise ValueError(f"invalid {name}")
    def is_number_tree(v):
        return all(is_number_tree(x) for x in v) if isinstance(v, list) else type(v) in (int, float)
    if not is_number_tree(value):
        raise ValueError(f"non-numeric {name}")
    try:
        result = np.asarray(value, dtype=np.float64)
    except (ValueError, TypeError, OverflowError) as exc:
        raise ValueError(f"invalid {name}") from exc
    if result.shape != shape or not np.isfinite(result).all() or np.any(np.abs(result) > limit):
        raise ValueError(f"invalid {name}")
    return result


def encode_target(frame: TargetFrame) -> bytes:
    packet = {
        "schema": SCHEMA, "frame": FRAME, "coordinate_version": COORDINATE_VERSION,
        "landmarks": list(LANDMARKS),
        "session": frame.session, "seq": frame.seq, "timestamp_unix_ns": frame.timestamp_unix_ns,
        "source": frame.source, "target_positions_m": np.asarray(frame.positions).tolist(),
        "command_velocity": np.asarray(frame.velocity).tolist(), "enabled": frame.enabled,
        "reset_id": frame.reset_id, "tracking_valid": frame.tracking_valid,
        "calibrated": frame.calibrated, "calibration_id": frame.calibration_id,
    }
    data = json.dumps(packet, separators=(",", ":"), allow_nan=False).encode("utf-8")
    decode_target(data)  # same validation for producers and consumers
    return data


def decode_target(data: bytes, *, now_unix_ns=None, max_age_s=2.0) -> TargetFrame:
    if len(data) > MAX_PACKET_BYTES:
        raise ValueError("oversize target packet")
    try:
        packet = json.loads(data)
    except (ValueError, UnicodeDecodeError, TypeError, RecursionError) as exc:
        raise ValueError("malformed target JSON") from exc
    if not isinstance(packet, dict) or packet.get("schema") != SCHEMA:
        raise ValueError("unsupported target schema")
    if (packet.get("frame") != FRAME or packet.get("coordinate_version") != COORDINATE_VERSION
            or packet.get("landmarks") != list(LANDMARKS)):
        raise ValueError("target frame/order mismatch")
    try:
        session = packet["session"]
        if not isinstance(session, str) or str(uuid.UUID(session)) != session:
            raise ValueError("invalid session UUID")
        source = packet["source"]
        if source not in ("openvr", "synthetic", "replay"):
            raise ValueError("invalid source")
        timestamp = _integer(packet["timestamp_unix_ns"], "timestamp_unix_ns", 1)
        if now_unix_ns is not None and abs(now_unix_ns - timestamp) > max_age_s * 1e9:
            raise ValueError("packet timestamp is too old or clock is unsynchronized")
        frame = TargetFrame(
            positions=_array(packet["target_positions_m"], (3, 3), 2.0, "target_positions_m"),
            velocity=_array(packet["command_velocity"], (3,), np.array([1.5, 1.0, 2.0]), "command_velocity"),
            enabled=_boolean(packet["enabled"], "enabled"),
            tracking_valid=_boolean(packet["tracking_valid"], "tracking_valid"),
            calibrated=_boolean(packet["calibrated"], "calibrated"),
            reset_id=_integer(packet["reset_id"], "reset_id"),
            # Added without changing v1 wire schema: historic packets have no
            # calibration generation and keep their previous behavior.
            calibration_id=_integer(packet.get("calibration_id", 0), "calibration_id"),
            seq=_integer(packet["seq"], "seq"), timestamp_unix_ns=timestamp,
            session=session, source=source,
        )
    except KeyError as exc:
        raise ValueError(f"missing field: {exc.args[0]}") from exc
    if frame.enabled and (not frame.tracking_valid or not frame.calibrated):
        raise ValueError("enabled target lacks tracking or calibration")
    return frame


class UDPReceiver:
    """Nonblocking single-source receiver; callers must honor enabled AND fresh.

    New sessions may take over only after the previous stream is stale. Old
    session IDs, duplicate/reordered packets and repeated reset IDs are ignored.
    This is local simulation IPC, not an authenticated Internet protocol.
    """

    def __init__(self, host="127.0.0.1", port=DEFAULT_PORT, timeout_s=0.25, nominal=None):
        if not 0.02 <= timeout_s <= 2.0:
            raise ValueError("timeout_s must be between 0.02 and 2 seconds")
        self.timeout_s = float(timeout_s)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind((host, port))
            self.socket.setblocking(False)
        except BaseException:
            self.socket.close()
            raise
        self.address = self.socket.getsockname()
        self._frame = TargetFrame()
        if nominal is not None:
            self._frame.positions = _array(np.asarray(nominal).tolist(), (3, 3), 2.0, "nominal")
        self._received_at = -math.inf
        self._reset_id = 0
        self._calibration_id = 0
        self._retired_sessions = deque(maxlen=32)
        self._peer = None
        self.accepted = 0
        self.rejected = 0

    def poll(self) -> TargetFrame:
        reset = False
        # Bound work per simulation step even if another process floods the port.
        for _ in range(256):
            try:
                data, peer = self.socket.recvfrom(MAX_PACKET_BYTES + 1)
            except BlockingIOError:
                break
            now = time.monotonic()
            try:
                frame = decode_target(data, now_unix_ns=time.time_ns(), max_age_s=self.timeout_s)
                changed_session = frame.session != self._frame.session
                if changed_session:
                    if frame.session in self._retired_sessions:
                        raise ValueError("retired session")
                    if self._frame.session and now - self._received_at <= self.timeout_s:
                        raise ValueError("another session is still active")
                elif (frame.seq <= self._frame.seq or peer != self._peer or frame.reset_id < self._reset_id
                      or frame.calibration_id < self._calibration_id):
                    raise ValueError("reordered packet or unexpected peer/reset/calibration")
                if changed_session:
                    if self._frame.session:
                        self._retired_sessions.append(self._frame.session)
                    self._reset_id = 0
                    self._calibration_id = 0
                if frame.reset_id > self._reset_id:
                    reset = True
                    self._reset_id = frame.reset_id
                self._calibration_id = frame.calibration_id
                self._frame, self._received_at, self._peer = frame, now, peer
                self.accepted += 1
            except (ValueError, TypeError):
                self.rejected += 1
        age = time.monotonic() - self._received_at
        fresh = age <= self.timeout_s and self._frame.tracking_valid
        enabled = fresh and self._frame.enabled and self._frame.calibrated
        return dataclasses.replace(
            self._frame, positions=self._frame.positions.copy(),
            velocity=self._frame.velocity.copy() if enabled else np.zeros(3),
            enabled=bool(enabled), fresh=bool(fresh), reset=bool(reset and age <= self.timeout_s), age_s=age,
        )

    def close(self):
        self.socket.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
