from __future__ import annotations

from contextlib import AbstractContextManager
from pathlib import Path
import socket
import struct
import threading
import time

import numpy as np
import pytest

from twist2_vr.models import PoseSample, TrackerFrame
from twist2_vr.providers.hybrid import QuestViveHybridProvider
from twist2_vr.providers.vive_linux import (
    DEVICE_INFO,
    DEV_READY,
    HEADER,
    HELLO_ACK,
    MAX_PAYLOAD,
    MSG_EVT_POSE,
    MSG_EVT_STATUS,
    MSG_HELLO,
    MSG_HELLO_ACK,
    MSG_LIST_DEVICES,
    MSG_LIST_DEVICES_RESP,
    POSE_EVENT,
    PROTOCOL_VERSION,
    STATUS_EVENT,
    VUTMessage,
    ViveUltimateLinuxProvider,
    decode_device_state,
)


def packet(message_type, request_id, payload):
    return HEADER.pack(message_type, PROTOCOL_VERSION, 0, request_id, len(payload)) + payload


def fixed_string(value, size=32):
    encoded = value.encode()[: size - 1]
    return encoded + b"\0" * (size - len(encoded))


class FakeVUTServer(AbstractContextManager):
    def __init__(self, path: Path, *, state=DEV_READY, serial="VIVE-ABC", pose_payload=None):
        self.path = path
        self.state = state
        self.serial = serial
        self.pose_payload = pose_payload
        self.listener = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.listener.bind(str(path))
        self.listener.listen(1)
        self.stop = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def _serve(self):
        connection = None
        try:
            connection, _ = self.listener.accept()
            hello = connection.recv(65548)
            message_type, version, _flags, request_id, size = HEADER.unpack_from(hello)
            assert message_type == MSG_HELLO and version == PROTOCOL_VERSION
            assert len(hello) == HEADER.size + size
            ack = HELLO_ACK.pack(PROTOCOL_VERSION, 0, 77, time.monotonic_ns())
            connection.sendall(packet(MSG_HELLO_ACK, request_id, ack))

            listing = connection.recv(65548)
            message_type, _version, _flags, request_id, size = HEADER.unpack_from(listing)
            assert message_type == MSG_LIST_DEVICES and size == 0
            device = DEVICE_INFO.pack(1, self.state, fixed_string(self.serial))
            connection.sendall(packet(MSG_LIST_DEVICES_RESP, request_id, struct.pack("<I", 1) + device))

            payload = self.pose_payload
            if payload is None:
                payload = POSE_EVENT.pack(
                    1,
                    0,
                    time.monotonic_ns(),
                    1.0,
                    2.0,
                    3.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    0.1,
                    0.2,
                    0.3,
                    0.0,
                    0.0,
                    0.0,
                    fixed_string(self.serial),
                )
            connection.sendall(packet(MSG_EVT_POSE, 0, payload))
            self.stop.wait(2.0)
        except Exception as error:  # surfaced in __exit__
            self.error = error
        finally:
            if connection is not None:
                connection.close()

    def __exit__(self, *exc):
        self.stop.set()
        self.listener.close()
        self.thread.join(timeout=2.0)
        if self.error is not None and exc[0] is None:
            raise self.error


def read_until_pose(provider, timeout=1.0):
    deadline = time.monotonic() + timeout
    frame = provider.read()
    while not frame.poses and time.monotonic() < deadline:
        time.sleep(0.005)
        frame = provider.read()
    return frame


def test_vive_hub_ready_pose_uses_serial_mapping(tmp_path):
    socket_path = tmp_path / "vut.sock"
    with FakeVUTServer(socket_path):
        with ViveUltimateLinuxProvider(
            {"VIVE-ABC": "Pelvis"}, socket_path=socket_path
        ) as provider:
            frame = read_until_pose(provider)
            pose = frame.poses["Pelvis"]
            assert pose.valid and pose.device_id == "VIVE-ABC"
            assert pose.tracking_state == "ready"
            np.testing.assert_allclose(pose.position_m, [1, 2, 3])
            np.testing.assert_allclose(pose.orientation_xyzw, [0, 0, 0, 1])
            assert pose.metadata["device_id"] == 1


@pytest.mark.parametrize("state", [0, 1, 3, 4, 5, 6, 7, 8,
                                   0x10, 0x11, 0x13, 0x14, 0x15, 0x16, 0x17,
                                   0x18, 0x1A, 0x22, 0x32, 0x102])
def test_vive_hub_accepts_only_ready_state(tmp_path, state):
    socket_path = tmp_path / f"vut-{state}.sock"
    with FakeVUTServer(socket_path, state=state):
        with ViveUltimateLinuxProvider(
            {"VIVE-ABC": "Pelvis"}, socket_path=socket_path
        ) as provider:
            pose = read_until_pose(provider).poses["Pelvis"]
            assert not pose.valid


def test_still_ready_snapshot_preserves_raw_state_and_pose_flags(tmp_path):
    socket_path = tmp_path / "vut.sock"
    with FakeVUTServer(socket_path, state=0x12):
        with ViveUltimateLinuxProvider({"VIVE-ABC": "Pelvis"}, socket_path=socket_path) as provider:
            pose = read_until_pose(provider).poses["Pelvis"]
            assert pose.valid and pose.tracking_state == "ready"
            assert pose.metadata["raw_state"] == 0x12
            assert pose.metadata["state"] == 2
            assert pose.metadata["state_flags"] == 0x10
            assert pose.metadata["flags"] == 0
            row = provider.device_inventory()[0]
            assert row["ready"] and row["raw_state"] == 0x12
    assert decode_device_state(8) == (8, 0)
    assert decode_device_state(0x32) == (0x32, 0)


def test_vive_hub_zero_quaternion_fails_closed(tmp_path):
    socket_path = tmp_path / "vut.sock"
    malformed_pose = POSE_EVENT.pack(
        1, 0, time.monotonic_ns(),
        1, 2, 3,
        0, 0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        fixed_string("VIVE-ABC"),
    )
    with FakeVUTServer(socket_path, pose_payload=malformed_pose):
        with ViveUltimateLinuxProvider(
            {"VIVE-ABC": "Pelvis"}, socket_path=socket_path
        ) as provider:
            pose = read_until_pose(provider).poses["Pelvis"]
            assert not pose.valid and pose.tracking_state == "invalid_pose"


def test_vive_hub_rejects_truncated_seqpacket(tmp_path):
    socket_path = tmp_path / "vut.sock"
    with FakeVUTServer(socket_path, pose_payload=b"x" * (MAX_PAYLOAD + 1)):
        with ViveUltimateLinuxProvider(
            {"VIVE-ABC": "Pelvis"}, socket_path=socket_path
        ) as provider:
            frame = provider.read()
            assert frame.poses == {}
            assert not provider.client.connected


def pose_event(serial, *, device_id=1):
    return VUTMessage(
        MSG_EVT_POSE,
        0,
        POSE_EVENT.pack(
            device_id, 0, time.monotonic_ns(),
            1, 2, 3,
            0, 0, 0, 1,
            0, 0, 0,
            0, 0, 0,
            fixed_string(serial) if serial else bytes(32),
        ),
    )


def status_event(state, *, device_id=1):
    return VUTMessage(
        MSG_EVT_STATUS,
        0,
        STATUS_EVENT.pack(device_id, state, 0, 0, time.monotonic_ns()),
    )


class QueuedClient:
    socket_path = "/fake/vut.sock"

    def __init__(self, batches):
        self.batches = list(batches)
        self.connected = False

    def connect(self, _timeout):
        self.connected = True

    def list_devices(self, _timeout):
        return [(1, DEV_READY, "VIVE-OLD")]

    def receive_available(self):
        return self.batches.pop(0) if self.batches else []

    def close(self):
        self.connected = False


def test_slot_reconnect_never_reuses_previous_pose_or_serial():
    client = QueuedClient([
        [pose_event("VIVE-OLD")],
        [status_event(8), status_event(DEV_READY), pose_event("")],
        [pose_event("VIVE-NEW")],
    ])
    provider = ViveUltimateLinuxProvider({"VIVE-OLD": "Pelvis"}, client=client)
    assert provider.read().poses["Pelvis"].valid
    assert provider.read().poses == {}
    assert provider.read().poses == {}
    inventory = provider.device_inventory(poll=False)
    assert inventory[0]["serial"] == "VIVE-NEW"
    provider.close()


def test_ready_transition_requires_a_fresh_pose():
    client = QueuedClient([
        [pose_event("VIVE-OLD")],
        [status_event(8), status_event(DEV_READY)],
    ])
    provider = ViveUltimateLinuxProvider({"VIVE-OLD": "Pelvis"}, client=client)
    assert provider.read().poses["Pelvis"].valid
    assert provider.read().poses == {}
    provider.close()


def test_still_flag_transitions_do_not_drop_healthy_pose_but_loss_does():
    client = QueuedClient([
        [pose_event("VIVE-OLD")],
        [status_event(0x12)],
        [status_event(2)],
        [status_event(0x13), status_event(0x12)],
        [pose_event("VIVE-OLD")],
    ])
    with ViveUltimateLinuxProvider({"VIVE-OLD": "Pelvis"}, client=client) as provider:
        assert provider.read().poses["Pelvis"].valid
        assert provider.read().poses["Pelvis"].valid
        assert provider.read().poses["Pelvis"].valid
        assert provider.read().poses == {}
        assert provider.read().poses["Pelvis"].valid


def test_still_flag_does_not_revive_stale_pose():
    timestamp = time.monotonic_ns()
    now = [timestamp * 1e-9]
    message = VUTMessage(MSG_EVT_POSE, 0, POSE_EVENT.pack(
        1, 0, timestamp, 1, 2, 3, 0, 0, 0, 1,
        0, 0, 0, 0, 0, 0, fixed_string("VIVE-OLD")))
    client = QueuedClient([[message], [status_event(0x12)]])
    with ViveUltimateLinuxProvider({"VIVE-OLD": "Pelvis"}, client=client, clock=lambda: now[0]) as provider:
        assert provider.read().poses["Pelvis"].valid
        now[0] += 1
        pose = provider.read().poses["Pelvis"]
        assert not pose.valid and pose.tracking_state == "stale"


class StaticProvider:
    def __init__(self, frame):
        self.frame = frame
        self.closed = False

    def read(self):
        return self.frame

    def close(self):
        self.closed = True


def sample(role, position, timestamp):
    return PoseSample("device-" + role, role, position, [0, 0, 0, 1], timestamp)


def test_hybrid_provider_applies_cross_space_transform():
    now = time.monotonic()
    quest_frame = TrackerFrame(
        now,
        {"Head": sample("Head", [0, 1, 0], now)},
        "openvr",
        "openvr_standing",
    )
    vive_frame = TrackerFrame(
        now,
        {"Pelvis": sample("Pelvis", [1, 0, 0], now)},
        "vive",
        "vive_hub_world",
    )
    transform = np.eye(4)
    transform[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    transform[:3, 3] = [2, 3, 4]
    provider = QuestViveHybridProvider(
        StaticProvider(quest_frame),
        StaticProvider(vive_frame),
        {
            "calibrated": True,
            "calibration_id": "test",
            "source_frame": "vive_hub_world",
            "target_frame": "openvr_standing",
            "source_to_target_matrix": transform.tolist(),
        },
    )
    frame = provider.read()
    np.testing.assert_allclose(frame.poses["Pelvis"].position_m, [2, 4, 4])
    assert set(frame.poses) == {"Head", "Pelvis"}
    provider.close()


def test_hybrid_requires_calibration_and_rejects_role_collision():
    now = time.monotonic()
    empty = StaticProvider(TrackerFrame(now, {}, "quest", "openvr_standing"))
    vive = StaticProvider(TrackerFrame(now, {}, "vive", "vive_hub_world"))
    with pytest.raises(RuntimeError, match="not calibrated"):
        QuestViveHybridProvider(empty, vive, {"calibrated": False})

    head = sample("Head", [0, 0, 0], now)
    provider = QuestViveHybridProvider(
        StaticProvider(TrackerFrame(now, {"Head": head}, "quest", "openvr_standing")),
        StaticProvider(TrackerFrame(now, {"Head": head}, "vive", "vive_hub_world")),
        {
            "calibrated": True,
            "source_frame": "vive_hub_world",
            "target_frame": "openvr_standing",
            "source_to_target_matrix": np.eye(4).tolist(),
        },
    )
    with pytest.raises(RuntimeError, match="role collision"):
        provider.read()
