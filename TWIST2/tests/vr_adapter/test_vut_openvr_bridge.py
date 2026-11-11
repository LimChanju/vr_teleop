import struct
import numpy as np
import pytest
from twist2_vr.models import PoseSample
from twist2_vr.vut_openvr_bridge import PACKET, encode_packet


def sample(**changes):
    args = dict(device_id='FA61N3B00314', role='Right_Foot', position_m=[0.2, 1.3, -0.4],
                orientation_xyzw=[0.1, 0.2, 0.3, 0.9], timestamp_s=1.0,
                tracking_state='ready', metadata={'source_timestamp_ns': 1_000_000_000})
    args.update(changes)
    return PoseSample(**args)


def test_protocol_layout_and_raw_pose():
    pose = sample()
    data = encode_packet(pose.device_id, pose, connected=True, now_ns=1_010_000_000)
    assert len(data) == 120
    magic, version, flags, sent, stamp, serial, *values = PACKET.unpack(data)
    assert (magic, version, flags, sent, stamp) == (b'VUT1', 1, 3, 1_010_000_000, 1_000_000_000)
    assert serial.rstrip(b'\0').decode() == pose.device_id
    np.testing.assert_array_equal(values[:3], pose.position_m)
    np.testing.assert_array_equal(values[3:], pose.orientation_xyzw)


@pytest.mark.parametrize('state', ['lost_tracking', 'rotation_only', 'stale', 'unknown_9', 'reconnected'])
def test_non_ready_is_never_valid(state):
    p = sample(tracking_state=state)
    assert PACKET.unpack(encode_packet(p.device_id, p, connected=True, now_ns=1_010_000_000))[2] == 1


@pytest.mark.parametrize('now', [800_000_000, 1_210_000_000])
def test_future_or_stale_pose(now):
    p = sample()
    assert PACKET.unpack(encode_packet(p.device_id, p, connected=True, now_ns=now))[2] == 1


def test_offline_and_missing():
    for pose in (None, sample()):
        assert PACKET.unpack(encode_packet('FA61N3B00314', pose, connected=False, now_ns=1_010_000_000))[2] == 0


@pytest.mark.parametrize('serial', ['', 'x'*32, 'bad/serial', 'bad\0serial'])
def test_invalid_serial(serial):
    with pytest.raises(ValueError):
        encode_packet(serial, None, connected=False, now_ns=1)


def test_driver_registration_is_not_proof_of_loading(tmp_path):
    from twist2_vr.spacecal_session import loaded_runtime_drivers
    proc = tmp_path / '123'
    proc.mkdir()
    (proc / 'exe').symlink_to('/unused/vrserver')
    (proc / 'maps').write_text('/some/driver_alvr_server.so\n')
    assert loaded_runtime_drivers(tmp_path) == set()
    (proc / 'maps').write_text('/some/driver_twist2_vut.so\n/some/driver_01spacecalibrator.so\n')
    assert loaded_runtime_drivers(tmp_path) == {'twist2_vut', '01spacecalibrator'}


def test_packet_has_zero_alignment_padding():
    packet = encode_packet('FA61N3B00314', sample(), connected=True, now_ns=1_010_000_000)
    assert packet[12:16] == b'\0' * 4
    assert struct.unpack_from('<d', packet, 64)[0] == 0.2
