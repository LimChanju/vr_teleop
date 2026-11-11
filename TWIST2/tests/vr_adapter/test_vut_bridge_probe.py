import errno
import fcntl
import json
import math
import signal
import socket
import struct

import pytest

from twist2_vr import vut_bridge_probe as probe


class FakeClock:
    def __init__(self):
        self.value = 1000.0

    def __call__(self):
        return self.value

    def ns(self):
        return round(self.value * 1e9)

    def sleep(self, duration):
        self.value += duration


class FakeSocket:
    def __init__(self, on_send=None):
        self.packets = []
        self.on_send = on_send
        self.blocking = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        return False

    def setblocking(self, blocking):
        self.blocking = blocking

    def sendto(self, data, address):
        if self.on_send:
            self.on_send(data)
        self.packets.append((data, address))
        return len(data)


def run_fake(tmp_path, sender, **options):
    clock = FakeClock()

    def factory(family, kind):
        assert (family, kind) == (socket.AF_UNIX, socket.SOCK_DGRAM)
        return sender

    return probe.run_probe(
        duration=0.1, lock_path=tmp_path / "bridge.lock", socket_factory=factory,
        clock=clock, clock_ns=clock.ns, sleeper=clock.sleep, **options,
    )


@pytest.mark.parametrize("mode,flags", [("invalid", 1), ("static", 3)])
def test_protocol_layout_flags_and_distinct_static_positions(mode, flags):
    positions = []
    for index in range(1, 6):
        data = probe.encode_probe_packet(index, mode, 123456789)
        assert len(data) == 120
        assert data[12:16] == b"\0" * 4
        fields = struct.unpack("<4sII4xQQ32s3d4d", data)
        assert fields[:5] == (b"VUT1", 1, flags, 123456789, 123456789)
        assert fields[5].rstrip(b"\0") == f"PROBE_VUT_{index:02d}".encode()
        assert all(math.isfinite(value) for value in fields[6:])
        assert sum(value * value for value in fields[9:]) == pytest.approx(1)
        assert fields[9:] != (0, 0, 0, 1)
        positions.append(fields[6:9])
    assert len(set(positions)) == 5


def test_sends_only_fake_socket_and_disconnects_all(tmp_path, capsys):
    sender = FakeSocket()
    assert run_fake(tmp_path, sender, mode="static", count=5) == 0
    assert sender.blocking is False
    decoded = [probe.PACKET.unpack(data) for data, _ in sender.packets]
    assert all(address == "\0twist2_vut_" + str(probe.os.getuid()) for _, address in sender.packets)
    assert all(fields[2] == 3 for fields in decoded[:-5])
    assert [fields[2] for fields in decoded[-5:]] == [0] * 5
    assert all(a[3] < b[3] for a, b in zip(decoded, decoded[1:]))
    assert all(fields[3] == fields[4] for fields in decoded)
    summary = json.loads(capsys.readouterr().out.splitlines()[-1])
    assert summary["disconnects_sent"] == 5
    assert summary["elapsed_s"] == pytest.approx(0.1)
    assert summary["no_robot_output"] is True


def test_real_bridge_lock_refuses_before_socket_creation(tmp_path):
    lock_path = tmp_path / "bridge.lock"
    with lock_path.open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(RuntimeError, match="bridge.lock"):
            probe.run_probe(lock_path=lock_path, socket_factory=lambda *_: pytest.fail("socket created"))


def test_sigterm_disconnects_and_restores_handlers(tmp_path, monkeypatch, capsys):
    handlers = {signal.SIGINT: object(), signal.SIGTERM: object()}
    original = handlers.copy()

    def install(signum, handler):
        previous = handlers[signum]
        handlers[signum] = handler
        return previous

    monkeypatch.setattr(probe.signal, "signal", install)

    def terminate(data):
        if probe.PACKET.unpack(data)[2]:
            handlers[signal.SIGTERM](signal.SIGTERM, None)

    sender = FakeSocket(terminate)
    assert run_fake(tmp_path, sender, count=2) == 0
    assert [probe.PACKET.unpack(data)[2] for data, _ in sender.packets] == [1, 0, 0]
    assert handlers == original
    assert json.loads(capsys.readouterr().out.splitlines()[-1])["signal"] == signal.SIGTERM


def test_send_failure_still_disconnects(tmp_path, capsys):
    def fail_active(data):
        if probe.PACKET.unpack(data)[2]:
            raise OSError(errno.EIO, "fake failure")

    sender = FakeSocket(fail_active)
    assert run_fake(tmp_path, sender, count=2) == 1
    assert [probe.PACKET.unpack(data)[2] for data, _ in sender.packets] == [0, 0]
    summary = json.loads(capsys.readouterr().out.splitlines()[-1])
    assert summary["send_errors"] == 1
    assert "fake failure" in summary["error"]


def test_absent_driver_times_out_with_failure_summary(tmp_path, capsys):
    def no_driver(_data):
        raise OSError(errno.ENOENT, "fake absent driver")

    sender = FakeSocket(no_driver)
    assert run_fake(tmp_path, sender, count=2) == 1
    summary = json.loads(capsys.readouterr().out.splitlines()[-1])
    assert summary["packets_sent"] == 0
    assert summary["send_errors"] > 0
    assert summary["disconnect_errors"] == 2
    assert summary["elapsed_s"] == pytest.approx(0.1)


@pytest.mark.parametrize("options", [
    {"count": 0}, {"count": 6}, {"duration": 0}, {"duration": 31},
    {"duration": float("nan")}, {"duration": float("inf")},
    {"frequency": 9}, {"frequency": 121}, {"frequency": float("nan")},
    {"mode": "raw"},
])
def test_invalid_options_rejected_without_socket(tmp_path, options):
    with pytest.raises(ValueError):
        probe.run_probe(lock_path=tmp_path / "bridge.lock",
                        socket_factory=lambda *_: pytest.fail("socket created"), **options)
    assert not (tmp_path / "bridge.lock").exists()


def test_cli_defaults_never_run_real_sender(monkeypatch):
    calls = []
    monkeypatch.setattr(probe, "run_probe", lambda *args: calls.append(args) or 0)
    assert probe.main([]) == 0
    assert calls == [("invalid", 1, 10.0, 30.0)]
