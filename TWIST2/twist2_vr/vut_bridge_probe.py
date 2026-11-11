"""Short-lived synthetic VUT protocol probe. No HTC SDK or robot output.

Use only for isolated SteamVR diagnosis. Probe devices remain registered until
SteamVR exits, although this sender disconnects them before returning.
"""
from __future__ import annotations

import argparse
import errno
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import time


ROOT = Path(__file__).resolve().parents[1]
LOCK_PATH = ROOT / "runtime/spacecal/bridge.lock"
# Matches tools/vut_openvr/protocol.h without importing the production feeder.
PACKET = struct.Struct("<4sII4xQQ32s3d4d")
MODES = ("invalid", "static")
RETRYABLE = {errno.ENOENT, errno.ECONNREFUSED, errno.EAGAIN, errno.ENOBUFS}


def validate(mode, count, duration, frequency):
    if mode not in MODES:
        raise ValueError("mode must be invalid or static")
    if not 1 <= count <= 5:
        raise ValueError("count must be 1..5")
    if not math.isfinite(duration) or not 0 < duration <= 30:
        raise ValueError("duration must be greater than 0 and at most 30 seconds")
    if not math.isfinite(frequency) or not 10 <= frequency <= 120:
        raise ValueError("frequency must be 10..120 Hz (avoid the 200 ms watchdog)")


def encode_probe_packet(index, mode, now_ns, *, connected=True):
    if not 1 <= index <= 5 or mode not in MODES or now_ns <= 0:
        raise ValueError("invalid probe packet parameters")
    flags = (1 if mode == "invalid" else 3) if connected else 0
    # Fixed, distinct, nonzero positions and a unit 60-degree yaw quaternion.
    position = (index * 0.25, 1.0 + index * 0.05, -0.5)
    orientation = (0.0, 0.5, 0.0, math.sqrt(3.0) / 2.0)
    return PACKET.pack(
        b"VUT1", 1, flags, now_ns, now_ns,
        f"PROBE_VUT_{index:02d}".encode(), *position, *orientation,
    )


def run_probe(mode="invalid", count=1, duration=10.0, frequency=30.0, *,
              lock_path=LOCK_PATH, socket_factory=socket.socket,
              clock=time.monotonic, clock_ns=time.monotonic_ns, sleeper=time.sleep):
    validate(mode, count, duration, frequency)
    lock_path = Path(lock_path)
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("a real bridge or another probe already holds bridge.lock") from exc

        address = "\0twist2_vut_" + str(os.getuid())
        summary = {
            "event": "probe_summary", "mode": mode, "count": count,
            "duration_s": duration, "frequency_hz": frequency,
            "packets_sent": 0, "send_errors": 0,
            "disconnects_sent": 0, "disconnect_errors": 0,
            "signal": None, "error": None, "no_robot_output": True,
        }
        start = clock()
        last_stamp = 0

        def timestamp():
            nonlocal last_stamp
            last_stamp = max(clock_ns(), last_stamp + 1)
            return last_stamp

        def stopping(signum, _frame):
            summary["signal"] = signum

        previous = {s: signal.signal(s, stopping) for s in (signal.SIGINT, signal.SIGTERM)}
        print(json.dumps({
            "event": "probe_start", "mode": mode, "count": count,
            "frequency_hz": frequency, "duration_s": duration,
            "socket": address, "no_robot_output": True,
            "notice": "Synthetic poses only; devices stay registered until SteamVR exits.",
        }), flush=True)
        try:
            with socket_factory(socket.AF_UNIX, socket.SOCK_DGRAM) as sender:
                sender.setblocking(False)
                try:
                    while summary["signal"] is None and clock() - start < duration:
                        for index in range(1, count + 1):
                            if summary["signal"] is not None:
                                break
                            try:
                                sender.sendto(encode_probe_packet(index, mode, timestamp()), address)
                                summary["packets_sent"] += 1
                            except OSError as exc:
                                summary["send_errors"] += 1
                                if exc.errno not in RETRYABLE:
                                    raise
                        if summary["signal"] is None:
                            sleeper(min(1.0 / frequency, max(0.0, duration - (clock() - start))))
                finally:
                    for index in range(1, count + 1):
                        try:
                            sender.sendto(encode_probe_packet(index, mode, timestamp(), connected=False), address)
                            summary["disconnects_sent"] += 1
                        except OSError:
                            summary["disconnect_errors"] += 1
        except OSError as exc:
            summary["error"] = str(exc)
        finally:
            for signum, handler in previous.items():
                signal.signal(signum, handler)
        summary["elapsed_s"] = round(clock() - start, 6)
        summary["exit_code"] = 1 if summary["error"] or not summary["packets_sent"] else 0
        print(json.dumps(summary), flush=True)
        return summary["exit_code"]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=MODES, default="invalid")
    parser.add_argument("--count", type=int, default=1)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--frequency", type=float, default=30.0)
    args = parser.parse_args(argv)
    try:
        return run_probe(args.mode, args.count, args.duration, args.frequency)
    except (ValueError, RuntimeError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    raise SystemExit(main())
