"""Forward raw VUT poses to the separate SteamVR driver, never to a robot.

No Quest/VIVE calibration is loaded here. xi-ve applies the transform inside
SteamVR, and consumers must then read OpenVR rather than the hybrid provider.
"""
from __future__ import annotations

import argparse
import errno
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import socket
import struct
import time

from .providers.vive_linux import ViveUltimateLinuxProvider

ROOT = Path(__file__).resolve().parents[1]
PACKET = struct.Struct("<4sII4xQQ32s3d4d")


def encode_packet(serial, pose, *, connected, now_ns):
    if not re.fullmatch(r"[A-Za-z0-9_-]{1,31}", serial):
        raise ValueError("invalid bridge serial")
    valid = bool(connected and pose is not None and pose.valid and pose.tracking_state == "ready")
    stamp = int(pose.metadata.get("source_timestamp_ns", round(pose.timestamp_s * 1e9))) if pose else 0
    valid = valid and stamp > 0 and -0.01 <= (now_ns - stamp) * 1e-9 <= 0.20
    position = pose.position_m if pose else (0, 0, 0)
    orientation = pose.orientation_xyzw if pose else (0, 0, 0, 1)
    return PACKET.pack(b"VUT1", 1, int(connected) | (int(valid) << 1), now_ns, stamp,
                       serial.encode(), *position, *orientation)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mapping", type=Path, default=ROOT / "config/local/tracker_mapping.vive_linux.json")
    parser.add_argument("--duration", type=float, default=0, help="0: run until Ctrl+C")
    parser.add_argument("--frequency", type=float, default=120)
    args = parser.parse_args(argv)
    if args.duration < 0 or not 1 <= args.frequency <= 240:
        parser.error("duration must be >=0 and frequency must be 1..240 Hz")
    mapping = json.loads(args.mapping.read_text())["serial_to_role"]
    if not 1 <= len(mapping) <= 5 or len(set(mapping.values())) != len(mapping):
        parser.error("mapping must have 1..5 unique roles")
    for serial in mapping:
        encode_packet(serial, None, connected=False, now_ns=1)
    runtime = ROOT / "runtime/spacecal"
    runtime.mkdir(parents=True, exist_ok=True)
    with (runtime / "bridge.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error("a bridge feeder is already running")
        address = "\0twist2_vut_" + str(os.getuid())
        stop = False
        def stopping(_signum, _frame):
            nonlocal stop
            stop = True
        previous = {s: signal.signal(s, stopping) for s in (signal.SIGINT, signal.SIGTERM)}
        provider = None
        last_retry = last_log = 0.0
        start = time.monotonic()
        print("VUT → SteamVR bridge: RAW poses, no spatial transform, no robot output.", flush=True)
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sender:
            sender.setblocking(False)
            try:
                while not stop and (not args.duration or time.monotonic() - start < args.duration):
                    now = time.monotonic()
                    if provider is None and now - last_retry >= 2:
                        last_retry = now
                        try:
                            provider = ViveUltimateLinuxProvider(mapping, max_pose_age_s=0.2)
                        except (OSError, RuntimeError) as exc:
                            print(f"Waiting for VIVE Hub: {exc}", flush=True)
                    poses, inventory = {}, {}
                    if provider is not None:
                        frame = provider.read()
                        poses = {p.device_id: p for p in frame.poses.values()}
                        inventory = {row["serial"]: row for row in provider.device_inventory(poll=False)}
                        if not provider.client.connected:
                            provider.close(); provider = None
                            poses, inventory = {}, {}
                    delivered = 0
                    for serial in mapping:
                        row = inventory.get(serial, {})
                        connected = bool(row) and row.get("state") not in (0, 8)
                        packet = encode_packet(serial, poses.get(serial), connected=connected, now_ns=time.monotonic_ns())
                        try:
                            sender.sendto(packet, address); delivered += 1
                        except OSError as exc:
                            if exc.errno not in (errno.ENOENT, errno.ECONNREFUSED, errno.EAGAIN, errno.ENOBUFS):
                                raise
                    if now - last_log >= 5:
                        states = ", ".join(f"{s[-4:]}={inventory.get(s, {}).get('state_name', 'offline')}" for s in mapping)
                        print(f"SteamVR driver {'receiving' if delivered else 'not loaded yet'}; {states}", flush=True)
                        last_log = now
                    time.sleep(1 / args.frequency)
            finally:
                # Explicit disconnect plus driver's independent 200ms watchdog.
                for serial in mapping:
                    try:
                        sender.sendto(encode_packet(serial, None, connected=False, now_ns=time.monotonic_ns()), address)
                    except OSError:
                        pass
                if provider is not None:
                    provider.close()
                for signum, handler in previous.items():
                    signal.signal(signum, handler)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
