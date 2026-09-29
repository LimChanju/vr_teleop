"""Receive Quest 3 controller input using Unitree's official TeleVuer transport.

This diagnostic wrapper is custom code. It does not launch Isaac Sim or publish
robot commands. Coordinates remain in the original OpenXR reference frame.
"""

import argparse
from datetime import datetime, timezone
import ipaddress
import json
import multiprocessing as mp
from pathlib import Path
import re
import signal
import socket
import ssl
import sys
import tempfile
import time
from urllib.parse import quote

import numpy as np

# params-proto otherwise consumes --help and parses this tool's CLI at import.
_probe_argv = sys.argv
try:
    sys.argv = [sys.argv[0]]
    from televuer import TeleVuer
finally:
    sys.argv = _probe_argv

ROOT = Path(__file__).resolve().parent
BUTTON_FIELDS = ("trigger", "triggerValue", "squeeze", "squeezeValue", "thumbstick", "aButton", "bButton")


def validate_pose(value):
    """Vuer sends sixteen column-major values (translation at indices 12–14)."""
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (16,) or not np.isfinite(matrix).all():
        raise ValueError("Expected 16 finite pose values")
    transform = matrix.reshape(4, 4, order="F")
    rotation = transform[:3, :3]
    if not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-4):
        raise ValueError("Invalid homogeneous pose")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=0.02) or not np.isclose(np.linalg.det(rotation), 1, atol=0.02):
        raise ValueError("Invalid pose rotation")
    return matrix


def validate_buttons(value):
    if not isinstance(value, dict):
        raise ValueError("Missing controller state")
    buttons = np.asarray([value.get(key, 0) for key in BUTTON_FIELDS], dtype=float)
    stick = np.asarray(value.get("thumbstickValue", [0, 0]), dtype=float)
    if buttons.shape != (7,) or stick.shape != (2,) or not np.isfinite(np.r_[buttons, stick]).all():
        raise ValueError("Invalid controller buttons")
    if np.any(buttons < 0) or np.any(buttons > 1) or np.any(np.abs(stick) > 1):
        raise ValueError("Controller value outside its valid range")
    return np.r_[buttons, stick]


class InputState:
    """Atomic shared snapshots, including freshness, for parent and Vuer child."""

    def __init__(self):
        # 16 matrix values + 9 button values + timestamp + packet count + valid flag.
        self.channels = {name: mp.Array("d", 28, lock=True) for name in ("head", "left", "right")}
        self.invalid_packets = mp.Value("Q", 0)

    def update(self, name, matrix, buttons=None, now=None):
        try:
            pose = validate_pose(matrix)
            state = np.zeros(9) if name == "head" else validate_buttons(buttons)
        except (ValueError, TypeError, OverflowError):
            with self.invalid_packets.get_lock():
                self.invalid_packets.value += 1
            # Do not retain a valid status after an explicit invalid/lost pose.
            with self.channels[name].get_lock():
                self.channels[name][27] = 0
            return False
        channel = self.channels[name]
        with channel.get_lock():
            channel[:25] = np.r_[pose, state]
            channel[25] = time.monotonic() if now is None else now
            channel[26] += 1
            channel[27] = 1
        return True

    def snapshot(self, stale_after=1.0, now=None):
        now = time.monotonic() if now is None else now
        output = {}
        for name, shared in self.channels.items():
            with shared.get_lock():
                data = list(shared[:])
            count = int(data[26])
            age = max(0.0, now - data[25]) if count else None
            fresh = bool(data[27]) and age is not None and age <= stale_after
            sample = {"fresh": fresh, "packets": count, "age_s": age,
                      "position_m": data[12:15] if count else None,
                      "matrix_column_major": data[:16] if count else None}
            if name != "head":
                sample["buttons"] = dict(zip(BUTTON_FIELDS, data[16:23]))
                sample["buttons"]["thumbstickValue"] = data[23:25]
            output[name] = sample
        all_fresh = all(sample["fresh"] for sample in output.values())
        any_received = any(sample["packets"] for sample in output.values())
        return {"status": "RECEIVING" if all_fresh else ("PARTIAL_OR_STALE" if any_received else "WAITING"),
                "utc": datetime.now(timezone.utc).isoformat(), "frame": "OpenXR: +X right, +Y up, -Z forward",
                "channels": output, "invalid_packets": self.invalid_packets.value}


class ControllerProbe(TeleVuer):
    def __init__(self, state, bind, port, static_root, cert, key, browser_url):
        self.probe_state = state
        self.probe_bind, self.probe_port = bind, port
        self.probe_static_root = static_root
        self.probe_browser_url = browser_url
        super().__init__(use_hand_tracking=False, binocular=False, img_shape=(480, 640),
                         display_mode="pass-through", zmq=False, webrtc=False,
                         cert_file=str(cert), key_file=str(key))

    def _vuer_run(self):
        # Configure only this server instance; upstream source remains unchanged.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        self.vuer.host, self.vuer.port = self.probe_bind, self.probe_port
        self.vuer.free_port = False  # Never stop another user's process to claim a port.
        self.vuer.static_root = self.probe_static_root
        self.vuer.get_url = lambda: self.probe_browser_url
        # Use aiohttp's bounded static paths, including for Vuer's bundled assets.
        self.vuer._add_static = lambda route, root: self.vuer.app.router.add_static(
            route, str(root), show_index=False, follow_symlinks=False)
        from aiohttp import web

        async def health(request):
            return web.json_response({"service": "quest3-controller-probe", "input": self.probe_state.snapshot()})

        self.vuer.app.router.add_get("/healthz", health)
        self.vuer.run(kill=False)

    async def on_cam_move(self, event, session, fps=60):
        value = event.value if isinstance(event.value, dict) else {}
        camera = value.get("camera", {})
        matrix = camera.get("matrix") if isinstance(camera, dict) else None
        if self.probe_state.update("head", matrix):
            await super().on_cam_move(event, session, fps)

    async def on_controller_move(self, event, session, fps=60):
        value = event.value if isinstance(event.value, dict) else {}
        left = self.probe_state.update("left", value.get("left"), value.get("leftState"))
        right = self.probe_state.update("right", value.get("right"), value.get("rightState"))
        if left and right:
            await super().on_controller_move(event, session, fps)


def save_json(path, value):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server-host", default="localhost", help="IP/DNS name reachable from Quest, used in the URL")
    parser.add_argument("--bind", default="127.0.0.1", help="Use 0.0.0.0 for a trusted LAN/VPN connection")
    parser.add_argument("--port", type=int, default=8012)
    parser.add_argument("--cert", type=Path, default=ROOT / "certs/cert.pem")
    parser.add_argument("--key", type=Path, default=ROOT / "certs/key.pem")
    parser.add_argument("--duration", type=float, default=0, help="Exit after N seconds; 0 waits until Ctrl+C")
    parser.add_argument("--log-dir", type=Path, default=ROOT / "logs" / datetime.now().strftime("%Y%m%d-%H%M%S-%f"))
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or not np.isfinite(args.duration) or args.duration < 0:
        parser.error("Invalid port or duration")
    try:
        address = ipaddress.ip_address(args.server_host)
        host = f"[{address}]" if address.version == 6 else str(address)
    except ValueError:
        if not re.fullmatch(r"[A-Za-z0-9](?:[A-Za-z0-9.-]*[A-Za-z0-9])?", args.server_host):
            parser.error("--server-host must be an IP or DNS name without a URL scheme")
        host = args.server_host
    # Validate key/certificate before creating the child process.
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    try:
        context.load_cert_chain(str(args.cert), str(args.key))
    except (OSError, ssl.SSLError) as error:
        parser.error(f"Cannot load certificate/key: {error}. Run scripts/make_cert.py first.")
    try:
        family, kind, proto, _, sockaddr = socket.getaddrinfo(args.bind, args.port, type=socket.SOCK_STREAM)[0]
        with socket.socket(family, kind, proto) as sock:
            sock.bind(sockaddr)
    except OSError as error:
        parser.error(f"Cannot bind {args.bind}:{args.port}: {error}. Choose a different --port.")
    args.log_dir.mkdir(parents=True, exist_ok=False)
    url = f"https://{host}:{args.port}/?ws=" + quote(f"wss://{host}:{args.port}", safe=":/")
    print(f"[Quest3] Open in the Quest browser: {url}", flush=True)
    print("[Quest3] Start the XR / Pass-through session, move both controllers, and press each trigger.", flush=True)
    print(f"[Quest3] Status file: {args.log_dir / 'latest.json'}", flush=True)
    state = InputState()
    stopping = False
    result = {"url": url, "mode": "controller", "controller_input_received": False, "status": "starting",
              "verification_note": "Packet reception alone does not verify a physical Quest 3; confirm motion and buttons on the headset."}
    previous_counts = dict.fromkeys(("head", "left", "right"), 0)
    started = last_print = time.monotonic()
    probe = None
    exit_code = 0

    def stop(signum, frame):
        nonlocal stopping
        stopping = True

    with tempfile.TemporaryDirectory(prefix="quest3-static-") as static_root:
        try:
            probe = ControllerProbe(state, args.bind, args.port, static_root, args.cert.resolve(), args.key.resolve(), url)
            signal.signal(signal.SIGINT, stop)
            signal.signal(signal.SIGTERM, stop)
            while not stopping and (not args.duration or time.monotonic() - started < args.duration):
                if not probe.process.is_alive():
                    raise RuntimeError(f"Input server stopped (exit code {probe.process.exitcode})")
                snapshot = state.snapshot()
                save_json(args.log_dir / "latest.json", snapshot)
                if snapshot["status"] == "RECEIVING":
                    result["controller_input_received"] = True
                now = time.monotonic()
                if now - last_print >= 1.0:
                    parts = []
                    for name, sample in snapshot["channels"].items():
                        hz = (sample["packets"] - previous_counts[name]) / (now - last_print)
                        previous_counts[name] = sample["packets"]
                        position = "/".join(f"{x:+.3f}" for x in sample["position_m"]) if sample["fresh"] else "waiting/stale"
                        trigger = f" trigger={sample['buttons']['triggerValue']:.2f}" if name != "head" and sample["fresh"] else ""
                        parts.append(f"{name}: {position} ({hz:.0f} Hz){trigger}")
                    print(f"[Quest3] {snapshot['status']} | " + " | ".join(parts), flush=True)
                    last_print = now
                time.sleep(0.1)
            result["status"] = "stopped"
        except Exception as error:
            result.update(status="failed", error=str(error))
            print(f"[Quest3] ERROR: {error}", file=sys.stderr, flush=True)
            exit_code = 1
        finally:
            if probe is not None:
                probe.close()
                if probe.process.is_alive():
                    probe.process.kill()
                    probe.process.join(timeout=2)
            result.update(duration_s=time.monotonic() - started, final_input=state.snapshot())
            save_json(args.log_dir / "summary.json", result)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
