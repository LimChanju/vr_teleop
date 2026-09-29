"""Host-only tests. Generated packets are not a physical Quest 3 test."""

import asyncio
import json
from pathlib import Path
import re
import signal
import socket
import ssl
import subprocess
import sys
import tempfile
import time
import unittest

import aiohttp
import msgpack
import numpy as np
from yarl import URL

from quest3_probe import InputState, validate_pose

ROOT = Path(__file__).resolve().parents[1]


def pose(x=0, y=1.5, z=-0.3):
    matrix = np.eye(4)
    matrix[:3, 3] = [x, y, z]
    return matrix.ravel(order="F").tolist()


class StateTests(unittest.TestCase):
    def test_column_major_positions_and_buttons(self):
        state = InputState()
        self.assertEqual(state.snapshot()["status"], "WAITING")
        state.update("head", pose(y=1.6), now=10)
        for name, x in (("left", -0.25), ("right", 0.25)):
            self.assertTrue(state.update(name, pose(x=x), {"triggerValue": 0.7, "thumbstickValue": [-0.4, 0.5]}, now=10))
        result = state.snapshot(now=10.2)
        self.assertEqual(result["status"], "RECEIVING")
        self.assertEqual(result["channels"]["left"]["position_m"], [-0.25, 1.5, -0.3])
        self.assertEqual(result["channels"]["right"]["buttons"]["triggerValue"], 0.7)
        self.assertEqual(result["channels"]["right"]["buttons"]["thumbstickValue"], [-0.4, 0.5])

    def test_stale_data_is_not_reported_as_current(self):
        state = InputState()
        state.update("left", pose(), {}, now=10)
        result = state.snapshot(now=11.1)
        self.assertFalse(result["channels"]["left"]["fresh"])
        self.assertEqual(result["status"], "PARTIAL_OR_STALE")

    def test_bad_and_missing_packets_invalidate_freshness(self):
        state = InputState()
        state.update("left", pose(), {}, now=10)
        for bad in (None, [0] * 16, [float("nan")] * 16, pose()[:15]):
            self.assertFalse(state.update("left", bad, {}, now=10.1))
            self.assertFalse(state.snapshot(now=10.2)["channels"]["left"]["fresh"])
        self.assertEqual(state.invalid_packets.value, 4)
        self.assertTrue(state.update("left", pose(), {}, now=10.3))
        self.assertTrue(state.snapshot(now=10.4)["channels"]["left"]["fresh"])

    def test_reflection_and_invalid_buttons_are_rejected(self):
        matrix = pose()
        matrix[0] = -1
        with self.assertRaises(ValueError):
            validate_pose(matrix)
        state = InputState()
        self.assertFalse(state.update("right", pose(), {"triggerValue": float("inf")}))
        self.assertFalse(state.update("right", pose(), {"thumbstickValue": [0]}))


class ServerTests(unittest.TestCase):
    def test_cli_help_is_our_cli(self):
        command = subprocess.run([sys.executable, str(ROOT / "quest3_probe.py"), "--help"], capture_output=True, text=True)
        self.assertEqual(command.returncode, 0)
        self.assertIn("--server-host", command.stdout)
        self.assertNotIn("--Vuer.free-port", command.stdout)

    def test_https_websocket_freshness_and_shutdown(self):
        with tempfile.TemporaryDirectory(prefix="quest3-probe-test-") as temporary:
            directory = Path(temporary)
            certs = directory / "certs"
            made = subprocess.run([sys.executable, str(ROOT / "scripts/make_cert.py"), "--server-host", "127.0.0.1",
                                   "--out-dir", str(certs)], capture_output=True, text=True)
            self.assertEqual(made.returncode, 0, made.stderr)
            self.assertEqual((certs / "key.pem").stat().st_mode & 0o777, 0o600)
            with socket.socket() as sock:
                sock.bind(("127.0.0.1", 0))
                port = sock.getsockname()[1]
            log_dir = directory / "result"
            command = [sys.executable, "-u", str(ROOT / "quest3_probe.py"), "--server-host", "127.0.0.1", "--port", str(port),
                       "--cert", str(certs / "cert.pem"), "--key", str(certs / "key.pem"), "--log-dir", str(log_dir)]
            context = ssl.create_default_context(cafile=str(certs / "cert.pem"))
            with (directory / "server.log").open("w+") as logfile:
                process = subprocess.Popen(command, stdout=logfile, stderr=subprocess.STDOUT)
                try:
                    async def exercise():
                        address = f"https://127.0.0.1:{port}"
                        async with aiohttp.ClientSession(connector=aiohttp.TCPConnector(ssl=context)) as client:
                            for _ in range(100):
                                if process.poll() is not None:
                                    logfile.seek(0)
                                    self.fail(logfile.read())
                                try:
                                    async with client.get(address + "/healthz") as response:
                                        health = await response.json()
                                        break
                                except aiohttp.ClientConnectorError:
                                    await asyncio.sleep(0.1)
                            else:
                                self.fail("HTTPS listener did not start")
                            self.assertEqual(health["input"]["status"], "WAITING")
                            async with client.get(address + "/") as response:
                                self.assertEqual(response.status, 200)
                                html = await response.text()
                                self.assertIn("<script", html)
                                script = re.search(r'src="(/assets/[^"]+\.js)"', html)
                                self.assertIsNotNone(script)
                            async with client.head(address + script.group(1)) as response:
                                self.assertEqual(response.status, 200)
                            for suffix in ("/static/certs/key.pem", "/assets/%2e%2e/%2e%2e/etc/passwd"):
                                async with client.get(URL(address + suffix, encoded=True)) as response:
                                    self.assertIn(response.status, (403, 404))
                            async with client.ws_connect(address + "/") as ws:
                                value = {"left": pose(-0.25), "right": pose(0.25),
                                         "leftState": {"triggerValue": 0.75}, "rightState": {"squeezeValue": 0.5}}
                                for _ in range(5):
                                    await ws.send_bytes(msgpack.packb({"etype": "CAMERA_MOVE", "value": {"camera": {"matrix": pose(y=1.6)}}}))
                                    await ws.send_bytes(msgpack.packb({"etype": "CONTROLLER_MOVE", "value": value}))
                                    await asyncio.sleep(0.08)
                                async with client.get(address + "/healthz") as response:
                                    snapshot = (await response.json())["input"]
                                self.assertEqual(snapshot["status"], "RECEIVING")
                                self.assertEqual(snapshot["channels"]["left"]["position_m"], [-0.25, 1.5, -0.3])
                                self.assertEqual(snapshot["channels"]["left"]["buttons"]["triggerValue"], 0.75)
                            await asyncio.sleep(1.2)
                            async with client.get(address + "/healthz") as response:
                                snapshot = (await response.json())["input"]
                            self.assertEqual(snapshot["status"], "PARTIAL_OR_STALE")
                            self.assertFalse(snapshot["channels"]["right"]["fresh"])
                            # Starting a second server must not terminate the first.
                            duplicate = subprocess.run(command[:-2] + ["--log-dir", str(directory / "duplicate")],
                                                       capture_output=True, text=True, timeout=10)
                            self.assertNotEqual(duplicate.returncode, 0)
                            self.assertIn("Cannot bind", duplicate.stderr)
                            self.assertIsNone(process.poll())
                        # Let aiohttp finish closing SSL transports before the loop exits.
                        await asyncio.sleep(0.25)
                    asyncio.run(exercise())
                finally:
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                logfile.seek(0)
                output = logfile.read()
                self.assertEqual(process.returncode, 0, output)
                self.assertIn(f"Visit: https://127.0.0.1:{port}/?ws=wss://127.0.0.1:{port}", output)
                summary = json.loads((log_dir / "summary.json").read_text())
                self.assertEqual(summary["status"], "stopped")
                self.assertTrue(summary["controller_input_received"])
                with socket.socket() as sock:
                    self.assertNotEqual(sock.connect_ex(("127.0.0.1", port)), 0)


if __name__ == "__main__":
    unittest.main()
