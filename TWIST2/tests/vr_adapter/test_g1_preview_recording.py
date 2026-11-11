import json
import os
from pathlib import Path
from queue import Queue
import signal
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest

from twist2_vr.g1_preview_recording import AsyncG1PreviewRecorder, G1PreviewRecorder, LABEL


class FakeRenderer:
    instances = []

    def __init__(self, model, *, height, width):
        self.height, self.width = height, width
        self.closed = False
        self.camera = None
        self.instances.append(self)

    def update_scene(self, data, *, camera, scene_option):
        self.camera = camera

    def render(self):
        return np.full((self.height, self.width, 3), 120, dtype=np.uint8)

    def close(self):
        self.closed = True


class Model:
    def __init__(self):
        self.vis = SimpleNamespace(global_=SimpleNamespace(offwidth=64, offheight=64))

    def __copy__(self):
        return Model()


@pytest.fixture
def fake_render(monkeypatch):
    FakeRenderer.instances.clear()
    writers = []

    def writer(path, size, **kwargs):
        frames = []
        writers.append((path, kwargs, frames))
        try:
            frame = yield
            while True:
                frames.append(frame.copy())
                frame = yield
        finally:
            Path(path).write_bytes(b"fake video")

    monkeypatch.setitem(sys.modules, "mujoco", SimpleNamespace(
        Renderer=FakeRenderer,
        MjvCamera=lambda: SimpleNamespace(lookat=np.zeros(3)),
        MjvOption=lambda: SimpleNamespace(flags=np.zeros(10, dtype=bool)),
        mjtCamera=SimpleNamespace(mjCAMERA_FREE=0),
        mjtVisFlag=SimpleNamespace(mjVIS_CONTACTPOINT=0),
    ))
    monkeypatch.setitem(sys.modules, "imageio_ffmpeg", SimpleNamespace(write_frames=writer))
    return writers


def data():
    return SimpleNamespace(qpos=np.array([.2, .3, .8, 1., 0, 0, 0]))


def test_fixed_rate_bounded_latest_frame_and_metadata(tmp_path, fake_render):
    model = Model()
    recorder = G1PreviewRecorder(model, tmp_path, fps=10, width=640, height=480)
    status = {"status": "tracking", "source_timestamp_s": 123.4,
              "refinement": {"success": False, "max_position_error_m": np.float64(.04)}}
    assert recorder.capture(data(), 0, status) == 1
    assert recorder.capture(data(), .04, status) == 0
    assert recorder.capture(data(), .31, status) == 3
    assert recorder.capture(data(), 5.0, status) == 5
    recorder.close()
    recorder.close()
    rows = [json.loads(line) for line in (tmp_path / "video_frames.jsonl").read_text().splitlines()]
    assert len(rows) == 9
    assert [row["frame_index"] for row in rows] == list(range(9))
    np.testing.assert_allclose([row["time_s"] for row in rows], np.arange(9) / 10)
    assert rows[-1]["wall_slot_time_s"] == 5
    assert rows[4]["skipped_wall_slots_before"] == 42
    assert rows[-1]["source_timestamp_s"] == 123.4
    assert rows[-1]["status"]["refinement"]["success"] is False
    assert all(row["physical_dynamics"] is False for row in rows)
    assert recorder.stats["skipped_wall_slots"] == 42
    assert recorder.stats["rendered_frames"] == 3
    assert recorder.stats["duplicated_frames"] == 6
    assert recorder.stats["closed"]
    assert FakeRenderer.instances[0].closed
    assert model.vis.global_.offwidth == 64
    assert model.vis.global_.offheight == 64
    np.testing.assert_allclose(FakeRenderer.instances[0].camera.lookat, [.2, .3, .85])
    assert FakeRenderer.instances[0].camera.azimuth == 180
    frame = fake_render[0][2][0]
    assert np.any(frame[:80] != 120)  # The label/status overlay is embedded.
    np.testing.assert_array_equal(fake_render[0][2][1], fake_render[0][2][2])


@pytest.mark.parametrize("filename", ["g1_front.mp4", "video_frames.jsonl"])
def test_exclusive_outputs_preserve_existing_files(tmp_path, fake_render, filename):
    path = tmp_path / filename
    path.write_bytes(b"existing recording")
    with pytest.raises(FileExistsError):
        G1PreviewRecorder(Model(), tmp_path)
    assert path.read_bytes() == b"existing recording"
    assert not fake_render


@pytest.mark.parametrize("options", [{"fps": 0}, {"fps": np.nan}, {"fps": True}, {"width": 95}, {"height": 0}])
def test_invalid_configuration(tmp_path, fake_render, options):
    with pytest.raises(ValueError):
        G1PreviewRecorder(Model(), tmp_path, **options)
    assert list(tmp_path.iterdir()) == []


def test_invalid_capture_and_close(tmp_path, fake_render):
    recorder = G1PreviewRecorder(Model(), tmp_path)
    for elapsed in (-1, np.nan, float("inf")):
        with pytest.raises(ValueError):
            recorder.capture(data(), elapsed, "tracking")
    with pytest.raises(ValueError):
        recorder.capture(data(), 0, {"fit": np.nan})
    recorder.capture(data(), .1, "tracking")
    with pytest.raises(ValueError, match="backwards"):
        recorder.capture(data(), .09, "tracking")
    recorder.close()
    with pytest.raises(RuntimeError):
        recorder.capture(data(), .2, "tracking")


@pytest.mark.parametrize("accepted", [True, False])
def test_overlay_contains_explicit_kinematic_label(tmp_path, fake_render, monkeypatch, accepted):
    recorder = G1PreviewRecorder(Model(), tmp_path)
    seen = []
    real_draw = recorder._draw.Draw

    def capture_draw(image):
        draw = real_draw(image)
        text = draw.text

        def capture_text(position, message, **kwargs):
            seen.append(message)
            return text(position, message, **kwargs)

        draw.text = capture_text
        return draw

    monkeypatch.setattr(recorder._draw, "Draw", capture_draw)
    recorder.capture(data(), 0, {"status": "tracking", "fit_accepted": accepted,
                               "fit": {"success": True}})
    recorder.close()
    assert LABEL in seen
    assert any("Elapsed" in text for text in seen)
    assert any("fit OK" in text for text in seen) is accepted
    if not accepted:
        assert any("fit outside tolerance" in text for text in seen)


def test_async_submission_replaces_pending_snapshot_without_mutating_inputs():
    recorder = AsyncG1PreviewRecorder.__new__(AsyncG1PreviewRecorder)
    recorder._closed = False
    recorder._process = SimpleNamespace(exitcode=None)
    recorder._queue = Queue(maxsize=1)
    recorder._calls = recorder._submitted = recorder._dropped = 0
    recorder._last_wall_time = -1
    snapshot_data = SimpleNamespace(qpos=np.array([0., 0., .8, 1, 0, 0, 0]), qvel=np.zeros(6))
    assert recorder.capture(snapshot_data, 0, {"status": "old"})
    snapshot_data.qpos[0] = .4
    assert recorder.capture(snapshot_data, .1, {"status": "latest"})
    snapshot_data.qpos[0] = .8
    packet = recorder._queue.get_nowait()
    assert packet[0][0] == .4
    assert packet[2] == .1
    assert packet[3]["status"] == "latest"
    assert recorder._dropped == 1
    assert recorder._submitted == 2


@pytest.mark.skipif(os.environ.get("TWIST2_TEST_EGL_RECORDING") != "1", reason="explicit EGL integration test")
def test_spawned_egl_recording_round_trip_with_actual_g1(tmp_path, monkeypatch):
    # Parent can import MuJoCo with its default GLFW backend; only the spawned
    # recorder selects EGL, and no viewer/window is initialized in either test.
    monkeypatch.delenv("MUJOCO_GL", raising=False)
    import mujoco
    import imageio_ffmpeg

    xml = Path(__file__).resolve().parents[2] / "assets/g1/g1_sim2sim_29dof.xml"
    model = mujoco.MjModel.from_xml_path(str(xml))
    robot = mujoco.MjData(model)
    with AsyncG1PreviewRecorder(xml, tmp_path, fps=10, width=640, height=480) as recorder:
        for index in range(8):
            if index == 3:
                # An initialized recorder must survive terminal Ctrl+C; only
                # its parent's close()/stop event owns orderly finalization.
                os.kill(recorder._process.pid, signal.SIGINT)
            robot.qpos[22] = .1 * index
            mujoco.mj_forward(model, robot)
            recorder.capture(robot, index / 10, {"status": "offline test", "source_timestamp_s": 100 + index / 10})
            time.sleep(.04)
    assert "MUJOCO_GL" not in os.environ
    stats = json.loads((tmp_path / "video_recording_stats.json").read_text())
    assert stats["graphics_backend"] == "egl"
    assert stats["completed"] and not stats["failed"]
    assert stats["frames_written"] >= 1
    assert recorder.stats["frames_written"] == stats["frames_written"]
    assert stats["worker_pid"] != os.getpid()
    assert not recorder._process.is_alive()
    reader = imageio_ffmpeg.read_frames(str(tmp_path / "g1_front.mp4"), pix_fmt="rgb24")
    metadata = next(reader)
    frames = list(reader)
    rows = [json.loads(line) for line in (tmp_path / "video_frames.jsonl").read_text().splitlines()]
    assert len(frames) == len(rows) == stats["frames_written"]
    assert metadata["size"] == (640, 480)
    assert metadata["fps"] == 10
    assert all(row["physical_dynamics"] is False for row in rows)


@pytest.mark.skipif(os.environ.get("TWIST2_TEST_EGL_RECORDING") != "1", reason="explicit spawned-worker integration test")
def test_spawned_worker_startup_error_is_reported(tmp_path):
    with pytest.raises(RuntimeError, match="does-not-exist"):
        AsyncG1PreviewRecorder(tmp_path / "does-not-exist.xml", tmp_path)
    stats = json.loads((tmp_path / "video_recording_stats.json").read_text())
    assert stats["failed"] and stats["completed"]
    assert "does-not-exist" in stats["worker_error"]
