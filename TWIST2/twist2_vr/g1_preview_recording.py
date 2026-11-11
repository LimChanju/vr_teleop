"""Offscreen MP4 recording for the G1 *kinematic* reference preview.

AsyncG1PreviewRecorder selects EGL only inside its spawned child. For direct
G1PreviewRecorder usage, set MUJOCO_GL=egl before importing MuJoCo in that
process. Neither recorder opens a window, connects to VR, or steps physics.
Use each recorder from the thread/process that constructs it.
"""

from __future__ import annotations

from collections.abc import Mapping
import copy
import json
import multiprocessing
from numbers import Real
import os
from pathlib import Path
from queue import Empty, Full
import signal
import traceback
from typing import Any

import numpy as np


LABEL = "G1 kinematic preview / NOT dynamics"


def _json_value(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return _json_value(value.tolist())
    if isinstance(value, np.generic):
        return _json_value(value.item())
    if isinstance(value, Mapping):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float) and np.isfinite(value):
        return value
    raise ValueError("recording status must contain finite JSON-compatible values")


class G1PreviewRecorder:
    """Record a constant-FPS MP4 plus one metadata row per encoded frame.

``capture`` renders the latest supplied state only when a new wall-clock FPS
slot is due. A late call repeats that latest image at most five times, dropping
older slots instead of doing unbounded catch-up. Thus long stalls compress the
video timeline; JSONL records both video time and wall slot/capture time, plus
the skipped count. This is a record of sampled references, not physics.

The output directory may already exist, but neither output file may exist.
The renderer owns a private model copy, leaving the caller's render settings
and simulation data unchanged. ``stats`` returns a fresh dictionary.
    """

    MAX_FRAMES_PER_CAPTURE = 5

    def __init__(self, model: Any, output_dir: str | Path, fps: float = 30,
                 width: int = 960, height: int = 720) -> None:
        if isinstance(fps, bool) or not isinstance(fps, Real) or not np.isfinite(fps) or fps <= 0:
            raise ValueError("fps must be finite and positive")
        # imageio-ffmpeg serializes the input rate to two decimal places.
        self.fps = round(float(fps), 2)
        if self.fps <= 0:
            raise ValueError("fps must remain positive when rounded to two decimal places")
        for name, value in (("width", width), ("height", height)):
            if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or value < 64 or value % 2:
                raise ValueError(f"{name} must be an even integer of at least 64 pixels")
        self.width, self.height = int(width), int(height)
        self.output_dir = Path(output_dir)
        self.video_path = self.output_dir / "g1_front.mp4"
        self.metadata_path = self.output_dir / "video_frames.jsonl"
        self._renderer = self._writer = self._metadata = None
        self._closed = False
        self._failed = False
        self._last_wall_time = -1.0
        self._next_slot = 0
        self._frames = self._captures = self._renders = self._duplicates = self._skipped = 0
        self.output_dir.mkdir(parents=True, exist_ok=True)
        owned: list[Path] = []
        try:
            # Reserve both names before starting ffmpeg (which otherwise uses
            # overwrite mode). Roll back only empty files created by this call.
            with self.video_path.open("xb"):
                pass
            owned.append(self.video_path)
            self._metadata = self.metadata_path.open("x", encoding="utf-8")
            owned.append(self.metadata_path)
            import mujoco
            import imageio_ffmpeg
            from PIL import Image, ImageDraw, ImageFont

            self._mujoco, self._ffmpeg = mujoco, imageio_ffmpeg
            self._image, self._draw = Image, ImageDraw
            self._model = copy.copy(model)
            self._model.vis.global_.offwidth = max(self.width, self._model.vis.global_.offwidth)
            self._model.vis.global_.offheight = max(self.height, self._model.vis.global_.offheight)
            self._renderer = mujoco.Renderer(self._model, height=self.height, width=self.width)
            self._camera = mujoco.MjvCamera()
            self._camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            # World +X side of the robot, facing its front; the camera follows
            # root translation but does not rotate with the robot's heading.
            self._camera.azimuth = 180.0
            self._camera.elevation = -12.0
            self._camera.distance = 2.8
            self._option = mujoco.MjvOption()
            self._option.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
            font_size = max(12, min(24, self.width // 45))
            try:
                self._font = ImageFont.truetype("DejaVuSans.ttf", font_size)
            except OSError:
                self._font = ImageFont.load_default()
        except BaseException:
            try:
                self.close()
            finally:
                for path in owned:
                    if path.exists() and path.stat().st_size == 0:
                        path.unlink()
            raise

    @property
    def stats(self) -> dict[str, Any]:
        return {
            "video_path": str(self.video_path),
            "metadata_path": str(self.metadata_path),
            "fps": self.fps, "width": self.width, "height": self.height,
            "frames_written": self._frames, "capture_calls": self._captures,
            "rendered_frames": self._renders, "duplicated_frames": self._duplicates,
            "skipped_wall_slots": self._skipped,
            "video_duration_s": self._frames / self.fps,
            "last_capture_elapsed_s": None if self._last_wall_time < 0 else self._last_wall_time,
            "sampling_policy": "latest_frame_repeat_capped",
            "max_frames_per_capture": self.MAX_FRAMES_PER_CAPTURE,
            "closed": self._closed, "failed": self._failed,
            "kinematic_preview": True, "physical_dynamics": False,
        }

    def _overlay(self, pixels: np.ndarray, elapsed: float, status: Any) -> np.ndarray:
        image = self._image.fromarray(pixels)
        draw = self._draw.Draw(image)
        summary = status if isinstance(status, str) else status.get("status", status.get("kind", "reference"))
        summary = str(summary).replace("\n", " ")[:110]
        lines = [LABEL, f"Elapsed {elapsed:.2f} s   |   {summary}"]
        if isinstance(status, Mapping):
            fit = status.get("refinement", status.get("fit"))
            if isinstance(fit, Mapping):
                accepted = status.get("fit_accepted", fit.get("fit_accepted", True))
                details = ("fit OK" if fit.get("success") is True and accepted is True
                           else "fit outside tolerance / unavailable")
                error = fit.get("max_position_error_m")
                if isinstance(error, Real) and np.isfinite(error):
                    details += f"   residual {1000 * error:.1f} mm"
                lines.append(details)
            elif fit is not None:
                lines.append(f"Fit: {str(fit)[:100]}")
        # Keep even the required disclaimer readable in a small test/export.
        wrapped = []
        max_width = self.width - 20
        for line in lines:
            current = ""
            for word in line.split():
                candidate = (current + " " + word).strip()
                if current and draw.textlength(candidate, font=self._font) > max_width:
                    wrapped.append(current)
                    current = word
                else:
                    current = candidate
            wrapped.append(current)
        line_height = self._font.getbbox("Ag")[3] + 6
        draw.rectangle((0, 0, self.width, min(self.height, 12 + line_height * len(wrapped))), fill=(12, 16, 24))
        for index, line in enumerate(wrapped):
            draw.text((10, 6 + index * line_height), line, font=self._font,
                      fill=(255, 215, 125) if index == 0 else (240, 240, 240))
        return np.ascontiguousarray(np.asarray(image), dtype=np.uint8)

    def capture(self, data: Any, wall_elapsed_s: float, status: Any) -> int:
        """Encode up to five due frames and return their count.

``status`` is a string or JSON-compatible mapping, retained in every JSONL
row. A mapping can include source_timestamp_s/timestamp_s, fit/refinement,
tracking status, or any other audit information supplied by the caller.
        """
        if self._closed or self._failed:
            raise RuntimeError("preview recorder is closed or failed")
        if (isinstance(wall_elapsed_s, bool) or not isinstance(wall_elapsed_s, Real)
                or not np.isfinite(wall_elapsed_s) or wall_elapsed_s < 0):
            raise ValueError("wall_elapsed_s must be finite and nonnegative")
        elapsed = float(wall_elapsed_s)
        if elapsed < self._last_wall_time:
            raise ValueError("wall_elapsed_s must not go backwards")
        if not isinstance(status, (str, Mapping)):
            raise ValueError("status must be a string or mapping")
        status = _json_value(status)
        if not np.isfinite(np.asarray(data.qpos)).all():
            raise ValueError("cannot record nonfinite robot qpos")
        self._captures += 1
        self._last_wall_time = elapsed
        last_slot = int(np.floor(elapsed * self.fps + 1e-8))
        count = last_slot - self._next_slot + 1
        if count <= 0:
            return 0
        skipped = max(0, count - self.MAX_FRAMES_PER_CAPTURE)
        first_slot = self._next_slot + skipped
        count = min(count, self.MAX_FRAMES_PER_CAPTURE)
        try:
            root = np.asarray(data.qpos[:3], dtype=float)
            if root.shape != (3,):
                raise ValueError("G1 preview requires free-root qpos")
            self._camera.lookat[:] = root + [0.0, 0.0, 0.05]
            self._renderer.update_scene(data, camera=self._camera, scene_option=self._option)
            pixels = self._overlay(self._renderer.render(), elapsed, status)
            self._renders += 1
            if self._writer is None:
                self._writer = self._ffmpeg.write_frames(
                    str(self.video_path), (self.width, self.height), fps=self.fps,
                    codec="libx264", pix_fmt_out="yuv420p", macro_block_size=1,
                    quality=7, ffmpeg_log_level="error", ffmpeg_timeout=10,
                    output_params=["-movflags", "+faststart"],
                )
                self._writer.send(None)
            source_time = None
            if isinstance(status, Mapping):
                source_time = status.get("source_timestamp_s", status.get("timestamp_s"))
            for duplicate in range(count):
                row = {
                    "frame_index": self._frames,
                    "time_s": self._frames / self.fps,
                    "wall_slot_time_s": (first_slot + duplicate) / self.fps,
                    "capture_elapsed_s": elapsed,
                    "source_timestamp_s": source_time,
                    "status": status,
                    "duplicate_of_frame": self._frames - duplicate if duplicate else None,
                    "skipped_wall_slots_before": skipped if duplicate == 0 else 0,
                    "kinematic_preview": True, "physical_dynamics": False,
                }
                encoded_row = json.dumps(row, allow_nan=False)
                self._writer.send(pixels)
                self._metadata.write(encoded_row + "\n")
                self._frames += 1
                self._duplicates += int(duplicate > 0)
            self._metadata.flush()
            self._skipped += skipped
            self._next_slot = last_slot + 1
            return count
        except BaseException:
            self._failed = True
            raise

    def close(self) -> None:
        """Finalize the encoder and release EGL/GL resources; safe to repeat."""
        if self._closed:
            return
        self._closed = True
        errors = []
        for resource in (self._writer, self._metadata, self._renderer):
            if resource is not None:
                try:
                    resource.close()
                except Exception as exc:
                    errors.append(exc)
        if errors:
            self._failed = True
            raise RuntimeError(f"preview recorder close failed: {errors[0]}") from errors[0]

    def __enter__(self) -> "G1PreviewRecorder":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


def _async_recorder_worker(xml_path: str, output_dir: str, fps: float, width: int,
                           height: int, snapshots: Any, stop: Any, ready: Any,
                           stats_path: str) -> None:
    """Spawn target: all MuJoCo/GL imports and resources belong to this child."""
    # Ctrl+C reaches the whole terminal process group. The parent handles it
    # and sets stop in close(); interrupting this child could truncate its MP4
    # or interrupt the terminal statistics write during finalization.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    os.environ["MUJOCO_GL"] = "egl"
    recorder = None
    received = 0
    error = None
    try:
        import mujoco

        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        recorder = G1PreviewRecorder(model, output_dir, fps=fps, width=width, height=height)
        ready.set()
        while True:
            try:
                snapshot = snapshots.get(timeout=0.1)
            except Empty:
                if stop.is_set():
                    break
                continue
            # Coalesce any packet queued while the worker was busy rendering.
            while True:
                try:
                    snapshot = snapshots.get_nowait()
                except Empty:
                    break
            qpos, qvel, elapsed, status = snapshot
            if qpos.shape != data.qpos.shape or qvel.shape != data.qvel.shape:
                raise ValueError("recording snapshot shape does not match the G1 XML")
            data.qpos[:] = qpos
            data.qvel[:] = qvel
            mujoco.mj_forward(model, data)
            recorder.capture(data, elapsed, status)
            received += 1
    except BaseException:
        error = traceback.format_exc()
    finally:
        if recorder is not None:
            try:
                recorder.close()
            except BaseException:
                error = (error or "") + traceback.format_exc()
        result = recorder.stats if recorder is not None else {"frames_written": 0, "closed": True}
        result.update({
            "worker_pid": os.getpid(), "received_snapshots": received,
            "worker_error": error, "failed": bool(error or result.get("failed")),
            "completed": True, "graphics_backend": "egl",
        })
        # The parent reserved this exact file exclusively before starting us.
        # It is read after process join, so terminal stats cannot be dropped
        # behind a full pose queue or consumed by the newest-frame policy.
        try:
            with Path(stats_path).open("w", encoding="utf-8") as stream:
                json.dump(result, stream, indent=2, allow_nan=False)
                stream.write("\n")
        finally:
            ready.set()  # Also wake the constructor on initialization failure.


class AsyncG1PreviewRecorder:
    """Nonblocking pose submission to a separate spawned EGL recorder process.

The calling module must put runtime MuJoCo/OpenGL imports inside its guarded
main function: Python spawn imports that module before entering our target.
This module itself performs no graphics imports at import time. Parent calls
never select EGL or touch VR. ``close`` waits for finalization and raises any
worker error; inspect ``stats`` or video_recording_stats.json for the result.
    """

    def __init__(self, xml_path: str | Path, output_dir: str | Path,
                 fps: float = 30, width: int = 960, height: int = 720) -> None:
        self.output_dir = Path(output_dir).resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.stats_path = self.output_dir / "video_recording_stats.json"
        self._closed = False
        self._submitted = self._dropped = self._calls = 0
        self._last_wall_time = -1.0
        self._terminal_stats: dict[str, Any] | None = None
        self._error: str | None = None
        # Reserve independently of the child, preventing accidental reuse of a
        # run directory even if its previous encoder failed before first frame.
        with self.stats_path.open("x", encoding="utf-8"):
            pass
        context = multiprocessing.get_context("spawn")
        self._queue = context.Queue(maxsize=1)
        self._stop = context.Event()
        self._ready = context.Event()
        self._process = context.Process(
            target=_async_recorder_worker,
            args=(str(Path(xml_path).resolve()), str(self.output_dir), fps, width,
                  height, self._queue, self._stop, self._ready, str(self.stats_path)),
            name="g1-preview-egl-recorder",
        )
        try:
            self._process.start()
            if not self._ready.wait(timeout=20):
                self._error = "EGL recording worker did not initialize within 20 seconds"
                raise RuntimeError(self._error)
            if self.stats_path.stat().st_size:
                self._terminal_stats = json.loads(self.stats_path.read_text(encoding="utf-8"))
                if self._terminal_stats.get("worker_error"):
                    raise RuntimeError(self._terminal_stats["worker_error"])
            self._raise_if_exited()
        except BaseException:
            self._terminate_owned_worker()
            self._close_queue()
            self._closed = True
            raise

    @property
    def stats(self) -> dict[str, Any]:
        result = dict(self._terminal_stats or {})
        result.update({
            "stats_path": str(self.stats_path), "asynchronous": True,
            "submitted_snapshots": self._submitted, "dropped_snapshots": self._dropped,
            "parent_capture_calls": self._calls, "parent_closed": self._closed,
        })
        if self._error:
            result.update({"failed": True, "worker_error": self._error})
        return result

    def _raise_if_exited(self) -> None:
        if self._process.exitcode is None:
            return
        if self.stats_path.stat().st_size:
            self._terminal_stats = json.loads(self.stats_path.read_text(encoding="utf-8"))
        self._error = str((self._terminal_stats or {}).get("worker_error") or
                          f"recording worker exited unexpectedly ({self._process.exitcode})")
        raise RuntimeError(self._error)

    def capture(self, data: Any, wall_elapsed_s: float, status: Any) -> bool:
        """Copy and submit the latest state without waiting for rendering.

Returns True if queued, False if a feeder race temporarily prevented replacing
the pending packet. Queue drops are counted; the next call retries normally.
        """
        if self._closed:
            raise RuntimeError("asynchronous recorder is closed")
        self._raise_if_exited()
        if (isinstance(wall_elapsed_s, bool) or not isinstance(wall_elapsed_s, Real)
                or not np.isfinite(wall_elapsed_s) or wall_elapsed_s < 0
                or wall_elapsed_s < self._last_wall_time):
            raise ValueError("wall_elapsed_s must be finite, nonnegative, and nondecreasing")
        if not isinstance(status, (str, Mapping)):
            raise ValueError("status must be a string or mapping")
        qpos = np.asarray(data.qpos, dtype=float).copy()
        qvel = np.asarray(data.qvel, dtype=float).copy()
        if qpos.ndim != 1 or qvel.ndim != 1 or not np.isfinite(qpos).all() or not np.isfinite(qvel).all():
            raise ValueError("recording requires finite one-dimensional qpos and qvel")
        snapshot = (qpos, qvel, float(wall_elapsed_s), _json_value(status))
        self._calls += 1
        self._last_wall_time = float(wall_elapsed_s)
        try:
            self._queue.put_nowait(snapshot)
        except Full:
            try:
                self._queue.get_nowait()
                self._dropped += 1
            except Empty:
                # multiprocessing's feeder may have reserved the queue slot
                # without making the packet readable yet. Never block the UI.
                self._dropped += 1
                return False
            try:
                self._queue.put_nowait(snapshot)
            except Full:
                self._dropped += 1
                return False
        self._submitted += 1
        return True

    def _terminate_owned_worker(self) -> None:
        if self._process.pid is None:
            return
        if self._process.is_alive():
            self._process.terminate()
        self._process.join(timeout=2)
        if self._process.is_alive():
            self._process.kill()
            self._process.join(timeout=2)

    def _close_queue(self) -> None:
        self._queue.cancel_join_thread()
        self._queue.close()

    def close(self) -> None:
        if self._closed:
            if self._error:
                raise RuntimeError(self._error)
            return
        self._closed = True
        self._stop.set()
        self._process.join(timeout=20)
        if self._process.is_alive():
            self._error = "recording worker did not finalize within 20 seconds; its process was stopped"
            self._terminate_owned_worker()
        self._close_queue()
        if self.stats_path.stat().st_size:
            self._terminal_stats = json.loads(self.stats_path.read_text(encoding="utf-8"))
        if self._terminal_stats is None:
            self._error = self._error or "recording worker exited without final statistics"
        elif self._terminal_stats.get("worker_error") or self._terminal_stats.get("failed"):
            self._error = self._error or str(self._terminal_stats.get("worker_error") or "recording failed")
        if self._process.exitcode != 0:
            self._error = self._error or f"recording worker exited with code {self._process.exitcode}"
        if self._error:
            raise RuntimeError(self._error)

    def __enter__(self) -> "AsyncG1PreviewRecorder":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
