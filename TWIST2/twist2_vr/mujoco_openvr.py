"""OpenVR stereo viewer for a MuJoCo simulation.

The official TWIST2 simulator uses ``mujoco.viewer.launch_passive``, which is a
desktop-only viewer.  This module keeps simulation and policy execution in the
original thread and owns OpenVR/OpenGL in a dedicated render thread.  Only a
small qpos snapshot crosses that boundary, so ``VRCompositor.waitGetPoses``
can run at the headset cadence without stalling the 1 kHz physics loop.

OpenVR and OpenGL are imported lazily.  Importing this module is therefore safe
on machines without a running VR runtime and in hardware-independent tests.
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
import traceback
from typing import Any

import numpy as np


@dataclass(frozen=True)
class ControllerButtonSample:
    """One polled legacy OpenVR button state and its stable device identity."""

    device_id: str | None
    available: bool
    pressed: bool


class ReleaseArmedButton:
    """Emit one edge only after observing a release on the current device.

    Requiring a release prevents a controller that wakes or reconnects while a
    button is held from unexpectedly moving the user's view.
    """

    def __init__(self) -> None:
        self._device_id: str | None = None
        self._armed = False
        self._was_pressed = False

    def update(self, sample: ControllerButtonSample) -> bool:
        if not sample.available or sample.device_id is None:
            self._device_id = None
            self._armed = False
            self._was_pressed = False
            return False

        if sample.device_id != self._device_id:
            self._device_id = sample.device_id
            self._armed = not sample.pressed
            self._was_pressed = sample.pressed
            return False

        if not self._armed:
            self._was_pressed = sample.pressed
            if not sample.pressed:
                self._armed = True
            return False

        rising = sample.pressed and not self._was_pressed
        self._was_pressed = sample.pressed
        return rising


class PendingSignal:
    """Thread-safe, coalescing signal whose take operation cannot lose a set."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._pending = False

    def request(self) -> None:
        with self._lock:
            self._pending = True

    def take(self) -> bool:
        with self._lock:
            pending = self._pending
            self._pending = False
            return pending


def _unit(vector: np.ndarray, *, name: str) -> np.ndarray:
    value = np.asarray(vector, dtype=np.float64)
    norm = float(np.linalg.norm(value))
    if norm < 1e-9:
        raise ValueError(f"{name} must be non-zero")
    return value / norm


def rigid_inverse(transform: np.ndarray) -> np.ndarray:
    """Invert a homogeneous rigid transform without a general matrix inverse."""

    transform = np.asarray(transform, dtype=np.float64)
    if transform.shape != (4, 4):
        raise ValueError(f"expected a 4x4 transform, got {transform.shape}")
    result = np.eye(4, dtype=np.float64)
    rotation = transform[:3, :3]
    result[:3, :3] = rotation.T
    result[:3, 3] = -rotation.T @ transform[:3, 3]
    return result


def matrix34_to_transform(matrix: Any) -> np.ndarray:
    """Convert an OpenVR ``HmdMatrix34_t`` (or compatible rows) to 4x4."""

    rows = matrix.m if hasattr(matrix, "m") else matrix
    array = np.asarray([[float(rows[r][c]) for c in range(4)] for r in range(3)])
    result = np.eye(4, dtype=np.float64)
    result[:3, :] = array
    return result


def _tracked_device_string(vr_system: Any, prop: int, index: int) -> str:
    try:
        return str(vr_system.getStringTrackedDeviceProperty(index, prop))
    except Exception:
        return ""


def poll_controller_button(
    vr_system: Any,
    openvr: Any,
    poses: Any,
    *,
    side: str,
    button_id: int,
) -> ControllerButtonSample:
    """Poll a physical controller button through OpenVR's legacy interface.

    ALVR may expose both sleeping Touch controllers and synthetic hand-tracking
    devices. The role API is preferred, then stable serial/model properties are
    used as a fallback. Device indexes are deliberately resolved every frame
    because SteamVR can reassign them after sleep or reconnect.
    """

    if side not in {"left", "right"}:
        raise ValueError(f"unsupported controller side: {side!r}")
    target_role = (
        openvr.TrackedControllerRole_LeftHand
        if side == "left"
        else openvr.TrackedControllerRole_RightHand
    )
    invalid_index = int(openvr.k_unTrackedDeviceIndexInvalid)

    candidates: list[tuple[int, int]] = []
    try:
        role_index = int(vr_system.getTrackedDeviceIndexForControllerRole(target_role))
    except Exception:
        role_index = invalid_index

    side_token = side.lower()
    for index, pose in enumerate(poses):
        if not pose.bDeviceIsConnected:
            continue
        try:
            if int(vr_system.getTrackedDeviceClass(index)) != int(
                openvr.TrackedDeviceClass_Controller
            ):
                continue
            role = int(vr_system.getControllerRoleForTrackedDeviceIndex(index))
        except Exception:
            continue
        serial = _tracked_device_string(vr_system, openvr.Prop_SerialNumber_String, index)
        model = _tracked_device_string(vr_system, openvr.Prop_ModelNumber_String, index)
        controller_type = _tracked_device_string(
            vr_system, openvr.Prop_ControllerType_String, index
        )
        identity = f"{serial} {model} {controller_type}".lower()
        role_matches = role == int(target_role)
        if not role_matches and side_token not in identity:
            continue
        # Synthetic hands expose controller states but have no physical button.
        # Never let one shadow a sleeping/waking Touch controller with the same
        # semantic role.
        if "hand" in controller_type.lower() or "hand tracker" in identity:
            continue
        score = 0
        if index == role_index and role_index != invalid_index:
            score += 1_000
        if role_matches:
            score += 500
        if "touch" in controller_type.lower():
            score += 100
        if "controller" in identity:
            score += 20
        candidates.append((score, index))

    if not candidates:
        return ControllerButtonSample(None, False, False)
    _, index = max(candidates, key=lambda item: (item[0], -item[1]))
    serial = _tracked_device_string(vr_system, openvr.Prop_SerialNumber_String, index)
    model = _tracked_device_string(vr_system, openvr.Prop_ModelNumber_String, index)
    device_id = serial or model or f"openvr-index-{index}"
    try:
        valid, state = vr_system.getControllerState(index)
    except Exception:
        return ControllerButtonSample(device_id, False, False)
    if not valid:
        return ControllerButtonSample(device_id, False, False)
    pressed = bool(int(state.ulButtonPressed) & (1 << int(button_id)))
    return ControllerButtonSample(device_id, True, pressed)


def look_at_head_pose(position: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Return a head pose whose OpenVR -Z axis looks at ``target``.

    OpenVR head coordinates are +X right, +Y up and -Z forward.  MuJoCo uses
    +Z as world up for this model.  The returned matrix maps head coordinates
    into the MuJoCo world frame and keeps the horizon level.
    """

    position = np.asarray(position, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    forward = _unit(target - position, name="camera forward")
    world_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    right = np.cross(forward, world_up)
    if float(np.linalg.norm(right)) < 1e-6:
        raise ValueError("camera forward cannot be parallel to world up")
    right = _unit(right, name="camera right")
    back = -forward
    up = _unit(np.cross(back, right), name="camera up")

    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = np.column_stack((right, up, back))
    result[:3, 3] = position
    return result


def front_view_head_pose(
    robot_position: np.ndarray,
    robot_forward: np.ndarray,
    *,
    distance_m: float = 2.2,
    eye_height_m: float = 1.35,
    target_height_m: float = 0.95,
) -> np.ndarray:
    """Construct the neutral third-person pose in front of the robot."""

    if distance_m <= 0:
        raise ValueError("front-view distance must be positive")
    robot_position = np.asarray(robot_position, dtype=np.float64)
    heading = np.asarray(robot_forward, dtype=np.float64).copy()
    heading[2] = 0.0
    heading = _unit(heading, name="robot horizontal heading")
    position = robot_position + heading * float(distance_m)
    position[2] = float(eye_height_m)
    target = robot_position.copy()
    target[2] = float(target_height_m)
    return look_at_head_pose(position, target)


def relative_hmd_pose(neutral_room_head: np.ndarray, current_room_head: np.ndarray) -> np.ndarray:
    """Express the current tracked head pose relative to its startup pose."""

    return rigid_inverse(neutral_room_head) @ np.asarray(current_room_head, dtype=np.float64)


def camera_vectors(world_eye: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return MuJoCo camera position, forward and up from a world eye pose."""

    world_eye = np.asarray(world_eye, dtype=np.float64)
    return (
        world_eye[:3, 3].copy(),
        -world_eye[:3, 2].copy(),
        world_eye[:3, 1].copy(),
    )


def projection_raw_to_mujoco(
    projection_raw: tuple[float, float, float, float],
    *,
    near_m: float,
    far_m: float,
) -> dict[str, float]:
    """Convert OpenVR projection tangents to ``mjvGLCamera`` frustum fields.

    OpenVR reports its upper edge as a negative tangent and its lower edge as
    a positive tangent. MuJoCo expects conventional signed near-plane
    coordinates, so OpenVR ``top`` becomes the lower coordinate and OpenVR
    ``bottom`` becomes the upper coordinate. This mapping reproduces the
    asymmetric vertical offset in ``IVRSystem::GetProjectionMatrix``.
    """

    if not 0 < near_m < far_m:
        raise ValueError("expected 0 < near plane < far plane")
    left, right, top, bottom = (float(value) for value in projection_raw)
    return {
        "frustum_bottom": top * near_m,
        "frustum_top": bottom * near_m,
        "frustum_center": 0.5 * (left + right) * near_m,
        # MuJoCo passes this value to glFrustum as center +/- width, so it is
        # the horizontal half-width rather than the complete frustum span.
        "frustum_width": 0.5 * (right - left) * near_m,
        "frustum_near": near_m,
        "frustum_far": far_m,
    }


@dataclass(frozen=True)
class OpenVRViewerConfig:
    """Configuration for the front-facing Quest/SteamVR viewer."""

    view: str = "front"
    distance_m: float = 2.2
    eye_height_m: float = 1.35
    target_height_m: float = 0.95
    render_scale: float = 0.75
    near_m: float = 0.05
    far_m: float = 50.0
    msaa_samples: int = 2
    mirror_width: int = 800
    mirror_height: int = 700
    recenter_button: str = "right-thumbstick"
    initialization_timeout_s: float = 20.0

    def validate(self) -> None:
        if self.view != "front":
            raise ValueError(f"unsupported VR view {self.view!r}; expected 'front'")
        if self.distance_m <= 0:
            raise ValueError("VR front distance must be positive")
        if not 0.25 <= self.render_scale <= 1.5:
            raise ValueError("VR render scale must be between 0.25 and 1.5")
        if self.msaa_samples not in {0, 2, 4, 8}:
            raise ValueError("VR MSAA samples must be one of 0, 2, 4, 8")
        if not 0 < self.near_m < self.far_m:
            raise ValueError("invalid VR near/far planes")
        if self.mirror_width < 64 or self.mirror_height < 64:
            raise ValueError("VR mirror dimensions must be at least 64 pixels")
        if self.recenter_button not in {"right-thumbstick", "right-a", "right-b", "off"}:
            raise ValueError(
                "VR recenter button must be right-thumbstick, right-a, right-b, or off"
            )


class OpenVRMuJoCoViewer:
    """Render a MuJoCo model to SteamVR while mirroring one eye on desktop.

    The first valid HMD pose becomes neutral.  At that instant the virtual head
    is placed in front of the G1, facing its torso.  Subsequent physical HMD
    motion is applied relative to that neutral pose.  Press ``R`` in the mirror
    window to recenter and ``Esc`` to close the viewer.
    """

    def __init__(self, model: Any, data: Any, config: OpenVRViewerConfig | None = None) -> None:
        self.model = model
        self.config = config or OpenVRViewerConfig()
        self.config.validate()

        self._snapshot_lock = threading.Lock()
        self._qpos = np.asarray(data.qpos, dtype=np.float64).copy()
        self._qvel = np.asarray(data.qvel, dtype=np.float64).copy()
        self._mocap_pos = np.asarray(data.mocap_pos, dtype=np.float64).copy()
        self._mocap_quat = np.asarray(data.mocap_quat, dtype=np.float64).copy()
        self._sim_time = float(data.time)

        self._ready = threading.Event()
        self._stop = threading.Event()
        self._recenter = PendingSignal()
        self._error: BaseException | None = None
        self._error_traceback = ""
        self._thread = threading.Thread(target=self._thread_main, name="twist2-openvr-render", daemon=True)
        self._thread.start()

        if not self._ready.wait(self.config.initialization_timeout_s):
            self._stop.set()
            # waitGetPoses normally returns at the compositor cadence. Give the
            # render thread a bounded opportunity to observe the stop request
            # so a failed constructor does not silently retain VR/GL resources.
            self._thread.join(timeout=5.0)
            if self._thread.is_alive():
                raise RuntimeError(
                    "timed out while initializing the OpenVR MuJoCo viewer; "
                    "the render thread also failed to stop within 5 seconds"
                )
            self.raise_if_failed()
            raise RuntimeError("timed out while initializing the OpenVR MuJoCo viewer")
        self.raise_if_failed()

    def update_state(self, data: Any) -> None:
        """Publish a lightweight visual state snapshot to the render thread."""

        with self._snapshot_lock:
            np.copyto(self._qpos, data.qpos)
            np.copyto(self._qvel, data.qvel)
            if self._mocap_pos.size:
                np.copyto(self._mocap_pos, data.mocap_pos)
                np.copyto(self._mocap_quat, data.mocap_quat)
            self._sim_time = float(data.time)

    def recenter(self) -> None:
        """Request a neutral front-facing pose on the next render frame."""

        self._recenter.request()

    def is_running(self) -> bool:
        return self._thread.is_alive() and not self._stop.is_set() and self._error is None

    def raise_if_failed(self) -> None:
        if self._error is not None:
            detail = f"\n{self._error_traceback}" if self._error_traceback else ""
            raise RuntimeError(f"OpenVR MuJoCo viewer failed: {self._error}{detail}") from self._error

    def close(self) -> None:
        self._stop.set()
        if self._thread.is_alive() and threading.current_thread() is not self._thread:
            self._thread.join(timeout=5.0)
        if self._thread.is_alive():
            raise RuntimeError("OpenVR render thread did not stop within 5 seconds")
        self.raise_if_failed()

    def _copy_snapshot(self, render_data: Any) -> None:
        with self._snapshot_lock:
            np.copyto(render_data.qpos, self._qpos)
            np.copyto(render_data.qvel, self._qvel)
            if self._mocap_pos.size:
                np.copyto(render_data.mocap_pos, self._mocap_pos)
                np.copyto(render_data.mocap_quat, self._mocap_quat)
            render_data.time = self._sim_time

    def _thread_main(self) -> None:
        try:
            self._render_loop()
        except BaseException as error:  # propagate renderer failures to the controller thread
            self._error = error
            self._error_traceback = traceback.format_exc()
            self._stop.set()
        finally:
            self._ready.set()

    def _render_loop(self) -> None:  # noqa: C901 - resource ownership is intentionally kept together
        import glfw
        import mujoco
        import openvr
        from OpenGL import GL

        vr_system = None
        compositor = None
        window = None
        context = None
        scene = None
        submit_texture = 0
        submit_fbo = 0
        glfw_initialized = False

        try:
            vr_system = openvr.init(openvr.VRApplication_Scene)
            compositor = openvr.VRCompositor()
            if compositor is None:
                raise RuntimeError("SteamVR compositor is unavailable")
            compositor.setTrackingSpace(openvr.TrackingUniverseStanding)

            recommended_width, recommended_height = vr_system.getRecommendedRenderTargetSize()
            projection_raw = [
                tuple(float(value) for value in vr_system.getProjectionRaw(openvr.Eye_Left)),
                tuple(float(value) for value in vr_system.getProjectionRaw(openvr.Eye_Right)),
            ]
            eye_height = max(1, int(round(recommended_height * self.config.render_scale)))
            # MuJoCo derives horizontal frustum extent from viewport aspect.
            # Match that aspect to OpenVR's raw tangent extents rather than
            # blindly using the recommended texture aspect.
            fov_aspects = [
                (right - left) / (bottom - top)
                for left, right, top, bottom in projection_raw
            ]
            eye_width = max(1, int(round(eye_height * float(np.mean(fov_aspects)))))
            stereo_width = 2 * eye_width

            if not glfw.init():
                raise RuntimeError("GLFW initialization failed; check DISPLAY/XAUTHORITY")
            glfw_initialized = True
            glfw.window_hint(glfw.SAMPLES, 0)
            glfw.window_hint(glfw.DOUBLEBUFFER, glfw.TRUE)
            glfw.window_hint(glfw.RESIZABLE, glfw.TRUE)
            window = glfw.create_window(
                self.config.mirror_width,
                self.config.mirror_height,
                "TWIST2 MuJoCo VR - front view (controller/R: recenter, Esc: quit)",
                None,
                None,
            )
            if not window:
                raise RuntimeError("could not create the OpenGL mirror window")
            glfw.make_context_current(window)
            glfw.swap_interval(0)

            def on_key(_window: Any, key: int, _scan: int, action: int, _mods: int) -> None:
                if action != glfw.PRESS:
                    return
                if key == glfw.KEY_ESCAPE:
                    glfw.set_window_should_close(_window, True)
                elif key == glfw.KEY_R:
                    self._recenter.request()

            glfw.set_key_callback(window, on_key)

            # The side-by-side offscreen buffer is copied once into a GL texture;
            # OpenVR receives its left/right halves through texture bounds.
            self.model.vis.global_.offwidth = stereo_width
            self.model.vis.global_.offheight = eye_height
            self.model.vis.quality.offsamples = self.config.msaa_samples

            render_data = mujoco.MjData(self.model)
            option = mujoco.MjvOption()
            abstract_camera = mujoco.MjvCamera()
            abstract_camera.distance = self.config.distance_m
            scene = mujoco.MjvScene(self.model, maxgeom=10_000)
            context = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_100)

            submit_texture = int(GL.glGenTextures(1))
            GL.glBindTexture(GL.GL_TEXTURE_2D, submit_texture)
            GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_LINEAR)
            GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_LINEAR)
            GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_S, GL.GL_CLAMP_TO_EDGE)
            GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_T, GL.GL_CLAMP_TO_EDGE)
            GL.glTexImage2D(
                GL.GL_TEXTURE_2D,
                0,
                GL.GL_RGBA8,
                stereo_width,
                eye_height,
                0,
                GL.GL_RGBA,
                GL.GL_UNSIGNED_BYTE,
                None,
            )
            submit_fbo = int(GL.glGenFramebuffers(1))
            GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, submit_fbo)
            GL.glFramebufferTexture2D(
                GL.GL_FRAMEBUFFER,
                GL.GL_COLOR_ATTACHMENT0,
                GL.GL_TEXTURE_2D,
                submit_texture,
                0,
            )
            if GL.glCheckFramebufferStatus(GL.GL_FRAMEBUFFER) != GL.GL_FRAMEBUFFER_COMPLETE:
                raise RuntimeError("OpenVR submit framebuffer is incomplete")
            GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)

            eye_to_head = [
                matrix34_to_transform(vr_system.getEyeToHeadTransform(openvr.Eye_Left)),
                matrix34_to_transform(vr_system.getEyeToHeadTransform(openvr.Eye_Right)),
            ]
            frustums = [
                projection_raw_to_mujoco(
                    projection_raw[0],
                    near_m=self.config.near_m,
                    far_m=self.config.far_m,
                ),
                projection_raw_to_mujoco(
                    projection_raw[1],
                    near_m=self.config.near_m,
                    far_m=self.config.far_m,
                ),
            ]
            texture = openvr.Texture_t(submit_texture, openvr.TextureType_OpenGL, openvr.ColorSpace_Gamma)
            left_bounds = openvr.VRTextureBounds_t(0.0, 0.0, 0.5, 1.0)
            right_bounds = openvr.VRTextureBounds_t(0.5, 0.0, 1.0, 1.0)
            poses_type = openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount
            poses = poses_type()

            neutral_room_head: np.ndarray | None = None
            initial_root_position: np.ndarray | None = None
            initial_heading: np.ndarray | None = None
            hmd_was_valid = False
            last_status_time = 0.0
            submit_gl_warning_shown = False
            recenter_edge = ReleaseArmedButton()
            button_bindings = {
                "right-thumbstick": ("right", openvr.k_EButton_Axis0, "right thumbstick"),
                "right-a": ("right", openvr.k_EButton_A, "right A"),
                "right-b": ("right", openvr.k_EButton_ApplicationMenu, "right B"),
            }
            recenter_binding = button_bindings.get(self.config.recenter_button)
            controller_hint = (
                f"Click {recenter_binding[2]} or press R to recenter."
                if recenter_binding is not None
                else "Press R to recenter (controller recenter is disabled)."
            )
            self._ready.set()

            print(
                "OpenVR MuJoCo viewer ready: "
                f"{eye_width}x{eye_height} per eye, front view {self.config.distance_m:.2f}m. "
                f"{controller_hint}"
            )

            while not self._stop.is_set() and not glfw.window_should_close(window):
                compositor.waitGetPoses(poses, None)
                hmd_pose = poses[openvr.k_unTrackedDeviceIndex_Hmd]
                hmd_valid = bool(hmd_pose.bDeviceIsConnected and hmd_pose.bPoseIsValid)

                controller_recenter = False
                if recenter_binding is not None:
                    side, button_id, _button_label = recenter_binding
                    button_sample = poll_controller_button(
                        vr_system,
                        openvr,
                        poses,
                        side=side,
                        button_id=button_id,
                    )
                    controller_recenter = recenter_edge.update(button_sample)
                keyboard_or_api_recenter = self._recenter.take()
                recenter_requested = controller_recenter or keyboard_or_api_recenter
                if recenter_requested and not hmd_valid:
                    print("Recenter ignored because the Quest HMD pose is not valid.")
                    recenter_requested = False

                self._copy_snapshot(render_data)
                mujoco.mj_forward(self.model, render_data)

                root_position = np.asarray(render_data.qpos[:3], dtype=np.float64).copy()
                root_rotation = np.empty(9, dtype=np.float64)
                mujoco.mju_quat2Mat(root_rotation, render_data.qpos[3:7])
                root_rotation = root_rotation.reshape(3, 3)
                if initial_heading is None:
                    initial_root_position = root_position.copy()
                    initial_heading = root_rotation[:, 0].copy()

                neutral_world_head = front_view_head_pose(
                    initial_root_position,
                    initial_heading,
                    distance_m=self.config.distance_m,
                    eye_height_m=self.config.eye_height_m,
                    target_height_m=self.config.target_height_m,
                )

                if hmd_valid:
                    current_room_head = matrix34_to_transform(hmd_pose.mDeviceToAbsoluteTracking)
                    if neutral_room_head is None or recenter_requested:
                        neutral_room_head = current_room_head.copy()
                        source = "controller" if controller_recenter else "HMD startup/R key"
                        print(
                            "OpenVR view recentered: G1 is directly in front of the user "
                            f"({source})."
                        )
                    world_head = neutral_world_head @ relative_hmd_pose(neutral_room_head, current_room_head)
                    hmd_was_valid = True
                else:
                    world_head = neutral_world_head
                    now = time.monotonic()
                    if hmd_was_valid or now - last_status_time > 2.0:
                        print("Waiting for a valid Quest HMD pose; mirror remains at the neutral front view.")
                        last_status_time = now
                    hmd_was_valid = False

                mujoco.mjv_updateScene(
                    self.model,
                    render_data,
                    option,
                    None,
                    abstract_camera,
                    mujoco.mjtCatBit.mjCAT_ALL,
                    scene,
                )
                scene.stereo = mujoco.mjtStereo.mjSTEREO_SIDEBYSIDE
                for eye_index in range(2):
                    world_eye = world_head @ eye_to_head[eye_index]
                    position, forward, up = camera_vectors(world_eye)
                    camera = scene.camera[eye_index]
                    camera.pos[:] = position.astype(np.float32)
                    camera.forward[:] = forward.astype(np.float32)
                    camera.up[:] = up.astype(np.float32)
                    camera.orthographic = 0
                    for field, value in frustums[eye_index].items():
                        setattr(camera, field, value)

                mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, context)
                if context.currentBuffer != mujoco.mjtFramebuffer.mjFB_OFFSCREEN:
                    raise RuntimeError("MuJoCo offscreen framebuffer is unavailable")
                viewport = mujoco.MjrRect(0, 0, stereo_width, eye_height)
                mujoco.mjr_render(viewport, scene, context)

                source_fbo = int(context.offFBO)
                if self.config.msaa_samples > 0 and int(context.offFBO_r):
                    GL.glBindFramebuffer(GL.GL_READ_FRAMEBUFFER, int(context.offFBO))
                    GL.glReadBuffer(GL.GL_COLOR_ATTACHMENT0)
                    GL.glBindFramebuffer(GL.GL_DRAW_FRAMEBUFFER, int(context.offFBO_r))
                    GL.glDrawBuffer(GL.GL_COLOR_ATTACHMENT0)
                    GL.glBlitFramebuffer(
                        0, 0, stereo_width, eye_height,
                        0, 0, stereo_width, eye_height,
                        GL.GL_COLOR_BUFFER_BIT,
                        GL.GL_NEAREST,
                    )
                    source_fbo = int(context.offFBO_r)

                GL.glBindFramebuffer(GL.GL_READ_FRAMEBUFFER, source_fbo)
                GL.glReadBuffer(GL.GL_COLOR_ATTACHMENT0)
                GL.glBindFramebuffer(GL.GL_DRAW_FRAMEBUFFER, submit_fbo)
                GL.glDrawBuffer(GL.GL_COLOR_ATTACHMENT0)
                GL.glBlitFramebuffer(
                    0, 0, stereo_width, eye_height,
                    0, 0, stereo_width, eye_height,
                    GL.GL_COLOR_BUFFER_BIT,
                    GL.GL_NEAREST,
                )

                # Mirror only the left eye.  This makes the desktop window useful
                # for supervision while the headset receives both eye textures.
                mirror_width, mirror_height = glfw.get_framebuffer_size(window)
                GL.glBindFramebuffer(GL.GL_READ_FRAMEBUFFER, submit_fbo)
                GL.glReadBuffer(GL.GL_COLOR_ATTACHMENT0)
                GL.glBindFramebuffer(GL.GL_DRAW_FRAMEBUFFER, 0)
                GL.glDrawBuffer(GL.GL_BACK)
                GL.glBlitFramebuffer(
                    0, 0, eye_width, eye_height,
                    0, 0, mirror_width, mirror_height,
                    GL.GL_COLOR_BUFFER_BIT,
                    GL.GL_LINEAR,
                )

                # SteamVR's Linux OpenGL submit path mutates GL state and can
                # leave a driver-side error flag even after accepting a frame.
                # First validate/flush our own work, then unbind the shared
                # texture, submit, and drain only errors produced inside the
                # successful compositor calls.
                GL.glBindFramebuffer(GL.GL_READ_FRAMEBUFFER, 0)
                GL.glBindFramebuffer(GL.GL_DRAW_FRAMEBUFFER, 0)
                GL.glBindTexture(GL.GL_TEXTURE_2D, 0)
                GL.glFlush()
                compositor.submit(openvr.Eye_Left, texture, left_bounds)
                compositor.submit(openvr.Eye_Right, texture, right_bounds)
                compositor_gl_errors = []
                while True:
                    gl_error = int(GL.glGetError())
                    if gl_error == int(GL.GL_NO_ERROR):
                        break
                    compositor_gl_errors.append(gl_error)
                unexpected_gl_errors = [
                    code
                    for code in compositor_gl_errors
                    if code != int(GL.GL_INVALID_VALUE)
                ]
                if unexpected_gl_errors:
                    raise RuntimeError(
                        "SteamVR frame submission left unexpected OpenGL "
                        f"error flag(s): {unexpected_gl_errors}"
                    )
                if compositor_gl_errors and not submit_gl_warning_shown:
                    print(
                        "SteamVR accepted the frame; cleared its known Linux "
                        "GL_INVALID_VALUE residue."
                    )
                    submit_gl_warning_shown = True
                GL.glFlush()
                glfw.swap_buffers(window)
                glfw.poll_events()

            self._stop.set()
        finally:
            # All GL and OpenVR resources are released on their owning thread.
            if window is not None:
                try:
                    glfw.make_context_current(window)
                except Exception:
                    pass
                if compositor is not None:
                    try:
                        compositor.clearLastSubmittedFrame()
                    except Exception:
                        pass
                try:
                    GL.glFinish()
                except Exception:
                    pass
                if submit_fbo:
                    try:
                        GL.glDeleteFramebuffers(1, [submit_fbo])
                    except Exception:
                        pass
                if submit_texture:
                    try:
                        GL.glDeleteTextures([submit_texture])
                    except Exception:
                        pass
                if context is not None:
                    try:
                        context.free()
                    except Exception:
                        pass
                try:
                    glfw.destroy_window(window)
                except Exception:
                    pass
            if glfw_initialized:
                try:
                    glfw.terminate()
                except Exception:
                    pass
            if vr_system is not None:
                try:
                    openvr.shutdown()
                except Exception:
                    pass
