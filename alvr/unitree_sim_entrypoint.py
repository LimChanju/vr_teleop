#!/usr/bin/env python3
"""Run the pinned Unitree simulator with an offline-safe scene overlay."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import runpy
import sys
from typing import Any


REMOTE_SCHEMES = ("http://", "https://", "omniverse://")
CAMERA_CONFIG = Path(__file__).with_name("sim_camera_config.yaml")
DISABLED_WEBRTC_WARNING = "[IsaacSimCamera] Failed to encode to WebRTC"


def should_suppress_image_warning(message: Any) -> bool:
    return str(message).startswith(DISABLED_WEBRTC_WARNING)


def prepare_environment_config(config: Any, *, full_scene: bool) -> list[str]:
    """Apply startup-only changes without modifying the pinned vendor checkout."""
    changes: list[str] = []

    # Isaac Lab's default is an unbounded render loop while texture assets are
    # reported as loading.  Cameras continue rendering after startup, so it is
    # safer for an interactive launcher to allow the first frames to settle
    # than to block controller startup forever on an unreachable texture.
    if getattr(config, "wait_for_textures", None) is not False:
        config.wait_for_textures = False
        changes.append("bounded texture startup")

    if full_scene:
        return changes

    scene = getattr(config, "scene", None)
    if scene is None:
        return changes

    # The fixed-base cylinder task references NVIDIA's public S3 warehouse,
    # while all task-critical assets are local.  Campus networks can leave its
    # material requests pending indefinitely, so omit only remote scenery.
    room = getattr(scene, "room_walls", None)
    spawn = getattr(room, "spawn", None)
    usd_path = str(getattr(spawn, "usd_path", ""))
    if usd_path.startswith(REMOTE_SCHEMES):
        scene.room_walls = None
        changes.append("remote room omitted")

    # Unitree's image server publishes the head and two wrist cameras.  The
    # spectator cameras are not consumed, despite the upstream exclude default
    # being applied too late (after gym.make) to prevent their creation.
    for camera_name in ("world_camera", "robot_camera"):
        if getattr(scene, camera_name, None) is not None:
            setattr(scene, camera_name, None)
            changes.append(f"unused {camera_name} omitted")

    return changes


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--upstream-script", type=Path, required=True)
    parser.add_argument(
        "--full-scene",
        action="store_true",
        help="keep remote scenery and unused spectator cameras",
    )
    return parser


def main() -> None:
    args, upstream_args = build_parser().parse_known_args()
    upstream_script = args.upstream_script.expanduser().resolve()
    if not upstream_script.is_file():
        raise SystemExit(f"Unitree simulator entrypoint not found: {upstream_script}")

    upstream_root = str(upstream_script.parent)
    inserted_path = not sys.path or sys.path[0] != upstream_root
    if inserted_path:
        sys.path.insert(0, upstream_root)

    # Unitree imports these modules before it creates AppLauncher, so installing
    # these narrow wrappers preserves the upstream import order.
    import gymnasium as gym
    import teleimager.image_server as image_server_module

    upstream_make = gym.make
    upstream_image_server = image_server_module.run_isaacsim_server
    upstream_camera_config = image_server_module.CONFIG_PATH
    upstream_image_warning = image_server_module.logger_mp.warning
    if not CAMERA_CONFIG.is_file():
        raise SystemExit(f"ALVR simulator camera config not found: {CAMERA_CONFIG}")
    image_server_module.CONFIG_PATH = str(CAMERA_CONFIG)

    def filtered_image_warning(message: Any, *warning_args: Any, **warning_kwargs: Any) -> Any:
        # Upstream logs this as a warning every frame when WebRTC is disabled;
        # it actually means the optional output was intentionally skipped.
        if should_suppress_image_warning(message):
            return None
        return upstream_image_warning(message, *warning_args, **warning_kwargs)

    image_server_module.logger_mp.warning = filtered_image_warning

    def configured_make(*make_args: Any, **make_kwargs: Any) -> Any:
        config = make_kwargs.get("cfg")
        if config is not None:
            changes = prepare_environment_config(config, full_scene=args.full_scene)
            if changes:
                print(f"[ALVR simulator] {', '.join(changes)}", flush=True)
        return upstream_make(*make_args, **make_kwargs)

    gym.make = configured_make

    def checked_image_server() -> Any:
        config_path = Path(image_server_module.CONFIG_PATH)
        print(
            f"[ALVR simulator] image config: {config_path} "
            f"(readable={config_path.is_file() and os.access(config_path, os.R_OK)})",
            flush=True,
        )
        try:
            return upstream_image_server()
        except BaseException as error:
            print(
                f"[ALVR simulator] image server startup failed: "
                f"{type(error).__name__}: {error}",
                flush=True,
            )
            raise

    image_server_module.run_isaacsim_server = checked_image_server
    sys.argv = [str(upstream_script), *upstream_args]
    try:
        runpy.run_path(str(upstream_script), run_name="__main__")
    finally:
        gym.make = upstream_make
        image_server_module.run_isaacsim_server = upstream_image_server
        image_server_module.CONFIG_PATH = upstream_camera_config
        image_server_module.logger_mp.warning = upstream_image_warning
        if inserted_path:
            try:
                sys.path.remove(upstream_root)
            except ValueError:
                pass


if __name__ == "__main__":
    main()
