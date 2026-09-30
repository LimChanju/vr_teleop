"""Pure host-side tests for the Unitree task catalog and launch commands."""

from __future__ import annotations

import io
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np


ALVR_ROOT = Path(__file__).resolve().parents[1]
if str(ALVR_ROOT) not in sys.path:
    sys.path.insert(0, str(ALVR_ROOT))

import unitree_task_launcher as launcher
import unitree_sim_entrypoint as sim_entrypoint
import unitree_tasks as tasks
import unitree_teleop_entrypoint as entrypoint
from arm_safety import ArmCommandSafety


EXPECTED_TASKS = {
    "pick-cylinder-dex1": ("Isaac-PickPlace-Cylinder-G129-Dex1-Joint", "dex1", False),
    "pick-cylinder-dex3": ("Isaac-PickPlace-Cylinder-G129-Dex3-Joint", "dex3", False),
    "pick-cylinder-inspire": ("Isaac-PickPlace-Cylinder-G129-Inspire-Joint", "inspire", False),
    "pick-redblock-dex1": ("Isaac-PickPlace-RedBlock-G129-Dex1-Joint", "dex1", False),
    "pick-redblock-dex3": ("Isaac-PickPlace-RedBlock-G129-Dex3-Joint", "dex3", False),
    "pick-redblock-inspire": ("Isaac-PickPlace-RedBlock-G129-Inspire-Joint", "inspire", False),
    "stack-rgy-dex1": ("Isaac-Stack-RgyBlock-G129-Dex1-Joint", "dex1", False),
    "stack-rgy-dex3": ("Isaac-Stack-RgyBlock-G129-Dex3-Joint", "dex3", False),
    "stack-rgy-inspire": ("Isaac-Stack-RgyBlock-G129-Inspire-Joint", "inspire", False),
    "drawer-dex1": ("Isaac-Pick-Redblock-Into-Drawer-G129-Dex1-Joint", "dex1", False),
    "drawer-dex3": ("Isaac-Pick-Redblock-Into-Drawer-G129-Dex3-Joint", "dex3", False),
    "move-cylinder-dex1": ("Isaac-Move-Cylinder-G129-Dex1-Wholebody", "dex1", True),
    "move-cylinder-dex3": ("Isaac-Move-Cylinder-G129-Dex3-Wholebody", "dex3", True),
    "move-cylinder-inspire": ("Isaac-Move-Cylinder-G129-Inspire-Wholebody", "inspire", True),
}


def option_value(command: list[str], option: str) -> str:
    return command[command.index(option) + 1]


class TaskCatalogTests(unittest.TestCase):
    def test_catalog_has_all_fourteen_unique_g1_tasks(self):
        self.assertEqual(len(tasks.TASKS), 14)
        self.assertEqual(len({task.alias for task in tasks.TASKS}), 14)
        self.assertEqual(len({task.task_id for task in tasks.TASKS}), 14)
        actual = {
            task.alias: (task.task_id, task.end_effector, task.wholebody)
            for task in tasks.TASKS
        }
        self.assertEqual(actual, EXPECTED_TASKS)

    def test_catalog_classifies_native_and_synergy_end_effectors(self):
        groups = {
            end_effector: [task for task in tasks.TASKS if task.end_effector == end_effector]
            for end_effector in ("dex1", "dex3", "inspire")
        }
        self.assertEqual({name: len(group) for name, group in groups.items()}, {
            "dex1": 5,
            "dex3": 5,
            "inspire": 4,
        })

        for task in groups["dex1"]:
            self.assertEqual(task.sim_dds_flag, "--enable_dex1_dds")
            self.assertEqual(task.teleop_input_mode, "controller")
            self.assertEqual(task.teleop_end_effector, "dex1")
            self.assertFalse(task.experimental_synergy)
            self.assertEqual(task.support_label, "native-controller")

        for task in groups["dex3"]:
            self.assertEqual(task.sim_dds_flag, "--enable_dex3_dds")
            self.assertEqual(task.teleop_input_mode, "hand")
            self.assertEqual(task.teleop_end_effector, "dex3")
            self.assertTrue(task.experimental_synergy)

        for task in groups["inspire"]:
            self.assertEqual(task.sim_dds_flag, "--enable_inspire_dds")
            self.assertEqual(task.teleop_input_mode, "hand")
            self.assertEqual(task.teleop_end_effector, "inspire_dfx")
            self.assertTrue(task.experimental_synergy)

    def test_task_can_be_resolved_by_alias_or_exact_id(self):
        for expected in tasks.TASKS:
            self.assertIs(tasks.resolve_task(expected.alias), expected)
            self.assertIs(tasks.resolve_task(expected.task_id), expected)
        with self.assertRaisesRegex(ValueError, "unknown G1 task"):
            tasks.resolve_task("not-a-task")

    def test_task_rows_expose_all_catalog_entries(self):
        rows = tasks.task_rows()
        self.assertEqual(len(rows), 14)
        self.assertEqual({row[0] for row in rows}, set(EXPECTED_TASKS))
        wholebody_aliases = {row[0] for row in rows if row[1] == "wholebody"}
        self.assertEqual(
            wholebody_aliases,
            {"move-cylinder-dex1", "move-cylinder-dex3", "move-cylinder-inspire"},
        )


class LaunchCommandTests(unittest.TestCase):
    def setUp(self):
        self.xr_root = Path("/tmp/test-xr-teleoperate")
        self.sim_root = Path("/tmp/test-unitree-sim")
        self.args = launcher.build_parser().parse_args(
            [
                "print",
                "pick-cylinder-dex1",
                "--xr-root",
                str(self.xr_root),
                "--sim-root",
                str(self.sim_root),
                "--sim-python",
                "conda run -n unitree-sim python",
                "--teleop-python",
                "python -u",
                "--device",
                "cuda:0",
                "--img-server-ip",
                "10.0.0.2",
                "--frequency",
                "60",
                "--record",
                "--headless-teleop",
                "--sim-arg=--no_render",
                "--teleop-arg=--ipc",
            ]
        )

    def test_simulator_command_selects_exactly_one_hand_dds_backend(self):
        all_dds_flags = {
            "--enable_dex1_dds",
            "--enable_dex3_dds",
            "--enable_inspire_dds",
        }
        for task in tasks.TASKS:
            with self.subTest(task=task.alias):
                command = launcher.build_sim_command(self.args, task)
                self.assertEqual(
                    command[:6],
                    ["conda", "run", "--no-capture-output", "-n", "unitree-sim", "python"],
                )
                self.assertEqual(option_value(command, "--task"), task.task_id)
                self.assertEqual(
                    option_value(command, "--upstream-script"),
                    str((self.sim_root / "sim_main.py").resolve()),
                )
                self.assertIn(str((ALVR_ROOT / "unitree_sim_entrypoint.py").resolve()), command)
                self.assertEqual(option_value(command, "--robot_type"), "g129")
                self.assertEqual(option_value(command, "--device"), "cuda:0")
                self.assertIn("--enable_cameras", command)
                self.assertEqual(set(command) & all_dds_flags, {task.sim_dds_flag})
                self.assertEqual(command[-1], "--no_render")

    def test_wholebody_simulator_flags_are_only_added_to_wholebody_tasks(self):
        for task in tasks.TASKS:
            with self.subTest(task=task.alias):
                command = launcher.build_sim_command(self.args, task)
                if task.wholebody:
                    self.assertEqual(option_value(command, "--action_source"), "dds_wholebody")
                    self.assertIn("--enable_wholebody_dds", command)
                    self.assertEqual(option_value(command, "--model_path"), "assets/model/policy.onnx")
                else:
                    self.assertNotIn("--action_source", command)
                    self.assertNotIn("--enable_wholebody_dds", command)
                    self.assertNotIn("--model_path", command)

    def test_teleop_command_matches_each_task_contract(self):
        for task in tasks.TASKS:
            with self.subTest(task=task.alias):
                command = launcher.build_teleop_command(self.args, task)
                self.assertEqual(command[:2], ["python", "-u"])
                self.assertEqual(option_value(command, "--input-mode"), task.teleop_input_mode)
                self.assertEqual(option_value(command, "--arm"), "G1_29")
                self.assertEqual(option_value(command, "--ee"), task.teleop_end_effector)
                self.assertEqual(option_value(command, "--frequency"), "60.0")
                self.assertEqual(option_value(command, "--img-server-ip"), "10.0.0.2")
                self.assertEqual(option_value(command, "--task-name"), task.alias)
                self.assertEqual(option_value(command, "--task-goal"), task.task_id)
                self.assertIn("--sim", command)
                self.assertIn("--record", command)
                self.assertIn("--headless", command)
                self.assertIn("--ipc", command)
                self.assertNotIn("--motion", command)

    def test_teleop_environment_selects_shim_and_wholebody_mode(self):
        fixed = tasks.resolve_task("pick-cylinder-dex1")
        wholebody = tasks.resolve_task("move-cylinder-dex3")
        with patch.dict(os.environ, {"PYTHONPATH": "/existing/pythonpath"}, clear=True):
            fixed_environment = launcher.teleop_environment(fixed)
            wholebody_environment = launcher.teleop_environment(wholebody)

        expected_prefix = [
            str((ALVR_ROOT / "shim").resolve()),
            str(ALVR_ROOT.resolve()),
        ]
        self.assertEqual(fixed_environment["PYTHONPATH"].split(os.pathsep)[:2], expected_prefix)
        self.assertEqual(fixed_environment["ALVR_WHOLEBODY"], "0")
        self.assertEqual(wholebody_environment["ALVR_WHOLEBODY"], "1")
        self.assertEqual(wholebody_environment["ALVR_TASK_ID"], wholebody.task_id)
        self.assertEqual(wholebody_environment["ALVR_END_EFFECTOR"], "dex3")
        self.assertTrue(Path(wholebody_environment["ALVR_ACTION_MANIFEST"]).is_absolute())

    def test_sim_environment_forces_utf8_for_upstream_camera_yaml(self):
        with patch.dict(os.environ, {"PYTHONUTF8": "0", "PYTHONIOENCODING": "ascii"}):
            environment = launcher.sim_environment(self.args)

        self.assertEqual(environment["PYTHONUTF8"], "1")
        self.assertEqual(environment["PYTHONIOENCODING"], "utf-8")
        self.assertEqual(environment["PYTHONUNBUFFERED"], "1")
        self.assertEqual(environment["PROJECT_ROOT"], str(self.sim_root.resolve()))

    def test_managed_options_cannot_be_overridden_by_extra_args(self):
        conflicts = (
            ("--sim-arg=--task=wrong", "--task"),
            ("--sim-arg=--dev", "--dev"),
            ("--sim-arg=--acti", "--acti"),
            ("--sim-arg=--full-scene", "--full-scene"),
            ("--teleop-arg=--motion", "--motion"),
            ("--teleop-arg=--mo", "--mo"),
            ("--teleop-arg=--upst", "--upst"),
        )
        for argument, expected in conflicts:
            with self.subTest(argument=argument):
                args = launcher.build_parser().parse_args(
                    ["print", "pick-cylinder-dex1", argument]
                )
                with self.assertRaisesRegex(ValueError, expected):
                    launcher.validate_extra_args(args)

    def test_vendor_root_and_explicit_sdk_root_select_checkout_defaults(self):
        vendor_root = Path("/tmp/custom-unitree-vendor")
        with patch.dict(os.environ, {"VENDOR_ROOT": str(vendor_root)}, clear=True):
            args = launcher.build_parser().parse_args(["print", "pick-cylinder-dex1"])
        self.assertEqual(args.xr_root, vendor_root / "xr_teleoperate")
        self.assertEqual(args.sim_root, vendor_root / "unitree_sim_isaaclab")
        self.assertEqual(args.sdk_root, vendor_root / "unitree_sdk2_python")

        explicit_sdk = Path("/tmp/explicit-unitree-sdk")
        args = launcher.build_parser().parse_args(
            ["print", "pick-cylinder-dex1", "--sdk-root", str(explicit_sdk)]
        )
        self.assertEqual(args.sdk_root, explicit_sdk)

    def test_startup_timeout_allows_first_run_shader_warmup(self):
        args = launcher.build_parser().parse_args(["print", "pick-cylinder-dex1"])

        self.assertEqual(args.startup_timeout, 900.0)
        self.assertEqual(args.device, "cuda:0")

    def test_full_scene_flag_is_forwarded_to_simulator_adapter(self):
        args = launcher.build_parser().parse_args(
            ["print", "pick-cylinder-dex1", "--full-scene"]
        )

        self.assertIn("--full-scene", launcher.build_sim_command(args, tasks.TASKS[0]))

    def test_default_sim_python_uses_exec_wrapper_for_activation_hooks(self):
        with tempfile.TemporaryDirectory() as directory:
            conda_root = Path(directory)
            alvr_root = conda_root / "alvr"
            conda_executable = conda_root / "bin" / "conda"
            sim_python = conda_root / "envs" / "unitree_sim_env" / "bin" / "python"
            conda_wrapper = alvr_root / "scripts" / "conda_env_exec.sh"
            conda_executable.parent.mkdir(parents=True)
            sim_python.parent.mkdir(parents=True)
            conda_wrapper.parent.mkdir(parents=True)
            conda_executable.touch()
            sim_python.touch()
            conda_wrapper.touch(mode=0o755)
            with patch.dict(
                os.environ,
                {"CONDA_EXE": str(conda_executable)},
                clear=True,
            ), patch.object(launcher, "ROOT", alvr_root):
                command = launcher._default_python("UNITREE_SIM_PYTHON")

        self.assertEqual(
            launcher._command_parts(command),
            [
                str(conda_wrapper),
                "unitree_sim_env",
                "python",
            ],
        )

    def test_default_sim_python_fails_closed_when_exec_wrapper_is_missing(self):
        with tempfile.TemporaryDirectory() as directory:
            conda_root = Path(directory)
            alvr_root = conda_root / "alvr"
            conda_executable = conda_root / "bin" / "conda"
            sim_python = conda_root / "envs" / "unitree_sim_env" / "bin" / "python"
            conda_executable.parent.mkdir(parents=True)
            sim_python.parent.mkdir(parents=True)
            conda_executable.touch()
            sim_python.touch()
            with patch.dict(
                os.environ,
                {"CONDA_EXE": str(conda_executable)},
                clear=True,
            ), patch.object(launcher, "ROOT", alvr_root):
                command = launcher._default_python("UNITREE_SIM_PYTHON")

        self.assertEqual(
            launcher._command_parts(command),
            [
                str(alvr_root / "scripts" / "conda_env_exec.sh"),
                "unitree_sim_env",
                "python",
            ],
        )
        self.assertIn("not executable", launcher._executable_problem(command) or "")


class SimulatorEntrypointTests(unittest.TestCase):
    @staticmethod
    def config(room_path: str = "https://assets.example/warehouse.usd"):
        return SimpleNamespace(
            wait_for_textures=True,
            scene=SimpleNamespace(
                room_walls=SimpleNamespace(
                    spawn=SimpleNamespace(usd_path=room_path)
                ),
                world_camera=object(),
                robot_camera=object(),
                front_camera=object(),
                left_wrist_camera=object(),
                right_wrist_camera=object(),
                packing_table=object(),
                object=object(),
            ),
        )

    def test_offline_overlay_omits_only_remote_scenery_and_spectator_camera(self):
        config = self.config()

        changes = sim_entrypoint.prepare_environment_config(config, full_scene=False)

        self.assertFalse(config.wait_for_textures)
        self.assertIsNone(config.scene.room_walls)
        self.assertIsNone(config.scene.world_camera)
        self.assertIsNone(config.scene.robot_camera)
        self.assertIsNotNone(config.scene.front_camera)
        self.assertIsNotNone(config.scene.left_wrist_camera)
        self.assertIsNotNone(config.scene.right_wrist_camera)
        self.assertIsNotNone(config.scene.packing_table)
        self.assertIsNotNone(config.scene.object)
        self.assertEqual(
            changes,
            [
                "bounded texture startup",
                "remote room omitted",
                "unused world_camera omitted",
                "unused robot_camera omitted",
            ],
        )

    def test_offline_overlay_keeps_local_room(self):
        config = self.config("/local/assets/warehouse.usd")

        sim_entrypoint.prepare_environment_config(config, full_scene=False)

        self.assertIsNotNone(config.scene.room_walls)
        self.assertIsNone(config.scene.world_camera)
        self.assertIsNone(config.scene.robot_camera)

    def test_full_scene_only_bounds_texture_wait(self):
        config = self.config()

        changes = sim_entrypoint.prepare_environment_config(config, full_scene=True)

        self.assertFalse(config.wait_for_textures)
        self.assertIsNotNone(config.scene.room_walls)
        self.assertIsNotNone(config.scene.world_camera)
        self.assertIsNotNone(config.scene.robot_camera)
        self.assertEqual(changes, ["bounded texture startup"])

    def test_only_disabled_webrtc_noise_is_suppressed(self):
        self.assertTrue(
            sim_entrypoint.should_suppress_image_warning(
                "[IsaacSimCamera] Failed to encode to WebRTC for head_camera"
            )
        )
        self.assertFalse(
            sim_entrypoint.should_suppress_image_warning(
                "[IsaacSimCamera] Failed to encode JPEG for head_camera"
            )
        )


class LauncherPreflightTests(unittest.TestCase):
    def test_git_revision_check_rejects_wrong_head(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / ".git").mkdir()
            expected = "a" * 40
            completed = subprocess.CompletedProcess(
                args=["git"], returncode=0, stdout="b" * 40 + "\n", stderr=""
            )
            with patch.object(launcher.subprocess, "run", return_value=completed):
                problem = launcher._git_head_problem("checkout", root, expected)
        self.assertIn("revision mismatch", problem or "")
        self.assertIn(expected, problem or "")

    def test_preflight_checks_all_three_pinned_checkouts(self):
        args = launcher.build_parser().parse_args(["doctor", "pick-cylinder-dex1"])
        with (
            patch.object(launcher, "_git_head_problem", return_value=None) as revision_check,
            patch.object(launcher, "_executable_problem", return_value=None),
            patch.object(launcher, "_usable_asset", return_value=True),
        ):
            launcher.preflight(args, tasks.TASKS[0], scope="all", imports=False)

        self.assertEqual(
            {(call.args[0], call.args[1], call.args[2]) for call in revision_check.call_args_list},
            {
                ("xr_teleoperate", args.xr_root, launcher.XR_TELEOPERATE_REVISION),
                ("unitree_sim_isaaclab", args.sim_root, launcher.UNITREE_SIM_REVISION),
                ("unitree_sdk2_python", args.sdk_root, launcher.UNITREE_SDK_REVISION),
            },
        )

    def test_missing_sim_task_registry_is_an_explicit_error(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            args = launcher.build_parser().parse_args(
                [
                    "sim",
                    "pick-cylinder-dex1",
                    "--sim-root",
                    str(root / "sim"),
                    "--sdk-root",
                    str(root / "sdk"),
                ]
            )
            with (
                patch.object(launcher, "_git_head_problem", return_value=None),
                patch.object(launcher, "_executable_problem", return_value=None),
                patch.object(launcher, "_usable_asset", return_value=True),
            ):
                problems = launcher.preflight(args, tasks.TASKS[0], scope="sim", imports=False)
        self.assertTrue(any("sim task registry missing" in problem for problem in problems))

    def test_dex1_preflight_also_requires_dex_retargeting_source(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            args = launcher.build_parser().parse_args(
                [
                    "teleop",
                    "pick-cylinder-dex1",
                    "--xr-root",
                    str(root / "xr"),
                    "--sdk-root",
                    str(root / "sdk"),
                ]
            )
            with (
                patch.object(launcher, "_git_head_problem", return_value=None),
                patch.object(launcher, "_executable_problem", return_value=None),
            ):
                problems = launcher.preflight(args, tasks.TASKS[0], scope="teleop", imports=False)
        self.assertTrue(any("xr dex-retargeting submodule missing" in problem for problem in problems))

    def test_preflight_uses_verified_safety_overlay(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            upstream = root / "xr" / "teleop" / "teleop_hand_and_arm.py"
            upstream.parent.mkdir(parents=True)
            upstream.write_text("pinned source", encoding="utf-8")
            args = launcher.build_parser().parse_args(
                [
                    "teleop",
                    "pick-cylinder-dex1",
                    "--xr-root",
                    str(root / "xr"),
                    "--sdk-root",
                    str(root / "sdk"),
                ]
            )
            with (
                patch.object(launcher, "_git_head_problem", return_value=None),
                patch.object(launcher, "_executable_problem", return_value=None),
                patch.object(entrypoint, "verified_safety_overlay") as verified_overlay,
            ):
                launcher.preflight(args, tasks.TASKS[0], scope="teleop", imports=False)
        verified_overlay.assert_called_once_with("pinned source")

    def test_teleop_import_probe_loads_real_image_and_controller_modules(self):
        expected_controller = {
            "dex1": "Dex1_1_Gripper_Controller",
            "dex3": "Dex3_1_Controller",
            "inspire": "Inspire_Controller_DFX",
        }
        for task in tasks.TASKS:
            with self.subTest(task=task.alias):
                code = launcher._teleop_import_code(task)
                self.assertIn("from teleimager.image_client import ImageClient", code)
                self.assertIn("from unitree_sdk2py.core.channel import ChannelFactoryInitialize", code)
                self.assertIn("import casadi, dex_retargeting", code)
                self.assertIn(expected_controller[task.end_effector], code)
                self.assertIn("import teleop.teleop_hand_and_arm", code)

    def test_execution_commands_run_import_checks_even_in_dry_run(self):
        expected_scopes = {
            "doctor": "all",
            "sim": "sim",
            "teleop": "teleop",
            "all": "all",
        }
        for command, scope in expected_scopes.items():
            for dry_run in (False, True):
                with self.subTest(command=command, dry_run=dry_run):
                    argv = [command, "pick-cylinder-dex1"]
                    if dry_run:
                        argv.append("--dry-run")
                    with (
                        patch.object(launcher, "preflight", return_value=[]) as preflight,
                        patch.object(launcher, "print_plan"),
                        patch.object(launcher, "_run_foreground", return_value=0),
                        patch.object(launcher, "run_all", return_value=0),
                    ):
                        self.assertEqual(launcher.main(argv), 0)
                    self.assertEqual(preflight.call_args.kwargs["scope"], scope)
                    self.assertTrue(preflight.call_args.kwargs["imports"])


class ImmediateThread:
    def __init__(self, *, target, args, daemon):
        del daemon
        self.target = target
        self.args = args

    def start(self):
        self.target(*self.args)

    def join(self, timeout=None):
        del timeout


class FakeProcess:
    def __init__(self, *, stdout=None, poll_values=(), pid=12345):
        self.stdout = stdout
        self.pid = pid
        self.returncode = None
        self.poll_values = list(poll_values)
        self.signals = []
        self.terminated = False
        self.killed = False

    def poll(self):
        if self.returncode is not None:
            return self.returncode
        if self.poll_values:
            result = self.poll_values.pop(0)
            if result is not None:
                self.returncode = result
            return result
        return None

    def send_signal(self, sent_signal):
        self.signals.append(sent_signal)
        self.returncode = -int(sent_signal)

    def terminate(self):
        self.terminated = True
        self.returncode = -signal.SIGTERM

    def kill(self):
        self.killed = True
        self.returncode = -signal.SIGKILL

    def wait(self, timeout=None):
        del timeout
        return self.returncode


class LauncherOrchestrationTests(unittest.TestCase):
    def test_process_group_cleanup_runs_after_group_leader_exits(self):
        simulator = FakeProcess(poll_values=(7,), pid=23456)
        kill_calls = []

        def fake_killpg(process_group_id, sent_signal):
            kill_calls.append((process_group_id, sent_signal))
            if sent_signal == 0:
                raise ProcessLookupError

        with patch.object(launcher.os, "killpg", side_effect=fake_killpg):
            launcher._stop_process_group(simulator)

        self.assertEqual(
            kill_calls,
            [(simulator.pid, signal.SIGINT), (simulator.pid, 0)],
        )

    def test_simulator_death_after_readiness_stops_teleop(self):
        simulator = FakeProcess(
            stdout=io.StringIO(launcher.READY_MARKER + "\n"),
            poll_values=(7,),
            pid=23456,
        )
        teleop = FakeProcess(poll_values=(None,), pid=23457)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            args = launcher.build_parser().parse_args(
                [
                    "all",
                    "pick-cylinder-dex1",
                    "--xr-root",
                    str(root / "xr"),
                    "--sim-root",
                    str(root / "sim"),
                    "--sdk-root",
                    str(root / "sdk"),
                ]
            )
            with (
                patch.object(launcher, "ROOT", root),
                patch.object(launcher.subprocess, "Popen", side_effect=(simulator, teleop)) as popen,
                patch.object(launcher.threading, "Thread", ImmediateThread),
            ):
                result = launcher.run_all(args, tasks.TASKS[0])

        self.assertEqual(result, 7)
        self.assertEqual(popen.call_count, 2)
        self.assertIn(signal.SIGINT, teleop.signals)
        self.assertFalse(teleop.terminated)
        self.assertFalse(teleop.killed)


class SafetyEntrypointTests(unittest.TestCase):
    def test_overlay_gates_ik_and_uses_measured_q_with_zero_torque(self):
        source = (
            entrypoint.UPSTREAM_NUMPY_ANCHOR
            + "def loop():\n"
            + entrypoint.UPSTREAM_ARM_INIT_ANCHOR
            + entrypoint.UPSTREAM_READY_ASSIGNMENT
            + entrypoint.UPSTREAM_IK_BLOCK
            + entrypoint.UPSTREAM_GO_HOME
        )
        overlaid = entrypoint.apply_safety_overlay(source)
        self.assertIn("import numpy as np", overlaid)
        self.assertIn("ArmCommandSafety(arm_ctrl, xr_motion_data_ready)", overlaid)
        self.assertIn("if tele_data.motion_data_ready and arm_safety.can_command()", overlaid)
        self.assertIn("current_lr_arm_q.copy()", overlaid)
        self.assertIn("np.zeros_like(current_lr_arm_q)", overlaid)
        self.assertIn("arm_safety.close()", overlaid)
        self.assertNotIn("ctrl_dual_arm_go_home", overlaid)

    def test_overlay_fails_closed_on_unknown_upstream_source(self):
        with self.assertRaisesRegex(RuntimeError, "unsupported xr_teleoperate revision"):
            entrypoint.apply_safety_overlay("print('changed upstream')\n")

    def test_verified_overlay_rejects_unpinned_source(self):
        with self.assertRaisesRegex(RuntimeError, "expected pinned"):
            entrypoint.verified_safety_overlay("print('different revision')\n")


class FakeMotionReady:
    def __init__(self):
        self.value = False
        self.lock = threading.Lock()

    def get_lock(self):
        return self.lock


class FakeArmController:
    def __init__(self):
        self.measured = np.linspace(-0.2, 0.2, 14)
        self.calls = []

    def get_current_dual_arm_q(self):
        return self.measured.copy()

    def ctrl_dual_arm(self, q_target, tauff_target):
        self.calls.append((np.asarray(q_target).copy(), np.asarray(tauff_target).copy()))


class ArmCommandSafetyTests(unittest.TestCase):
    def test_timeout_holds_measured_pose_and_requires_release_before_rearm(self):
        controller = FakeArmController()
        motion_ready = FakeMotionReady()
        gate = ArmCommandSafety(
            controller,
            motion_ready,
            timeout=0.1,
            start_thread=False,
        )
        np.testing.assert_allclose(controller.calls[-1][0], controller.measured)
        np.testing.assert_allclose(controller.calls[-1][1], np.zeros(14))

        target = np.linspace(0.1, 0.8, 14)
        self.assertTrue(gate.command(target, np.ones(14)))
        self.assertTrue(motion_ready.value)
        controller.measured = np.linspace(-0.4, 0.4, 14)
        self.assertTrue(gate._watchdog_step(gate._last_command_at + 0.2))
        self.assertTrue(gate.timeout_latched)
        self.assertFalse(motion_ready.value)
        np.testing.assert_allclose(controller.calls[-1][0], controller.measured)
        np.testing.assert_allclose(controller.calls[-1][1], np.zeros(14))

        self.assertFalse(gate.command(target, np.ones(14)))
        gate.hold(controller.measured, released=True)
        self.assertFalse(gate.timeout_latched)
        self.assertTrue(gate.command(target, np.ones(14)))
        gate.close()

    def test_invalid_command_latches_measured_hold(self):
        controller = FakeArmController()
        gate = ArmCommandSafety(controller, timeout=0.1, start_thread=False)
        with self.assertRaisesRegex(ValueError, "finite"):
            gate.command(np.full(14, np.nan), np.zeros(14))
        self.assertTrue(gate.timeout_latched)
        np.testing.assert_allclose(controller.calls[-1][0], controller.measured)
        np.testing.assert_allclose(controller.calls[-1][1], np.zeros(14))
        gate.close()

    def test_wrong_length_command_is_rejected(self):
        controller = FakeArmController()
        gate = ArmCommandSafety(controller, timeout=0.1, start_thread=False)
        with self.assertRaisesRegex(ValueError, "shaped"):
            gate.command(np.zeros(1), np.zeros(1))
        self.assertTrue(gate.timeout_latched)
        np.testing.assert_allclose(controller.calls[-1][0], controller.measured)
        gate.close()

    def test_watchdog_retries_a_transient_hold_failure(self):
        controller = FakeArmController()
        gate = ArmCommandSafety(controller, timeout=0.1, start_thread=False)
        self.assertTrue(gate.command(np.ones(14), np.ones(14)))
        original = controller.ctrl_dual_arm
        failures = 1

        def fail_once(q_target, tauff_target):
            nonlocal failures
            if failures:
                failures -= 1
                raise RuntimeError("temporary controller failure")
            original(q_target, tauff_target)

        controller.ctrl_dual_arm = fail_once
        with self.assertRaisesRegex(RuntimeError, "temporary"):
            gate._watchdog_step(gate._last_command_at + 0.2)
        self.assertTrue(gate._hold_pending)
        self.assertFalse(gate._watchdog_step(gate._last_command_at + 0.3))
        self.assertFalse(gate._hold_pending)
        np.testing.assert_allclose(controller.calls[-1][0], controller.measured)
        np.testing.assert_allclose(controller.calls[-1][1], np.zeros(14))
        gate.close()


if __name__ == "__main__":
    unittest.main()
