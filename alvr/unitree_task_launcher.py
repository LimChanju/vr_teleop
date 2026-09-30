#!/usr/bin/env python3
"""Launch every official Unitree Isaac Lab G1 task with the ALVR adapter."""

from __future__ import annotations

import argparse
from datetime import datetime
import os
from pathlib import Path
import queue
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
from typing import TextIO

from unitree_tasks import TASKS, TaskSpec, resolve_task


ROOT = Path(__file__).resolve().parent
REPOSITORY_ROOT = ROOT.parent
DEFAULT_VENDOR_ROOT = REPOSITORY_ROOT / "vendor"
READY_MARKER = "========= start controller success ========="
XR_TELEOPERATE_REVISION = "817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b"
UNITREE_SIM_REVISION = "e30c25b1dffdf92ada1d6c8c1fe9a47bdde0fecc"
UNITREE_SDK_REVISION = "814556d15970dd2ecf1c9984e845ca02ab07e206"
RESERVED_SIM_OPTIONS = {
    "--upstream-script",
    "--full-scene",
    "--device",
    "--task",
    "--robot_type",
    "--enable_cameras",
    "--enable_dex1_dds",
    "--enable_dex3_dds",
    "--enable_inspire_dds",
    "--action_source",
    "--enable_wholebody_dds",
    "--model_path",
}
RESERVED_TELEOP_OPTIONS = {
    "--upstream-script",
    "--input-mode",
    "--arm",
    "--ee",
    "--sim",
    "--motion",
    "--frequency",
    "--img-server-ip",
    "--task-name",
    "--task-goal",
}


def _environment_path(name: str, default: Path) -> Path:
    return Path(os.environ.get(name, str(default))).expanduser()


def _default_python(environment_name: str, fallback: str = "python") -> str:
    explicit = os.environ.get(environment_name)
    if explicit:
        return explicit
    conda_executable = Path(
        os.environ.get("CONDA_EXE", "/home/railab/miniconda3/bin/conda")
    ).expanduser()
    conda_prefix = conda_executable.parent.parent
    candidate_name = "unitree_sim_env" if environment_name == "UNITREE_SIM_PYTHON" else "tv"
    candidate = conda_prefix / "envs" / candidate_name / "bin" / "python"
    if not candidate.is_file():
        return fallback
    # A binary Isaac Sim installation exposes its modules through the conda
    # environment's activation hook.  Calling env/bin/python directly skips
    # that hook, so activate the environment before replacing the wrapper
    # process with Python.  Keeping Python as the process-group leader also
    # makes simulator shutdown reliable.
    if environment_name == "UNITREE_SIM_PYTHON" and conda_executable.is_file():
        conda_wrapper = ROOT / "scripts" / "conda_env_exec.sh"
        # Return the wrapper path even when it is missing or non-executable so
        # preflight fails with a concrete setup error.  Falling back to
        # ``conda run`` would let Isaac Sim descendants escape the process
        # group that the launcher owns and make a clean shutdown impossible.
        return shlex.join([str(conda_wrapper), candidate_name, "python"])
    return str(candidate)


def build_parser() -> argparse.ArgumentParser:
    vendor_root = _environment_path("VENDOR_ROOT", DEFAULT_VENDOR_ROOT)
    parser = argparse.ArgumentParser(
        description="Run Unitree G1 Isaac Lab tasks using Quest 3 ALVR input",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "command",
        choices=("list", "doctor", "print", "sim", "teleop", "all"),
        help="operation to perform",
    )
    parser.add_argument("task", nargs="?", help="task alias or exact Isaac task id")
    parser.add_argument(
        "--xr-root",
        type=Path,
        default=_environment_path("XR_TELEOPERATE_ROOT", vendor_root / "xr_teleoperate"),
        help="xr_teleoperate checkout",
    )
    parser.add_argument(
        "--sim-root",
        type=Path,
        default=_environment_path("UNITREE_SIM_ROOT", vendor_root / "unitree_sim_isaaclab"),
        help="unitree_sim_isaaclab checkout",
    )
    parser.add_argument(
        "--sdk-root",
        type=Path,
        default=_environment_path("UNITREE_SDK_ROOT", vendor_root / "unitree_sdk2_python"),
        help="unitree_sdk2_python checkout",
    )
    parser.add_argument(
        "--sim-python",
        default=_default_python("UNITREE_SIM_PYTHON"),
        help="simulator Python command; quoted commands such as 'conda run -n unitree_sim_env python' work",
    )
    parser.add_argument(
        "--teleop-python",
        default=_default_python("XR_TELEOPERATE_PYTHON"),
        help="xr_teleoperate Python command",
    )
    parser.add_argument("--device", default="cuda:0", help="Isaac Lab device")
    parser.add_argument("--img-server-ip", default="127.0.0.1")
    parser.add_argument("--frequency", type=float, default=30.0)
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=900.0,
        help="seconds to allow for first-run RTX camera and shader warm-up",
    )
    parser.add_argument("--record", action="store_true", help="enable official episode recording")
    parser.add_argument("--headless-teleop", action="store_true", help="disable xr_teleoperate rerun UI")
    parser.add_argument(
        "--full-scene",
        action="store_true",
        help="keep remote simulator scenery and unused spectator cameras",
    )
    parser.add_argument("--dry-run", action="store_true", help="validate and print without starting processes")
    parser.add_argument(
        "--sim-arg",
        action="append",
        default=[],
        help="extra simulator argument (repeat; use --sim-arg=--option for leading dashes)",
    )
    parser.add_argument(
        "--teleop-arg",
        action="append",
        default=[],
        help="extra teleop argument (repeat; use --teleop-arg=--option for leading dashes)",
    )
    return parser


def _command_parts(value: str) -> list[str]:
    parts = shlex.split(value)
    if not parts:
        raise ValueError("Python command cannot be empty")
    if (
        Path(parts[0]).name == "conda"
        and len(parts) > 1
        and parts[1] == "run"
        and "--no-capture-output" not in parts
        and "--live-stream" not in parts
    ):
        parts.insert(2, "--no-capture-output")
    return parts


def validate_extra_args(args: argparse.Namespace) -> None:
    for label, values, reserved in (
        ("--sim-arg", args.sim_arg, RESERVED_SIM_OPTIONS),
        ("--teleop-arg", args.teleop_arg, RESERVED_TELEOP_OPTIONS),
    ):
        # Both upstream parsers use argparse's default allow_abbrev=True.  An
        # exact-name check alone would let --mo enable the managed --motion
        # flag, for example, so reject every unambiguous or ambiguous prefix.
        conflicts = sorted(
            {
                option
                for value in values
                if (option := value.split("=", 1)[0]).startswith("--")
                and any(managed.startswith(option) for managed in reserved)
            }
        )
        if conflicts:
            raise ValueError(f"{label} cannot override managed option(s): {', '.join(conflicts)}")


def build_sim_command(args: argparse.Namespace, task: TaskSpec) -> list[str]:
    command = [
        *_command_parts(args.sim_python),
        str((ROOT / "unitree_sim_entrypoint.py").resolve()),
        "--upstream-script",
        str((args.sim_root / "sim_main.py").resolve()),
        "--device",
        args.device,
        "--enable_cameras",
        "--task",
        task.task_id,
        "--robot_type",
        "g129",
        task.sim_dds_flag,
    ]
    if args.full_scene:
        command.append("--full-scene")
    if task.wholebody:
        command.extend(
            [
                "--action_source",
                "dds_wholebody",
                "--enable_wholebody_dds",
                "--model_path",
                "assets/model/policy.onnx",
            ]
        )
    command.extend(args.sim_arg)
    return command


def build_teleop_command(args: argparse.Namespace, task: TaskSpec) -> list[str]:
    command = [
        *_command_parts(args.teleop_python),
        str((ROOT / "unitree_teleop_entrypoint.py").resolve()),
        "--upstream-script",
        str((args.xr_root / "teleop" / "teleop_hand_and_arm.py").resolve()),
        "--input-mode",
        task.teleop_input_mode,
        "--arm",
        "G1_29",
        "--ee",
        task.teleop_end_effector,
        "--sim",
        "--frequency",
        str(args.frequency),
        "--img-server-ip",
        args.img_server_ip,
        "--task-name",
        task.alias,
        "--task-goal",
        task.task_id,
    ]
    if args.record:
        command.append("--record")
    if args.headless_teleop:
        command.append("--headless")
    # Do not add --motion for Wholebody.  The official flag uses LocoClient,
    # while unitree_sim_isaaclab consumes rt/run_command/cmd.  The ALVR shim
    # publishes that exact topic and keeps arm commands on rt/lowcmd.
    command.extend(args.teleop_arg)
    return command


def teleop_environment(task: TaskSpec) -> dict[str, str]:
    environment = os.environ.copy()
    python_path = [str((ROOT / "shim").resolve()), str(ROOT.resolve())]
    if environment.get("PYTHONPATH"):
        python_path.append(environment["PYTHONPATH"])
    environment.update(
        {
            "PYTHONPATH": os.pathsep.join(python_path),
            "PYTHONUNBUFFERED": "1",
            "ALVR_ACTION_MANIFEST": str((ROOT / "actions.json").resolve()),
            # Official Unitree mapping expects an absolute ray/aim pose.  Do
            # not silently command the robot from calibrated raw deltas.
            "ALVR_POSE_MODE": environment.get("ALVR_POSE_MODE", "aim"),
            "ALVR_WHOLEBODY": "1" if task.wholebody else "0",
            "ALVR_TASK_ID": task.task_id,
            "ALVR_END_EFFECTOR": task.end_effector,
        }
    )
    return environment


def sim_environment(args: argparse.Namespace) -> dict[str, str]:
    environment = os.environ.copy()
    environment["PYTHONUNBUFFERED"] = "1"
    # teleimager's upstream YAML loader does not pass encoding=.  Isaac Sim
    # can otherwise fall back to ASCII and fail on the config's Unicode
    # comments before DDS/controller startup.
    environment["PYTHONUTF8"] = "1"
    environment["PYTHONIOENCODING"] = "utf-8"
    environment.setdefault("PROJECT_ROOT", str(args.sim_root.resolve()))
    return environment


def _executable_problem(command: str) -> str | None:
    first = _command_parts(command)[0]
    if os.sep in first:
        path = Path(first).expanduser()
        return None if path.is_file() and os.access(path, os.X_OK) else f"not executable: {path}"
    return None if shutil.which(first) else f"not found on PATH: {first}"


def _git_head_problem(label: str, root: Path, expected_revision: str) -> str | None:
    if not root.is_dir():
        return f"{label} checkout missing: {root}"
    if not (root / ".git").exists():
        return f"{label} is not a Git checkout: {root}"
    try:
        check = subprocess.run(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            text=True,
            capture_output=True,
            timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        return f"{label} revision check failed: {type(error).__name__}: {error}"
    if check.returncode != 0:
        detail = (check.stderr or check.stdout).strip().splitlines()
        return f"{label} revision check failed: {detail[-1] if detail else f'exit {check.returncode}'}"
    actual_revision = check.stdout.strip()
    if actual_revision != expected_revision:
        return (
            f"{label} revision mismatch: expected {expected_revision}, "
            f"found {actual_revision or '<empty>'}"
        )
    return None


def _usable_asset(path: Path) -> bool:
    if not path.is_file() or path.stat().st_size == 0:
        return False
    try:
        with path.open("rb") as asset_file:
            header = asset_file.read(80)
        return not header.startswith(b"version https://git-lfs.github.com/spec")
    except OSError:
        return False


def _subprocess_problem(
    label: str,
    command: list[str],
    *,
    cwd: Path,
    environment: dict[str, str],
) -> str | None:
    try:
        check = subprocess.run(
            command,
            cwd=cwd,
            env=environment,
            text=True,
            capture_output=True,
            timeout=90,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        return f"{label} failed: {type(error).__name__}: {error}"
    if check.returncode == 0:
        return None
    detail = (check.stderr or check.stdout).strip().splitlines()
    return f"{label} failed: {detail[-1] if detail else f'exit {check.returncode}'}"


def _teleop_import_code(task: TaskSpec) -> str:
    controller_import = {
        "dex1": (
            "from teleop.robot_control.robot_hand_unitree "
            "import Dex1_1_Gripper_Controller; "
        ),
        "dex3": (
            "from teleop.robot_control.robot_hand_unitree "
            "import Dex3_1_Controller; "
        ),
        "inspire": (
            "from teleop.robot_control.robot_hand_inspire "
            "import Inspire_Controller_DFX; "
        ),
    }[task.end_effector]
    shim_file = str((ROOT / "shim" / "televuer" / "__init__.py").resolve())
    return (
        "from pathlib import Path; "
        "import casadi, dex_retargeting, numpy, openvr, pinocchio; "
        "from unitree_sdk2py.core.channel import ChannelFactoryInitialize; "
        "import televuer; "
        f"assert Path(televuer.__file__).resolve() == Path({shim_file!r}); "
        # The upstream entrypoint configures logging_mp before importing its
        # controllers.  logging_mp 0.2 correctly rejects late configuration,
        # so preserve that import order in the doctor check as well.
        "import teleop.teleop_hand_and_arm; "
        "from teleimager.image_client import ImageClient; "
        "from teleop.robot_control.robot_arm import G1_29_ArmController; "
        "from teleop.robot_control.robot_arm_ik import G1_29_ArmIK; "
        f"{controller_import}"
        "print('teleop imports and ALVR shim: OK')"
    )


def preflight(
    args: argparse.Namespace,
    task: TaskSpec | None,
    *,
    scope: str = "all",
    imports: bool = False,
) -> list[str]:
    problems: list[str] = []
    check_xr = scope in {"teleop", "all"}
    check_sim = scope in {"sim", "all"}
    check_sdk = check_xr or check_sim
    checks: dict[str, Path] = {}
    if check_xr:
        checks.update(
            {
                "xr entrypoint": args.xr_root / "teleop" / "teleop_hand_and_arm.py",
                "xr G1 model": args.xr_root / "assets" / "g1" / "g1_body29_hand14.urdf",
                "xr teleimager submodule": args.xr_root / "teleop" / "teleimager" / "pyproject.toml",
                "ALVR action manifest": ROOT / "actions.json",
                "ALVR shim": ROOT / "shim" / "televuer" / "__init__.py",
                "ALVR safety entrypoint": ROOT / "unitree_teleop_entrypoint.py",
                "xr dex-retargeting submodule": (
                    args.xr_root / "teleop" / "robot_control" / "dex-retargeting" / "pyproject.toml"
                ),
            }
        )
    if check_sim:
        checks.update(
            {
                "sim entrypoint": args.sim_root / "sim_main.py",
                "ALVR simulator adapter": ROOT / "unitree_sim_entrypoint.py",
                "ALVR simulator camera config": ROOT / "sim_camera_config.yaml",
                "sim teleimager submodule": args.sim_root / "teleimager" / "pyproject.toml",
            }
        )
    if check_sdk:
        checks["unitree SDK source"] = args.sdk_root / "setup.py"
    for label, path in checks.items():
        if not path.exists():
            problems.append(f"{label} missing: {path}")

    revision_checks: list[tuple[str, Path, str]] = []
    if check_xr:
        revision_checks.append(("xr_teleoperate", args.xr_root, XR_TELEOPERATE_REVISION))
    if check_sim:
        revision_checks.append(("unitree_sim_isaaclab", args.sim_root, UNITREE_SIM_REVISION))
    if check_sdk:
        revision_checks.append(("unitree_sdk2_python", args.sdk_root, UNITREE_SDK_REVISION))
    for label, root, revision in revision_checks:
        problem = _git_head_problem(label, root, revision)
        if problem:
            problems.append(problem)

    python_checks = []
    if check_sim:
        python_checks.append(("sim Python", args.sim_python))
    if check_xr:
        python_checks.append(("teleop Python", args.teleop_python))
    for label, command in python_checks:
        problem = _executable_problem(command)
        if problem:
            problems.append(f"{label}: {problem}")

    if task is not None and check_sim:
        registrations = args.sim_root / "tasks" / "g1_tasks"
        if not registrations.is_dir():
            problems.append(f"sim task registry missing: {registrations}")
        else:
            registered = any(
                task.task_id in path.read_text(encoding="utf-8", errors="ignore")
                for path in registrations.rglob("__init__.py")
            )
            if not registered:
                problems.append(f"task id is not registered by this simulator checkout: {task.task_id}")
        for relative in task.required_sim_assets:
            asset = args.sim_root / relative
            if not _usable_asset(asset):
                problems.append(f"sim asset missing or is an LFS pointer: {asset}")

    if check_xr and (args.xr_root / "teleop" / "teleop_hand_and_arm.py").is_file():
        from unitree_teleop_entrypoint import verified_safety_overlay

        try:
            verified_safety_overlay(
                (args.xr_root / "teleop" / "teleop_hand_and_arm.py").read_text(encoding="utf-8")
            )
        except (OSError, RuntimeError) as error:
            problems.append(f"ALVR arm safety overlay incompatible: {error}")

    if imports and check_xr and _executable_problem(args.teleop_python) is None:
        import_task = task or TASKS[0]
        environment = teleop_environment(import_task)
        source_path = os.pathsep.join((str(args.xr_root), environment.get("PYTHONPATH", "")))
        environment["PYTHONPATH"] = source_path
        problem = _subprocess_problem(
            "teleop Python imports",
            [*_command_parts(args.teleop_python), "-c", _teleop_import_code(import_task)],
            cwd=args.xr_root / "teleop",
            environment=environment,
        )
        if problem:
            problems.append(problem)

    if imports and check_sim and _executable_problem(args.sim_python) is None:
        problem = _subprocess_problem(
            "sim Python imports",
            [
                *_command_parts(args.sim_python),
                "-c",
                "import gymnasium, isaaclab, onnxruntime, pinocchio, torch, unitree_sdk2py; "
                "print('sim imports: OK')",
            ],
            cwd=args.sim_root,
            environment=sim_environment(args),
        )
        if problem:
            problems.append(problem)
    return problems


def print_task_table() -> None:
    headers = ("ALIAS", "MODE", "EE", "INPUT SUPPORT")
    rows = [
        (
            task.alias,
            "wholebody" if task.wholebody else "fixed",
            task.end_effector,
            task.support_label,
        )
        for task in TASKS
    ]
    widths = [max(len(headers[index]), *(len(row[index]) for row in rows)) for index in range(4)]
    print("  ".join(value.ljust(widths[index]) for index, value in enumerate(headers)))
    print("  ".join("-" * width for width in widths))
    for row in rows:
        print("  ".join(value.ljust(widths[index]) for index, value in enumerate(row)))
    print(f"\n{len(rows)} G1 tasks: Dex1 is native; Dex3/Inspire use controller grasp synergy.")


def _quoted(command: list[str]) -> str:
    return shlex.join(command)


def print_plan(args: argparse.Namespace, task: TaskSpec) -> None:
    print(f"Task:       {task.task_id}")
    print(f"Mode:       {'wholebody' if task.wholebody else 'fixed'}")
    print(f"End effector: {task.end_effector} ({task.support_label})")
    print(f"Simulator:  {_quoted(build_sim_command(args, task))}")
    print(f"Teleop:     {_quoted(build_teleop_command(args, task))}")
    print(f"ALVR shim:  {ROOT / 'shim'}")
    print("Dead-man:   hold both grip buttons; release either grip to freeze/stop")


def _run_foreground(command: list[str], cwd: Path, environment: dict[str, str]) -> int:
    try:
        return subprocess.call(command, cwd=cwd, env=environment)
    except KeyboardInterrupt:
        return 130


def _pump_output(
    process: subprocess.Popen[str],
    log: TextIO,
    ready: threading.Event,
    events: queue.Queue[str],
) -> None:
    assert process.stdout is not None
    try:
        for line in process.stdout:
            log.write(line)
            log.flush()
            print(f"[sim] {line}", end="", flush=True)
            if READY_MARKER in line:
                ready.set()
    finally:
        events.put("closed")


def _process_group_exists(process_group_id: int) -> bool:
    try:
        os.killpg(process_group_id, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _wait_for_process_group_exit(
    process: subprocess.Popen[str], timeout: float
) -> bool:
    deadline = time.monotonic() + timeout
    while _process_group_exists(process.pid):
        # Reap the direct child as soon as it exits.  Its descendants can keep
        # the same process group alive after that point.
        process.poll()
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False
        time.sleep(min(0.1, remaining))
    return True


def _stop_process_group(process: subprocess.Popen[str]) -> None:
    # Always target the group, even if the direct simulator process has already
    # exited: Isaac Sim may have left Kit/renderer descendants in that group.
    for sent_signal, timeout in (
        (signal.SIGINT, 15.0),
        (signal.SIGTERM, 10.0),
        (signal.SIGKILL, 5.0),
    ):
        try:
            os.killpg(process.pid, sent_signal)
        except ProcessLookupError:
            break
        if _wait_for_process_group_exit(process, timeout):
            break

    if process.poll() is None:
        try:
            process.wait(timeout=1)
        except subprocess.TimeoutExpired:
            # SIGKILL was already attempted above.  Avoid masking the original
            # launcher result if an uninterruptible kernel task remains.
            pass


def _stop_process(process: subprocess.Popen[object]) -> None:
    """Stop one foreground child without signaling the launcher's process group."""
    if process.poll() is not None:
        return
    try:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=15)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    if process.poll() is None:
        try:
            process.terminate()
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            try:
                process.kill()
            except ProcessLookupError:
                pass
            process.wait(timeout=5)


def run_all(args: argparse.Namespace, task: TaskSpec) -> int:
    log_dir = ROOT / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    log_path = log_dir / f"{stamp}-{task.alias}-sim.log"
    print(f"Simulator log: {log_path}")
    ready = threading.Event()
    events: queue.Queue[str] = queue.Queue()
    with log_path.open("w", encoding="utf-8") as log:
        simulator = subprocess.Popen(
            build_sim_command(args, task),
            cwd=args.sim_root,
            env=sim_environment(args),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        pump = threading.Thread(target=_pump_output, args=(simulator, log, ready, events), daemon=True)
        pump.start()
        deadline = time.monotonic() + args.startup_timeout
        try:
            while not ready.is_set():
                if simulator.poll() is not None:
                    print(f"Simulator exited before readiness (code {simulator.returncode}).", file=sys.stderr)
                    return simulator.returncode or 1
                if time.monotonic() >= deadline:
                    print(f"Simulator did not become ready within {args.startup_timeout:g}s.", file=sys.stderr)
                    return 1
                try:
                    events.get(timeout=0.5)
                except queue.Empty:
                    pass
            print("Simulator DDS controller is ready; starting ALVR teleop.")
            teleop = subprocess.Popen(
                build_teleop_command(args, task),
                cwd=args.xr_root / "teleop",
                env=teleop_environment(task),
            )
            try:
                while True:
                    simulator_status = simulator.poll()
                    if simulator_status is not None:
                        print(
                            f"Simulator exited during teleoperation (code {simulator_status}).",
                            file=sys.stderr,
                        )
                        _stop_process(teleop)
                        return simulator_status or 1
                    teleop_status = teleop.poll()
                    if teleop_status is not None:
                        return teleop_status
                    time.sleep(0.1)
            except KeyboardInterrupt:
                return 130
            finally:
                _stop_process(teleop)
        finally:
            _stop_process_group(simulator)
            pump.join(timeout=5)


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        validate_extra_args(args)
    except ValueError as error:
        parser.error(str(error))
    if args.command == "list":
        print_task_table()
        return 0
    if args.task is None:
        parser.error(f"{args.command} requires TASK; run '{Path(sys.argv[0]).name} list'")
    try:
        task = resolve_task(args.task)
    except ValueError as error:
        parser.error(str(error))

    imports = args.command in {"doctor", "all", "sim", "teleop"}
    scope = {
        "doctor": "all",
        "all": "all",
        "sim": "sim",
        "teleop": "teleop",
        "print": "none",
    }[args.command]
    problems = preflight(args, task, scope=scope, imports=imports) if scope != "none" else []
    if problems:
        print("Preflight failed:", file=sys.stderr)
        for problem in problems:
            print(f"  - {problem}", file=sys.stderr)
        print(
            f"See {ROOT / 'README.md'}; clone sources with "
            f"'{ROOT / 'scripts' / 'setup_unitree_sources.sh'}' and install official simulator assets.",
            file=sys.stderr,
        )
        return 2

    if args.command in {"doctor", "print"} or args.dry_run:
        print_plan(args, task)
        if args.command == "doctor":
            print("Doctor: source, revision, asset, executable, import, and shim checks passed.")
        return 0
    if args.command == "sim":
        return _run_foreground(build_sim_command(args, task), args.sim_root, sim_environment(args))
    if args.command == "teleop":
        return _run_foreground(
            build_teleop_command(args, task),
            args.xr_root / "teleop",
            teleop_environment(task),
        )
    return run_all(args, task)


if __name__ == "__main__":
    raise SystemExit(main())
