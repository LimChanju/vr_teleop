import os
from pathlib import Path
import shutil
import subprocess
import time

import pytest


ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def isolated_entry(tmp_path):
    scripts = tmp_path / "project with spaces" / "scripts"
    scripts.mkdir(parents=True)
    entry = scripts / "steamvr_pinned_entry.sh"
    shutil.copy2(ROOT / "scripts/steamvr_pinned_entry.sh", entry)
    (scripts / "run_steamvr_pinned.sh").write_text(
        '#!/usr/bin/env bash\n'
        'printf "%s\\n" "$@"\n'
        'exit "${PINNED_STUB_EXIT:-0}"\n'
    )
    supplied_command = tmp_path / "managed-steamvr-command"
    supplied_command.write_text(
        '#!/usr/bin/env bash\n'
        'printf executed > "$PINNED_TEST_MARKER"\n'
        'exit 99\n'
    )
    supplied_command.chmod(0o755)
    marker = tmp_path / "supplied-command-executed"
    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    systemctl = fake_bin / "systemctl"
    systemctl.write_text(
        '#!/usr/bin/env bash\n'
        'printf "%s\\n" "$*" >> "$PINNED_TEST_SYSTEMCTL_LOG"\n'
        'case "$*" in\n'
        '  "--user is-active --quiet twist2-steamvr-pinned.service")\n'
        '    [[ "${PINNED_TEST_UNIT_ACTIVE:-0}" == 1 && '
        '! -e "$PINNED_TEST_UNIT_STOPPED" ]] ;;\n'
        '  "--user stop twist2-steamvr-pinned.service")\n'
        '    printf stopped > "$PINNED_TEST_UNIT_STOPPED" ;;\n'
        '  *) exit 98 ;;\n'
        'esac\n'
    )
    systemctl.chmod(0o755)
    env = {
        **os.environ,
        "PATH": str(fake_bin) + os.pathsep + os.environ.get("PATH", os.defpath),
        "PINNED_TEST_MARKER": str(marker),
        "PINNED_STUB_EXIT": "0",
        "PINNED_TEST_UNIT_ACTIVE": "0",
        "PINNED_TEST_SYSTEMCTL_LOG": str(tmp_path / "systemctl-calls"),
        "PINNED_TEST_UNIT_STOPPED": str(tmp_path / "unit-stopped"),
    }
    return entry, supplied_command, marker, env


@pytest.mark.parametrize("arguments", ["none", "steam_command", "shell_syntax"])
def test_entry_ignores_steam_command_and_arguments(isolated_entry, tmp_path, arguments):
    entry, supplied_command, marker, env = isolated_entry
    argv = {
        "none": [],
        "steam_command": [str(supplied_command), "--check", "--verb=waitforexitandrun"],
        "shell_syntax": [
            str(supplied_command),
            '; printf injected > "$PINNED_TEST_MARKER"; '
            '$(printf injected > "$PINNED_TEST_MARKER") '
            '`printf injected > "$PINNED_TEST_MARKER"`',
        ],
    }[arguments]

    result = subprocess.run(
        ["bash", str(entry), *argv],
        cwd=tmp_path,
        env=env,
        capture_output=True,
        text=True,
        timeout=5,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == ["--start"]
    assert not marker.exists()
    assert Path(env["PINNED_TEST_SYSTEMCTL_LOG"]).read_text().splitlines() == [
        "--user is-active --quiet twist2-steamvr-pinned.service"
    ]


@pytest.mark.parametrize("exit_code", [7, 64])
def test_entry_propagates_pinned_launcher_failure(isolated_entry, tmp_path, exit_code):
    entry, supplied_command, marker, env = isolated_entry
    env["PINNED_STUB_EXIT"] = str(exit_code)

    result = subprocess.run(
        ["bash", str(entry), str(supplied_command)],
        cwd=tmp_path,
        env=env,
        capture_output=True,
        text=True,
        timeout=5,
    )

    assert result.returncode == exit_code, result.stderr
    assert result.stdout.splitlines() == ["--start"]
    assert not marker.exists()
    assert not Path(env["PINNED_TEST_SYSTEMCTL_LOG"]).exists()


def test_entry_stays_alive_and_stops_only_pinned_unit_on_term(isolated_entry, tmp_path):
    entry, supplied_command, marker, env = isolated_entry
    env["PINNED_TEST_UNIT_ACTIVE"] = "1"
    calls = Path(env["PINNED_TEST_SYSTEMCTL_LOG"])
    process = subprocess.Popen(
        ["bash", str(entry), str(supplied_command)],
        cwd=tmp_path,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        deadline = time.monotonic() + 3
        while not calls.exists() and process.poll() is None and time.monotonic() < deadline:
            time.sleep(0.01)
        assert calls.exists(), "Entry did not begin monitoring the isolated unit"
        assert process.poll() is None, "Entry exited while the isolated unit was active"
        process.terminate()
        stdout, stderr = process.communicate(timeout=5)
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate(timeout=5)

    assert process.returncode == 0, stderr
    assert stdout.splitlines() == ["--start"]
    assert not marker.exists()
    assert Path(env["PINNED_TEST_UNIT_STOPPED"]).read_text() == "stopped"
    commands = calls.read_text().splitlines()
    assert commands.count("--user stop twist2-steamvr-pinned.service") == 1
    assert set(commands) <= {
        "--user is-active --quiet twist2-steamvr-pinned.service",
        "--user stop twist2-steamvr-pinned.service",
    }
