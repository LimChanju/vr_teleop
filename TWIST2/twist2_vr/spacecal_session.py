"""One-terminal VUT feeder + upstream xi-ve desktop/VR calibration UI."""
from __future__ import annotations
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def executable_pids(executable):
    result = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if proc.stat().st_uid == os.getuid() and (proc / 'exe').resolve() == executable.resolve():
                result.append(int(proc.name))
        except OSError:
            pass
    return result


def loaded_runtime_drivers(proc_root=Path('/proc')):
    """DriverManager lists registered drivers too; inspect actual server mappings."""
    names = set()
    for proc in proc_root.iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if (proc / 'exe').resolve().name != 'vrserver':
                continue
            mappings = (proc / 'maps').read_text()
            for name in ('twist2_vut', '01spacecalibrator'):
                if f'/driver_{name}.so' in mappings:
                    names.add(name)
        except OSError:
            pass
    return names


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-ui', action='store_true', help='only forward poses; useful when UI already runs')
    args = parser.parse_args(argv)
    ui = ROOT / 'runtime/spacecal/drivers/01spacecalibrator/bin/linux64/space-calibrator'
    if not ui.is_file():
        parser.error('Run bash scripts/setup_space_calibrator.sh --install first')
    names = loaded_runtime_drivers()
    if not {'twist2_vut', '01spacecalibrator'} <= names:
        parser.error('New drivers are not loaded. Restart SteamVR once after installation.')
    logdir = ROOT / 'runtime/spacecal'
    stop = False
    def stopping(_signum, _frame):
        nonlocal stop
        stop = True
    previous = {s: signal.signal(s, stopping) for s in (signal.SIGINT, signal.SIGTERM)}
    bridge = overlay = None
    print('Space Calibrator session. No robot/simulation commands are sent.', flush=True)
    print('Reference: Quest RIGHT controller. Target: twist2_vut / FA61N3B00314.', flush=True)
    print('Use ordinary calibration, not Continuous. Keep the two devices rigidly together.', flush=True)
    try:
        with (logdir / 'overlay-launch.log').open('a') as log:
            bridge = subprocess.Popen([sys.executable, '-u', '-m', 'twist2_vr.vut_openvr_bridge'], cwd=ROOT)
            if not args.no_ui:
                if executable_pids(ui):
                    print('Using already-running xi-ve UI.', flush=True)
                else:
                    overlay = subprocess.Popen([str(ui)], cwd=ui.parent, stdout=log, stderr=subprocess.STDOUT)
            print('Keep this terminal running. Ctrl+C stops this session’s feeder.', flush=True)
            while not stop:
                if bridge.poll() is not None:
                    return bridge.returncode
                if overlay is not None and overlay.poll() is not None:
                    print(f'xi-ve UI exited ({overlay.returncode}); see {logdir / "overlay-launch.log"}', flush=True)
                    return overlay.returncode or 0
                time.sleep(0.2)
    finally:
        for process in (bridge, overlay):
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    print(f'Owned process {process.pid} did not exit; inspect it before stopping manually.', flush=True)
        for signum, handler in previous.items():
            signal.signal(signum, handler)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
