"""Preview regressions without creating windows or connecting to OpenVR."""

import json
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

pytest.importorskip("scipy")

from twist2_vr import sparseposer_cli as cli
from twist2_vr.models import PoseSample, TrackerFrame


def frame_with_poses(*, invalid=(), missing=()):
    poses = {
        role: PoseSample(role, role, [index / 10, 1, 0.2], [0, 0, 0, 1], 10.,
                         valid=role not in invalid)
        for index, role in enumerate(cli.ROLES) if role not in missing
    }
    return TrackerFrame(10., poses, "synthetic_test_only", "openvr_standing")


@pytest.fixture
def preview(monkeypatch):
    display = cli.SkeletonPreview.__new__(cli.SkeletonPreview)
    display.plt = Mock()
    display.plt.fignum_exists.return_value = True
    display.fig = SimpleNamespace(number=1)
    display.axes = Mock()
    display.last_draw = 0.
    monkeypatch.setattr(cli.time, "monotonic", lambda: 10.)
    return display


def test_warmup_draws_sensors_and_pumps_gui(preview):
    frame = frame_with_poses()
    preview.update({"kind": "warming_up", "timestamp_s": 10.}, frame)

    preview.plt.pause.assert_called_once()
    preview.axes.clear.assert_called_once()
    preview.axes.plot.assert_not_called()
    assert len(preview.axes.scatter.call_args.args[0]) == len(cli.ROLES)
    assert "warm" in preview.axes.set_title.call_args.args[0].lower()


def test_blocked_displays_missing_roles_and_only_valid_sensors(preview):
    frame = frame_with_poses(invalid=("Head",), missing=("Left_Wrist", "Right_Wrist"))
    preview.update({"kind": "tracking_blocked", "timestamp_s": 10.,
                    "reason": "missing/invalid Head"}, frame)

    preview.plt.pause.assert_called_once()
    preview.axes.clear.assert_called_once()
    preview.axes.plot.assert_not_called()
    assert len(preview.axes.scatter.call_args.args[0]) == len(cli.ROLES) - 3
    title = preview.axes.set_title.call_args.args[0]
    assert all(role in title for role in ("Head", "Left_Wrist", "Right_Wrist"))


def test_blocked_with_no_sensors_keeps_gui_responsive(preview):
    preview.update({"kind": "tracking_blocked", "timestamp_s": 10.,
                    "reason": "missing/invalid Pelvis"},
                   frame_with_poses(missing=cli.ROLES))

    preview.plt.pause.assert_called_once()
    preview.axes.plot.assert_not_called()
    title = preview.axes.set_title.call_args.args[0]
    assert all(role in title for role in cli.ROLES)


def test_estimate_draws_bones_in_plot_z_up_and_loss_clears_them(preview):
    estimate = {"kind": "sparseposer_eight_sensor_estimate", "timestamp_s": 10.,
                "positions_m": [[0., 1., 2.], [0.1, 1.5, 2.2]], "parents": [0, 0],
                "refinement": {"success": True}}
    preview.update(estimate, frame_with_poses())
    bone = np.array(preview.axes.plot.call_args.args)
    np.testing.assert_allclose(bone, [[0., 0.1], [2., 2.2], [1., 1.5]])

    preview.axes.reset_mock()
    preview.last_draw = 0.
    preview.update({"kind": "tracking_blocked", "timestamp_s": 10.,
                    "reason": "missing/invalid Head"}, frame_with_poses(missing=("Head",)))
    preview.axes.clear.assert_called_once()
    preview.axes.plot.assert_not_called()


def test_throttled_redraw_still_pumps_gui(preview):
    preview.last_draw = 9.95
    preview.update({"kind": "warming_up", "timestamp_s": 10.}, frame_with_poses())

    preview.plt.pause.assert_called_once()
    preview.axes.clear.assert_not_called()


def test_closed_window_interrupts_even_while_redraw_is_throttled(preview):
    preview.last_draw = 9.95
    preview.plt.fignum_exists.return_value = False
    with pytest.raises(KeyboardInterrupt):
        preview.update({"kind": "warming_up", "timestamp_s": 10.}, frame_with_poses())


def test_cli_preview_receives_warmup_blocked_and_estimated_frames(monkeypatch, tmp_path):
    # Exercise the real loop and recording writer with a synthetic replay.
    frame = frame_with_poses()
    recording = tmp_path / "raw_poses.jsonl"
    recording.write_text((json.dumps(cli._raw_record(frame)) + "\n") * 3)
    calibration = tmp_path / "calibration.json"
    calibration.write_text("{}")
    reference = tmp_path / "reference.txt"
    reference.write_text("synthetic reference for metadata hashing")
    estimate = {"kind": "sparseposer_eight_sensor_estimate", "timestamp_s": 10.}
    estimator = Mock()
    estimator.process.side_effect = [None, ValueError("missing/invalid Head"), estimate]
    display = Mock()

    monkeypatch.setitem(sys.modules, "torch", SimpleNamespace(set_num_threads=Mock()))
    monkeypatch.setattr(cli, "REFERENCE", reference)
    monkeypatch.setattr(cli, "MODELS", tmp_path / "no_models_needed")
    monkeypatch.setattr(cli, "load_skeleton", Mock(return_value=object()))
    monkeypatch.setattr(cli, "SparsePoserBackend", Mock(return_value=object()))
    monkeypatch.setattr(cli, "EightTrackerEstimator", Mock(return_value=estimator))
    monkeypatch.setattr(cli, "SkeletonPreview", Mock(return_value=display))
    provider = Mock(side_effect=AssertionError("replay must never initialize OpenVR"))
    monkeypatch.setattr(cli, "_provider", provider)
    output = tmp_path / "output"

    assert cli.main(["replay", "--recording", str(recording), "--calibration", str(calibration),
                     "--spacecal-profile", str(tmp_path / "no_live_profile"),
                     "--output", str(output), "--preview"]) == 0

    statuses = [call.args[0]["kind"] for call in display.update.call_args_list]
    assert statuses == ["warming_up", "tracking_blocked", "sparseposer_eight_sensor_estimate"]
    provider.assert_not_called()
    saved = [json.loads(line) for line in (output / "body_estimates.jsonl").read_text().splitlines()]
    assert [row["kind"] for row in saved] == statuses
    assert saved[1]["reason"] == "missing/invalid Head"
