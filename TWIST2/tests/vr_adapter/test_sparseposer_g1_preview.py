import json
from queue import Queue
from types import SimpleNamespace

import numpy as np
import pytest

from twist2_vr.sparseposer_g1_preview import (XML, apply_preview_pose, joint_signature,
                                            parser, put_latest, validate_args)


def test_preview_has_no_policy_or_publish_switch():
    args = parser().parse_args([])
    assert args.viewer == "vr"
    assert not args.record
    assert parser().parse_args(["--record"]).record
    assert not parser().parse_args(["--no-record"]).record
    assert not any("redis" in k or "publish" in k or "policy" in k for k in vars(args))


def test_latest_queue_replaces_oldest():
    q = Queue(maxsize=2)
    put_latest(q, 1); put_latest(q, 2); put_latest(q, 3)
    assert q.get_nowait() == 2
    assert q.get_nowait() == 3


def test_kinematic_assignment_never_steps_physics(monkeypatch):
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(XML))
    data = mujoco.MjData(model)
    qpos = np.r_[[0, 0, .8, 1, 0, 0, 0], np.zeros(29)]
    def forbidden(*args):
        pytest.fail("kinematic preview must not call mj_step")
    monkeypatch.setattr(mujoco, "mj_step", forbidden)
    apply_preview_pose(model, data, {"qpos": qpos})
    np.testing.assert_array_equal(data.qpos, qpos)
    np.testing.assert_array_equal(data.qvel, 0)
    assert data.time == 0
    assert len(joint_signature(model)) == 29
    for bad in ([0]*35, [np.nan]*36, [0]*36):
        with pytest.raises(ValueError):
            apply_preview_pose(model, data, {"qpos": bad})


def test_replay_checks_recording_calibration_hash(tmp_path):
    profile = tmp_path / "calibration.json"
    profile.write_text(json.dumps({"spacecal_sha256": None}))
    recording = tmp_path / "raw_poses.jsonl"
    recording.write_text("")
    (tmp_path / "run.json").write_text(json.dumps({"calibration_sha256": "not-the-same-profile"}))
    args = parser().parse_args(["--viewer", "headless", "--recording", str(recording),
                               "--calibration", str(profile)])
    with pytest.raises(ValueError, match="different calibration"):
        validate_args(args)


def test_fast_cannot_be_used_for_live_tracking():
    with pytest.raises(ValueError, match="offline"):
        validate_args(parser().parse_args(["--fast"]))


@pytest.mark.parametrize("flag,value", [("--seconds", "-1"), ("--frames", "-1"),
                                       ("--hz", "0"), ("--threads", "0"), ("--video-fps", "0")])
def test_invalid_runtime_values(flag, value):
    with pytest.raises(ValueError, match="invalid duration"):
        validate_args(parser().parse_args([flag, value]))
