import json
from types import SimpleNamespace

import numpy as np
import pytest

from twist2_vr import spatial_check
from twist2_vr.models import PoseSample, TrackerFrame
from twist2_vr.spatial_check import BATCH_ROLES, build_parser, summarize_batch, summarize_proximity


def frame(t=1.0, *, distance=.08, shift=0, invalid=False, age=0, skew=0,
          generation=0, serial="vive", missing=False, coordinate_frame="openvr_standing"):
    poses = {
        "Right_Wrist": PoseSample("touch", "Right_Wrist", [shift, 1, 0], [0, 0, 0, 1], t-age),
        "Pelvis": PoseSample(serial, "Pelvis", [shift+distance, 1, 0], [0, 0, 0, 1],
                            t-age-skew, valid=not invalid,
                            metadata={"connection_generation": generation}),
    }
    if missing:
        poses.pop("Right_Wrist")
    return TrackerFrame(t, poses, "test", coordinate_frame)


def test_nearby_pair_passes_without_marking_any_calibration_complete():
    result = summarize_proximity([frame(), frame(1.1)])
    assert result["proximity_consistent"]
    assert result["valid_pairs"] == 2
    assert result["mean_separation_m"] == pytest.approx(.08)
    np.testing.assert_allclose(result["mean_quest_minus_vive_m"], [-.08, 0, 0])
    assert not result["mounts_calibrated"]
    assert not result["full_spatial_alignment_verified"]


@pytest.mark.parametrize("distance", [.3, 33.5])
def test_stable_but_wrong_position_fails(distance):
    result = summarize_proximity([frame(distance=distance), frame(1.1, distance=distance)])
    assert not result["proximity_consistent"]
    assert "reported_positions_too_far_apart_for_nearby_devices" in result["issues"]


@pytest.mark.parametrize("kwargs", [{"invalid": True}, {"missing": True},
                                  {"age": .3}, {"age": -.1}, {"skew": .1}])
def test_invalid_missing_stale_future_or_unsynchronized_pairs_rejected(kwargs):
    result = summarize_proximity([frame(**kwargs), frame(1.1, **kwargs)])
    assert not result["proximity_consistent"]
    assert result["valid_pairs"] == 0


@pytest.mark.parametrize("kwargs", [{"generation": 1}, {"serial": "replacement"}])
def test_changed_device_identity_or_connection_rejected(kwargs):
    result = summarize_proximity([frame(), frame(1.1, **kwargs)])
    assert not result["proximity_consistent"]
    assert "missing_or_changed_device_identity_or_generation" in result["issues"]


def test_nearby_pair_moving_together_is_not_a_static_check():
    result = summarize_proximity([frame(), frame(1.1, shift=.2)])
    assert not result["proximity_consistent"]
    assert "devices_not_held_still" in result["issues"]


def test_one_sample_and_low_coverage_do_not_pass():
    assert not summarize_proximity([frame()])["proximity_consistent"]
    result = summarize_proximity([frame(), frame(1.1), frame(1.2, invalid=True)])
    assert not result["proximity_consistent"]
    assert "insufficient_fresh_synchronized_tracking" in result["issues"]


def test_wrong_frame_or_role_or_threshold_rejected():
    with pytest.raises(ValueError, match="no frames"):
        summarize_proximity([])
    with pytest.raises(ValueError, match="openvr_standing"):
        summarize_proximity([frame(coordinate_frame="vive_hub_world")])
    with pytest.raises(ValueError, match="one VIVE"):
        summarize_proximity([frame()], vive_role="Right_Wrist")
    for value in (0, -1, float("nan"), float("inf")):
        with pytest.raises(ValueError, match="finite and positive"):
            summarize_proximity([frame()], max_separation=value)


def test_capture_defaults_have_no_calibration_or_robot_output_switch():
    args = build_parser().parse_args([])
    assert args.vive_role is None and args.quest_hand == "right"
    assert args.mapping.parts[-2:] == ("local", "tracker_mapping.vive_linux.json")
    assert args.countdown == 10 and args.seconds == 5
    assert not hasattr(args, "publish_sim")
    assert not hasattr(args, "output_calibration")


def batch_stage(role, *, distance=.08, generation=0):
    frames = []
    for t in (1, 1.1):
        sample = frame(t, distance=distance)
        frames.append(TrackerFrame(t, {
            "Right_Wrist": sample.poses["Right_Wrist"].replaced(
                metadata={"connection_generation": generation}),
            role: sample.poses["Pelvis"].replaced(role=role, device_id="vive:"+role),
        }, "test", "openvr_standing"))
    return summarize_proximity(frames, vive_role=role)


def test_batch_success_keeps_calibration_and_robot_output_disabled():
    result = summarize_batch([batch_stage(r) for r in BATCH_ROLES], BATCH_ROLES)
    assert result["all_proximity_checks_consistent"]
    assert len(result["stages"]) == 5
    assert not result["mounts_calibrated"]
    assert not result["full_spatial_alignment_verified"]
    assert not result["simulation_publish_enabled"]


def test_batch_failure_does_not_lose_other_stages():
    stages = [batch_stage(r, distance=.5 if r == "Left_Knee" else .08) for r in BATCH_ROLES]
    result = summarize_batch(stages, BATCH_ROLES)
    assert not result["all_proximity_checks_consistent"]
    assert len(result["stages"]) == 5
    assert sum(s["proximity_consistent"] for s in result["stages"]) == 4
    assert not summarize_batch(stages[:-1], BATCH_ROLES)["all_proximity_checks_consistent"]


def test_batch_detects_reconnected_controller_between_individually_good_stages():
    stages = [batch_stage(r, generation=i) for i, r in enumerate(BATCH_ROLES)]
    assert all(s["proximity_consistent"] for s in stages)
    result = summarize_batch(stages, BATCH_ROLES)
    assert not result["all_proximity_checks_consistent"]
    assert "device_identity_or_generation_changed_between_stages" in result["issues"]


def test_batch_and_single_role_are_mutually_exclusive():
    parser = build_parser()
    args = parser.parse_args(["--all-trackers", "--speech", "--no-speech"])
    assert args.all_trackers and not args.speech
    with pytest.raises(SystemExit):
        parser.parse_args(["--all-trackers", "--vive-role", "Pelvis"])


def test_batch_cli_records_all_stages_even_when_one_fails(tmp_path, monkeypatch):
    closed = []

    class FakeSource:
        def __init__(self, *args, **kwargs):
            self.client = SimpleNamespace(session_id=7)

        def __enter__(self):
            return self

        def __exit__(self, *args):
            closed.append(self)

    called = []

    def capture(provider, args, *, role, serial, index, total):
        called.append(role)
        assert serial == "vive:" + role
        assert total == 5
        samples = []
        for t in (1, 1.1):
            f = frame(t, distance=33.5 if role == "Left_Knee" else .08)
            samples.append(TrackerFrame(t, {
                "Right_Wrist": f.poses["Right_Wrist"],
                role: f.poses["Pelvis"].replaced(role=role, device_id=serial),
            }, "test", "openvr_standing"))
        return samples

    monkeypatch.setattr(spatial_check, "OpenVRProvider", FakeSource)
    monkeypatch.setattr(spatial_check, "ViveUltimateLinuxProvider", FakeSource)
    monkeypatch.setattr(spatial_check, "QuestViveHybridProvider", lambda *a: object())
    monkeypatch.setattr(spatial_check, "_vive_linux_mapping", lambda p: {"vive:"+r: r for r in BATCH_ROLES})
    monkeypatch.setattr(spatial_check, "load_json", lambda p: {"calibrated": True})
    monkeypatch.setattr(spatial_check, "capture_stage", capture)

    status = spatial_check.main(["--all-trackers", "--no-speech", "--output-root", str(tmp_path)])
    assert status == 2
    assert called == list(BATCH_ROLES)
    assert len(closed) == 2
    output, = tmp_path.iterdir()
    result = json.loads((output/"summary.json").read_text())
    assert len(result["stages"]) == 5
    assert not result["all_proximity_checks_consistent"]
    for index, role in enumerate(BATCH_ROLES, 1):
        directory = output / f"{index:02d}_{role}"
        records = [json.loads(line) for line in (directory/"poses.jsonl").read_text().splitlines()]
        assert len(records) == 2
        assert records[0]["poses"][role]["timestamp_s"] == 1
        assert json.loads((directory/"summary.json").read_text())["vive_session_id"] == 7
