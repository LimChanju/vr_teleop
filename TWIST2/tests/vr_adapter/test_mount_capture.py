import json

import numpy as np
import pytest

from twist2_vr import mount_capture
from twist2_vr.models import PoseSample, TrackerFrame
from twist2_vr.mount_capture import ROLES, build_parser, summarize


LOWER_HEAD_ROLES = ("Head", "Pelvis", "Left_Knee", "Right_Knee", "Left_Foot", "Right_Foot")
WRIST_ROLES = ("Left_Wrist", "Right_Wrist")


def frame(t=1.0, *, sign=1, missing=None, invalid=None, shift=0, generation=0):
    poses = {
        role: PoseSample(role, role, [shift, 0, 1], [0, 0, 0, sign], t,
                         valid=role != invalid,
                         tracking_state="unknown_18" if role == invalid else "ready",
                         metadata={"connection_generation": generation})
        for role in ROLES if role != missing
    }
    return TrackerFrame(t, poses, "test", "openvr_standing")


def test_stable_antipodal_quaternions_are_diagnostics_not_calibration():
    result = summarize([frame(), frame(1.1, sign=-1)])
    assert result["stable_capture"]
    assert result["all_eight_valid_frames"] == 2
    assert result["all_required_valid_frames"] == 2
    assert result["excluded_roles"] == []
    assert result["required_roles"] == list(ROLES)
    assert result["partial_capture"] is False
    assert result["capture_mode"] == "full_body"
    assert result["full_body_stable_capture"] is True
    assert all(row["included_in_stability_check"] for row in result["roles"].values())
    assert result["mounts_calibrated"] is False
    np.testing.assert_allclose(result["roles"]["Pelvis"]["mean_orientation_xyzw"], [0, 0, 0, 1])


@pytest.mark.parametrize("kwargs", [{"missing": "Head"}, {"invalid": "Left_Knee"}])
def test_missing_or_unknown_tracking_is_not_accepted(kwargs):
    result = summarize([frame(**kwargs), frame(1.1, **kwargs)])
    assert not result["stable_capture"]
    assert result["all_eight_valid_frames"] == 0


def test_motion_and_reconnection_are_not_accepted_as_stable():
    assert not summarize([frame(), frame(1.1, shift=0.1)])["stable_capture"]
    assert not summarize([frame(), frame(1.1, generation=1)])["stable_capture"]


def test_stale_samples_rejected_and_frame_required():
    sample = frame()
    stale = TrackerFrame(2, sample.poses, "test", "openvr_standing")
    assert not summarize([stale])["stable_capture"]
    assert summarize([stale])["roles"]["Head"]["valid_samples"] == 0
    with pytest.raises(ValueError, match="no frames"):
        summarize([])
    with pytest.raises(ValueError, match="openvr_standing"):
        summarize([TrackerFrame(1, sample.poses, "test", "wrong")])


def test_defaults_use_local_mapping_and_do_not_offer_publication():
    args = build_parser().parse_args([])
    assert args.mapping.parts[-2:] == ("local", "tracker_mapping.vive_linux.json")
    assert args.countdown == 10 and args.seconds == 5
    assert args.source == "vive-linux"
    assert args.exclude_role == []
    assert args.lower_body_head is False
    assert not hasattr(args, "publish_sim")


def pelvis_problem(t, condition):
    sample = frame(t, missing="Pelvis" if condition == "missing" else None,
                   invalid="Pelvis" if condition == "invalid" else None)
    if condition == "stale":
        poses = dict(sample.poses)
        poses["Pelvis"] = poses["Pelvis"].replaced(timestamp_s=t - 1.0)
        sample = TrackerFrame(t, poses, "test", "openvr_standing")
    return sample


@pytest.mark.parametrize("condition", ["missing", "invalid", "stale"])
def test_bad_pelvis_requires_explicit_exclusion_and_remains_in_report(condition):
    frames = [pelvis_problem(1.0, condition), pelvis_problem(1.1, condition)]
    default = summarize(frames)
    assert not default["stable_capture"]
    assert not default["full_body_stable_capture"]
    assert default["all_required_valid_frames"] == 0

    result = summarize(frames, exclude_roles=("Pelvis",))
    assert result["stable_capture"]
    assert result["partial_capture"] is True
    assert result["capture_mode"] == "pelvis_excluded"
    assert result["full_body_stable_capture"] is False
    assert result["mounts_calibrated"] is False
    assert result["all_eight_valid_frames"] == 0
    assert result["all_required_valid_frames"] == 2
    assert result["excluded_roles"] == ["Pelvis"]
    assert result["required_roles"] == [role for role in ROLES if role != "Pelvis"]
    assert set(result["roles"]) == set(ROLES)
    pelvis = result["roles"]["Pelvis"]
    assert pelvis["included_in_stability_check"] is False
    assert pelvis["valid_samples"] == 0
    assert pelvis["valid_fraction"] == 0
    assert pelvis["stable_capture"] is False
    expected_state = {"missing": "missing", "invalid": "unknown_18", "stale": "ready"}[condition]
    assert pelvis["tracking_states"] == {expected_state: 2}
    assert "mean_position_m" not in pelvis
    assert all(result["roles"][role]["included_in_stability_check"]
               for role in result["required_roles"])


@pytest.mark.parametrize("change", [
    {"position_m": [0.1, 0, 1]},
    {"metadata": {"connection_generation": 1}},
])
def test_excluded_pelvis_motion_or_reconnection_is_reported_not_used_for_stability(change):
    moved = frame(1.1)
    poses = dict(moved.poses)
    poses["Pelvis"] = poses["Pelvis"].replaced(**change)
    moved = TrackerFrame(1.1, poses, "test", "openvr_standing")
    assert not summarize([frame(), moved])["stable_capture"]
    result = summarize([frame(), moved], exclude_roles=["Pelvis"])
    assert result["stable_capture"]
    assert not result["roles"]["Pelvis"]["stable_capture"]
    assert result["all_eight_valid_frames"] == result["all_required_valid_frames"] == 2
    assert result["partial_capture"] and not result["full_body_stable_capture"]


def test_exclusion_is_partial_even_when_all_eight_roles_happen_to_be_stable():
    result = summarize([frame(), frame(1.1)], exclude_roles=["Pelvis"])
    assert result["stable_capture"]
    assert result["all_eight_valid_frames"] == result["all_required_valid_frames"] == 2
    assert result["roles"]["Pelvis"]["stable_capture"]
    assert not result["roles"]["Pelvis"]["included_in_stability_check"]
    assert result["partial_capture"] is True
    assert result["full_body_stable_capture"] is False


@pytest.mark.parametrize("problem", ["missing", "invalid", "stale", "motion", "reconnected"])
def test_pelvis_exclusion_does_not_hide_other_role_failure(problem):
    first = frame()
    second = frame(1.1, missing="Left_Foot" if problem == "missing" else None,
                   invalid="Left_Foot" if problem == "invalid" else None)
    if problem in {"stale", "motion", "reconnected"}:
        changes = {
            "stale": {"timestamp_s": 0.0},
            "motion": {"position_m": [0.1, 0, 1]},
            "reconnected": {"metadata": {"connection_generation": 1}},
        }
        poses = dict(second.poses)
        poses["Left_Foot"] = poses["Left_Foot"].replaced(**changes[problem])
        second = TrackerFrame(1.1, poses, "test", "openvr_standing")
    result = summarize([first, second], exclude_roles=["Pelvis"])
    assert not result["stable_capture"]
    assert result["partial_capture"] and not result["full_body_stable_capture"]
    assert result["roles"]["Left_Foot"]["included_in_stability_check"]
    assert not result["roles"]["Left_Foot"]["stable_capture"]


@pytest.mark.parametrize("excluded", [(role,) for role in ROLES if role != "Pelvis"]
                         + [("Unknown",), ("pelvis",), ("Pelvis", "Head")])
def test_only_pelvis_may_be_excluded(excluded):
    with pytest.raises(ValueError):
        summarize([frame(), frame(1.1)], exclude_roles=excluded)


def test_cli_pelvis_exclusion_is_explicit_and_rejects_other_roles():
    assert build_parser().parse_args(["--exclude-role", "Pelvis"]).exclude_role == ["Pelvis"]
    for role in ("Head", "Unknown"):
        with pytest.raises(SystemExit):
            build_parser().parse_args(["--exclude-role", role])


def lower_body_head_frame(t=1.0, *, wrist_condition="missing"):
    sample = frame(t)
    poses = dict(sample.poses)
    for role in WRIST_ROLES:
        if wrist_condition == "missing":
            del poses[role]
        elif wrist_condition == "stale":
            poses[role] = poses[role].replaced(timestamp_s=t - 1.0, position_m=[8, 9, 10])
        else:
            poses[role] = poses[role].replaced(
                valid=False, tracking_state="unknown_18", position_m=[8, 9, 10],
            )
    return TrackerFrame(t, poses, "test", "openvr_standing")


@pytest.mark.parametrize("wrist_condition", ["missing", "invalid", "stale"])
def test_lower_body_head_mode_accepts_six_roles_without_controllers(wrist_condition):
    frames = [lower_body_head_frame(t, wrist_condition=wrist_condition) for t in (1.0, 1.1)]
    assert not summarize(frames)["stable_capture"]
    assert not summarize(frames, exclude_roles=["Pelvis"])["stable_capture"]
    result = summarize(frames, lower_body_head=True)
    assert mount_capture.LOWER_BODY_HEAD_ROLES == LOWER_HEAD_ROLES
    assert result["capture_mode"] == "lower_body_head"
    assert result["required_roles"] == list(LOWER_HEAD_ROLES)
    assert result["excluded_roles"] == list(WRIST_ROLES)
    assert result["stable_capture"] and result["partial_capture"]
    assert not result["full_body_stable_capture"] and not result["mounts_calibrated"]
    assert result["all_required_valid_frames"] == 2
    assert result["all_eight_valid_frames"] == 0
    assert set(result["roles"]) == set(ROLES)
    for role in LOWER_HEAD_ROLES:
        assert result["roles"][role]["included_in_stability_check"]
        assert result["roles"][role]["stable_capture"]
    for role in WRIST_ROLES:
        row = result["roles"][role]
        assert not row["included_in_stability_check"]
        assert not row["stable_capture"]
        assert row["valid_samples"] == 0
        state = {"missing": "missing", "invalid": "unknown_18", "stale": "ready"}[wrist_condition]
        assert row["tracking_states"] == {state: 2}


@pytest.mark.parametrize("role", LOWER_HEAD_ROLES)
@pytest.mark.parametrize("problem", ["missing", "invalid", "stale", "motion", "reconnected"])
def test_lower_body_head_mode_requires_head_and_every_lower_tracker_to_be_stable(role, problem):
    first = lower_body_head_frame()
    second = lower_body_head_frame(1.1)
    poses = dict(second.poses)
    if problem == "missing":
        del poses[role]
    else:
        changes = {
            "invalid": {"valid": False, "tracking_state": "unknown_18"},
            "stale": {"timestamp_s": 0.0},
            "motion": {"position_m": [0.1, 0, 1]},
            "reconnected": {"metadata": {"connection_generation": 1}},
        }
        poses[role] = poses[role].replaced(**changes[problem])
    second = TrackerFrame(1.1, poses, "test", "openvr_standing")
    result = summarize([first, second], lower_body_head=True)
    assert not result["stable_capture"]
    assert result["partial_capture"] and not result["full_body_stable_capture"]
    assert result["required_roles"] == list(LOWER_HEAD_ROLES)
    assert not result["roles"][role]["stable_capture"]
    assert result["roles"][role]["included_in_stability_check"]


def test_lower_body_head_mode_never_claims_full_body_even_when_all_eight_are_stable():
    result = summarize([frame(), frame(1.1)], lower_body_head=True)
    assert result["stable_capture"]
    assert result["all_required_valid_frames"] == result["all_eight_valid_frames"] == 2
    assert result["capture_mode"] == "lower_body_head"
    assert result["partial_capture"] and not result["full_body_stable_capture"]
    assert all(row["stable_capture"] for row in result["roles"].values())
    for role in WRIST_ROLES:
        assert not result["roles"][role]["included_in_stability_check"]


def test_lower_body_head_mode_cannot_be_combined_with_pelvis_exclusion():
    with pytest.raises(ValueError):
        summarize([frame(), frame(1.1)], lower_body_head=True, exclude_roles=["Pelvis"])


@pytest.mark.parametrize("flags", [
    ["--lower-body-head", "--exclude-role", "Pelvis"],
    ["--exclude-role", "Pelvis", "--lower-body-head"],
])
def test_cli_incompatible_capture_modes_fail_before_settings_or_providers(monkeypatch, flags):
    monkeypatch.setattr(mount_capture, "load_json", forbidden)
    monkeypatch.setattr(mount_capture, "_vive_linux_mapping", forbidden)
    monkeypatch.setattr(mount_capture, "OpenVRProvider", forbidden)
    monkeypatch.setattr(mount_capture, "ViveUltimateLinuxProvider", forbidden)
    monkeypatch.setattr(mount_capture, "QuestViveHybridProvider", forbidden)
    with pytest.raises(SystemExit):
        mount_capture.main(flags)


@pytest.fixture
def capture_environment(monkeypatch, tmp_path):
    class Clock:
        now = 1.0

        def monotonic(self):
            return self.now

        def sleep(self, seconds):
            self.now += seconds

    clock = Clock()
    monkeypatch.setattr(mount_capture.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(mount_capture.time, "sleep", clock.sleep)
    mapping = {f"serial-{role}": role for role in ROLES
               if role not in {"Head", "Left_Wrist", "Right_Wrist"}}
    configs = {
        "mapping": {"serial_to_role": mapping},
        "coordinates": {"mounts_calibrated": False,
                        "mount_observations": {"source": "test"},
                        "tracker_to_body": {"Pelvis": {"translation_m": [123, 0, 0]}}},
        "alignment": {"test_only_alignment": True},
        "profile": {"reference_tracking_system": "test", "rotation": [0, 90, 0]},
    }
    paths = {name: tmp_path / f"{name}.json" for name in configs}
    for name, config in configs.items():
        paths[name].write_text(json.dumps(config), encoding="utf-8")
    originals = {name: path.read_bytes() for name, path in paths.items()}
    args = ["--mapping", str(paths["mapping"]), "--coordinates", str(paths["coordinates"]),
            "--cross-calibration", str(paths["alignment"]),
            "--spacecal-profile", str(paths["profile"]),
            "--output-root", str(tmp_path / "output"),
            "--countdown", "0", "--seconds", "0.06", "--frequency", "50"]
    return clock, mapping, paths, configs, originals, args


def forbidden(*args, **kwargs):
    pytest.fail("the unselected input/calibration path must not be accessed")


def test_openvr_capture_never_loads_sdk_alignment_or_transforms_poses(
    monkeypatch, tmp_path, capture_environment,
):
    clock, mapping, paths, configs, originals, args = capture_environment
    connections = []

    class Provider:
        def __init__(self, serial_to_role, *, action_manifest):
            assert serial_to_role == mapping
            connections.append(self)
            self.closed = False

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            self.closed = True

        def read(self):
            return frame(clock.now, shift=0.37)

    original_load_json = mount_capture.load_json

    def load_without_alignment(path):
        assert path != paths["alignment"]
        return original_load_json(path)

    monkeypatch.setattr(mount_capture, "load_json", load_without_alignment)
    monkeypatch.setattr(mount_capture, "OpenVRProvider", Provider)
    monkeypatch.setattr(mount_capture, "ViveUltimateLinuxProvider", forbidden)
    monkeypatch.setattr(mount_capture, "QuestViveHybridProvider", forbidden)
    assert mount_capture.main([*args, "--source", "openvr"]) == 0
    assert len(connections) == 1 and connections[0].closed
    output = next((tmp_path / "output").iterdir())
    report = json.loads((output / "summary.json").read_text())
    assert report["capture_source"] == "openvr"
    assert report["stable_capture"] and not report["mounts_calibrated"]
    alignment = report["spatial_alignment"]
    assert alignment["expected_runtime_alignment"] == "xi-ve SpaceCalibrator"
    assert not alignment["application_alignment_applied"]
    assert not alignment["runtime_alignment_verified"]
    assert not alignment["profile_changed_during_capture"]
    assert alignment["profile_before"]["profile"] == configs["profile"]
    assert alignment["profile_before"]["sha256"] == alignment["profile_after"]["sha256"]
    assert report["mount_observations"] == {"source": "test"}
    records = [json.loads(line) for line in (output / "poses.jsonl").read_text().splitlines()]
    assert len(records) == report["frame_count"] >= 2
    for record in records:
        np.testing.assert_allclose(record["poses"]["Pelvis"]["position_m"], [0.37, 0, 1])
        assert record["poses"]["Pelvis"]["timestamp_s"] == record["timestamp_s"]
    for name, path in paths.items():
        assert path.read_bytes() == originals[name]


@pytest.mark.parametrize("condition", ["missing", "invalid", "stale"])
def test_mocked_openvr_cli_pelvis_exclusion_preserves_raw_data_and_settings(
    monkeypatch, tmp_path, capsys, capture_environment, condition,
):
    clock, mapping, paths, configs, originals, args = capture_environment
    connections = []

    class Provider:
        def __init__(self, serial_to_role, *, action_manifest):
            assert serial_to_role == mapping
            self.closed = False
            connections.append(self)

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            self.closed = True

        def read(self):
            return pelvis_problem(clock.now, condition)

    original_load_json = mount_capture.load_json

    def load_without_alignment(path):
        assert path != paths["alignment"]
        return original_load_json(path)

    monkeypatch.setattr(mount_capture, "load_json", load_without_alignment)
    monkeypatch.setattr(mount_capture, "OpenVRProvider", Provider)
    monkeypatch.setattr(mount_capture, "ViveUltimateLinuxProvider", forbidden)
    monkeypatch.setattr(mount_capture, "QuestViveHybridProvider", forbidden)
    assert mount_capture.main([*args, "--source", "openvr", "--exclude-role", "Pelvis"]) == 0
    assert len(connections) == 1 and connections[0].closed
    stdout = capsys.readouterr().out
    assert "Partial capture" in stdout
    assert "NOT full-body" in stdout
    assert "NOT calibrated" in stdout

    output = next((tmp_path / "output").iterdir())
    assert {path.name for path in output.iterdir()} == {"poses.jsonl", "summary.json"}
    report = json.loads((output / "summary.json").read_text())
    assert report["stable_capture"] and report["partial_capture"]
    assert not report["full_body_stable_capture"] and not report["mounts_calibrated"]
    assert report["excluded_roles"] == ["Pelvis"]
    assert report["required_roles"] == [role for role in ROLES if role != "Pelvis"]
    assert report["all_required_valid_frames"] == report["frame_count"] >= 2
    assert report["all_eight_valid_frames"] == 0
    assert set(report["roles"]) == set(ROLES)
    assert report["roles"]["Pelvis"]["valid_samples"] == 0
    assert not report["roles"]["Pelvis"]["included_in_stability_check"]
    assert not report["spatial_alignment"]["application_alignment_applied"]
    assert not report["spatial_alignment"]["runtime_alignment_verified"]
    assert not report["spatial_alignment"]["profile_changed_during_capture"]
    records = [json.loads(line) for line in (output / "poses.jsonl").read_text().splitlines()]
    assert len(records) == report["frame_count"]
    for record in records:
        if condition == "missing":
            assert "Pelvis" not in record["poses"]
            continue
        pelvis = record["poses"]["Pelvis"]
        np.testing.assert_allclose(pelvis["position_m"], [0, 0, 1])
        assert pelvis["metadata"] == {"connection_generation": 0}
        if condition == "invalid":
            assert pelvis["valid"] is False
            assert pelvis["tracking_state"] == "unknown_18"
            assert pelvis["timestamp_s"] == record["timestamp_s"]
        else:
            assert pelvis["valid"] is True
            assert pelvis["tracking_state"] == "ready"
            assert pelvis["timestamp_s"] == pytest.approx(record["timestamp_s"] - 1.0)
    for name, path in paths.items():
        assert path.read_bytes() == originals[name]


@pytest.mark.parametrize("wrist_condition, missing_head", [
    ("missing", False), ("invalid", False), ("stale", False), ("missing", True),
])
def test_mocked_cli_lower_body_head_preserves_raw_data_and_requires_hmd(
    monkeypatch, tmp_path, capsys, capture_environment, wrist_condition, missing_head,
):
    clock, mapping, paths, configs, originals, args = capture_environment
    connections = []
    samples = []

    class Provider:
        def __init__(self, serial_to_role, *, action_manifest):
            assert serial_to_role == mapping
            self.closed = False
            connections.append(self)

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            self.closed = True

        def read(self):
            sample = lower_body_head_frame(clock.now, wrist_condition=wrist_condition)
            if missing_head:
                sample = TrackerFrame(
                    clock.now, {role: pose for role, pose in sample.poses.items() if role != "Head"},
                    "test", "openvr_standing",
                )
            samples.append(sample)
            return sample

    original_load_json = mount_capture.load_json

    def load_without_alignment(path):
        assert path != paths["alignment"]
        return original_load_json(path)

    monkeypatch.setattr(mount_capture, "load_json", load_without_alignment)
    monkeypatch.setattr(mount_capture, "OpenVRProvider", Provider)
    monkeypatch.setattr(mount_capture, "ViveUltimateLinuxProvider", forbidden)
    monkeypatch.setattr(mount_capture, "QuestViveHybridProvider", forbidden)
    exit_code = mount_capture.main([*args, "--source", "openvr", "--lower-body-head"])
    assert exit_code == (2 if missing_head else 0)
    assert len(connections) == 1 and connections[0].closed
    stdout = capsys.readouterr().out
    assert "Lower-body + head" in stdout and "6 roles" in stdout
    assert "NOT full-body" in stdout and "NOT calibrated" in stdout
    assert "controllers in both hands" not in stdout

    output = next((tmp_path / "output").iterdir())
    assert {path.name for path in output.iterdir()} == {"poses.jsonl", "summary.json"}
    report = json.loads((output / "summary.json").read_text())
    assert report["capture_mode"] == "lower_body_head"
    assert report["required_roles"] == list(LOWER_HEAD_ROLES)
    assert report["excluded_roles"] == list(WRIST_ROLES)
    assert report["partial_capture"] and not report["full_body_stable_capture"]
    assert not report["mounts_calibrated"]
    assert report["stable_capture"] is (not missing_head)
    assert report["all_eight_valid_frames"] == 0
    assert report["all_required_valid_frames"] == (0 if missing_head else report["frame_count"])
    assert set(report["roles"]) == set(ROLES)
    assert not report["spatial_alignment"]["application_alignment_applied"]
    assert not report["spatial_alignment"]["runtime_alignment_verified"]
    assert not report["spatial_alignment"]["profile_changed_during_capture"]
    records = [json.loads(line) for line in (output / "poses.jsonl").read_text().splitlines()]
    assert len(records) == len(samples) == report["frame_count"] >= 2
    for record, sample in zip(records, samples):
        assert set(record["poses"]) == set(sample.poses)
        for role, pose in sample.poses.items():
            raw = record["poses"][role]
            np.testing.assert_allclose(raw["position_m"], pose.position_m)
            np.testing.assert_allclose(raw["orientation_xyzw"], pose.orientation_xyzw)
            assert raw["timestamp_s"] == pose.timestamp_s
            assert raw["valid"] == pose.valid
            assert raw["tracking_state"] == pose.tracking_state
            assert raw["metadata"] == pose.metadata
    for name, path in paths.items():
        assert path.read_bytes() == originals[name]


def test_default_sdk_capture_preserves_alignment_and_never_reads_spacecal_profile(
    monkeypatch, tmp_path, capture_environment,
):
    clock, mapping, paths, configs, originals, args = capture_environment
    connections = []

    class Provider:
        def __init__(self, serial_to_role, **kwargs):
            connections.append((serial_to_role, kwargs))

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            pass

        def read(self):
            return frame(clock.now)

    def hybrid(quest, vive, alignment):
        assert isinstance(quest, Provider) and isinstance(vive, Provider)
        assert alignment == configs["alignment"]
        return quest

    monkeypatch.setattr(mount_capture, "OpenVRProvider", Provider)
    monkeypatch.setattr(mount_capture, "ViveUltimateLinuxProvider", Provider)
    monkeypatch.setattr(mount_capture, "QuestViveHybridProvider", hybrid)
    monkeypatch.setattr(mount_capture, "_spacecal_profile_snapshot", forbidden)
    assert mount_capture.main(args) == 0
    assert [connection[0] for connection in connections] == [{}, mapping]
    report_path = next((tmp_path / "output").glob("*/summary.json"))
    report = json.loads(report_path.read_text())
    assert report["capture_source"] == "vive-linux"
    assert report["spatial_alignment"] == configs["alignment"]
    assert report["stable_capture"] and not report["mounts_calibrated"]
    for name, path in paths.items():
        assert path.read_bytes() == originals[name]


@pytest.mark.parametrize("contents, expected_status", [(None, "unavailable"), (b"{", "invalid_json")])
def test_profile_snapshot_failure_is_diagnostic_only(tmp_path, contents, expected_status):
    path = tmp_path / "profile.json"
    if contents is not None:
        path.write_bytes(contents)
    snapshot = mount_capture._spacecal_profile_snapshot(path)
    assert snapshot["status"] == expected_status
    assert "profile" not in snapshot
    assert "error" in snapshot
    if contents is not None:
        assert path.read_bytes() == contents


def test_openvr_capture_reports_profile_changes_without_claiming_alignment(
    monkeypatch, tmp_path, capture_environment,
):
    clock, mapping, paths, configs, originals, args = capture_environment

    class Provider:
        def __init__(self, *args, **kwargs):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            pass

        def read(self):
            paths["profile"].write_text('{"changed_by_runtime": true}', encoding="utf-8")
            return frame(clock.now)

    monkeypatch.setattr(mount_capture, "OpenVRProvider", Provider)
    monkeypatch.setattr(mount_capture, "ViveUltimateLinuxProvider", forbidden)
    monkeypatch.setattr(mount_capture, "QuestViveHybridProvider", forbidden)
    assert mount_capture.main([*args, "--source", "openvr"]) == 0
    report_path = next((tmp_path / "output").glob("*/summary.json"))
    report = json.loads(report_path.read_text())
    alignment = report["spatial_alignment"]
    assert alignment["profile_changed_during_capture"]
    assert alignment["profile_before"]["profile"] == configs["profile"]
    assert alignment["profile_after"]["profile"] == {"changed_by_runtime": True}
    assert not alignment["runtime_alignment_verified"] and not report["mounts_calibrated"]
