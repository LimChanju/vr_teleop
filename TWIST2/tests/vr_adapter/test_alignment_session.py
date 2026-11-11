import copy
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from twist2_vr import alignment_session as session
from twist2_vr.math3d import matrix_to_quaternion, quaternion_to_matrix
from twist2_vr.models import PoseSample, TrackerFrame


SERIAL = "FA61N3B00314"


def transform(rng):
    q = rng.normal(size=4)
    result = np.eye(4)
    result[:3, :3] = quaternion_to_matrix(q / np.linalg.norm(q))
    result[:3, 3] = rng.uniform(-0.5, 0.5, 3)
    return result


def pose(matrix, role, timestamp, device_id):
    return PoseSample(device_id=device_id, role=role, position_m=matrix[:3, 3],
                      orientation_xyzw=matrix_to_quaternion(matrix[:3, :3]),
                      timestamp_s=timestamp, tracking_state="ready",
                      metadata={"connection_generation": 0})


def make_records():
    rng = np.random.default_rng(20261014)
    world, fixture = transform(rng), transform(rng)
    stages = {phase: [] for phase in session.PHASES}
    for i in range(64):
        v = transform(rng)
        q = world @ v @ fixture
        t = 10 + i * 0.02
        phase = session.PHASES[i // 32]
        stages[phase].append({
            "timestamp_s": t + .001, "stage": phase, "vive_session_id": 5,
            "quest_coordinate_frame": "openvr_standing", "vive_coordinate_frame": "vive_hub_world",
            "quest": session.pose_record(pose(q, "Right_Wrist", t, "touch-right")),
            "vive": session.pose_record(pose(v, session.TRACKER_ROLE, t - .003, SERIAL)),
        })
    return stages, world, fixture


def args_for_test():
    return session.build_parser().parse_args(["--tracker-serial", SERIAL, "--fit-max-samples", "24"])


def test_exact_fixture_passes_and_only_training_enters_solver(monkeypatch):
    stages, world, fixture = make_records()
    original_solver = session.solve_hand_eye
    calls = []

    def solver(q, v):
        calls.append((q, v))
        return original_solver(q, v)

    monkeypatch.setattr(session, "solve_hand_eye", solver)
    report, candidate = session.analyse_session(stages, args_for_test())
    assert report["held_out_validation_passed"]
    assert len(calls) == 1 and len(calls[0][0]) == 24
    assert report["solver"]["used_validation_samples"] is False
    np.testing.assert_allclose(candidate["source_to_target_matrix"], world, atol=1e-8)
    np.testing.assert_allclose(candidate["calibration_fixture"]["tracker_from_controller_matrix"], fixture, atol=1e-8)
    assert candidate["held_out_validation"]["position_rmse_m"] < 1e-8
    assert report["phases"]["calibration"]["residuals"]["sample_count"] == 32
    assert candidate["installed"] is False
    assert report["mounts_calibrated"] is False
    assert report["simulation_publish_enabled"] is False


def test_holdout_offset_fails_without_refitting_world():
    stages, world, fixture = make_records()
    for r in stages["validation"]:
        r["quest"]["position_m"][0] += .08
    report, candidate = session.analyse_session(stages, args_for_test())
    assert candidate is None
    assert "validation: position_rmse_exceeds_threshold" in report["issues"]
    assert report["phases"]["validation"]["residuals"]["position_rmse_m"] == pytest.approx(.08)
    np.testing.assert_allclose(report["estimated_transform_for_diagnostics"]["source_to_target_matrix"], world, atol=1e-8)


def test_fixture_slip_in_validation_fails():
    stages, world, fixture = make_records()
    slipped = fixture.copy()
    slipped[:3, :3] = slipped[:3, :3] @ quaternion_to_matrix([0, 0, np.sin(.2), np.cos(.2)])
    for r in stages["validation"]:
        v = session.pose_to_matrix(PoseSample(**r["vive"]))
        r["quest"] = session.pose_record(pose(world @ v @ slipped, "Right_Wrist",
                                              r["quest"]["timestamp_s"], "touch-right"))
    report, candidate = session.analyse_session(stages, args_for_test())
    assert candidate is None
    assert "validation: orientation_rmse_exceeds_threshold" in report["issues"]


@pytest.mark.parametrize("fault", ["invalid", "stale", "future", "skew", "frame", "serial", "missing", "duplicate", "nan_pose"])
def test_bad_input_is_retained_but_not_accepted(fault):
    stages, _, _ = make_records()
    for r in stages["validation"]:
        if fault == "invalid":
            r["vive"]["valid"] = False
            r["vive"]["tracking_state"] = "rotation_only"
        elif fault == "stale":
            r["vive"]["timestamp_s"] -= 1
        elif fault == "future":
            r["quest"]["timestamp_s"] += 1
        elif fault == "skew":
            r["vive"]["timestamp_s"] -= .08
        elif fault == "frame":
            r["vive_coordinate_frame"] = "openvr_standing"
        elif fault == "serial":
            r["vive"]["device_id"] = "wrong-tracker"
        elif fault == "missing":
            r["quest"] = None
        elif fault == "duplicate":
            r["vive"]["timestamp_s"] = stages["validation"][0]["vive"]["timestamp_s"]
        else:
            r["quest"]["position_m"][0] = float("nan")
    report, candidate = session.analyse_session(stages, args_for_test())
    assert candidate is None and not report["held_out_validation_passed"]
    assert report["phases"]["validation"]["valid_fraction"] < .8
    assert report["phases"]["validation"]["discarded_reasons"]


@pytest.mark.parametrize("change", ["generation", "identity", "server_session"])
def test_reconnection_between_stages_rejected(change):
    stages, _, _ = make_records()
    for r in stages["validation"]:
        if change == "generation":
            r["quest"]["metadata"]["connection_generation"] = 1
        elif change == "identity":
            r["quest"]["device_id"] = "other-controller"
        else:
            r["vive_session_id"] = 6
    report, candidate = session.analyse_session(stages, args_for_test())
    assert candidate is None
    assert any("changed" in issue for issue in report["issues"])


def test_validation_must_be_new_samples():
    stages, _, _ = make_records()
    stages["validation"] = copy.deepcopy(stages["calibration"])
    report, candidate = session.analyse_session(stages, args_for_test())
    assert candidate is None
    assert "validation_reuses_or_precedes_calibration_samples" in report["issues"]


def test_stationary_validation_does_not_certify_alignment():
    stages, _, _ = make_records()
    first = copy.deepcopy(stages["validation"][0])
    for r in stages["validation"]:
        for source in ("quest", "vive"):
            r[source]["position_m"] = first[source]["position_m"][:]
            r[source]["orientation_xyzw"] = first[source]["orientation_xyzw"][:]
    report, candidate = session.analyse_session(stages, args_for_test())
    assert candidate is None
    assert "validation: vive_motion_coverage_too_small" in report["issues"]


def test_underconstrained_calibration_produces_diagnostics_not_candidate():
    stages, _, _ = make_records()
    first = copy.deepcopy(stages["calibration"][0])
    for r in stages["calibration"]:
        for source in ("quest", "vive"):
            r[source]["position_m"] = first[source]["position_m"][:]
            r[source]["orientation_xyzw"] = first[source]["orientation_xyzw"][:]
    report, candidate = session.analyse_session(stages, args_for_test())
    assert candidate is None
    assert any(issue.startswith("calibration_solver:") for issue in report["issues"])


def test_bad_frame_not_used_even_when_pose_is_fresh():
    stages, _, _ = make_records()
    stages["calibration"][1]["quest_coordinate_frame"] = "unexpected"
    report, candidate = session.analyse_session(stages, args_for_test())
    assert candidate is not None  # one rejected sample is reported, coverage still adequate
    assert report["phases"]["calibration"]["discarded_reasons"] == {"unexpected_coordinate_frame": 1}


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(seconds, .001)


class FakeProvider:
    def __init__(self, clock, source, fail_after=None, valid=True):
        self.clock, self.source = clock, source
        self.client = SimpleNamespace(session_id=5)
        self.fail_after, self.valid = fail_after, valid
        self.closed = False
        self.reads = 0

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.closed = True

    def read(self):
        self.reads += 1
        if self.fail_after and self.reads >= self.fail_after:
            raise KeyboardInterrupt
        t = self.clock.monotonic()
        matrix = transform(np.random.default_rng(int(t * 1e5)))
        role = "Right_Wrist" if self.source == "quest" else session.TRACKER_ROLE
        serial = "touch-right" if self.source == "quest" else SERIAL
        p = pose(matrix, role, t, serial).replaced(valid=self.valid)
        return TrackerFrame(t, {role: p}, self.source,
                            "openvr_standing" if self.source == "quest" else "vive_hub_world")


def install_fakes(monkeypatch, tmp_path, *, fail_after=None, valid=True):
    clock = FakeClock()
    providers = []

    def factory(source):
        def create(*a, **kw):
            p = FakeProvider(clock, source, fail_after=fail_after, valid=valid)
            providers.append(p)
            return p
        return create

    monkeypatch.setattr(session, "time", clock)
    monkeypatch.setattr(session, "announce", lambda *a, **kw: None)
    monkeypatch.setattr(session, "OpenVRProvider", factory("quest"))
    monkeypatch.setattr(session, "ViveUltimateLinuxProvider", factory("vive"))
    project = tmp_path / "project"
    monkeypatch.setattr(session, "ROOT", project)
    active = project / "config/local/calibration.quest_vive.json"
    active.parent.mkdir(parents=True)
    active.write_text('{"keep": "active calibration"}\n')
    return providers, active


@pytest.mark.parametrize("mode", ["pass", "tracking_timeout", "interrupt"])
def test_complete_cli_preserves_active_file_and_failure_records(monkeypatch, tmp_path, mode):
    providers, active = install_fakes(monkeypatch, tmp_path,
                                     valid=mode != "tracking_timeout",
                                     fail_after=12 if mode == "interrupt" else None)
    original = active.read_bytes()
    out = tmp_path / "records"
    result = session.main(["--tracker-serial", SERIAL, "--countdown", "0", "--frequency", "10",
                           "--fit-seconds", "1", "--validation-seconds", "1", "--ready-timeout", "1",
                           "--no-speech", "--output-root", str(out)])
    folder, = out.iterdir()
    report = json.loads((folder / "summary.json").read_text())
    records = [json.loads(l) for l in (folder / "poses.jsonl").read_text().splitlines()]
    assert records and (folder / "session.log").stat().st_size
    assert active.read_bytes() == original
    assert (folder / "active_calibration_snapshot.json").read_bytes() == original
    assert all(p.closed for p in providers)
    if mode == "pass":
        assert result == 0 and report["held_out_validation_passed"]
        assert (folder / "candidate.json").exists()
        assert {"calibration", "validation"} <= {r["stage"] for r in records}
    elif mode == "interrupt":
        assert result == 130 and report["status"] == "interrupted"
        assert not (folder / "candidate.json").exists()
    else:
        assert result == 2 and report["status"] == "failed"
        assert not (folder / "candidate.json").exists()


@pytest.mark.parametrize("option,value", [("--fit-seconds", "nan"), ("--validation-seconds", "0"),
                                          ("--max-skew", "-1"), ("--fit-max-samples", "2")])
def test_invalid_args_fail_before_hardware_or_writes(tmp_path, option, value):
    out = tmp_path / "uncreated"
    with pytest.raises(SystemExit) as result:
        session.main(["--tracker-serial", SERIAL, "--output-root", str(out), option, value])
    assert result.value.code == 2 and not out.exists()


def test_write_json_never_overwrites(tmp_path):
    path = tmp_path / "candidate.json"
    session.write_json(path, {"first": True})
    with pytest.raises(FileExistsError):
        session.write_json(path, {"second": True})
    assert json.loads(path.read_text()) == {"first": True}
