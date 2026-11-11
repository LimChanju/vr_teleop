"""Offline checks against the optional, unchanged upstream checkout and weights."""

from pathlib import Path
import json

import numpy as np
import pytest

pytest.importorskip("scipy")
pytest.importorskip("pymotion")
from scipy.spatial.transform import Rotation
from pymotion.ops.forward_kinematics import fk as upstream_fk

from twist2_vr.models import PoseSample, TrackerFrame
from twist2_vr.shin_refinement import forward_kinematics
from twist2_vr.sparseposer_adapter import (
    EightTrackerEstimator, MotionWindow, MountCalibration, ROLE_JOINT,
    fit_tpose_calibration, network_sample,
)
from twist2_vr.sparseposer_backend import SparsePoserBackend, load_skeleton


CHECKOUT = Path(__file__).resolve().parents[2] / "external" / "SparsePoser"
REFERENCE = CHECKOUT / "SparsePoserUnity" / "Assets" / "Data" / "male_170_85.txt"


@pytest.fixture(scope="module")
def skeleton():
    if not REFERENCE.is_file():
        pytest.skip("optional SparsePoser checkout is absent")
    return load_skeleton(CHECKOUT, REFERENCE)


def reference_world(skeleton):
    local_wxyz = skeleton.reference_local_rotations[:, [3, 0, 1, 2]]
    positions, matrices = upstream_fk(local_wxyz, np.zeros(3), skeleton.offsets, skeleton.parents)
    ground = min(positions[skeleton.names.index(name), 1] for name in ("LeftToe", "RightToe"))
    positions[:, 1] -= ground
    return positions, Rotation.from_matrix(matrices)


def synthetic_capture(skeleton, *, yaw=0.4, height=1.7, start=100.0, count=66):
    positions, _ = reference_world(skeleton)
    room = Rotation.from_euler("y", yaw)
    origin = np.array([1.1, 0.04, -2.0])
    scale = 1.7 / height
    frames = []
    for index in range(count):
        stamp = start + index / 60
        poses = {}
        for slot, (role, name) in enumerate(ROLE_JOINT.items()):
            joint = skeleton.names.index(name)
            # HMD optical -Z points along model +Z. The other sensors have
            # deliberate arbitrary mounting rotations and lever-arm offsets.
            mount_rotation = Rotation.from_euler("y", np.pi) if role == "Head" else Rotation.from_euler("xyz", [0.1 * slot, -0.08 * slot, 0.03 * slot])
            rotation = room * mount_rotation
            offset = np.zeros(3) if role == "Head" else np.array([0.015, -0.025, 0.01])
            position = room.apply(positions[joint] / scale) + origin - rotation.apply(offset)
            poses[role] = PoseSample(
                f"synthetic_{role}", role, position, rotation.as_quat(), stamp,
                metadata={"connection_generation": 1},
            )
        frames.append(TrackerFrame(stamp, poses, "synthetic_test_only", "openvr_standing"))
    return frames, positions, room, origin


def test_reference_fk_agrees_with_upstream_metres_and_root_convention(skeleton):
    local_wxyz = skeleton.reference_local_rotations[:, [3, 0, 1, 2]]
    expected_p, expected_r = upstream_fk(local_wxyz, np.array([0, 0.886734, 0]), skeleton.offsets, skeleton.parents)
    actual_p, actual_q = forward_kinematics(skeleton, skeleton.reference_local_rotations, [0, 0.886734, 0])
    np.testing.assert_allclose(actual_p, expected_p, atol=1e-7)
    np.testing.assert_allclose(Rotation.from_quat(actual_q).as_matrix(), expected_r, atol=1e-7)
    assert np.linalg.norm(skeleton.offsets[skeleton.names.index("LeftKnee")]) == pytest.approx(0.377351, abs=1e-6)


@pytest.mark.parametrize("yaw,height", [(0.4, 1.7), (-1.2, 1.85)])
def test_calibration_removes_room_transform_scale_and_rigid_mounts(skeleton, yaw, height):
    frames, expected_p, room, origin = synthetic_capture(skeleton, yaw=yaw, height=height)
    data = fit_tpose_calibration(frames, skeleton, height_m=height)
    calibration = MountCalibration(data, skeleton)
    np.testing.assert_allclose(calibration.origin, origin, atol=1e-7)
    np.testing.assert_allclose(calibration.world_from_model.as_matrix(), room.as_matrix(), atol=1e-7)
    targets = calibration.targets(frames[-1])
    _, expected_r = reference_world(skeleton)
    for role, name in ROLE_JOINT.items():
        joint = skeleton.names.index(name)
        np.testing.assert_allclose(targets[role][0], expected_p[joint], atol=1e-7)
        np.testing.assert_allclose(Rotation.from_quat(targets[role][1]).as_matrix(), expected_r[joint].as_matrix(), atol=1e-7)
    sample = network_sample(targets)
    np.testing.assert_allclose(sample[0, :3], targets["Pelvis"][0], atol=1e-7)
    root_rotation = Rotation.from_quat(targets["Pelvis"][1])
    np.testing.assert_allclose(sample[1, :3], root_rotation.inv().apply(targets["Left_Foot"][0] - targets["Pelvis"][0]), atol=1e-7)


def test_history_resamples_past_only_and_resets_after_gap():
    history = MotionWindow(hz=60, max_gap=0.2)
    result = None
    for index in range(81):
        stamp = index / 40
        sample = np.zeros((6, 7))
        sample[:, 0] = stamp
        sample[:, 3:] = Rotation.from_euler("y", 0.5 * stamp).as_quat()[[3, 0, 1, 2]]
        if index % 2:
            sample[:, 3:] *= -1
        result = history.append(stamp, sample)
    assert result is not None
    expected_times = np.linspace(2 - 63 / 60, 2, 64)
    np.testing.assert_allclose(result[:, 0, 0], expected_times, atol=1e-7)
    actual = Rotation.from_quat(result[:, 0, 3:][:, [1, 2, 3, 0]])
    np.testing.assert_allclose(actual.as_matrix(), Rotation.from_euler("y", 0.5 * expected_times).as_matrix(), atol=1e-7)
    assert history.append(2.3, sample) is None
    assert len(history.samples) == 1


@pytest.mark.parametrize("roles", [("Left_Foot", "Right_Foot"), ("Left_Knee", "Right_Knee")])
def test_tpose_rejects_swapped_lower_body_assignments(skeleton, roles):
    frames, *_ = synthetic_capture(skeleton)
    swapped = []
    left, right = roles
    for frame in frames:
        poses = dict(frame.poses)
        poses[left], poses[right] = poses[right].replaced(role=left), poses[left].replaced(role=right)
        swapped.append(TrackerFrame(frame.timestamp_s, poses, frame.source, frame.coordinate_frame))
    with pytest.raises(ValueError):
        fit_tpose_calibration(swapped, skeleton)


def test_tpose_rejects_reusing_one_device_for_multiple_roles(skeleton):
    frames, *_ = synthetic_capture(skeleton)
    duplicated = []
    for frame in frames:
        poses = dict(frame.poses)
        poses["Right_Knee"] = poses["Right_Knee"].replaced(device_id=poses["Left_Knee"].device_id)
        duplicated.append(TrackerFrame(frame.timestamp_s, poses, frame.source, frame.coordinate_frame))
    with pytest.raises(ValueError):
        fit_tpose_calibration(duplicated, skeleton)


def test_real_models_calibration_refinement_and_world_output(skeleton):
    torch = pytest.importorskip("torch")
    model_dir = CHECKOUT / "python" / "models" / "model_xsens"
    if not all((model_dir / name).is_file() for name in ("generator.pt", "ik.pt", "data.pt")):
        pytest.skip("optional official pretrained files are absent")
    frames, _, room, origin = synthetic_capture(skeleton)
    calibration = fit_tpose_calibration(frames, skeleton)
    old_threads = torch.get_num_threads()
    try:
        torch.set_num_threads(2)
        model = SparsePoserBackend(CHECKOUT, REFERENCE, model_dir)
        estimator = EightTrackerEstimator(model, calibration)
        outputs = [estimator.process(frame) for frame in frames]
    finally:
        torch.set_num_threads(old_threads)
    assert all(output is None for output in outputs[:63])
    output = outputs[-1]
    assert output is not None
    assert output["coordinate_frame"] == "openvr_standing"
    assert output["robot_output"] is False
    positions = np.asarray(output["positions_m"])
    rotations = np.asarray(output["orientations_xyzw"])
    assert positions.shape == (22, 3) and np.isfinite(positions).all()
    assert rotations.shape == (22, 4) and np.isfinite(rotations).all()
    np.testing.assert_allclose(np.linalg.norm(rotations, axis=-1), 1, atol=1e-7)
    expected_root = room.apply(MountCalibration(calibration, skeleton).targets(frames[-1])["Pelvis"][0]) + origin
    np.testing.assert_allclose(positions[0], expected_root, atol=1e-7)
    assert output["refinement"]["success"]


def test_cli_calibration_replay_and_overwrite_protection(skeleton, tmp_path):
    from twist2_vr.sparseposer_cli import main, _raw_record
    frames, *_ = synthetic_capture(skeleton)
    recording = tmp_path / "synthetic_input.jsonl"
    recording.write_text("".join(json.dumps(_raw_record(f))+"\n" for f in frames))
    profile = tmp_path / "synthetic_tpose.json"
    capture_output = tmp_path / "capture"
    assert main(["calibrate", "--recording", str(recording), "--calibration", str(profile),
                 "--output", str(capture_output)]) == 0
    original_profile = profile.read_bytes()
    with pytest.raises(ValueError, match="preserving existing"):
        main(["calibrate", "--recording", str(recording), "--calibration", str(profile)])
    assert profile.read_bytes() == original_profile
    replay_output = tmp_path / "replay"
    assert main(["replay", "--recording", str(recording), "--calibration", str(profile),
                 "--output", str(replay_output)]) == 0
    records = [json.loads(line) for line in (replay_output / "body_estimates.jsonl").read_text().splitlines()]
    assert sum(r["kind"] == "warming_up" for r in records) == 63
    last = records[-1]
    assert last["fit_accepted"] and last["robot_output"] is False
    assert len(last["positions_m"]) == 22
    assert json.loads((replay_output / "summary.json").read_text())["estimates"] == 3
