import copy
import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from twist2_vr.models import PoseSample, TrackerFrame
from twist2_vr.shin_refinement import forward_kinematics
from twist2_vr.sparseposer_adapter import (
    ROLES, ROLE_JOINT, EightTrackerEstimator, MotionWindow, MountCalibration,
    check_frame, fit_tpose_calibration, network_sample, read_recording,
)
from twist2_vr.sparseposer_backend import Skeleton


@pytest.fixture
def skeleton():
    names = ("Hips", "LeftHip", "LeftKnee", "LeftAnkle", "LeftToe",
             "RightHip", "RightKnee", "RightAnkle", "RightToe", "Chest", "Chest3", "Chest4",
             "Neck", "Head", "LeftCollar", "LeftShoulder", "LeftElbow", "LeftWrist",
             "RightCollar", "RightShoulder", "RightElbow", "RightWrist")
    parents = np.array([0,0,1,2,3,0,5,6,7,0,9,10,11,12,11,14,15,16,11,18,19,20])
    offsets = np.array([[0,0,0],[.1,0,0],[0,-.4,0],[0,-.4,0],[0,-.1,.15],
                        [-.1,0,0],[0,-.4,0],[0,-.4,0],[0,-.1,.15],
                        [0,.1,0],[0,.2,0],[0,.1,0],[0,.1,0],[0,.1,0],
                        [.05,.05,0],[.1,0,0],[.25,0,0],[.25,0,0],
                        [-.05,.05,0],[-.1,0,0],[-.25,0,0],[-.25,0,0]])
    return Skeleton(names, parents, offsets, np.tile([0.,0,0,1], (22,1)))


def frames_for(skeleton, count=80):
    positions, _ = forward_kinematics(skeleton, skeleton.reference_local_rotations, [0,.9,0])
    return [TrackerFrame(i/60, {
        r: PoseSample(r, r, positions[skeleton.names.index(j)],
                      Rotation.from_euler("y", np.pi).as_quat() if r == "Head" else [0,0,0,1], i/60)
        for r,j in ROLE_JOINT.items()}, "synthetic", "openvr_standing") for i in range(count)]


@pytest.mark.parametrize("role", ROLES)
def test_every_sensor_required(skeleton, role):
    frame = frames_for(skeleton)[0]
    poses = dict(frame.poses)
    del poses[role]
    with pytest.raises(ValueError, match="missing/invalid"):
        check_frame(TrackerFrame(frame.timestamp_s, poses, frame.source, frame.coordinate_frame))


@pytest.mark.parametrize("change", [{"valid": False}, {"tracking_state":"rotation_only"},
                                  {"timestamp_s":-1.}, {"timestamp_s":1.}])
def test_rejects_bad_sensor_quality(skeleton, change):
    frame = frames_for(skeleton)[0]
    poses = dict(frame.poses)
    poses["Left_Knee"] = poses["Left_Knee"].replaced(**change)
    with pytest.raises(ValueError):
        check_frame(TrackerFrame(frame.timestamp_s, poses, frame.source, frame.coordinate_frame))


def test_arms_down_capture_not_accepted_as_calibration(skeleton):
    frames = frames_for(skeleton)
    for f in frames:
        for role in ("Left_Wrist", "Right_Wrist"):
            p = f.poses[role].position_m.copy()
            p[0] *= .2
            p[1] = .6
            f.poses[role] = f.poses[role].replaced(position_m=p)
    with pytest.raises(ValueError, match="T-pose"):
        fit_tpose_calibration(frames, skeleton)


def test_calibration_schema_and_identity_rejected(skeleton):
    frames = frames_for(skeleton)
    profile = fit_tpose_calibration(frames, skeleton)
    calibration = MountCalibration(profile, skeleton)
    frame = frames[-1]
    frame.poses["Left_Knee"] = frame.poses["Left_Knee"].replaced(device_id="another_tracker")
    with pytest.raises(ValueError, match="device changed"):
        calibration.targets(frame)
    for key, value in (("kind", "neutral_mount_diagnostics_only"),
                       ("skeleton_sha256", "wrong"), ("coordinate_frame", "vive_world")):
        broken = {**profile, key:value}
        with pytest.raises(ValueError):
            MountCalibration(broken, skeleton)


def test_legs_used_no_upperbody_change_and_block_resets_history(skeleton):
    class FakeBackend:
        def infer(self, window):
            self.last_window = window
            return self.skeleton.reference_local_rotations.copy()
    backend = FakeBackend()
    backend.skeleton = skeleton
    frames = frames_for(skeleton, count=140)
    estimator = EightTrackerEstimator(backend, fit_tpose_calibration(frames[:80], skeleton))
    outputs = [estimator.process(f) for f in frames[:80]]
    assert outputs[-1]["fit_accepted"] and not outputs[-1]["robot_eligible"]
    for f in frames[80:]:
        f.poses["Left_Knee"] = f.poses["Left_Knee"].replaced(
            orientation_xyzw=Rotation.from_euler("x", .4).as_quat())
    result = estimator.process(frames[80])
    # Extra shin orientation influences the answer, not merely the six-input prior.
    assert not np.allclose(result["local_rotations_xyzw"][2], skeleton.reference_local_rotations[2])
    np.testing.assert_allclose(result["local_rotations_xyzw"][9:], skeleton.reference_local_rotations[9:])
    lost = copy.deepcopy(frames[81])
    lost.poses["Left_Knee"] = lost.poses["Left_Knee"].replaced(valid=False)
    with pytest.raises(ValueError):
        estimator.process(lost)
    assert estimator.process(frames[82]) is None


def test_network_features_do_not_make_arms_follow_head_yaw():
    targets = {r:(np.array([.1*i, .2, 0]), np.array([0.,0,0,1])) for i,r in enumerate(ROLES)}
    original = network_sample(targets)
    targets["Head"] = (targets["Head"][0], Rotation.from_euler("y", 1).as_quat())
    changed = network_sample(targets)
    np.testing.assert_array_equal(original[[0,1,2,4,5]], changed[[0,1,2,4,5]])
    assert not np.allclose(original[3], changed[3])


def test_window_duplicate_reversal_and_empty_warmup():
    w = MotionWindow()
    sample = np.zeros((6,7)); sample[:,3] = 1
    assert w.append(0., sample) is None
    assert w.append(0., sample) is None
    assert len(w.samples) == 1
    with pytest.raises(ValueError, match="reversed"):
        w.append(-1., sample)
    assert not w.samples


def test_replay_preserves_sensor_age(tmp_path, skeleton):
    from twist2_vr.sparseposer_cli import _raw_record
    frame = frames_for(skeleton)[0]
    frame.poses["Left_Foot"] = frame.poses["Left_Foot"].replaced(timestamp_s=-.15)
    path = tmp_path / "record.jsonl"
    path.write_text(json.dumps(_raw_record(frame))+"\n")
    replayed = list(read_recording(path))[0]
    assert replayed.timestamp_s == 0.
    assert replayed.poses["Left_Foot"].timestamp_s == -.15
    check_frame(replayed)
