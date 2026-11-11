from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from twist2_vr.shin_refinement import forward_kinematics
from twist2_vr.sparseposer_adapter import ROLE_JOINT, skeleton_fingerprint
from twist2_vr.sparseposer_backend import Skeleton, load_skeleton
from twist2_vr.sparseposer_gmr import (
    BVH_TO_GMR, GMR_JOINTS, SparsePoserGMRConverter, SparsePoserG1Retargeter,
)


ROOT = Path(__file__).resolve().parents[2]


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


def calibration(skeleton, height=1.7, yaw=71):
    return {
        "schema_version": 1, "kind": "sparseposer_tpose_mounts", "coordinate_frame": "openvr_standing",
        "height_m": height, "model_scale": 1.7 / height,
        "world_origin_m": [2.3, .07, -4.8],
        "world_from_model_xyzw": Rotation.from_euler("y", yaw, degrees=True).as_quat().tolist(),
        "skeleton_sha256": skeleton_fingerprint(skeleton),
        "mounts": {role: {"device_id": role, "joint": joint, "translation_model_m": [0,0,0],
                          "rotation_xyzw": [0,0,0,1]} for role, joint in ROLE_JOINT.items()},
    }


def estimate(skeleton, profile, local=None, root=None, stamp=1.):
    local = skeleton.reference_local_rotations.copy() if local is None else local.copy()
    root = np.array([0, .9, 0]) if root is None else np.asarray(root)
    positions, rotations = forward_kinematics(skeleton, local, root)
    w = Rotation.from_quat(profile["world_from_model_xyzw"])
    return {
        "schema_version": 1, "kind": "sparseposer_eight_sensor_estimate", "timestamp_s": stamp,
        "coordinate_frame": "openvr_standing", "fit_accepted": True,
        "joint_names": list(skeleton.names), "parents": skeleton.parents.tolist(),
        "root_position_model_m": root.tolist(), "local_rotations_xyzw": local.tolist(),
        "positions_m": (w.apply(positions / profile["model_scale"]) + profile["world_origin_m"]).tolist(),
        "orientations_xyzw": (w * Rotation.from_quat(rotations)).as_quat().tolist(),
    }


def rot(target):
    return Rotation.from_quat(np.asarray(target[1])[[1,2,3,0]])


def test_right_handed_axes_units_and_root_frame(skeleton):
    profile = calibration(skeleton, height=1.87)
    converted = SparsePoserGMRConverter(skeleton, profile).convert(
        estimate(skeleton, profile, root=[.2,.9,.4]))
    assert np.linalg.det(BVH_TO_GMR) == pytest.approx(1)
    np.testing.assert_allclose(converted["Hips"][0], np.array([.4,.2,.9]) * 1.1)
    np.testing.assert_allclose(rot(converted["Hips"]).as_matrix(), np.eye(3), atol=1e-12)
    assert set(converted) == set(GMR_JOINTS)


@pytest.mark.parametrize("side,sign", [("Left",1), ("Right",-1)])
def test_tpose_arm_orientation_agrees_with_bone_direction(skeleton, side, sign):
    profile = calibration(skeleton)
    targets = SparsePoserGMRConverter(skeleton, profile).convert(estimate(skeleton, profile))
    # GMR's Xsens upper/lower arms use local -Z after basis conversion.
    for name, child in (("Shoulder", "Elbow"), ("Elbow", "Wrist")):
        direction = targets[side + child][0] - targets[side + name][0]
        direction /= np.linalg.norm(direction)
        np.testing.assert_allclose(rot(targets[side + name]).apply([0,0,-1]), direction, atol=1e-12)
        np.testing.assert_allclose(direction, [0,sign,0], atol=1e-12)


def test_npose_arm_bases_are_identity_not_sideways(skeleton):
    profile = calibration(skeleton)
    local = skeleton.reference_local_rotations.copy()
    local[skeleton.names.index("LeftShoulder")] = Rotation.from_euler("z", -90, degrees=True).as_quat()
    local[skeleton.names.index("RightShoulder")] = Rotation.from_euler("z", 90, degrees=True).as_quat()
    targets = SparsePoserGMRConverter(skeleton, profile).convert(estimate(skeleton, profile, local))
    for side in ("Left", "Right"):
        for part in ("Shoulder", "Elbow", "Wrist"):
            np.testing.assert_allclose(rot(targets[side + part]).as_matrix(), np.eye(3), atol=1e-12)


def test_footmod_uses_ankle_pose_not_toe(skeleton):
    profile = calibration(skeleton)
    local = skeleton.reference_local_rotations.copy()
    local[skeleton.names.index("LeftToe")] = Rotation.from_euler("x", .9).as_quat()
    result = SparsePoserGMRConverter(skeleton, profile).convert(estimate(skeleton, profile, local))
    np.testing.assert_allclose(result["LeftFootMod"][0], [0,.1,.1], atol=1e-12)
    np.testing.assert_allclose(rot(result["LeftFootMod"]).as_matrix(), np.eye(3), atol=1e-12)


def test_room_yaw_origin_and_quaternion_sign_do_not_change_robot_frame(skeleton):
    first = calibration(skeleton)
    second = calibration(skeleton, yaw=-43)
    second["world_origin_m"] = [-9, .4, 7]
    a = SparsePoserGMRConverter(skeleton, first).convert(estimate(skeleton, first))
    e = estimate(skeleton, second)
    e["orientations_xyzw"] = (-np.asarray(e["orientations_xyzw"])).tolist()
    e["local_rotations_xyzw"] = (-np.asarray(e["local_rotations_xyzw"])).tolist()
    b = SparsePoserGMRConverter(skeleton, second).convert(e)
    for name in a:
        np.testing.assert_allclose(a[name][0], b[name][0], atol=1e-12)
        np.testing.assert_allclose(rot(a[name]).as_matrix(), rot(b[name]).as_matrix(), atol=1e-12)


@pytest.mark.parametrize("field,value", [
    ("coordinate_frame", "vive_world"), ("kind", "raw_poses"),
    ("joint_names", ["wrong"]), ("parents", [0]), ("timestamp_s", float("nan")),
    ("root_position_model_m", [0, float("nan"), 0]),
    ("local_rotations_xyzw", [[0,0,0,0]] * 22),
    ("orientations_xyzw", [[0,0,0,0]] * 22), ("positions_m", [[1,1,1]] * 22),
])
def test_bad_or_mismatched_estimates_rejected(skeleton, field, value):
    profile = calibration(skeleton)
    e = estimate(skeleton, profile)
    e[field] = value
    with pytest.raises(ValueError):
        SparsePoserGMRConverter(skeleton, profile).convert(e)


def test_unknown_arm_axes_rejected(skeleton):
    skeleton.offsets[skeleton.names.index("LeftElbow")] = [0,-.25,0]
    with pytest.raises(ValueError, match="T-pose arm"):
        SparsePoserGMRConverter(skeleton, calibration(skeleton))


def test_pinned_gmr_actual_ik_respects_limits_and_moves_arms():
    pytest.importorskip("mujoco")
    pytest.importorskip("mink")
    pytest.importorskip("pymotion")
    checkout = ROOT / "external" / "GMR"
    sparse = ROOT / "external" / "SparsePoser"
    reference = sparse / "SparsePoserUnity/Assets/Data/male_170_85.txt"
    if not reference.is_file() or not (checkout / "general_motion_retargeting").is_dir():
        pytest.skip("official checkouts not installed")
    skel = load_skeleton(sparse, reference)
    profile = calibration(skel)
    retargeter = SparsePoserG1Retargeter(checkout, skel, profile)
    local = skel.reference_local_rotations.copy()
    for step in range(8):
        ref, info = retargeter.retarget(estimate(skel, profile, local, stamp=1+step*.02))
    assert ref.valid and not info["joint_limit_violations"]
    assert ref.qpos.shape == (36,) and ref.mimic_obs.shape == (35,)
    assert .5 < ref.qpos[2] < 1.2
    initial = ref.qpos.copy()
    # Raise/lower through the upper arms, without editing the legs or IK table.
    for step in range(1,9):
        local[skel.names.index("LeftShoulder")] = Rotation.from_euler("z", -step*8, degrees=True).as_quat()
        local[skel.names.index("RightShoulder")] = Rotation.from_euler("z", step*8, degrees=True).as_quat()
        e = estimate(skel, profile, local, stamp=2+step*.02)
        ref, info = retargeter.retarget(e)
        assert not info["joint_limit_violations"]
    addresses = [retargeter.model.joint(side + "_shoulder_roll_joint").qposadr[0] for side in ("left", "right")]
    assert np.linalg.norm(ref.qpos[addresses] - initial[addresses]) > .5
    bad_fit = deepcopy(e)
    bad_fit["timestamp_s"] += .02
    bad_fit["fit_accepted"] = False
    preview_ref, diagnostics = retargeter.retarget(bad_fit)
    assert not preview_ref.valid and np.isfinite(preview_ref.qpos).all()
    assert not diagnostics["source_fit_accepted"]
    assert not diagnostics["robot_output"] and not diagnostics["policy_or_balance_verified"]
    with pytest.raises(ValueError, match="timestamps"):
        retargeter.retarget(bad_fit)
