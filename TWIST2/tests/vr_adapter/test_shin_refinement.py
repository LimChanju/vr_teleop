from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

import twist2_vr.shin_refinement as refinement
from twist2_vr.shin_refinement import ShinRefinementConfig, forward_kinematics, refine_legs


def skeleton_fixture():
    # A 22-joint skeleton, with articulated upper body and two complete legs.
    names = [
        "Hips", "LeftHip", "LeftKnee", "LeftAnkle", "LeftToe",
        "RightHip", "RightKnee", "RightAnkle", "RightToe",
        "Spine", "Spine1", "Spine2", "Neck", "Head",
        "LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand",
        "RightShoulder", "RightArm", "RightForeArm", "RightHand",
    ]
    parents = [-1, 0, 1, 2, 3, 0, 5, 6, 7, 0, 9, 10, 11, 12, 11, 14, 15, 16, 11, 18, 19, 20]
    offsets = np.array([
        [0, 0, 0], [.10, 0, 0], [0, -.42, 0], [0, -.40, 0], [0, -.05, .14],
        [-.10, 0, 0], [0, -.42, 0], [0, -.40, 0], [0, -.05, .14],
        [0, .12, 0], [0, .12, 0], [0, .12, 0], [0, .12, 0], [0, .12, 0],
        [.08, .08, 0], [.10, 0, 0], [.25, 0, 0], [.23, 0, 0],
        [-.08, .08, 0], [-.10, 0, 0], [-.25, 0, 0], [-.23, 0, 0],
    ])
    reference = np.tile([0., 0., 0., 1.], (22, 1))
    return SimpleNamespace(names=names, parents=np.array(parents), offsets=offsets, reference_local_rotations=reference)


def pose_fixture(skeleton):
    local = skeleton.reference_local_rotations.copy()
    local[0] = Rotation.from_euler("y", 23, degrees=True).as_quat()
    local[[1, 2, 3, 5, 6, 7]] = Rotation.from_euler("x", [-12, 24, -12, -12, 24, -12], degrees=True).as_quat()
    local[[10, 15, 19]] = Rotation.from_euler("z", [8, -20, 16], degrees=True).as_quat()
    return local, np.array([.25, .95, -.40])


def targets_for(skeleton, local, root):
    positions, rotations = forward_kinematics(skeleton, local, root)
    return {
        role: (positions[skeleton.names.index(joint)].copy(), rotations[skeleton.names.index(joint)].copy())
        for role, joint in (
            ("Left_Knee", "LeftKnee"), ("Right_Knee", "RightKnee"),
            ("Left_Foot", "LeftToe"), ("Right_Foot", "RightToe"),
        )
    }


@pytest.mark.parametrize("side,knee,other", [("Left", 2, [5, 6, 7]), ("Right", 6, [1, 2, 3])])
def test_each_shin_changes_its_leg_even_with_unchanged_feet(side, knee, other):
    skeleton = skeleton_fixture()
    local, root = pose_fixture(skeleton)
    targets = targets_for(skeleton, local, root)
    role = side + "_Knee"
    position, rotation = targets[role]
    targets[role] = (position + [.04, 0, .02], (Rotation.from_euler("x", 12, degrees=True) * Rotation.from_quat(rotation)).as_quat())
    result, info = refine_legs(local, root, skeleton, targets, {"reference_prior_weight": 0})
    assert np.linalg.norm(result[knee] - local[knee]) > 1e-3
    assert info["final"][role]["position_error_m"] < info["initial"][role]["position_error_m"]
    assert info["final"][role]["rotation_error_rad"] < info["initial"][role]["rotation_error_rad"]
    np.testing.assert_allclose(result[other], local[other], atol=1e-10)
    assert info["final_cost"] < info["initial_cost"]


def test_reachable_pose_preserves_upper_body_lengths_and_inputs():
    skeleton = skeleton_fixture()
    local, root = pose_fixture(skeleton)
    desired = local.copy()
    desired[[1, 2, 3, 5, 6, 7]] = Rotation.from_euler("x", [-22, 43, -21, -18, 38, -20], degrees=True).as_quat()
    targets = targets_for(skeleton, desired, root)
    saved_local, saved_root, saved_offsets = local.copy(), root.copy(), skeleton.offsets.copy()
    saved_targets = {role: (p.copy(), q.copy()) for role, (p, q) in targets.items()}
    result, info = refine_legs(local, root, skeleton, targets)
    before_pos, before_rot = forward_kinematics(skeleton, local, root)
    after_pos, after_rot = forward_kinematics(skeleton, result, root)
    unchanged = [0, 4, 8] + list(range(9, 22))
    np.testing.assert_array_equal(result[unchanged], local[unchanged])
    np.testing.assert_array_equal(after_pos[[0] + list(range(9, 22))], before_pos[[0] + list(range(9, 22))])
    np.testing.assert_array_equal(after_rot[[0] + list(range(9, 22))], before_rot[[0] + list(range(9, 22))])
    for joint in range(1, 22):
        assert np.linalg.norm(after_pos[joint] - after_pos[skeleton.parents[joint]]) == pytest.approx(np.linalg.norm(skeleton.offsets[joint]), abs=1e-12)
    assert info["success"] and info["converged"] and info["targets_satisfied"]
    assert info["max_position_error_m"] < .003
    np.testing.assert_array_equal(local, saved_local)
    np.testing.assert_array_equal(root, saved_root)
    np.testing.assert_array_equal(skeleton.offsets, saved_offsets)
    for role in targets:
        np.testing.assert_array_equal(targets[role][0], saved_targets[role][0])
        np.testing.assert_array_equal(targets[role][1], saved_targets[role][1])


def test_quaternion_sign_invariance():
    skeleton = skeleton_fixture()
    local, root = pose_fixture(skeleton)
    desired = local.copy()
    desired[2] = Rotation.from_euler("x", 35, degrees=True).as_quat()
    targets = targets_for(skeleton, desired, root)
    first, info = refine_legs(local, root, skeleton, targets)
    skeleton.reference_local_rotations *= -1
    second, other_info = refine_legs(-local, root, skeleton, {role: (p, -q) for role, (p, q) in targets.items()})
    first_pos, first_rot = forward_kinematics(skeleton, first, root)
    second_pos, second_rot = forward_kinematics(skeleton, second, root)
    np.testing.assert_allclose(first_pos, second_pos, atol=1e-10)
    np.testing.assert_allclose(Rotation.from_quat(first_rot).as_matrix(), Rotation.from_quat(second_rot).as_matrix(), atol=1e-10)
    assert info["final_cost"] == pytest.approx(other_info["final_cost"], abs=1e-12)


def test_unreachable_targets_report_residual_failure():
    skeleton = skeleton_fixture()
    local, root = pose_fixture(skeleton)
    targets = targets_for(skeleton, local, root)
    targets["Left_Foot"][0][0] += 3
    result, info = refine_legs(local, root, skeleton, targets, {"max_nfev": 2})
    assert np.all(np.isfinite(result))
    assert not info["success"] and not info["targets_satisfied"]
    assert info["max_position_error_m"] > 1
    assert info["nfev"] <= 2
    assert info["status"] == 0
    assert not info["converged"]


def test_optimizer_failure_is_explicit_and_keeps_prediction(monkeypatch):
    skeleton = skeleton_fixture()
    local, root = pose_fixture(skeleton)

    def fail(*args, **kwargs):
        raise np.linalg.LinAlgError("synthetic numerical failure")

    monkeypatch.setattr(refinement, "least_squares", fail)
    result, info = refine_legs(local, root, skeleton, targets_for(skeleton, local, root))
    np.testing.assert_array_equal(result, local)
    assert info["fallback_used"] and not info["success"] and not info["converged"]
    assert "synthetic numerical failure" in info["message"]


@pytest.mark.parametrize("role", ["Left_Knee", "Right_Knee", "Left_Foot", "Right_Foot"])
def test_all_four_calibrated_targets_are_required(role):
    skeleton = skeleton_fixture()
    local, root = pose_fixture(skeleton)
    targets = targets_for(skeleton, local, root)
    del targets[role]
    with pytest.raises(ValueError, match="targets require"):
        refine_legs(local, root, skeleton, targets)


@pytest.mark.parametrize("problem", ["nan_rotation", "zero_rotation", "root", "offset", "cycle", "target_position", "target_rotation", "chain"])
def test_invalid_inputs_raise(problem):
    skeleton = skeleton_fixture()
    local, root = pose_fixture(skeleton)
    targets = targets_for(skeleton, local, root)
    if problem == "nan_rotation":
        local[2, 0] = np.nan
    elif problem == "zero_rotation":
        local[2] = 0
    elif problem == "root":
        root[0] = np.inf
    elif problem == "offset":
        skeleton.offsets[2, 0] = np.nan
    elif problem == "cycle":
        skeleton.parents[1] = 2
    elif problem == "target_position":
        targets["Left_Knee"][0][0] = np.nan
    elif problem == "target_rotation":
        targets["Left_Knee"][1][:] = 0
    elif problem == "chain":
        skeleton.parents[3] = 1
    with pytest.raises(ValueError):
        refine_legs(local, root, skeleton, targets)


@pytest.mark.parametrize("config", [{"max_nfev": 0}, {"max_nfev": 1.5}, {"foot_rotation_weight": 0}, {"prediction_prior_weight": -1}, {"gtol": 0}, {"unknown": 1}, {"gtol": "invalid"}, {"reference_prior_weight": np.nan}])
def test_invalid_configuration_raises(config):
    skeleton = skeleton_fixture()
    local, root = pose_fixture(skeleton)
    with pytest.raises(ValueError):
        refine_legs(local, root, skeleton, targets_for(skeleton, local, root), config)


def test_fk_uses_rotated_offsets_and_root_world_position():
    skeleton = skeleton_fixture()
    local = skeleton.reference_local_rotations.copy()
    local[0] = Rotation.from_euler("z", 90, degrees=True).as_quat()
    skeleton.offsets[0] = [100, 100, 100]
    root = np.array([1., 2., 3.])
    positions, rotations = forward_kinematics(vars(skeleton), local, root)
    np.testing.assert_allclose(positions[0], root)
    np.testing.assert_allclose(positions[1], root + [0, .1, 0], atol=1e-12)
    np.testing.assert_allclose(positions[2], root + [.42, .1, 0], atol=1e-12)
    np.testing.assert_allclose(Rotation.from_quat(rotations[2]).as_matrix(), Rotation.from_quat(local[0]).as_matrix(), atol=1e-12)


def test_pymotion_self_parented_root_matches_minus_one_without_mutating_skeleton():
    skeleton = skeleton_fixture()
    local, root = pose_fixture(skeleton)
    targets = targets_for(skeleton, local, root)
    expected_positions, expected_rotations = forward_kinematics(skeleton, local, root)
    expected_result, expected_info = refine_legs(local, root, skeleton, targets)
    skeleton.parents[0] = 0
    saved_parents = skeleton.parents.copy()
    positions, rotations = forward_kinematics(skeleton, local, root)
    result, info = refine_legs(local, root, skeleton, targets)
    np.testing.assert_array_equal(positions, expected_positions)
    np.testing.assert_array_equal(rotations, expected_rotations)
    np.testing.assert_array_equal(result, expected_result)
    np.testing.assert_array_equal(skeleton.parents, saved_parents)
    assert info["success"] == expected_info["success"]
    assert info["final"] == expected_info["final"]


def test_multiple_root_conventions_in_one_skeleton_are_rejected():
    skeleton = skeleton_fixture()
    skeleton.parents[9] = 9
    with pytest.raises(ValueError, match="exactly one root"):
        forward_kinematics(skeleton, skeleton.reference_local_rotations, np.zeros(3))


def test_actual_backend_skeleton_type_with_pymotion_root():
    from twist2_vr.sparseposer_backend import Skeleton

    source = skeleton_fixture()
    source.parents[0] = 0
    skeleton = Skeleton(tuple(source.names), source.parents, source.offsets, source.reference_local_rotations)
    local, root = pose_fixture(skeleton)
    targets = targets_for(skeleton, local, root)
    result, info = refine_legs(local, root, skeleton, targets)
    assert result.shape == (22, 4)
    assert info["success"]
    assert skeleton.parents[0] == 0


def test_reference_pose_prior_is_a_soft_cost_and_cannot_claim_exact_fit():
    skeleton = skeleton_fixture()
    local, root = pose_fixture(skeleton)
    targets = targets_for(skeleton, local, root)
    free, _ = refine_legs(local, root, skeleton, targets, {"reference_prior_weight": 0})
    regularized, info = refine_legs(local, root, skeleton, targets, {
        "reference_prior_weight": 2,
        "position_tolerance_m": 1e-8,
        "rotation_tolerance_rad": 1e-8,
    })
    leg_joints = [1, 2, 3, 5, 6, 7]
    assert np.linalg.norm(Rotation.from_quat(regularized[leg_joints]).as_rotvec()) < np.linalg.norm(Rotation.from_quat(free[leg_joints]).as_rotvec())
    assert info["converged"]
    assert not info["success"] and not info["targets_satisfied"]


def test_analytic_jacobian_matches_finite_differences(monkeypatch):
    skeleton = skeleton_fixture()
    local, root = pose_fixture(skeleton)
    targets = targets_for(skeleton, local, root)
    real_solver = refinement.least_squares

    def checking_solver(fun, x0, *, jac, **kwargs):
        delta = np.linspace(-.25, .30, 18)
        epsilon = 1e-6
        numerical = np.column_stack([
            (fun(delta + np.eye(18)[j] * epsilon) - fun(delta - np.eye(18)[j] * epsilon)) / (2 * epsilon)
            for j in range(18)
        ])
        np.testing.assert_allclose(jac(delta), numerical, atol=2e-8, rtol=2e-6)
        return real_solver(fun, x0, jac=jac, **kwargs)

    monkeypatch.setattr(refinement, "least_squares", checking_solver)
    refine_legs(local, root, skeleton, targets, ShinRefinementConfig())
