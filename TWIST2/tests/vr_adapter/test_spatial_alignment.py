import numpy as np
import pytest

from twist2_vr.spatial_alignment import solve_hand_eye, validate_transform


def random_transform(rng):
    quaternion = rng.normal(size=4)
    quaternion /= np.linalg.norm(quaternion)
    x, y, z, w = quaternion
    rotation = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = rng.uniform(-1, 1, size=3)
    return result


def test_hand_eye_recovers_world_and_fixture_transforms():
    rng = np.random.default_rng(42)
    vive_to_quest = random_transform(rng)
    tracker_to_controller = random_transform(rng)
    vive_samples = [random_transform(rng) for _ in range(30)]
    quest_samples = [
        vive_to_quest @ pose @ tracker_to_controller for pose in vive_samples
    ]
    result = solve_hand_eye(quest_samples, vive_samples)
    np.testing.assert_allclose(result.vive_to_quest, vive_to_quest, atol=1e-8)
    np.testing.assert_allclose(result.tracker_from_controller, tracker_to_controller, atol=1e-8)
    assert result.position_rmse_m < 1e-10
    assert result.orientation_rmse_deg < 1e-4


def test_hand_eye_rejects_stationary_underconstrained_samples():
    samples = [np.eye(4) for _ in range(10)]
    with pytest.raises(ValueError, match="rotational diversity"):
        solve_hand_eye(samples, samples)


def test_validate_transform_rejects_reflection():
    reflection = np.eye(4)
    reflection[0, 0] = -1
    with pytest.raises(ValueError, match="determinant"):
        validate_transform(reflection)
