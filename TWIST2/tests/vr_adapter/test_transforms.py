import math

import numpy as np
import pytest

from twist2_vr.calibration import NeutralCalibration
from twist2_vr.math3d import matrix_to_quaternion, quaternion_to_matrix
from twist2_vr.models import PoseSample, TrackerFrame
from twist2_vr.normalization import PoseNormalizer


AXIS = [[0, 0, -1], [-1, 0, 0], [0, 1, 0]]


def pose(role="Pelvis", position=(0, 0, 0), quat=(0, 0, 0, 1), timestamp=1.0):
    return PoseSample("device", role, position, quat, timestamp)


def test_openvr_axis_mapping_position_and_rotation():
    normalizer = PoseNormalizer({
        "source_frame": "openvr_standing", "target_frame": "twist2_world",
        "axis_matrix": AXIS, "position_scale": 1.0,
    })
    source_rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
    sample = pose(position=(1, 2, 3), quat=matrix_to_quaternion(source_rotation))
    out = normalizer.normalize_pose(sample)
    np.testing.assert_allclose(out.position_m, [-3, -1, 2])
    axis = np.asarray(AXIS)
    np.testing.assert_allclose(
        quaternion_to_matrix(out.orientation_xyzw), axis @ source_rotation @ axis.T, atol=1e-12
    )


def test_tracker_mount_offset_is_applied_in_tracker_frame():
    normalizer = PoseNormalizer({
        "source_frame": "source", "target_frame": "target", "axis_matrix": np.eye(3).tolist(),
        "tracker_to_body": {"Pelvis": {"translation_m": [0.1, 0, 0], "orientation_xyzw": [0, 0, 0, 1]}},
    })
    out = normalizer.normalize_pose(pose(position=(1, 2, 3)))
    np.testing.assert_allclose(out.position_m, [1.1, 2, 3])


def test_calibration_aligns_anchor_yaw_and_position():
    neutral = {"Pelvis": [0, 0, 0.95]}
    calibration = NeutralCalibration({"calibrated": False, "anchor_role": "Pelvis", "neutral_positions_m": neutral})
    yaw90 = [0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)]
    frame = TrackerFrame(1.0, {"Pelvis": pose(position=(2, 3, 1), quat=yaw90)}, "test", "twist2_world")
    matrix = calibration.calibrate_from_frame(frame)
    assert matrix.shape == (4, 4)
    aligned = calibration.apply(frame).poses["Pelvis"]
    np.testing.assert_allclose(aligned.position_m, neutral["Pelvis"], atol=1e-7)
    np.testing.assert_allclose(quaternion_to_matrix(aligned.orientation_xyzw), np.eye(3), atol=1e-7)


def test_rejects_reflection_axis_and_nan_pose():
    with pytest.raises(ValueError, match="proper"):
        PoseNormalizer({"axis_matrix": [[-1, 0, 0], [0, 1, 0], [0, 0, 1]]})
    with pytest.raises(ValueError, match="finite"):
        pose(position=(np.nan, 0, 0))
