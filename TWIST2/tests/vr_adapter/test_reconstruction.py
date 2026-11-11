from pathlib import Path

import numpy as np

from twist2_vr.calibration import NeutralCalibration
from twist2_vr.config import load_json
from twist2_vr.models import PoseSample, TrackerFrame
from twist2_vr.normalization import PoseNormalizer
from twist2_vr.providers.mock import SPARSE_TARGET_POSES, MockVRProvider
from twist2_vr.reconstruction import GMR_BODY_JOINTS, SparseKinematicReconstructor, StrictDirectReconstructor
from twist2_vr.retargeting import body_frame_to_gmr


ROOT = Path(__file__).resolve().parents[2]


def test_sparse_mock_reconstruction_marks_estimates_and_exact_gmr_schema():
    raw = MockVRProvider(target_poses=SPARSE_TARGET_POSES).read()
    normalized = PoseNormalizer(load_json(ROOT / "config/coordinates.openvr.json"))(raw)
    calibrated = NeutralCalibration(load_json(ROOT / "config/calibration.mock.json")).apply(normalized)
    body = SparseKinematicReconstructor(load_json(ROOT / "config/reconstruction.sparse.json"))(calibrated)
    assert body.valid
    assert set(GMR_BODY_JOINTS).issubset(body.joints)
    assert body.provenance["Pelvis"] == "measured"
    assert body.provenance["Left_Elbow"] == "estimated_two_bone_ik"
    assert body.provenance["Left_Hip"] == "estimated_neutral"
    gmr = body_frame_to_gmr(body)
    assert set(gmr) == set(GMR_BODY_JOINTS)
    assert all(np.asarray(value[0]).shape == (3,) and np.asarray(value[1]).shape == (4,) for value in gmr.values())
    # Identity xyzw becomes scalar-first identity wxyz at the GMR boundary.
    np.testing.assert_allclose(gmr["Pelvis"][1], [1, 0, 0, 0])


def test_strict_mode_and_sparse_minimum_fail_closed():
    one = PoseSample("hmd", "Head", [0, 0, 1.7], [0, 0, 0, 1], 1.0)
    frame = TrackerFrame(1.0, {"Head": one}, "test", "twist2_neutral")
    assert not StrictDirectReconstructor()(frame).valid
    assert not SparseKinematicReconstructor(load_json(ROOT / "config/reconstruction.sparse.json"))(frame).valid
