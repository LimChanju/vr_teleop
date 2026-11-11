from pathlib import Path

import numpy as np

from twist2_vr.calibration import NeutralCalibration
from twist2_vr.config import load_json
from twist2_vr.normalization import PoseNormalizer
from twist2_vr.providers.mock import MockVRProvider
from twist2_vr.reconstruction import SparseKinematicReconstructor
from twist2_vr.retargeting import OfficialGMRRetargeter


ROOT = Path(__file__).resolve().parents[2]


def test_official_gmr_mock_frame_to_g1_reference():
    raw = MockVRProvider().read()
    normalized = PoseNormalizer(load_json(ROOT / "config/coordinates.openvr.json"))(raw)
    calibrated = NeutralCalibration(load_json(ROOT / "config/calibration.mock.json")).apply(normalized)
    body = SparseKinematicReconstructor(load_json(ROOT / "config/reconstruction.sparse.json"))(calibrated)
    result = OfficialGMRRetargeter(ROOT / "external/GMR").retarget(body)
    assert result.source == "official_gmr"
    assert result.qpos.shape == (36,)
    assert result.mimic_obs.shape == (35,)
    assert np.all(np.isfinite(result.qpos))
    assert np.all(np.isfinite(result.mimic_obs))
