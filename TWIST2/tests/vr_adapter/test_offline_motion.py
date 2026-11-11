from pathlib import Path

import numpy as np

from twist2_vr.offline_motion import OfficialExampleMotionStream


ROOT = Path(__file__).resolve().parents[2]


def test_official_example_motion_builds_controller_reference():
    stream = OfficialExampleMotionStream(
        ROOT / "assets/example_motions/0807_yanjie_walk_003.pkl"
    )
    reference = stream.reference(0, 123.0)
    assert stream.num_steps > 100
    assert reference.mimic_obs.shape == (35,)
    assert reference.qpos.shape == (36,)
    assert np.all(np.isfinite(reference.mimic_obs))
    np.testing.assert_allclose(reference.mimic_obs[6:], reference.qpos[7:], atol=1e-6)
    # Motion PKLs and MotionLib use xyzw; MuJoCo qpos must be wxyz.
    assert abs(np.linalg.norm(reference.qpos[3:7]) - 1.0) < 1e-5
