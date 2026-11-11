import numpy as np

from twist2_vr.retargeting import DEFAULT_G1_Q29, Twist2ReferenceEncoder


def qpos(x=0.0, quat=(1, 0, 0, 0)):
    return np.concatenate(([x, 0, 0.8], quat, DEFAULT_G1_Q29))


def test_reference_encoder_shape_and_velocity_layout():
    encoder = Twist2ReferenceEncoder()
    first = encoder.encode(qpos(), 1.0, "test")
    assert first.mimic_obs.shape == (35,)
    np.testing.assert_allclose(first.mimic_obs[:6], [0, 0, 0.8, 0, 0, 0], atol=1e-6)
    np.testing.assert_allclose(first.mimic_obs[6:], DEFAULT_G1_Q29)
    second = encoder.encode(qpos(0.02), 1.02, "test")
    np.testing.assert_allclose(second.mimic_obs[:2], [1, 0], atol=1e-5)


def test_quaternion_sign_flip_does_not_create_angular_velocity():
    encoder = Twist2ReferenceEncoder()
    encoder.encode(qpos(quat=(1, 0, 0, 0)), 1.0, "test")
    result = encoder.encode(qpos(quat=(-1, 0, 0, 0)), 1.02, "test")
    assert abs(float(result.mimic_obs[5])) < 1e-6
