"""Regression tests for the signed WXYZ features seen by SparsePoser.

Geometrically equal rotations are not enough here: the neural-network input
must retain the same quaternion hemisphere throughout the accepted stream.
These tests intentionally compare quaternion components, not rotation matrices.
"""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

import twist2_vr.sparseposer_adapter as adapter
from twist2_vr.sparseposer_adapter import MotionWindow


_AXES = np.asarray([
    [1., 0., 0.], [0., 1., 0.], [0., 0., 1.],
    [1., 1., 0.], [0., 1., 1.], [1., 0., 1.],
])
_AXES /= np.linalg.norm(_AXES, axis=1, keepdims=True)


def _sample(degrees, frame=0):
    """An analytic continuous lift, without forcing the scalar part positive."""
    angles = np.deg2rad(degrees) * (1. + .04 * np.arange(6))
    xyzw = Rotation.from_rotvec(angles[:, None] * _AXES).as_quat()
    result = np.empty((6, 7))
    result[:, :3] = np.arange(18).reshape(6, 3) * .01 + frame * .001
    result[:, 3:] = xyzw[:, [3, 0, 1, 2]]
    return result


def _feed(window, samples, start=0.):
    return [window.append(start + index / 60., sample)
            for index, sample in enumerate(samples)]


@pytest.mark.parametrize("seed", [1, 7, 19])
def test_arbitrary_q_minus_q_per_frame_and_sensor_gives_identical_features(seed):
    reference = np.stack([_sample(index * 2., index) for index in range(230)])
    flipped = reference.copy()
    signs = np.random.default_rng(seed).choice([-1., 1.], (len(reference), 6))
    signs[0] = -1.  # Explicitly exercise identity anchoring at startup.
    flipped[:, :, 3:] *= signs[:, :, None]

    clean_windows = _feed(MotionWindow(), reference)
    flipped_windows = _feed(MotionWindow(), flipped)

    for index, (clean, changed) in enumerate(zip(clean_windows, flipped_windows)):
        if index < 63:
            assert clean is None and changed is None
        else:
            np.testing.assert_allclose(changed, clean, atol=2e-6, rtol=0.)
            np.testing.assert_allclose(changed, reference[index - 63:index + 1],
                                       atol=2e-6, rtol=0.)


def test_continuous_rotation_retains_negative_w_across_180_and_360_degrees():
    samples = np.stack([_sample(index * 2., index) for index in range(211)])
    windows = _feed(MotionWindow(), samples)

    # A per-frame "w >= 0" fix would silently negate these input features.
    at_240 = windows[120]
    assert at_240[-1, 0, 3] < 0.
    np.testing.assert_allclose(at_240[-1], samples[120], atol=2e-6, rtol=0.)
    at_360 = windows[180]
    assert at_360[-1, 0, 3] == pytest.approx(-1.)
    np.testing.assert_allclose(windows[-1], samples[-64:], atol=2e-6, rtol=0.)


def test_rolling_windows_keep_identical_components_on_their_overlap():
    rng = np.random.default_rng(42)
    samples = np.stack([_sample(index * 2.5, index) for index in range(240)])
    samples[:, :, 3:] *= rng.choice([-1., 1.], (len(samples), 6, 1))
    windows = [result for result in _feed(MotionWindow(), samples) if result is not None]

    for previous, current in zip(windows, windows[1:]):
        np.testing.assert_allclose(current[:-1], previous[1:], atol=2e-6, rtol=0.)


@pytest.mark.parametrize("reset", ["clear", "gap", "reversal"])
def test_new_stream_after_reset_is_anchored_to_identity(reset):
    window = MotionWindow()
    _feed(window, [_sample(index * 2.) for index in range(131)])
    assert window.samples[-1][1][0, 3] < 0.
    last_stamp = window.samples[-1][0]
    first_stamp = last_stamp + 1. / 60.

    if reset == "clear":
        window.clear()
    elif reset == "gap":
        first_stamp = last_stamp + .3
    else:
        with pytest.raises(ValueError, match="reversed"):
            window.append(last_stamp - 1., _sample(270.))
        assert not window.samples

    sample = _sample(270.)
    assert np.all(sample[:, 3] < 0.)
    result = _feed(window, [sample] * 64, start=first_stamp)[-1]
    assert result is not None
    expected = sample.copy()
    expected[:, 3:] *= -1.
    np.testing.assert_allclose(result, np.repeat(expected[None], 64, axis=0),
                               atol=2e-6, rtol=0.)


def test_ignored_duplicate_does_not_change_history_or_next_signed_window():
    baseline, with_duplicate = MotionWindow(), MotionWindow()
    samples = [_sample(index * 2., index) for index in range(151)]
    _feed(baseline, samples[:-1])
    _feed(with_duplicate, samples[:-1])
    previous = [(stamp, value.copy()) for stamp, value in with_duplicate.samples]
    duplicate = _sample(120.)
    duplicate[:, 3:] *= -1.
    assert with_duplicate.append(149 / 60., duplicate) is None
    assert len(with_duplicate.samples) == len(previous)
    for (stamp, value), (old_stamp, old_value) in zip(with_duplicate.samples, previous):
        assert stamp == old_stamp
        np.testing.assert_array_equal(value, old_value)
    np.testing.assert_allclose(with_duplicate.append(150 / 60., samples[-1]),
                               baseline.append(150 / 60., samples[-1]),
                               atol=2e-6, rtol=0.)


def test_input_sample_is_never_mutated_and_buffer_owns_its_copy():
    window = MotionWindow()
    sample = _sample(30.)
    sample[:, 3:] *= -1.
    expected_raw = sample.copy()
    sample.setflags(write=False)
    assert window.append(0., sample) is None
    np.testing.assert_array_equal(sample, expected_raw)
    assert not np.shares_memory(window.samples[-1][1], sample)
    assert np.all(window.samples[-1][1][:, 3] > 0.)


def test_slerp_output_sign_changes_are_anchored_to_stream_not_accepted_as_features(monkeypatch):
    real_slerp = adapter.Slerp

    class SignFlippingSlerp:
        def __init__(self, times, rotations):
            self.inner = real_slerp(times, rotations)

        def __call__(self, grid):
            q = self.inner(grid).as_quat()
            q[::2] *= -1.  # Includes q[0]; each quaternion still means the same rotation.
            return Rotation.from_quat(q)

    samples = [_sample(index * 2., index) for index in range(151)]
    baseline = _feed(MotionWindow(), samples)
    monkeypatch.setattr(adapter, "Slerp", SignFlippingSlerp)
    changed = _feed(MotionWindow(), samples)
    for expected, actual in zip(baseline[63:], changed[63:]):
        np.testing.assert_allclose(actual, expected, atol=2e-6, rtol=0.)


@pytest.mark.parametrize("quaternion", [[0., 0., 0., 0.], [2., 0., 0., 0.]])
def test_invalid_nonunit_quaternion_is_rejected_without_altering_history(quaternion):
    window = MotionWindow()
    window.append(0., _sample(0.))
    sample = _sample(1.)
    sample[2, 3:] = quaternion
    with pytest.raises(ValueError, match="unit"):
        window.append(1. / 60., sample)
    assert len(window.samples) == 1
