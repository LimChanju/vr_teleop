import numpy as np

from twist2_vr.filtering import PoseFilter
from twist2_vr.models import PoseSample, TrackerFrame


def frame(timestamp, position=(0, 0, 0), valid=True):
    sample = PoseSample("p", "Pelvis", position, [0, 0, 0, 1], timestamp, valid=valid)
    return TrackerFrame(timestamp, {"Pelvis": sample}, "test", "target")


def test_filter_passes_pose_directly_and_rejects_stale_nonmonotonic():
    filt = PoseFilter({"max_stale_s": 0.2})
    assert filt(frame(1.0), now_s=1.0).poses["Pelvis"].valid
    direct = filt(frame(1.1, (10, -5, 3)), now_s=1.1).poses["Pelvis"]
    assert direct.valid
    np.testing.assert_allclose(direct.position_m, [10, -5, 3])
    stale = filt(frame(1.2), now_s=2.0).poses["Pelvis"]
    assert not stale.valid and stale.tracking_state == "stale"
    backwards = filt(frame(1.0), now_s=1.0).poses["Pelvis"]
    assert not backwards.valid and backwards.tracking_state == "non_monotonic"


def test_missing_or_reconnected_role_resets_filter_history():
    filt = PoseFilter({"max_stale_s": 0.2})
    filt(frame(1.0), now_s=1.0)
    filt(TrackerFrame(1.1, {}, "test", "target"), now_s=1.1)
    after_missing = filt(frame(1.2, (5, 0, 0)), now_s=1.2).poses["Pelvis"]
    assert after_missing.valid

    old = PoseSample(
        "p", "Pelvis", [5, 0, 0], [0, 0, 0, 1], 1.3,
        metadata={"connection_generation": 0},
    )
    filt(TrackerFrame(1.3, {"Pelvis": old}, "test", "target"), now_s=1.3)
    reconnected = old.replaced(
        position_m=[-5, 0, 0], timestamp_s=1.4,
        metadata={"connection_generation": 2},
    )
    accepted = filt(
        TrackerFrame(1.4, {"Pelvis": reconnected}, "test", "target"), now_s=1.4
    ).poses["Pelvis"]
    assert accepted.valid
    np.testing.assert_allclose(accepted.position_m, [-5, 0, 0])


def test_filter_rejects_nonpositive_stale_window():
    import pytest

    with pytest.raises(ValueError, match="max_stale_s"):
        PoseFilter({"max_stale_s": 0.0})
