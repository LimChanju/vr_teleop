from pathlib import Path

import numpy as np

from twist2_vr.config import load_json
from twist2_vr.cli import _ik_summary
from twist2_vr.models import PoseSample, TrackerFrame
from twist2_vr.providers.mock import FULL_TARGET_POSES
from twist2_vr.reconstruction import GMR_BODY_JOINTS
from twist2_vr.retargeting import DEFAULT_G1_Q29, QuestArmIKRetargeter
from twist2_vr.upperbody import QuestUpperBodyReconstructor


ROOT = Path(__file__).resolve().parents[2]


def control(grip=0.0, *, active=True, stick=False, secondary=False):
    return {
        "active": active,
        "available": {"grip": active},
        "grip": grip,
        "thumbstick_click": stick,
        "secondary": secondary,
    }


def frame(timestamp, *, left=(0.3, 0.3, 1.2), right=(0.3, -0.3, 1.2),
          head=(0.0, 0.0, 1.7), head_quat=(0, 0, 0, 1), grips=(0.0, 0.0),
          valid=True, generation=0, extra=False):
    poses = {}
    for role, value, serial in (
        ("Head", head, "hmd"),
        ("Left_Wrist", left, "left-touch"),
        ("Right_Wrist", right, "right-touch"),
    ):
        poses[role] = PoseSample(
            serial, role, value, head_quat if role == "Head" else [0, 0, 0, 1],
            timestamp, valid=valid, metadata={"connection_generation": generation},
        )
    if extra:
        poses["Pelvis"] = PoseSample("vive-extra", "Pelvis", [9, 9, 9], [0, 0, 0, 1], timestamp)
    return TrackerFrame(
        timestamp, poses, "test", "twist2_neutral",
        controls={"left": control(grips[0]), "right": control(grips[1])},
    )


def fast_config():
    config = load_json(ROOT / "config/quest_arms.json")
    config.update({
        "neutral_samples": 3,
        "neutral_duration_s": 0.02,
    })
    return config


def calibrate(reconstructor):
    for i in range(3):
        body = reconstructor(frame(1.0 + 0.02 * i))
    assert body.valid and reconstructor.calibrated
    assert reconstructor.status.state == "READY"
    return body


def test_release_before_arm_fixed_head_and_extra_trackers_are_ignored():
    reconstructor = QuestUpperBodyReconstructor(fast_config())
    reconstructor(frame(0.0, grips=(1.0, 1.0)))
    assert not reconstructor.calibrated and not reconstructor.armed

    neutral = calibrate(reconstructor)
    reconstructor(frame(1.08, grips=(1.0, 1.0), extra=True))
    assert reconstructor.armed
    moved_head_only = reconstructor(frame(
        1.10, grips=(1.0, 1.0), head=(0.5, -0.2, 1.3),
        head_quat=(0, 0, np.sin(0.5), np.cos(0.5)), extra=True,
    ))
    for side in ("Left", "Right"):
        np.testing.assert_allclose(
            moved_head_only.joints[f"{side}_Wrist"].position_m,
            neutral.joints[f"{side}_Wrist"].position_m,
        )
    assert moved_head_only.joints["Pelvis"].device_id == "canonical:Pelvis"
    assert moved_head_only.provenance["Pelvis"] == "fixed_neutral"


def test_controller_translation_moves_only_wrist_target_and_release_freezes():
    reconstructor = QuestUpperBodyReconstructor(fast_config())
    neutral = calibrate(reconstructor)
    reconstructor(frame(1.08, grips=(1.0, 1.0)))
    moved = reconstructor(frame(1.10, left=(0.4, 0.3, 1.35), grips=(1.0, 1.0)))
    left_delta = moved.joints["Left_Wrist"].position_m - neutral.joints["Left_Wrist"].position_m
    np.testing.assert_allclose(left_delta, [0.1, 0.0, 0.15], atol=1e-9)
    np.testing.assert_allclose(
        moved.joints["Right_Wrist"].position_m,
        neutral.joints["Right_Wrist"].position_m,
    )
    held = reconstructor(frame(1.12, left=(0.8, 0.3, 1.6), grips=(0.0, 1.0)))
    np.testing.assert_allclose(held.joints["Left_Wrist"].position_m, moved.joints["Left_Wrist"].position_m)
    assert not reconstructor.armed and reconstructor.status.state == "READY"


def test_tracking_loss_requires_new_release_and_calibration():
    reconstructor = QuestUpperBodyReconstructor(fast_config())
    calibrate(reconstructor)
    reconstructor(frame(1.08, grips=(1.0, 1.0)))
    assert reconstructor.armed
    safe = reconstructor(frame(1.10, left=(0.35, 0.3, 1.25), grips=(1.0, 1.0)))
    lost = reconstructor(frame(1.12, valid=False, grips=(1.0, 1.0)))
    assert not reconstructor.armed and not reconstructor.calibrated
    assert "Head" in reconstructor.status.detail
    assert "Left_Wrist" in reconstructor.status.detail
    np.testing.assert_allclose(lost.joints["Left_Wrist"].position_m, safe.joints["Left_Wrist"].position_m)
    reconstructor(frame(1.14, generation=1, grips=(1.0, 1.0)))
    assert reconstructor.status.state == "WAIT_RELEASE"


def test_controller_target_is_not_cartesian_clipped():
    reconstructor = QuestUpperBodyReconstructor(fast_config())
    calibrate(reconstructor)
    reconstructor(frame(1.08, grips=(1.0, 1.0)))
    moved = reconstructor(frame(
        1.18,
        left=(0.8, 0.5, 1.8),
        right=(-0.2, -0.5, 0.6),
        grips=(1.0, 1.0),
    ))
    neutral_left = np.asarray(FULL_TARGET_POSES["Left_Wrist"][0])
    neutral_right = np.asarray(FULL_TARGET_POSES["Right_Wrist"][0])
    np.testing.assert_allclose(
        moved.joints["Left_Wrist"].position_m - neutral_left,
        [0.5, 0.2, 0.6],
    )
    np.testing.assert_allclose(
        moved.joints["Right_Wrist"].position_m - neutral_right,
        [-0.5, -0.2, -0.6],
    )


def test_unclipped_controller_target_uses_only_physical_arm_joint_ranges():
    reconstructor = QuestUpperBodyReconstructor(fast_config())
    calibrate(reconstructor)
    reconstructor(frame(1.08, grips=(1.0, 1.0)))
    moved = reconstructor(frame(
        1.18,
        left=(0.8, 0.5, 1.8),
        right=(-0.2, -0.5, 0.6),
        grips=(1.0, 1.0),
    ))

    config = load_json(ROOT / "config/quest_arms.json")
    retargeter = QuestArmIKRetargeter(ROOT / "assets/g1/g1_mocap_29dof.xml", config)
    reference = retargeter.retarget(moved)

    assert np.isfinite(reference.mimic_obs).all()
    for side, arm_slice in (("Left", slice(15, 19)), ("Right", slice(22, 26))):
        lower, upper = retargeter.joint_limits[side]
        q = reference.mimic_obs[6:][arm_slice]
        assert np.all(q >= lower - 1e-6) and np.all(q <= upper + 1e-6)


def canonical_body(timestamp, left_delta=(0, 0, 0), right_delta=(0, 0, 0)):
    joints = {}
    provenance = {}
    for name in (*GMR_BODY_JOINTS, "Head"):
        position, quaternion = FULL_TARGET_POSES[name]
        delta = left_delta if name == "Left_Wrist" else right_delta if name == "Right_Wrist" else (0, 0, 0)
        joints[name] = PoseSample(
            f"test:{name}", name, np.asarray(position) + np.asarray(delta), quaternion, timestamp
        )
        provenance[name] = "test"
    from twist2_vr.models import BodyPoseFrame
    return BodyPoseFrame(timestamp, joints, provenance, True)


def test_quest_arm_ik_changes_only_proximal_arms_and_raises_wrists():
    config = load_json(ROOT / "config/quest_arms.json")
    retargeter = QuestArmIKRetargeter(ROOT / "assets/g1/g1_mocap_29dof.xml", config)
    reference = retargeter.retarget(canonical_body(1.0))
    np.testing.assert_allclose(reference.mimic_obs[6:], DEFAULT_G1_Q29, atol=1e-7)

    for i in range(1, 31):
        reference = retargeter.retarget(canonical_body(
            1.0 + 0.02 * i, left_delta=(0.08, 0.0, 0.20), right_delta=(0.08, 0.0, 0.20)
        ))
    q29 = reference.mimic_obs[6:]
    protected = np.ones(29, dtype=bool)
    protected[15:19] = False
    protected[22:26] = False
    np.testing.assert_allclose(q29[protected], DEFAULT_G1_Q29[protected], atol=0.0)
    assert np.max(np.abs(q29[~protected] - DEFAULT_G1_Q29[~protected])) > 0.05
    np.testing.assert_allclose(reference.mimic_obs[:6], [0, 0, 0.8, 0, 0, 0], atol=0.0)

    retargeter.data.qpos[:] = reference.qpos
    retargeter.mujoco.mj_forward(retargeter.model, retargeter.data)
    for side in ("Left", "Right"):
        achieved = retargeter.data.xpos[retargeter.wrist_body_ids[side]]
        assert achieved[2] > retargeter.neutral_wrist_positions[side][2] + 0.12

    summary = _ik_summary(retargeter)
    assert "L=" in summary and "R=" in summary and "mm" in summary
    assert "scale=" not in summary


def test_unreachable_arm_target_reports_residual_without_software_projection():
    config = load_json(ROOT / "config/quest_arms.json")
    retargeter = QuestArmIKRetargeter(ROOT / "assets/g1/g1_mocap_29dof.xml", config)
    reference = retargeter.retarget(canonical_body(
        1.0, left_delta=(2.0, -2.0, 2.0), right_delta=(2.0, 2.0, 2.0)
    ))
    assert np.isfinite(reference.mimic_obs).all()
    for side, arm_slice in (("Left", slice(15, 19)), ("Right", slice(22, 26))):
        lower, upper = retargeter.joint_limits[side]
        q = reference.mimic_obs[6:][arm_slice]
        assert np.all(q >= lower - 1e-6) and np.all(q <= upper + 1e-6)
        assert retargeter.last_residual_m[side] > 1.0
    summary = _ik_summary(retargeter)
    assert "scale=" not in summary
