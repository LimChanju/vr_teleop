"""SparsePoser BVH poses -> unmodified official GMR -> G1 preview references.

This is a simulation/kinematic adapter, NOT a robot command transport. In
particular a finite retargeted pose is not a dynamically balanced policy action.

The pinned GMR Xsens parser uses BVH [X,Y,Z] -> robot [Z,X,Y] (right-handed).
Its bundled ``251021_04_boxing_120Hz_cm_3DsMax.bvh`` has N-pose arm axes:
both upper/lower arm offsets point along local -Y. SparsePoser's reference BVH
has T-pose arm axes, +X left / -X right. Copying names or applying only a world
rotation therefore makes the arms wrong by 90 degrees. We change each arm's
LOCAL segment basis before the world/local [Z,X,Y] basis conversion.

Source contracts: GMR utils/xsens.py (3DSM ankle position AND orientation),
utils/xsens_vendor/BVHParser.py, ik_configs/bvh_xsens_to_g1.json. No upstream
source, IK weights, robot joint limits or velocity limits are changed here.
"""
from __future__ import annotations

from dataclasses import replace
import importlib
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

from .retargeting import Twist2ReferenceEncoder
from .shin_refinement import forward_kinematics
from .sparseposer_adapter import MountCalibration


BVH_TO_GMR = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
GMR_JOINTS = (
    "Hips", "Chest4", "LeftHip", "RightHip", "LeftKnee", "RightKnee",
    "LeftFootMod", "RightFootMod", "LeftShoulder", "RightShoulder",
    "LeftElbow", "RightElbow", "LeftWrist", "RightWrist",
)


class SparsePoserGMRConverter:
    """Validated coordinate adapter, independent of MuJoCo, Redis and VR.

    Retargeting is in the calibrated body-forward frame, not SteamVR's arbitrary
    room yaw/origin. Positions restore actual human metres (undo model scaling).
    Quaternions for GMR are GLOBAL WXYZ, not network LOCAL XYZW.
    """

    def __init__(self, skeleton, calibration):
        self.skeleton = skeleton
        self.calibration = MountCalibration(calibration, skeleton)
        self.indices = {name: i for i, name in enumerate(skeleton.names)}
        missing = {"LeftAnkle", "RightAnkle", *GMR_JOINTS} - {"LeftFootMod", "RightFootMod"} - self.indices.keys()
        if missing:
            raise ValueError(f"SparsePoser skeleton missing required GMR joints: {sorted(missing)}")
        self.basis = Rotation.from_matrix(BVH_TO_GMR)
        self.arm_basis = {}
        # Do not infer an arbitrary twist from a single bone direction. These
        # explicit rotations preserve the reference BVH's local forward +Z.
        for side, sign in (("Left", 1), ("Right", -1)):
            for child in ("Elbow", "Wrist"):
                offset = np.asarray(skeleton.offsets[self.indices[side + child]])
                unit = offset / np.linalg.norm(offset)
                if not np.allclose(unit, [sign, 0, 0], atol=0.005):
                    raise ValueError("supported SparsePoser reference requires T-pose arm local +/-X axes")
            correction = Rotation.from_euler("z", sign * 90, degrees=True)
            for segment in ("Shoulder", "Elbow", "Wrist"):
                self.arm_basis[side + segment] = correction

    def convert(self, estimate):
        if (estimate.get("schema_version") != 1
                or estimate.get("kind") != "sparseposer_eight_sensor_estimate"
                or estimate.get("coordinate_frame") != "openvr_standing"):
            raise ValueError("expected an EightTrackerEstimator estimate in OpenVR standing space")
        if tuple(estimate.get("joint_names", ())) != tuple(self.skeleton.names):
            raise ValueError("estimate joint ordering differs from calibrated reference skeleton")
        if not np.array_equal(estimate.get("parents"), self.skeleton.parents):
            raise ValueError("estimate skeleton parent ordering differs")
        if not np.isfinite(float(estimate["timestamp_s"])):
            raise ValueError("nonfinite estimate timestamp")
        count = len(self.skeleton.names)
        local = np.asarray(estimate["local_rotations_xyzw"], dtype=float)
        root = np.asarray(estimate["root_position_model_m"], dtype=float)
        if (local.shape != (count, 4) or root.shape != (3,)
                or not np.isfinite(local).all() or not np.isfinite(root).all()
                or not np.allclose(np.linalg.norm(local, axis=1), 1., atol=1e-3)):
            raise ValueError("estimate must contain finite root and unit local XYZW rotations")
        positions, rotations = forward_kinematics(self.skeleton, local, root)
        c = self.calibration
        # Cross-check redundant world output: rejects a different calibration,
        # accidental double SpaceCal transform, or world/local quaternion mixup.
        world_positions = np.asarray(estimate["positions_m"], dtype=float)
        world_rotations = np.asarray(estimate["orientations_xyzw"], dtype=float)
        if (world_positions.shape != (count, 3) or world_rotations.shape != (count, 4)
                or not np.isfinite(world_positions).all() or not np.isfinite(world_rotations).all()
                or not np.allclose(np.linalg.norm(world_rotations, axis=1), 1., atol=1e-3)):
            raise ValueError("estimate world poses must be finite, correctly sized and unit quaternion")
        expected_positions = c.world_from_model.apply(positions / c.scale) + c.origin
        expected_rotations = c.world_from_model * Rotation.from_quat(rotations)
        if (not np.allclose(world_positions, expected_positions, atol=1e-5)
                or np.max((expected_rotations.inv() * Rotation.from_quat(world_rotations)).magnitude()) > 1e-4):
            raise ValueError("estimate world/canonical poses disagree with this calibration")
        result = {}
        for name in GMR_JOINTS:
            source_name = name.replace("FootMod", "Ankle")
            j = self.indices[source_name]
            r = Rotation.from_quat(rotations[j])
            if name in self.arm_basis:
                r = r * self.arm_basis[name]
            # BOTH local and world bases change. C*R alone would turn a neutral
            # upright pelvis into a 120-degree-tipped one.
            r = self.basis * r * self.basis.inv()
            result[name] = [self.basis.apply(positions[j] / c.scale),
                            r.as_quat()[[3, 0, 1, 2]]]
        return result


class SparsePoserG1Retargeter:
    """Official Xsens->G1 IK with diagnostics, for the simulation preview only.

    ``retarget`` returns ``(RobotReference, diagnostics)``. A reference can
    contain a useful preview qpos while ``valid=False``: e.g. shin constraints
    were not satisfied. Nothing in this class transmits it to a controller.
    """

    def __init__(self, gmr_checkout, skeleton, calibration, *, offset_to_ground=True):
        self.converter = SparsePoserGMRConverter(skeleton, calibration)
        checkout = Path(gmr_checkout).expanduser().resolve()
        if not (checkout / "general_motion_retargeting" / "motion_retarget.py").is_file():
            raise ValueError(f"GMR checkout not found: {checkout}")
        existing = sys.modules.get("general_motion_retargeting")
        if existing is not None and Path(existing.__file__).resolve().parent.parent != checkout:
            raise RuntimeError("a different GMR checkout is already imported in this process")
        sys.path.insert(0, str(checkout))
        try:
            upstream = importlib.import_module("general_motion_retargeting")
            self.gmr = upstream.GeneralMotionRetargeting(
                src_human="bvh_xsens", tgt_robot="unitree_g1",
                actual_human_height=float(calibration["height_m"]), verbose=False,
            )
        except ImportError as error:
            raise RuntimeError(f"official GMR dependencies unavailable: {error}") from error
        finally:
            sys.path.remove(str(checkout))
        self.model = self.gmr.model
        self.xml_file = self.gmr.xml_file
        if self.model.nq != 36 or self.model.nv != 35:
            raise ValueError("expected official free-root G1 29-DoF (nq=36, nv=35)")
        self.offset_to_ground = bool(offset_to_ground)
        self.encoder = Twist2ReferenceEncoder()
        self.last_timestamp = None

    def retarget(self, estimate):
        timestamp = float(estimate["timestamp_s"])
        if self.last_timestamp is not None and timestamp <= self.last_timestamp:
            raise ValueError("GMR preview timestamps must increase; create a new retargeter for replay restart")
        targets = self.converter.convert(estimate)
        qpos = np.asarray(self.gmr.retarget(targets, offset_to_ground=self.offset_to_ground), dtype=float)
        if (qpos.shape != (36,) or not np.isfinite(qpos).all()
                or not np.isclose(np.linalg.norm(qpos[3:7]), 1., atol=1e-4)):
            raise ValueError("official GMR returned invalid qpos")
        violations = []
        for j in range(self.model.njnt):
            if not self.model.jnt_limited[j]:
                continue
            value = float(qpos[self.model.jnt_qposadr[j]])
            lower, upper = self.model.jnt_range[j]
            if value < lower - 1e-4 or value > upper + 1e-4:
                violations.append({"joint": self.model.joint(j).name, "value_rad": value,
                                   "range_rad": [float(lower), float(upper)]})
        fit_ok = bool(estimate.get("fit_accepted", False))
        reason = []
        if not fit_ok:
            reason.append("human shin/foot refinement outside tolerance")
        if violations:
            reason.append("GMR joint limit violation")
        reference = self.encoder.encode(qpos, timestamp, "sparseposer_official_gmr_preview")
        reference = replace(reference, valid=not reason, reason="; ".join(reason))
        self.last_timestamp = timestamp
        diagnostics = {
            "kind": "sparseposer_gmr_kinematic_preview", "simulation_only": True,
            "robot_output": False, "policy_or_balance_verified": False,
            "gmr_source": "bvh_xsens", "gmr_target": "unitree_g1",
            "coordinate_frame": "calibrated_forward_x_left_y_up_z",
            "source_fit_accepted": fit_ok, "joint_limit_violations": violations,
            "ik_error_stage1": float(self.gmr.error1()),
            "ik_error_stage2": float(self.gmr.error2()),
            "offset_to_ground": self.offset_to_ground,
            "ground_behavior": ("official GMR aligns lowest ankle to 0.10m each frame; not jump-height faithful"
                                if self.offset_to_ground else "no per-frame ground alignment"),
        }
        return reference, diagnostics
