"""Eight-sensor Quest/VIVE adapter; upstream SparsePoser still has SIX inputs.

All sensors must already share OpenVR standing space (SpaceCal applied ONCE).
Separate T-pose calibration estimates rigid mounts against the reference body;
this is an anthropometric approximation, not measured anatomical joint centers.
The learned prior is refined using calibrated shin AND toe position/orientation.
This module cannot publish robot commands.
"""
from __future__ import annotations

from collections import deque
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from .models import PoseSample, TrackerFrame
from .shin_refinement import forward_kinematics, refine_legs


ROLE_JOINT = {
    "Pelvis": "Hips", "Left_Foot": "LeftToe", "Right_Foot": "RightToe",
    "Head": "Head", "Left_Wrist": "LeftWrist", "Right_Wrist": "RightWrist",
    "Left_Knee": "LeftKnee", "Right_Knee": "RightKnee",
}
ROLES = tuple(ROLE_JOINT)
NETWORK_ROLES = ROLES[:6]
LOWER_ROLES = ("Left_Knee", "Right_Knee", "Left_Foot", "Right_Foot")


def file_sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def skeleton_fingerprint(skeleton):
    data = {k: v.tolist() if isinstance(v, np.ndarray) else v
            for k, v in asdict(skeleton).items()}
    return hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()


def check_frame(frame, identities=None, max_age=0.20):
    if frame.coordinate_frame != "openvr_standing":
        raise ValueError("expected already-aligned openvr_standing input; do not apply SDK transform again")
    for role in ROLES:
        pose = frame.poses.get(role)
        if pose is None or not pose.valid:
            raise ValueError(f"missing/invalid {role}")
        if pose.tracking_state not in ("200", "tracked", "ready"):
            raise ValueError(f"{role}: not full position tracking ({pose.tracking_state})")
        if not -0.01 <= frame.timestamp_s - pose.timestamp_s <= max_age:
            raise ValueError(f"stale/future {role}")
        if identities and identities.get(role) != pose.device_id:
            raise ValueError(f"{role} device changed: recalibrate mounts")
    if len({frame.poses[r].device_id for r in ROLES}) != len(ROLES):
        raise ValueError("each role requires a unique device identity")


def read_recording(path):
    """Preserve capture timestamps, unlike the generic wall-clock replay provider."""
    with Path(path).open() as stream:
        for line in stream:
            if not line.strip():
                continue
            row = json.loads(line)
            if row.get("schema_version") != 1:
                raise ValueError("expected schema_version=1 raw pose recording")
            stamp = float(row["timestamp_s"])
            poses = {role: PoseSample(
                device_id=p["device_id"], role=role, position_m=p["position_m"],
                orientation_xyzw=p["orientation_xyzw"], timestamp_s=p.get("timestamp_s", stamp),
                valid=p.get("valid", True), tracking_state=p.get("tracking_state", "tracked"),
                metadata=p.get("metadata", {})) for role, p in row["poses"].items()}
            yield TrackerFrame(stamp, poses, row["source"], row["coordinate_frame"],
                               row.get("controls", {}), row.get("sequence", 0))


def fit_tpose_calibration(frames, skeleton, *, height_m=1.70, spacecal_sha256=None):
    """Fit mounts ONLY to a deliberate stable, forward-facing T-pose capture.

    Template proportions are scaled uniformly, not secretly fitted to an
    arms-down recording. Head sensor center approximates the template head.
    Foot targets are BVH toes; knee targets follow each rigid LOWER leg.
    """
    if not 1.2 <= height_m <= 2.2:
        raise ValueError("height_m outside supported calibration range 1.2–2.2 m")
    if len(frames) < 20 or frames[-1].timestamp_s - frames[0].timestamp_s < 1.0:
        raise ValueError("capture at least one second / 20 valid T-pose frames")
    identities = {r: frames[0].poses[r].device_id for r in ROLES if r in frames[0].poses}
    means, quality = {}, {}
    for f in frames:
        check_frame(f, identities)
    times = np.array([f.timestamp_s for f in frames])
    if np.any(np.diff(times) <= 0) or np.max(np.diff(times)) > 0.20:
        raise ValueError("T-pose capture has out-of-order timestamps or tracking gaps")
    for role in ROLES:
        poses = [f.poses[role] for f in frames]
        if len({p.metadata.get("connection_generation", 0) for p in poses}) != 1:
            raise ValueError(f"{role} reconnected during capture")
        p = np.stack([s.position_m for s in poses])
        r = Rotation.from_quat([s.orientation_xyzw for s in poses])
        mean_r = r.mean()
        std = float(np.max(p.std(axis=0)))
        degrees = float(np.degrees((mean_r.inv() * r).magnitude()).max())
        if std > 0.02 or degrees > 8.0:
            raise ValueError(f"{role} moved during T-pose: std={std:.3f}m rotation={degrees:.1f}deg")
        means[role] = (p.mean(axis=0), mean_r)
        quality[role] = {"position_std_m": std, "max_rotation_deviation_deg": degrees}

    # HMD's local -Z is forward in OpenVR; BVH template forward is +Z.
    forward = means["Head"][1].apply([0, 0, -1])
    forward[1] = 0
    if np.linalg.norm(forward) < 0.5:
        raise ValueError("look straight ahead, not down/up, during calibration")
    yaw = np.arctan2(forward[0], forward[2])
    world_from_model = Rotation.from_euler("y", yaw)
    model_from_world = world_from_model.inv()
    scale = 1.70 / height_m  # canonical model is the supplied male_170_85 template
    ref_pos, ref_rot = forward_kinematics(skeleton, skeleton.reference_local_rotations, np.zeros(3))
    root = np.array([0., -min(ref_pos[skeleton.names.index(ROLE_JOINT[r]), 1]
                              for r in ("Left_Foot", "Right_Foot")), 0.])
    ref_pos += root
    origin = means["Head"][0] - world_from_model.apply(ref_pos[skeleton.names.index("Head")]) / scale
    for suffix in ("Knee", "Foot"):
        left_p = model_from_world.apply(means[f"Left_{suffix}"][0] - origin)
        right_p = model_from_world.apply(means[f"Right_{suffix}"][0] - origin)
        if left_p[0] - right_p[0] < 0.03:
            raise ValueError(f"left/right {suffix} mapping reversed or feet crossed: check tracker roles")
    for side in ("Left", "Right"):
        if not (means[f"{side}_Foot"][0][1] + 0.10 < means[f"{side}_Knee"][0][1]
                < means["Pelvis"][0][1] - 0.10):
            raise ValueError(f"{side}: expected foot below upper-shin tracker below pelvis")
    mounts = {}
    for role, joint in ROLE_JOINT.items():
        p, r = means[role]
        p_model = model_from_world.apply(p - origin) * scale
        r_model = model_from_world * r
        j = skeleton.names.index(joint)
        offset = r_model.inv().apply(ref_pos[j] - p_model)
        # Calibration quality checks, NOT motion clamps in live estimation.
        if np.linalg.norm(offset) / scale > 0.30:
            raise ValueError(f"{role} is >30cm from T-pose template; check T-pose, mounts, height, SpaceCal")
        mounts[role] = {"device_id": identities[role], "joint": joint,
                        "translation_model_m": offset.tolist(),
                        "rotation_xyzw": (r_model.inv() * Rotation.from_quat(ref_rot[j])).as_quat().tolist()}
        quality[role]["estimated_mount_distance_m"] = float(np.linalg.norm(offset) / scale)
    # Explicitly reject arms-down, crossed hands and bad lower-body role assignments.
    left = model_from_world.apply(means["Left_Wrist"][0] - means["Pelvis"][0])
    right = model_from_world.apply(means["Right_Wrist"][0] - means["Pelvis"][0])
    if left[0] < 0.35 * height_m / 1.7 or right[0] > -0.35 * height_m / 1.7:
        raise ValueError("T-pose required: extend both arms horizontally to their own sides")
    if abs(means["Left_Wrist"][0][1] - means["Right_Wrist"][0][1]) > 0.15:
        raise ValueError("hold both hands at the same shoulder height")
    return {"schema_version": 1, "kind": "sparseposer_tpose_mounts", "coordinate_frame": "openvr_standing",
            "height_m": height_m, "model_scale": scale, "world_origin_m": origin.tolist(),
            "world_from_model_xyzw": world_from_model.as_quat().tolist(), "mounts": mounts,
            "skeleton_sha256": skeleton_fingerprint(skeleton), "spacecal_sha256": spacecal_sha256,
            "quality": quality, "capture_frames": len(frames),
            "assumptions": "T-pose, uniform 170cm-template scaling; HMD center approximates head joint. Mounts are estimates, not anatomical measurements."}


class MountCalibration:
    def __init__(self, data, skeleton):
        if data.get("schema_version") != 1 or data.get("kind") != "sparseposer_tpose_mounts":
            raise ValueError("requires dedicated SparsePoser T-pose calibration, not a stable mount-capture summary")
        if data.get("skeleton_sha256") != skeleton_fingerprint(skeleton):
            raise ValueError("calibration reference skeleton differs")
        if data.get("coordinate_frame") != "openvr_standing":
            raise ValueError("calibration must use openvr_standing space")
        self.data = data
        self.scale = float(data["model_scale"])
        self.origin = np.asarray(data["world_origin_m"], dtype=float)
        if not np.isfinite(self.scale) or self.scale <= 0 or self.origin.shape != (3,) or not np.isfinite(self.origin).all():
            raise ValueError("invalid calibration origin/scale")
        self.world_from_model = Rotation.from_quat(data["world_from_model_xyzw"])
        self.identities = {r: data["mounts"][r]["device_id"] for r in ROLES}
        self.mounts = {}
        for role in ROLES:
            row = data["mounts"][role]
            t = np.asarray(row["translation_model_m"], dtype=float)
            if t.shape != (3,) or not np.isfinite(t).all() or row["joint"] != ROLE_JOINT[role]:
                raise ValueError(f"invalid mount for {role}")
            self.mounts[role] = (t, Rotation.from_quat(row["rotation_xyzw"]))

    def targets(self, frame):
        check_frame(frame, self.identities)
        result = {}
        inverse = self.world_from_model.inv()
        for role in ROLES:
            pose = frame.poses[role]
            r = inverse * Rotation.from_quat(pose.orientation_xyzw)
            p = inverse.apply(pose.position_m - self.origin) * self.scale
            t_mount, r_mount = self.mounts[role]
            result[role] = (p + r.apply(t_mount), (r * r_mount).as_quat())
        return result


def network_sample(targets):
    """Global root plus five root-relative targets, official order and WXYZ."""
    root_p, root_q = targets["Pelvis"]
    root_r = Rotation.from_quat(root_q)
    sample = []
    for i, role in enumerate(NETWORK_ROLES):
        p, q = targets[role]
        if i:
            p = root_r.inv().apply(p - root_p)
            q = (root_r.inv() * Rotation.from_quat(q)).as_quat()
        sample.append(np.concatenate((p, np.asarray(q)[[3, 0, 1, 2]])))
    return np.asarray(sample, dtype=np.float32)


class MotionWindow:
    """64 uniformly spaced samples ending NOW; interpolate, never extrapolate.

    The sample rate is explicit. A gap invalidates history rather than making
    a stationary fake body. Duplicate timestamps are ignored; reversal errors.
    Quaternion signs follow the official Unity stream: initialize against
    identity, then choose the sign nearest the preceding accepted sample.
    q and -q are the same rotation but NOT interchangeable network features.
    """
    def __init__(self, hz=60., max_gap=0.20):
        if not np.isfinite(hz) or hz <= 0 or not np.isfinite(max_gap) or max_gap <= 0:
            raise ValueError("window rate/gap must be positive")
        self.hz, self.max_gap = hz, max_gap
        self.samples = deque()

    def clear(self):
        self.samples.clear()

    def append(self, timestamp, sample):
        sample = np.array(sample, dtype=float, copy=True)
        if not np.isfinite(timestamp) or sample.shape != (6, 7) or not np.isfinite(sample).all():
            raise ValueError("invalid temporal sample")
        if not np.allclose(np.linalg.norm(sample[:, 3:], axis=-1), 1., atol=1e-3, rtol=0.):
            raise ValueError("temporal sample requires unit WXYZ quaternions")
        if self.samples:
            dt = timestamp - self.samples[-1][0]
            if dt < 0:
                self.clear()
                raise ValueError("timestamps reversed")
            if dt == 0:
                return None
            if dt > self.max_gap:
                self.clear()
        # Match PythonCommunication.cs's identity-initialized RotBuffer and
        # continuity against the previous STREAM sample, not just the previous
        # element of a newly created 64-frame window. Retained samples preserve
        # the chosen hemisphere as the window advances, including beyond 180°.
        previous = self.samples[-1][1][:, 3:] if self.samples else np.array([1., 0., 0., 0.])
        flip = np.sum(sample[:, 3:] * previous, axis=-1) < 0
        sample[flip, 3:] *= -1
        self.samples.append((timestamp, sample))
        start = timestamp - 63 / self.hz
        while len(self.samples) > 2 and self.samples[1][0] <= start:
            self.samples.popleft()
        if self.samples[0][0] > start + 1e-8:
            return None
        times = np.array([s[0] for s in self.samples])
        data = np.stack([s[1] for s in self.samples])
        grid = np.linspace(max(start, times[0]), times[-1], 64)
        result = np.empty((64, 6, 7), dtype=np.float32)
        for j in range(6):
            for k in range(3):
                result[:, j, k] = np.interp(grid, times, data[:, j, k])
            rotations = Rotation.from_quat(data[:, j, 3:][:, [1, 2, 3, 0]])
            q = Slerp(times, rotations)(grid).as_quat()[:, [3, 0, 1, 2]]
            # Slerp only promises rotations, not their quaternion signs. Anchor
            # its first output to the already aligned source; otherwise a
            # sign change here can negate an entire network input trajectory.
            if np.dot(q[0], data[0, j, 3:]) < 0:
                q[0] *= -1
            for i in range(1, len(q)):
                if np.dot(q[i-1], q[i]) < 0:
                    q[i] *= -1
            result[:, j, 3:] = q
        return result


class EightTrackerEstimator:
    def __init__(self, backend, calibration, *, hz=60., refinement_config=None):
        self.backend = backend
        self.calibration = MountCalibration(calibration, backend.skeleton)
        self.window = MotionWindow(hz)
        self.refinement_config = refinement_config
        self.generations = None

    def process(self, frame):
        try:
            targets = self.calibration.targets(frame)
            generations = tuple(frame.poses[r].metadata.get("connection_generation", 0) for r in ROLES)
            if self.generations is not None and generations != self.generations:
                self.window.clear()
            self.generations = generations
            window = self.window.append(frame.timestamp_s, network_sample(targets))
        except ValueError:
            self.window.clear()
            raise
        if window is None:
            return None
        local = self.backend.infer(window)
        # Match official demo's EnforceRootRot=true, never make root follow head.
        local[0] = targets["Pelvis"][1]
        root = targets["Pelvis"][0]
        refined, diagnostics = refine_legs(local, root, self.backend.skeleton,
                                          {r: targets[r] for r in LOWER_ROLES}, self.refinement_config)
        positions, rotations = forward_kinematics(self.backend.skeleton, refined, root)
        c = self.calibration
        world_positions = c.world_from_model.apply(positions / c.scale) + c.origin
        world_rotations = (c.world_from_model * Rotation.from_quat(rotations)).as_quat()
        return {"schema_version": 1, "kind": "sparseposer_eight_sensor_estimate",
                "timestamp_s": frame.timestamp_s, "coordinate_frame": "openvr_standing",
                "estimated_not_ground_truth": True, "robot_output": False,
                "fit_accepted": bool(diagnostics["success"]), "robot_eligible": False,
                "status": "estimated" if diagnostics["success"] else "fit_outside_tolerance",
                "upstream_network_input_roles": list(NETWORK_ROLES),
                "extra_shin_roles": ["Left_Knee", "Right_Knee"],
                "root_position_model_m": root.tolist(), "local_rotations_xyzw": refined.tolist(),
                "joint_names": list(self.backend.skeleton.names),
                "parents": self.backend.skeleton.parents.tolist(),
                "positions_m": world_positions.tolist(), "orientations_xyzw": world_rotations.tolist(),
                "refinement": diagnostics}
