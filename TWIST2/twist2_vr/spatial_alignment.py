"""Rigid alignment between Quest/OpenVR and VIVE tracker map spaces."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence

import numpy as np

from .math3d import matrix_to_quaternion, quaternion_to_matrix
from .models import PoseSample


def validate_transform(matrix: Iterable[Iterable[float]], name: str = "transform") -> np.ndarray:
    result = np.asarray(matrix, dtype=np.float64)
    if result.shape != (4, 4) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite 4x4 matrix")
    if not np.allclose(result[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError(f"{name} has an invalid homogeneous bottom row")
    rotation = result[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
        raise ValueError(f"{name} rotation must be orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5):
        raise ValueError(f"{name} rotation must have determinant +1")
    return result


def pose_to_matrix(pose: PoseSample) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = quaternion_to_matrix(pose.orientation_xyzw)
    matrix[:3, 3] = pose.position_m
    return matrix


def transform_pose(pose: PoseSample, transform: np.ndarray, *, source_frame: str) -> PoseSample:
    transform = validate_transform(transform)
    rotation = transform[:3, :3]
    output_rotation = rotation @ quaternion_to_matrix(pose.orientation_xyzw)
    return pose.replaced(
        position_m=rotation @ pose.position_m + transform[:3, 3],
        orientation_xyzw=matrix_to_quaternion(output_rotation),
        metadata={**pose.metadata, "spatially_aligned_from": source_frame},
    )


def _inverse(matrix: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    rotation = matrix[:3, :3]
    result[:3, :3] = rotation.T
    result[:3, 3] = -rotation.T @ matrix[:3, 3]
    return result


def _project_rotation(matrix: np.ndarray) -> np.ndarray:
    u, _singular, vt = np.linalg.svd(matrix)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0:
        u[:, -1] *= -1
        rotation = u @ vt
    return rotation


def _mean_rotation(rotations: Sequence[np.ndarray]) -> np.ndarray:
    if not rotations:
        raise ValueError("at least one rotation is required")
    return _project_rotation(np.sum(rotations, axis=0))


def _rotation_error_rad(left: np.ndarray, right: np.ndarray) -> float:
    error = left.T @ right
    cosine = float(np.clip((np.trace(error) - 1.0) * 0.5, -1.0, 1.0))
    return math.acos(cosine)


@dataclass(frozen=True)
class HandEyeResult:
    vive_to_quest: np.ndarray
    tracker_from_controller: np.ndarray
    sample_count: int
    pair_count: int
    position_rmse_m: float
    orientation_rmse_deg: float
    rotation_system_rank: int
    translation_system_rank: int


def solve_hand_eye(
    quest_controller_poses: Sequence[np.ndarray],
    vive_tracker_poses: Sequence[np.ndarray],
    *,
    minimum_rotation_deg: float = 4.0,
    max_pairs: int = 4000,
) -> HandEyeResult:
    """Solve ``Q_i = A V_i B`` from a rigid controller/tracker fixture.

    ``A`` maps the VIVE world into Quest/OpenVR standing space. ``B`` is the
    fixed tracker-to-controller mount transform used only during calibration.
    The method is a closed-form ``AX = XB`` solution followed by averaging.
    It requires rotations about multiple axes; translating without rotating is
    intentionally rejected as under-constrained.
    """

    if len(quest_controller_poses) != len(vive_tracker_poses):
        raise ValueError("Quest and VIVE calibration sample counts differ")
    if len(quest_controller_poses) < 8:
        raise ValueError("hand-eye calibration requires at least 8 paired samples")
    quest = [validate_transform(value, "Quest pose") for value in quest_controller_poses]
    vive = [validate_transform(value, "VIVE pose") for value in vive_tracker_poses]

    candidates: list[tuple[np.ndarray, np.ndarray]] = []
    minimum_rotation = math.radians(minimum_rotation_deg)
    for first in range(len(quest) - 1):
        for second in range(first + 1, len(quest)):
            vive_relative = _inverse(vive[first]) @ vive[second]
            quest_relative = _inverse(quest[first]) @ quest[second]
            rotation_motion = max(
                _rotation_error_rad(np.eye(3), vive_relative[:3, :3]),
                _rotation_error_rad(np.eye(3), quest_relative[:3, :3]),
            )
            if rotation_motion >= minimum_rotation:
                candidates.append((vive_relative, quest_relative))
    if len(candidates) > max_pairs:
        indices = np.linspace(0, len(candidates) - 1, max_pairs, dtype=int)
        candidates = [candidates[index] for index in indices]
    if len(candidates) < 6:
        raise ValueError(
            "calibration motion lacks rotational diversity; rotate the rigid controller/tracker fixture "
            "around pitch, yaw, and roll"
        )

    rotation_rows = []
    identity = np.eye(3)
    for vive_relative, quest_relative in candidates:
        rotation_rows.append(
            np.kron(identity, vive_relative[:3, :3])
            - np.kron(quest_relative[:3, :3].T, identity)
        )
    rotation_system = np.vstack(rotation_rows)
    rotation_rank = int(np.linalg.matrix_rank(rotation_system, tol=1e-7))
    if rotation_rank < 8:
        raise ValueError(
            f"calibration rotation system is under-constrained (rank {rotation_rank}/8); "
            "use larger rotations around multiple axes"
        )
    _u, _s, vh = np.linalg.svd(rotation_system, full_matrices=False)
    raw_rotation = vh[-1].reshape((3, 3), order="F")

    best: tuple[float, np.ndarray, np.ndarray, int] | None = None
    for sign in (1.0, -1.0):
        mount_rotation = _project_rotation(sign * raw_rotation)
        translation_left = []
        translation_right = []
        for vive_relative, quest_relative in candidates:
            translation_left.append(vive_relative[:3, :3] - identity)
            translation_right.append(
                mount_rotation @ quest_relative[:3, 3] - vive_relative[:3, 3]
            )
        left = np.vstack(translation_left)
        right = np.concatenate(translation_right)
        translation_rank = int(np.linalg.matrix_rank(left, tol=1e-7))
        if translation_rank < 3:
            continue
        mount_translation, *_ = np.linalg.lstsq(left, right, rcond=None)
        mount = np.eye(4, dtype=np.float64)
        mount[:3, :3] = mount_rotation
        mount[:3, 3] = mount_translation

        world_candidates = [
            quest_pose @ _inverse(mount) @ _inverse(vive_pose)
            for quest_pose, vive_pose in zip(quest, vive)
        ]
        world = np.eye(4, dtype=np.float64)
        world[:3, :3] = _mean_rotation([value[:3, :3] for value in world_candidates])
        world[:3, 3] = np.mean([value[:3, 3] for value in world_candidates], axis=0)

        position_errors = []
        orientation_errors = []
        for quest_pose, vive_pose in zip(quest, vive):
            predicted = world @ vive_pose @ mount
            position_errors.append(float(np.linalg.norm(predicted[:3, 3] - quest_pose[:3, 3])))
            orientation_errors.append(
                _rotation_error_rad(predicted[:3, :3], quest_pose[:3, :3])
            )
        position_rmse = float(np.sqrt(np.mean(np.square(position_errors))))
        orientation_rmse = float(np.sqrt(np.mean(np.square(orientation_errors))))
        score = position_rmse + orientation_rmse
        if best is None or score < best[0]:
            best = (score, world, mount, translation_rank)

    if best is None:
        raise ValueError("calibration translation system is under-constrained")
    _score, world, mount, translation_rank = best
    position_errors = []
    orientation_errors = []
    for quest_pose, vive_pose in zip(quest, vive):
        predicted = world @ vive_pose @ mount
        position_errors.append(float(np.linalg.norm(predicted[:3, 3] - quest_pose[:3, 3])))
        orientation_errors.append(_rotation_error_rad(predicted[:3, :3], quest_pose[:3, :3]))
    return HandEyeResult(
        vive_to_quest=world,
        tracker_from_controller=mount,
        sample_count=len(quest),
        pair_count=len(candidates),
        position_rmse_m=float(np.sqrt(np.mean(np.square(position_errors)))),
        orientation_rmse_deg=math.degrees(float(np.sqrt(np.mean(np.square(orientation_errors))))),
        rotation_system_rank=rotation_rank,
        translation_system_rank=translation_rank,
    )


__all__ = [
    "HandEyeResult",
    "pose_to_matrix",
    "solve_hand_eye",
    "transform_pose",
    "validate_transform",
]
