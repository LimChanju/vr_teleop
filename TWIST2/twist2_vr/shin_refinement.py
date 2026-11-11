"""Fit calibrated shin/foot poses without changing the predicted upper body.

All positions use the skeleton's metric world frame (normally BVH Y-up), and
all quaternions are xyzw. Targets are *joint* poses, after tracker mounting
calibration: Left/Right_Knee refer to BVH Knee, Left/Right_Foot to BVH Toe.
Local rotations are complete BVH rotations, not deltas from the reference pose.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, fields
from numbers import Real
from time import perf_counter
from typing import Any

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class ShinRefinementConfig:
    """Residual multipliers, solver budget, and explicit acceptance tolerances.

Position residuals are metres; rotation/prior residuals are radians. These
weights multiply residuals before squaring. Priors are soft costs, not joint
limits. ``max_nfev`` bounds optimizer evaluations, not elapsed wall time.
    """

    knee_position_weight: float = 1.0
    knee_rotation_weight: float = 0.15
    foot_position_weight: float = 2.0
    foot_rotation_weight: float = 0.15
    prediction_prior_weight: float = 0.01
    reference_prior_weight: float = 0.002
    max_nfev: int = 30
    ftol: float = 1e-6
    xtol: float = 1e-6
    gtol: float = 1e-6
    position_tolerance_m: float = 0.03
    rotation_tolerance_rad: float = 0.20

    def __post_init__(self) -> None:
        for field in fields(self):
            value = getattr(self, field.name)
            if field.name == "max_nfev":
                if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or value < 1:
                    raise ValueError("max_nfev must be a positive integer")
            elif isinstance(value, bool) or not isinstance(value, Real) or not np.isfinite(value) or value < 0:
                raise ValueError(f"{field.name} must be finite and nonnegative")
        for name in ("knee_position_weight", "knee_rotation_weight", "foot_position_weight", "foot_rotation_weight"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive so every measured pose is used")
        for name in ("ftol", "xtol", "gtol"):
            if getattr(self, name) <= np.finfo(float).eps:
                raise ValueError(f"{name} must exceed machine epsilon")


def _array(value: Any, shape: tuple[int, ...], label: str) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a finite array of shape {shape}") from exc
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must be a finite array of shape {shape}")
    return result.copy()


def _quaternions(value: Any, count: int, label: str) -> np.ndarray:
    result = _array(value, (count, 4), label)
    norms = np.linalg.norm(result, axis=1)
    if not np.all(np.isfinite(norms)) or np.any(norms < 1e-8):
        raise ValueError(f"{label} contains a zero or invalid quaternion")
    return result / norms[:, None]


@dataclass(frozen=True)
class _Skeleton:
    names: tuple[str, ...]
    parents: np.ndarray
    offsets: np.ndarray
    references: np.ndarray
    order: tuple[int, ...]
    root: int


def _read_skeleton(skeleton: Any) -> _Skeleton:
    def get(name: str) -> Any:
        try:
            return skeleton[name] if isinstance(skeleton, Mapping) else getattr(skeleton, name)
        except (KeyError, AttributeError) as exc:
            raise ValueError(f"skeleton is missing {name}") from exc

    try:
        names = tuple(get("names"))
    except TypeError as exc:
        raise ValueError("skeleton names must be a sequence") from exc
    count = len(names)
    if not count or any(not isinstance(name, str) or not name for name in names) or len(set(names)) != count:
        raise ValueError("skeleton names must be unique nonempty strings")
    raw_parents = _array(get("parents"), (count,), "skeleton parents")
    if np.any(raw_parents != np.floor(raw_parents)) or np.any(raw_parents < -1) or np.any(raw_parents >= count):
        raise ValueError("skeleton parents must be integer indices or -1")
    parents = raw_parents.astype(np.int64)
    # PyMotion represents the root as its own parent (normally parents[0]=0).
    # Normalize our private copy, retaining both public skeleton conventions.
    roots = np.flatnonzero((parents == -1) | (parents == np.arange(count)))
    if len(roots) != 1:
        raise ValueError("skeleton must have exactly one root with parent -1 or itself")
    parents[roots[0]] = -1
    offsets = _array(get("offsets"), (count, 3), "skeleton offsets")
    references = _quaternions(get("reference_local_rotations"), count, "reference local rotations")
    order: list[int] = []
    state = np.zeros(count, dtype=np.uint8)

    def visit(joint: int) -> None:
        if state[joint] == 1:
            raise ValueError("skeleton parents contain a cycle")
        if state[joint] == 2:
            return
        state[joint] = 1
        if parents[joint] >= 0:
            visit(int(parents[joint]))
        state[joint] = 2
        order.append(joint)

    for joint in range(count):
        visit(joint)
    return _Skeleton(names, parents, offsets, references, tuple(order), int(roots[0]))


def _fk(skeleton: _Skeleton, local_matrices: np.ndarray, root_position: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    positions = np.empty((len(skeleton.names), 3))
    matrices = np.empty_like(local_matrices)
    for joint in skeleton.order:
        parent = skeleton.parents[joint]
        if parent < 0:
            # root_position is the root joint's world position, already including
            # any BVH root OFFSET applied by the caller.
            positions[joint] = root_position
            matrices[joint] = local_matrices[joint]
        else:
            positions[joint] = positions[parent] + matrices[parent] @ skeleton.offsets[joint]
            matrices[joint] = matrices[parent] @ local_matrices[joint]
    return positions, matrices


def forward_kinematics(skeleton: Any, local_rotations: Any, root_position: Any) -> tuple[np.ndarray, np.ndarray]:
    """Return world joint positions and xyzw rotations in skeleton name order.

``root_position`` is the actual world root joint position, not a translation
channel to which the root OFFSET should be added. A root parent of -1 or the
root's own index is accepted. Inputs are never modified.
    """
    spec = _read_skeleton(skeleton)
    local = _quaternions(local_rotations, len(spec.names), "local rotations")
    root = _array(root_position, (3,), "root position")
    positions, matrices = _fk(spec, Rotation.from_quat(local).as_matrix(), root)
    return positions, Rotation.from_matrix(matrices).as_quat()


def _hat(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def _right_jacobian(vector: np.ndarray, inverse: bool = False) -> np.ndarray:
    """SO(3) right Jacobian, including its stable small-angle limit."""
    angle_sq = float(vector @ vector)
    skew = _hat(vector)
    if inverse:
        coefficient = (1.0 / 12.0 + angle_sq / 720.0) if angle_sq < 1e-8 else (
            1.0 - 0.5 * np.sqrt(angle_sq) / np.tan(0.5 * np.sqrt(angle_sq))
        ) / angle_sq
        return np.eye(3) + 0.5 * skew + coefficient * (skew @ skew)
    if angle_sq < 1e-8:
        a, b = 0.5 - angle_sq / 24.0, 1.0 / 6.0 - angle_sq / 120.0
    else:
        angle = np.sqrt(angle_sq)
        a, b = (1.0 - np.cos(angle)) / angle_sq, (angle - np.sin(angle)) / (angle_sq * angle)
    return np.eye(3) - a * skew + b * (skew @ skew)


def _leg_chains(spec: _Skeleton) -> list[tuple[int, int, int, int]]:
    normalized = {name.replace("_", "").lower(): index for index, name in enumerate(spec.names)}
    if len(normalized) != len(spec.names):
        raise ValueError("skeleton joint names are ambiguous after removing underscores")
    chains = []
    for side in ("left", "right"):
        try:
            chain = tuple(normalized[side + name] for name in ("hip", "knee", "ankle", "toe"))
        except KeyError as exc:
            raise ValueError(f"skeleton requires {side} Hip/Knee/Ankle/Toe joints") from exc
        if any(spec.parents[child] != parent for parent, child in zip(chain, chain[1:])) or spec.parents[chain[0]] < 0:
            raise ValueError(f"{side} leg must be a non-root Hip -> Knee -> Ankle -> Toe chain")
        if any(np.linalg.norm(spec.offsets[joint]) < 1e-8 for joint in chain[1:]):
            raise ValueError(f"{side} leg contains a zero-length segment")
        chains.append(chain)
    if len(set(chains[0] + chains[1])) != 8:
        raise ValueError("left and right leg chains must be disjoint")
    # Neither leg may be an ancestor of the other; their parents remain fixed.
    leg_joints = set(chains[0] + chains[1])
    for chain in chains:
        parent = int(spec.parents[chain[0]])
        while parent >= 0:
            if parent in leg_joints:
                raise ValueError("leg chains must have independent fixed parents")
            parent = int(spec.parents[parent])
    return chains


def refine_legs(
    local_rotations: Any,
    root_position: Any,
    skeleton: Any,
    targets: Mapping[str, tuple[Any, Any]],
    config: ShinRefinementConfig | Mapping[str, Any] | None = None,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Refine only bilateral Hip/Knee/Ankle local rotations against four poses.

All four roles are required. Invalid input raises ValueError. An optimizer
failure returns a finite pose with explicit failure diagnostics; convergence
alone does not imply the measured targets are reachable. ``success`` requires
both convergence and all position/orientation acceptance tolerances to pass.
    """
    started = perf_counter()
    if config is None:
        cfg = ShinRefinementConfig()
    elif isinstance(config, ShinRefinementConfig):
        cfg = config
    elif isinstance(config, Mapping):
        try:
            cfg = ShinRefinementConfig(**config)
        except TypeError as exc:
            raise ValueError(f"invalid refinement config: {exc}") from exc
    else:
        raise ValueError("config must be a ShinRefinementConfig or mapping")
    spec = _read_skeleton(skeleton)
    original = _array(local_rotations, (len(spec.names), 4), "local rotations")
    local = _quaternions(original, len(spec.names), "local rotations")
    root = _array(root_position, (3,), "root position")
    chains = _leg_chains(spec)
    roles = ("Left_Knee", "Left_Foot", "Right_Knee", "Right_Foot")
    if not isinstance(targets, Mapping) or any(role not in targets for role in roles):
        raise ValueError("targets require Left_Knee, Right_Knee, Left_Foot, and Right_Foot")
    target_positions, target_quaternions = [], []
    for role in roles:
        try:
            position, quaternion = targets[role]
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{role} target must be (world position, xyzw rotation)") from exc
        target_positions.append(_array(position, (3,), f"{role} position"))
        target_quaternions.append(_quaternions([quaternion], 1, f"{role} quaternion")[0])
    target_positions = np.asarray(target_positions)
    target_matrices = Rotation.from_quat(target_quaternions).as_matrix()
    local_matrices = Rotation.from_quat(local).as_matrix()
    initial_positions, initial_matrices = _fk(spec, local_matrices, root)
    optimized = np.array([joint for chain in chains for joint in chain[:3]])
    measured = np.array([joint for chain in chains for joint in (chain[1], chain[3])])
    reference_matrices = Rotation.from_quat(spec.references[optimized]).as_matrix()
    weights = np.array([
        cfg.knee_position_weight, cfg.knee_rotation_weight,
        cfg.foot_position_weight, cfg.foot_rotation_weight,
    ] * 2)

    def evaluate(delta: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        vectors = delta.reshape(6, 3)
        changes = Rotation.from_rotvec(vectors).as_matrix()
        candidate = local_matrices[optimized] @ changes
        right_jacobians = np.array([_right_jacobian(vector) for vector in vectors])
        positions, matrices = [], []
        chain_positions, chain_matrices = [], []
        for leg, chain in enumerate(chains):
            parent = int(spec.parents[chain[0]])
            pos, mat = initial_positions[parent], initial_matrices[parent]
            for depth, joint in enumerate(chain):
                pos = pos + mat @ spec.offsets[joint]
                mat = mat @ (candidate[leg * 3 + depth] if depth < 3 else local_matrices[joint])
                if depth < 3:
                    chain_positions.append(pos)
                    chain_matrices.append(mat)
                if depth in (1, 3):
                    positions.append(pos)
                    matrices.append(mat)
        positions, matrices = np.asarray(positions), np.asarray(matrices)
        angle_errors = Rotation.from_matrix(target_matrices.transpose(0, 2, 1) @ matrices).as_rotvec()
        residuals = np.empty((8, 3))
        residuals[0::2] = positions - target_positions
        residuals[1::2] = angle_errors
        residuals *= weights[:, None]
        jacobian = np.zeros((24, 18))
        for target_index in range(4):
            leg = target_index // 2
            target_depth = 1 if target_index % 2 == 0 else 3
            rotation_log_jacobian = _right_jacobian(angle_errors[target_index], inverse=True)
            for depth in range(min(target_depth + 1, 3)):
                variable = leg * 3 + depth
                axes = chain_matrices[variable] @ right_jacobians[variable]
                column = slice(variable * 3, variable * 3 + 3)
                row = target_index * 6
                jacobian[row:row + 3, column] = (
                    -_hat(positions[target_index] - chain_positions[variable]) @ axes
                ) * weights[target_index * 2]
                jacobian[row + 3:row + 6, column] = (
                    rotation_log_jacobian @ matrices[target_index].T @ axes
                ) * weights[target_index * 2 + 1]
        residual_blocks, jacobian_blocks = [residuals.ravel()], [jacobian]
        for weight, prior_matrices in (
            (cfg.prediction_prior_weight, local_matrices[optimized]),
            (cfg.reference_prior_weight, reference_matrices),
        ):
            if weight == 0:
                continue
            errors = Rotation.from_matrix(prior_matrices.transpose(0, 2, 1) @ candidate).as_rotvec()
            prior_jacobian = np.zeros((18, 18))
            for variable in range(6):
                block = slice(variable * 3, variable * 3 + 3)
                prior_jacobian[block, block] = (
                    _right_jacobian(errors[variable], inverse=True) @ right_jacobians[variable]
                ) * weight
            residual_blocks.append((errors * weight).ravel())
            jacobian_blocks.append(prior_jacobian)
        return np.concatenate(residual_blocks), np.vstack(jacobian_blocks)

    # least_squares requests residual and Jacobian separately at the same point.
    cache_x, cache_value = None, None

    def cached(delta: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        nonlocal cache_x, cache_value
        if cache_x is None or not np.array_equal(cache_x, delta):
            cache_x, cache_value = delta.copy(), evaluate(delta)
        return cache_value

    initial_residual, _ = cached(np.zeros(18))
    initial_cost = float(initial_residual @ initial_residual / 2.0)
    output = original.copy()
    fallback, result = False, None
    failure_message = ""
    try:
        result = least_squares(
            lambda delta: cached(delta)[0], np.zeros(18),
            jac=lambda delta: cached(delta)[1], method="trf",
            max_nfev=cfg.max_nfev, ftol=cfg.ftol, xtol=cfg.xtol, gtol=cfg.gtol,
        )
        if not np.all(np.isfinite(result.x)) or not np.isfinite(result.cost):
            raise FloatingPointError("optimizer returned nonfinite values")
        if result.cost > initial_cost + 1e-12:
            raise FloatingPointError("optimizer increased the objective")
        output[optimized] = (
            Rotation.from_quat(local[optimized]) * Rotation.from_rotvec(result.x.reshape(6, 3))
        ).as_quat()
        # Keep quaternion representatives continuous with the predicted frame.
        signs = np.sum(output[optimized] * local[optimized], axis=1) < 0
        output[optimized[signs]] *= -1
    except (ValueError, FloatingPointError, np.linalg.LinAlgError) as exc:
        fallback, failure_message = True, f"refinement failed: {exc}"

    final_positions, final_matrices = _fk(spec, Rotation.from_quat(output).as_matrix(), root)

    def metrics(positions: np.ndarray, matrices: np.ndarray) -> dict[str, dict[str, float]]:
        distances = np.linalg.norm(positions[measured] - target_positions, axis=1)
        angles = Rotation.from_matrix(target_matrices.transpose(0, 2, 1) @ matrices[measured]).magnitude()
        return {
            role: {"position_error_m": float(distance), "rotation_error_rad": float(angle)}
            for role, distance, angle in zip(roles, distances, angles)
        }

    final_metrics = metrics(final_positions, final_matrices)
    max_position = max(value["position_error_m"] for value in final_metrics.values())
    max_rotation = max(value["rotation_error_rad"] for value in final_metrics.values())
    converged = bool(result is not None and result.success and not fallback)
    satisfied = bool(max_position <= cfg.position_tolerance_m and max_rotation <= cfg.rotation_tolerance_rad)
    diagnostics = {
        "success": converged and satisfied,
        "converged": converged,
        "targets_satisfied": satisfied,
        "fallback_used": fallback,
        "status": -1 if fallback or result is None else int(result.status),
        "message": failure_message or str(result.message),
        "nfev": 0 if result is None else int(result.nfev),
        "njev": 0 if result is None else int(result.njev or 0),
        "initial_cost": initial_cost,
        "final_cost": initial_cost if fallback or result is None else float(result.cost),
        "initial": metrics(initial_positions, initial_matrices),
        "final": final_metrics,
        "max_position_error_m": max_position,
        "max_rotation_error_rad": max_rotation,
        "elapsed_s": perf_counter() - started,
        "optimized_joints": [spec.names[index] for index in optimized],
    }
    return output, diagnostics
