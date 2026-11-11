"""Small, dependency-free rigid-transform helpers."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def normalize_quaternion(q: Iterable[float]) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError("quaternion must be a finite four-vector")
    norm = float(np.linalg.norm(q))
    if norm < 1e-8:
        raise ValueError("zero quaternion")
    return q / norm


def quaternion_to_matrix(q_xyzw: Iterable[float]) -> np.ndarray:
    x, y, z, w = normalize_quaternion(q_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    m = np.asarray(matrix, dtype=np.float64)
    if m.shape != (3, 3) or not np.all(np.isfinite(m)):
        raise ValueError("rotation matrix must be finite and 3x3")
    u, _, vt = np.linalg.svd(m)
    m = u @ vt
    if np.linalg.det(m) < 0:
        u[:, -1] *= -1
        m = u @ vt
    trace = float(np.trace(m))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = np.array([(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
                      (m[1, 0] - m[0, 1]) / s, 0.25 * s])
    else:
        i = int(np.argmax(np.diag(m)))
        if i == 0:
            s = math.sqrt(max(0.0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) * 2
            q = np.array([0.25 * s, (m[0, 1] + m[1, 0]) / s,
                          (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s])
        elif i == 1:
            s = math.sqrt(max(0.0, 1 + m[1, 1] - m[0, 0] - m[2, 2])) * 2
            q = np.array([(m[0, 1] + m[1, 0]) / s, 0.25 * s,
                          (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s])
        else:
            s = math.sqrt(max(0.0, 1 + m[2, 2] - m[0, 0] - m[1, 1])) * 2
            q = np.array([(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s,
                          0.25 * s, (m[1, 0] - m[0, 1]) / s])
    return normalize_quaternion(q)


def quaternion_multiply(left_xyzw: Iterable[float], right_xyzw: Iterable[float]) -> np.ndarray:
    x1, y1, z1, w1 = normalize_quaternion(left_xyzw)
    x2, y2, z2, w2 = normalize_quaternion(right_xyzw)
    return normalize_quaternion([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def quaternion_inverse(q_xyzw: Iterable[float]) -> np.ndarray:
    x, y, z, w = normalize_quaternion(q_xyzw)
    return np.array([-x, -y, -z, w], dtype=np.float64)


def nlerp(q0: Iterable[float], q1: Iterable[float], alpha: float) -> np.ndarray:
    a = normalize_quaternion(q0)
    b = normalize_quaternion(q1)
    if float(np.dot(a, b)) < 0:
        b = -b
    return normalize_quaternion((1.0 - alpha) * a + alpha * b)


def yaw_quaternion(q_xyzw: Iterable[float]) -> np.ndarray:
    r = quaternion_to_matrix(q_xyzw)
    forward = r[:, 0].copy()
    forward[2] = 0.0
    if np.linalg.norm(forward) < 1e-8:
        return np.array([0.0, 0.0, 0.0, 1.0])
    yaw = math.atan2(float(forward[1]), float(forward[0]))
    return np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])


def quaternion_angle(q0: Iterable[float], q1: Iterable[float]) -> float:
    dot = abs(float(np.dot(normalize_quaternion(q0), normalize_quaternion(q1))))
    return 2.0 * math.acos(float(np.clip(dot, -1.0, 1.0)))


def rotation_vector(q_xyzw: Iterable[float]) -> np.ndarray:
    q = normalize_quaternion(q_xyzw)
    if q[3] < 0:
        q = -q
    sin_half = float(np.linalg.norm(q[:3]))
    if sin_half < 1e-8:
        return np.zeros(3)
    angle = 2 * math.atan2(sin_half, float(q[3]))
    return q[:3] * (angle / sin_half)


def roll_pitch(q_xyzw: Iterable[float]) -> tuple[float, float]:
    x, y, z, w = normalize_quaternion(q_xyzw)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(float(np.clip(2 * (w * y - z * x), -1.0, 1.0)))
    return roll, pitch
