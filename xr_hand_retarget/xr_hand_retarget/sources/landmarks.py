"""OpenXR 26 → MediaPipe-style 21 landmarks + wrist/palm preprocess (SomeHand-style)."""

from __future__ import annotations

import numpy as np

from . import openxr_joints as J

# OpenXR 26 indices → MediaPipe 21 (same as somehand pico_input).
# Skips Palm(0) and non-thumb metacarpals (6,11,16,21).
OPENXR_TO_MEDIAPIPE: list[int] = [
    J.WRIST,  # 0
    J.THUMB_METACARPAL,  # 1
    J.THUMB_PROXIMAL,  # 2
    J.THUMB_DISTAL,  # 3
    J.THUMB_TIP,  # 4
    J.INDEX_PROXIMAL,  # 5
    J.INDEX_INTERMEDIATE,  # 6
    J.INDEX_DISTAL,  # 7
    J.INDEX_TIP,  # 8
    J.MIDDLE_PROXIMAL,  # 9
    J.MIDDLE_INTERMEDIATE,  # 10
    J.MIDDLE_DISTAL,  # 11
    J.MIDDLE_TIP,  # 12
    J.RING_PROXIMAL,  # 13
    J.RING_INTERMEDIATE,  # 14
    J.RING_DISTAL,  # 15
    J.RING_TIP,  # 16
    J.LITTLE_PROXIMAL,  # 17
    J.LITTLE_INTERMEDIATE,  # 18
    J.LITTLE_DISTAL,  # 19
    J.LITTLE_TIP,  # 20
]

_OPERATOR2ROBOT_RIGHT = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)
_LEFT_RIGHT_ROBOT_MIRROR = np.diag([1.0, -1.0, 1.0]).astype(np.float64)
_OPERATOR2ROBOT_LEFT = _OPERATOR2ROBOT_RIGHT @ _LEFT_RIGHT_ROBOT_MIRROR


def openxr26_to_mediapipe21(joints26: np.ndarray) -> np.ndarray:
    """Convert OpenXR (26, ≥3) xyz to MediaPipe-style (21, 3)."""
    joints26 = np.asarray(joints26, dtype=np.float64)
    if joints26.ndim != 2 or joints26.shape[0] != J.NUM_JOINTS:
        raise ValueError(f"expected (26, ≥3), got {joints26.shape}")
    out = np.empty((21, 3), dtype=np.float64)
    for mp_i, ox_i in enumerate(OPENXR_TO_MEDIAPIPE):
        out[mp_i] = joints26[ox_i, :3]
    return out


def _estimate_wrist_frame(landmarks_3d: np.ndarray, *, hand_side: str) -> np.ndarray:
    """Palm frame from wrist(0), index_mcp(5), middle_mcp(9). Columns = axes."""
    points = landmarks_3d[[0, 5, 9], :]
    x_vector = points[0] - points[2]

    centered = points - np.mean(points, axis=0, keepdims=True)
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    n = float(np.linalg.norm(normal))
    if n < 1e-8:
        raise ValueError("degenerate palm normal")
    normal = normal / n

    x_axis = x_vector - np.dot(x_vector, normal) * normal
    xn = float(np.linalg.norm(x_axis))
    if xn < 1e-8:
        raise ValueError("degenerate palm x-axis")
    x_axis = x_axis / xn

    if hand_side == "left":
        z_axis = np.cross(normal, x_axis)
    else:
        z_axis = np.cross(x_axis, normal)
    zn = float(np.linalg.norm(z_axis))
    if zn < 1e-8:
        raise ValueError("degenerate palm z-axis")
    z_axis = z_axis / zn

    if float(np.dot(z_axis, points[1] - points[2])) < 0.0:
        normal = -normal
        z_axis = -z_axis

    return np.stack([x_axis, normal, z_axis], axis=1)


def _mediapipe_to_mujoco_fallback(landmarks_3d: np.ndarray) -> np.ndarray:
    out = np.empty_like(landmarks_3d)
    out[:, 0] = -landmarks_3d[:, 2]
    out[:, 1] = landmarks_3d[:, 0]
    out[:, 2] = -landmarks_3d[:, 1]
    return out


def preprocess_landmarks(landmarks_3d: np.ndarray, hand_side: str = "right") -> np.ndarray:
    """Wrist-origin + palm frame + operator→robot (SomeHand preprocess)."""
    side = "left" if str(hand_side).lower().startswith("l") else "right"
    lm = np.asarray(landmarks_3d, dtype=np.float64)
    centered = lm - lm[0:1, :]
    operator_to_robot = _OPERATOR2ROBOT_LEFT if side == "left" else _OPERATOR2ROBOT_RIGHT
    try:
        wrist_frame = _estimate_wrist_frame(centered, hand_side=side)
        return centered @ wrist_frame @ operator_to_robot
    except ValueError:
        fallback = _mediapipe_to_mujoco_fallback(centered)
        if side == "left":
            return fallback @ _LEFT_RIGHT_ROBOT_MIRROR
        return fallback


class TemporalFilter:
    """EMA filter: filtered = alpha * current + (1-alpha) * previous."""

    def __init__(self, alpha: float = 0.65):
        self.alpha = float(np.clip(alpha, 1e-3, 1.0))
        self._prev: np.ndarray | None = None

    def reset(self) -> None:
        self._prev = None

    def filter(self, x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=np.float64)
        if self._prev is None or self.alpha >= 1.0 - 1e-12:
            self._prev = x.copy()
            return x.copy()
        out = self.alpha * x + (1.0 - self.alpha) * self._prev
        self._prev = out.copy()
        return out
