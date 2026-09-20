"""OpenXR 26 → MediaPipe 21 (xyz only). No palm-frame preprocess.

Official wuji-retargeting applies its own wrist-frame transform. Do not run
SomeHand ``preprocess_landmarks`` before ``Retargeter.retarget``.
"""

from __future__ import annotations

import numpy as np

from xr_hand_retarget.sources.landmarks import OPENXR_TO_MEDIAPIPE, openxr26_to_mediapipe21
from xr_hand_retarget.algorithms.safety_xhand1 import pose_is_zero

__all__ = [
    "OPENXR_TO_MEDIAPIPE",
    "openxr26_to_mediapipe21",
    "pose_is_zero",
    "wrist_origin_mp21",
    "palm_triangle_area_m2",
]


def wrist_origin_mp21(landmarks21: np.ndarray) -> np.ndarray:
    """Subtract wrist (index 0). Official appendix: wrist is the coordinate origin."""
    lm = np.asarray(landmarks21, dtype=np.float64)
    if lm.shape != (21, 3):
        raise ValueError(f"expected (21, 3), got {lm.shape}")
    return lm - lm[0:1, :]


def palm_triangle_area_m2(landmarks21: np.ndarray) -> float:
    """Area of wrist / index-MCP / middle-MCP (MediaPipe 0, 5, 9), m².

    Official ``estimate_frame_from_hand_points`` uses the same three points.
    A near-zero area means the palm frame SVD will flip.
    """
    lm = np.asarray(landmarks21, dtype=np.float64)
    if lm.shape[0] < 10:
        return 0.0
    a = lm[5, :3] - lm[0, :3]
    b = lm[9, :3] - lm[0, :3]
    return 0.5 * float(np.linalg.norm(np.cross(a, b)))
