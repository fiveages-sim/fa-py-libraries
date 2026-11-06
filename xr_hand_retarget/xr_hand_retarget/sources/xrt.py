"""XRoboToolkit SDK → HandFrame (vendor-specific source)."""

from __future__ import annotations

import time

import numpy as np

from xr_hand_retarget.sources.frame import HandFrame
from xr_hand_retarget.sources.landmarks import openxr26_to_mediapipe21


def _as_active(flag) -> int:
    try:
        v = np.asarray(flag).reshape(-1)
        if v.size == 0:
            return 0
        return 1 if float(v[0]) > 0.5 else 0
    except Exception:
        return 0


def _normalize_hand26(raw) -> np.ndarray:
    arr = np.asarray(raw, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[0] != 26:
        arr = arr.reshape(26, -1)
    return arr


def read_hand_frame(xrt, side: str, *, timestamp: float | None = None) -> HandFrame:
    """Pull one OpenXR hand sample from an already-open SDK client."""
    if side not in ("left", "right"):
        raise ValueError(f"side must be left|right, got {side!r}")
    get_active = (
        xrt.get_left_hand_is_active if side == "left" else xrt.get_right_hand_is_active
    )
    get_hand = (
        xrt.get_left_hand_tracking_state
        if side == "left"
        else xrt.get_right_hand_tracking_state
    )
    joints26 = _normalize_hand26(get_hand())
    active = _as_active(get_active())
    xyz = openxr26_to_mediapipe21(joints26)
    return HandFrame(
        side=side,
        active=active,
        joints26=joints26,
        xyz=xyz,
        timestamp=time.time() if timestamp is None else timestamp,
    )


def controller_button_pressed(xrt, side: str, button: str) -> bool:
    """Read A/B (right) or X/Y (left) for optional Linker j2 lock."""
    btn = (button or "").strip().lower()
    if btn in ("", "off", "none", "false", "0"):
        return False
    if btn == "a":
        attr = "get_X_button" if side == "left" else "get_A_button"
    elif btn == "b":
        attr = "get_Y_button" if side == "left" else "get_B_button"
    else:
        return False
    fn = getattr(xrt, attr, None)
    if not callable(fn):
        return False
    try:
        return bool(fn())
    except Exception:
        return False
