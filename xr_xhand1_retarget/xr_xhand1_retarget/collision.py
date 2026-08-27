"""URDF-informed soft self-collision guards (no mesh / FCL dependency).

True mesh collision needs Pinocchio+HppFCL / Bullet against xhand1 collision
geometries. Here we encode the kinematic facts that matter for teleop:

- Joint limits already come from xhand1.xacro.
- Thumb CMC (joint1) low + strong thumb/index curl drives the thumb into the
  index/palm corridor on this hand.
- Adjacent finger tips crowd when all MCP/PIP are near upper limits.

Projection is soft: nudge opposition up / ease flexions, then re-clamp.
"""

from __future__ import annotations

import numpy as np

# q layout matches XHAND1_JOINT_NAMES
_T1, _T2, _T3 = 0, 1, 2
_I1, _I2, _I3 = 3, 4, 5
_M1, _M2 = 6, 7
_R1, _R2 = 8, 9
_P1, _P2 = 10, 11


def _clamp(q: np.ndarray, limits: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64).reshape(-1)
    return np.clip(q, limits[:, 0], limits[:, 1])


def soft_collision_project(
    q: np.ndarray,
    limits: np.ndarray,
    *,
    enabled: bool = True,
    thumb_min_opposition_when_curled: float = 0.45,
    thumb_curl_trigger: float = 1.0,
    index_curl_trigger: float = 1.4,
    finger_crowd_trigger: float = 3.2,
    ease: float = 0.9,
) -> np.ndarray:
    """Project q away from known XHand1 self-collision envelopes."""
    q = _clamp(np.asarray(q, dtype=np.float64), limits)
    if not enabled:
        return q

    out = q.copy()
    thumb_curl = float(out[_T2] + max(0.0, out[_T3]))
    index_curl = float(out[_I2] + out[_I3])

    # Thumb stabbing into index/palm: low CMC bend + high curls.
    if out[_T1] < thumb_min_opposition_when_curled and thumb_curl >= thumb_curl_trigger:
        if index_curl >= index_curl_trigger or thumb_curl >= thumb_curl_trigger + 0.4:
            out[_T1] = min(
                limits[_T1, 1],
                max(out[_T1], thumb_min_opposition_when_curled),
            )
            out[_I2] *= ease
            out[_I3] *= ease
            out[_T2] = limits[_T2, 0] + (out[_T2] - limits[_T2, 0]) * ease
            out[_T3] *= ease

    # Fist crowding: all non-thumb fingers near max → ease outer fingers first.
    finger_sum = float(
        out[_I2]
        + out[_I3]
        + out[_M1]
        + out[_M2]
        + out[_R1]
        + out[_R2]
        + out[_P1]
        + out[_P2]
    )
    if finger_sum >= finger_crowd_trigger:
        out[_P1] *= ease
        out[_P2] *= ease
        out[_R1] *= ease
        out[_R2] *= ease
        out[_I1] *= ease

    return _clamp(out, limits)
