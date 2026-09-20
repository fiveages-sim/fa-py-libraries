"""URDF-informed soft self-collision guards (no mesh / FCL dependency).

Only the thumb joints (q[0:3]) are modified. Index/middle curl are read as
triggers for the palm-corridor rule; four-finger commands are never written.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# q layout matches XHAND1_JOINT_NAMES
_T1, _T2, _T3 = 0, 1, 2
_I2, _I3 = 4, 5
_M1, _M2 = 6, 7


@dataclass
class CollisionGains:
    """Thumb-only soft collision (XHand1)."""

    thumb_index_guard: bool = True
    thumb_min_opposition_when_curled: float = 0.38
    thumb_curl_trigger: float = 0.85
    index_curl_trigger: float = 1.15
    middle_curl_trigger: float = 1.0
    ease: float = 0.88


def _clamp(q: np.ndarray, limits: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64).reshape(-1)
    return np.clip(q, limits[:, 0], limits[:, 1])


def soft_collision_project(
    q: np.ndarray,
    limits: np.ndarray,
    *,
    enabled: bool = True,
    gains: CollisionGains | None = None,
    **legacy_kwargs,
) -> np.ndarray:
    """Project thumb joints away from the index/palm corridor. Four fingers unchanged."""
    q = _clamp(np.asarray(q, dtype=np.float64), limits)
    if not enabled:
        return q

    g = gains or CollisionGains()
    if legacy_kwargs:
        # Ignore obsolete fist_crowd_* keys; accept renamed thumb fields.
        for key, val in legacy_kwargs.items():
            if hasattr(g, key):
                setattr(g, key, val)

    if not g.thumb_index_guard:
        return q

    out = q.copy()
    thumb_curl = float(out[_T2] + max(0.0, out[_T3]))
    index_curl = float(out[_I2] + out[_I3])  # read-only trigger
    middle_curl = float(out[_M1] + out[_M2])  # read-only trigger

    oppose_lo = float(g.thumb_min_opposition_when_curled)
    if out[_T1] < oppose_lo and thumb_curl >= g.thumb_curl_trigger:
        index_near = index_curl >= g.index_curl_trigger
        middle_near = middle_curl >= g.middle_curl_trigger
        thumb_deep = thumb_curl >= g.thumb_curl_trigger + 0.35
        if index_near or middle_near or thumb_deep:
            out[_T1] = min(limits[_T1, 1], max(out[_T1], oppose_lo))
            out[_T2] = limits[_T2, 0] + (out[_T2] - limits[_T2, 0]) * g.ease
            out[_T3] *= g.ease

    if out[_T1] < 0.28 and index_curl >= 0.9:
        out[_T1] = min(limits[_T1, 1], max(out[_T1], 0.32))

    return _clamp(out, limits)
