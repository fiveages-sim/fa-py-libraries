"""O6/L6 nest route: fingertip→四指 PIP 中心 + RViz pinch_q 锚点.

j2 from thumb−nest palm-plane *lateral* (radial / palm-width), not the 7°
metacarpal-az knots. j1 from nest radius; thumb curl is a small add-on.
Modes only gate pinch attract / j2 slot blend. O7 is not this curve.
See docs/O6_NEST.md.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from xr_hand_retarget.sources import openxr_joints as J
from xr_hand_retarget.algorithms.curl_xhand1 import _interp_knots, _unit_t

from xr_hand_retarget.algorithms.palm_tip import (
    FINGERS,
    O6Workspace,
    PalmTipCalib,
    PalmTipFeat,
    _THUMB_J1,
    _THUMB_J2,
    _fist_blend_w,
    _j1_open_curl,
    _j2_lat_knots,
    _lead_w,
    _xyz,
    apply_fist_envelope,
    apply_j2_lock,
    attract_pinch,
    extract_palm_tip,
    map_palm_tip,
    nest_radius_lat,
)

_NEST_PIPS = (
    J.INDEX_INTERMEDIATE,
    J.MIDDLE_INTERMEDIATE,
    J.RING_INTERMEDIATE,
    J.LITTLE_INTERMEDIATE,
)


@dataclass
class NestDecision:
    mode: str  # along | fist | pinch
    slot: str  # index | middle | none
    fist_w: float
    w_pinch: float
    w_index: float
    w_ulnar: float
    nest_r: float
    nest_lat: float = 0.0


@dataclass
class NestGeom:
    """Thumb tip relative to the four-finger PIP nest, palm frame."""

    r: float  # ||tip − nest|| / palm_width
    lat: float  # in-plane radial offset / palm_width
    r_p: float = 0.0  # ||in-plane (tip − nest)|| / palm_width


def human_nest_xyz(joints26: np.ndarray) -> np.ndarray:
    """Four-finger reachable center on the person: mean of PIP joints."""
    pts = np.stack([_xyz(joints26, i) for i in _NEST_PIPS], axis=0)
    return pts.mean(axis=0)


def extract_nest_geom(joints26: np.ndarray, side: str) -> NestGeom:
    r, lat, r_p = nest_radius_lat(joints26, side)
    return NestGeom(r=r, lat=lat, r_p=r_p)


def nest_radius_width(joints26: np.ndarray, side: str) -> float:
    """||thumb_tip − nest|| / palm_width. Along is large; in-palm / pinch is small."""
    return extract_nest_geom(joints26, side).r


def classify_nest(
    feat: PalmTipFeat,
    calib: PalmTipCalib,
    ws: O6Workspace,
    *,
    nest_r: float = 1.0,
    nest_lat: float = 0.0,
) -> NestDecision:
    """Split 掌侧 / 掌内 / 对掌. Fist gate is four-finger curl, not thumb_az."""
    fist_w = _fist_blend_w(feat, calib, ws)
    d = feat.pinch_d or {}
    near = float(ws.slot_near)
    far = float(ws.slot_far)
    w_d = {n: _lead_w(d.get(n, 1e9), near, far) for n in FINGERS}
    w_index = float(w_d["index"])
    w_ulnar = float(max(w_d["middle"], w_d["ring"], w_d["pinky"]))
    w_pinch = max(w_index, w_ulnar)
    isolated = w_pinch >= 0.28 and fist_w < 0.70
    if fist_w >= 0.55 and not isolated:
        return NestDecision(
            "fist", "none", fist_w, w_pinch, w_index, w_ulnar, nest_r, nest_lat
        )
    if w_pinch >= 0.20:
        slot = "index" if w_index >= w_ulnar else "middle"
        return NestDecision(
            "pinch", slot, fist_w, w_pinch, w_index, w_ulnar, nest_r, nest_lat
        )
    return NestDecision(
        "along", "none", fist_w, w_pinch, w_index, w_ulnar, nest_r, nest_lat
    )


def nest_j2_from_lat(
    lat: float, ws: O6Workspace, calib: PalmTipCalib | None = None
) -> float:
    """Radial offset → j2. Prefer captured along/index/middle nest_lat knots."""
    hi = float(ws.j2_middle)
    knots = _j2_lat_knots(calib, ws) if calib is not None else None
    if knots is not None:
        xs, qs = knots
        return float(np.clip(_interp_knots(float(lat), xs, qs), 0.0, hi))
    return float(
        np.clip(
            _interp_knots(
                float(lat),
                [float(ws.nest_lat_along), float(ws.nest_lat_opp)],
                [0.0, hi],
            ),
            0.0,
            hi,
        )
    )


def nest_j2_rad(
    geom: NestGeom,
    feat: PalmTipFeat,
    calib: PalmTipCalib,
    ws: O6Workspace,
) -> float:
    """j2 from nest lat, pulled to 0 when the thumb is extended and far (掌侧 / good).

    Fingers in a fist must not keep j2 at opposition: good is 四指握 + 拇指贴掌外侧.
    Over-nest (small r) still uses lat so an open-palm sweep can reach j2_middle.
    """
    j2 = nest_j2_from_lat(geom.lat, ws, calib)
    curl_t = _unit_t(
        float(feat.thumb_curl),
        _j1_open_curl(calib),
        float(calib.flex_thumb_curl),
    )
    r_near = float(ws.nest_r_near)
    r_far = float(ws.nest_r_far)
    r_side = 0.5 * (r_near + r_far)
    w_far = _unit_t(float(geom.r), r_side, r_far)
    w_up = 0.0
    if float(geom.r_p) < 0.18 and float(geom.r) > r_near + 0.15:
        w_up = 1.0
    w_side = (1.0 - curl_t) * max(w_far, w_up)
    j2 = (1.0 - w_side) * j2
    return float(np.clip(j2, 0.0, float(ws.j2_middle)))


def nest_j1_from_r(
    r: float,
    feat: PalmTipFeat,
    calib: PalmTipCalib,
    ws: O6Workspace,
    j1_hi: float,
) -> float:
    """Depth from nest radius. Far → 0; near → close_j1. Curl adds curl_span."""
    j1 = float(
        _interp_knots(
            float(r),
            [float(ws.nest_r_near), float(ws.nest_r_far)],
            [float(ws.close_j1), 0.0],
        )
    )
    curl_t = _unit_t(
        float(feat.thumb_curl),
        _j1_open_curl(calib),
        float(calib.flex_thumb_curl),
    )
    j1 = j1 + curl_t * float(ws.nest_curl_span)
    return float(np.clip(j1, 0.0, float(j1_hi)))


def apply_nest_thumb(
    q: np.ndarray,
    feat: PalmTipFeat,
    calib: PalmTipCalib,
    ws: O6Workspace,
    decision: NestDecision,
    j1_hi: float,
    *,
    j2_lat: float,
    j1_r: float,
) -> np.ndarray:
    """Set thumb from nest lateral + radius. Modes do not park j2."""
    out = np.asarray(q, dtype=np.float64).copy()
    j2_index = float(ws.j2_index)
    j2_middle = float(ws.j2_middle)
    close_j1 = float(ws.close_j1)
    j2 = float(np.clip(j2_lat, 0.0, j2_middle))
    j1 = float(np.clip(j1_r, 0.0, float(j1_hi)))

    if decision.mode == "pinch":
        den = decision.w_index + decision.w_ulnar
        if den <= 1e-9:
            j2_star = j2_index if decision.slot == "index" else j2_middle
        else:
            j2_star = (decision.w_index * j2_index + decision.w_ulnar * j2_middle) / den
        w = float(np.clip(decision.w_pinch, 0.0, 1.0))
        j2 = (1.0 - w) * j2 + w * float(j2_star)
        j1 = max(j1, w * close_j1)

    out[_THUMB_J2] = float(np.clip(j2, 0.0, j2_middle))
    out[_THUMB_J1] = float(np.clip(j1, 0.0, float(j1_hi)))
    return out


def map_nest(
    feat: PalmTipFeat,
    calib: PalmTipCalib,
    limits: np.ndarray,
    ws: O6Workspace,
    joints26: np.ndarray,
    side: str,
) -> tuple[np.ndarray, NestDecision]:
    geom = extract_nest_geom(joints26, side)
    decision = classify_nest(
        feat, calib, ws, nest_r=geom.r, nest_lat=geom.lat
    )
    q = map_palm_tip(feat, calib, limits, ws)
    j1_hi = float(limits[_THUMB_J1, 1])
    if getattr(ws, "fist_j1", None) is not None:
        j1_hi = float(ws.fist_j1)
    j2_lat = nest_j2_rad(geom, feat, calib, ws)
    j1_r = nest_j1_from_r(geom.r, feat, calib, ws, j1_hi)
    q = apply_nest_thumb(
        q, feat, calib, ws, decision, j1_hi, j2_lat=j2_lat, j1_r=j1_r
    )
    return np.clip(q, limits[:, 0], limits[:, 1]), decision


def retarget_nest(
    joints26: np.ndarray,
    side: str,
    calib: PalmTipCalib,
    limits: np.ndarray,
    ws: O6Workspace,
    *,
    j2_lock: bool = False,
) -> tuple[np.ndarray, NestDecision]:
    feat = extract_palm_tip(joints26, side)
    q, decision = map_nest(feat, calib, limits, ws, joints26, side)
    q = apply_j2_lock(q, ws, j2_lock)
    if decision.mode == "pinch" and bool(getattr(ws, "attract_enabled", True)):
        q = attract_pinch(q, feat, ws, calib)
    q = apply_fist_envelope(q, feat, calib, ws, float(limits[_THUMB_J1, 1]))
    return np.clip(q, limits[:, 0], limits[:, 1]), decision
