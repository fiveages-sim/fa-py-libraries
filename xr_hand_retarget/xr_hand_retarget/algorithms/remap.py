"""Hybrid OpenXR 26 → XHand1 12-DOF remap.

- Index + middle / ring / pinky: bone-chain flexion (URDF U) + index abduction on j1.
- Thumb: tip palm-frame opposition (j1) + bone-chain flex at tip (j2/j3).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from xr_hand_retarget.sources import openxr_joints as J

XHAND1_JOINT_NAMES = [
    "thumb_joint1",
    "thumb_joint2",
    "thumb_joint3",
    "index_joint1",
    "index_joint2",
    "index_joint3",
    "middle_joint1",
    "middle_joint2",
    "ring_joint1",
    "ring_joint2",
    "pinky_joint1",
    "pinky_joint2",
]

XHAND1_LIMITS = np.array(
    [
        [0.0, 1.832],
        [-0.698, 1.57],
        [0.0, 1.57],
        [-0.174, 0.174],
        [0.0, 1.919],
        [0.0, 1.919],
        [0.0, 1.919],
        [0.0, 1.919],
        [0.0, 1.919],
        [0.0, 1.919],
        [0.0, 1.919],
        [0.0, 1.919],
    ],
    dtype=np.float64,
)


@dataclass
class FingerChainGains:
    """Bone-chain flexion → full URDF range (open→lower, fist→upper)."""

    mcp_fist_ref_rad: float = 1.15
    pip_fist_ref_rad: float = 1.05
    scale: float = 1.0


@dataclass
class IndexAbductionGains:
    """Index joint1 from INDEX_PROXIMAL − INDEX_METACARPAL in palm plane.

    Palm lateral axis must NOT use INDEX_PROXIMAL (that made j1 insensitive:
    abducting the index also rotated the palm x-axis).
    """

    abduction_sign: float = -1.0
    # |bone·x| at palm plane (m) mapped to ±0.174 rad abduction.
    lateral_span_m: float = 0.028
    abduction_curl_fade: float = 0.40
    # Subtract open-hand resting lateral (m) so neutral pose → j1≈0.
    lat_rest_m: float = 0.0


@dataclass
class ThumbTipGains:
    """Thumb j1: lateral anchor (default); j2/j3: bone flex at prox / IP."""

    j1_mode: str = "lateral_anchor"
    # lateral_anchor: open palm → j1≈lo; tip x aligns little proximal → j1≈hi
    lateral_open_margin_m: float = 0.022
    lateral_y_span_m: float = 0.038
    lateral_open_deadband: float = 0.10
    lateral_open_x_m: float | None = None
    # Legacy phi mode (meta→tip bearing)
    rest_phi_rad: float = 0.28
    opposition_scale: float = 1.30
    opposition_offset: float = 0.02
    invert_cmc: bool = True
    cmc_span: float = 1.832
    mcp_fist_ref_rad: float = 1.10
    ip_fist_ref_rad: float = 1.05
    mcp_bias: float = -0.08
    flex_scale: float = 1.0


def _pos(joints26: np.ndarray, idx: int) -> np.ndarray:
    return np.asarray(joints26[idx, :3], dtype=np.float64)


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return np.zeros(3, dtype=np.float64)
    return v / n


def _angle(v1: np.ndarray, v2: np.ndarray) -> float:
    return float(
        np.arccos(np.clip(float(np.dot(_unit(v1), _unit(v2))), -1.0, 1.0))
    )


def _flexion(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    return max(0.0, np.pi - _angle(a - b, c - b))


def _pip_dip(prox, inter, dist, tip) -> float:
    return 0.65 * _flexion(prox, inter, dist) + 0.35 * _flexion(inter, dist, tip)


def _palm_axes(
    joints26: np.ndarray, side: str
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    wrist = _pos(joints26, J.WRIST)
    middle = _pos(joints26, J.MIDDLE_PROXIMAL) - wrist
    index = _pos(joints26, J.INDEX_PROXIMAL) - wrist
    pinky = _pos(joints26, J.LITTLE_PROXIMAL) - wrist

    y = _unit(middle)
    x = _unit(index - pinky)
    z = _unit(np.cross(x, y))
    if side.lower().startswith("l"):
        z = -z
        x = _unit(np.cross(y, z))
    else:
        x = _unit(np.cross(y, z))
        z = _unit(np.cross(x, y))
    return wrist, x, y, z


def _palm_axes_for_index_abd(
    joints26: np.ndarray, side: str
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Palm frame for index abduction: lateral = middle–pinky (no index)."""
    wrist = _pos(joints26, J.WRIST)
    middle = _pos(joints26, J.MIDDLE_PROXIMAL) - wrist
    pinky = _pos(joints26, J.LITTLE_PROXIMAL) - wrist

    y = _unit(middle)
    x = _unit(middle - pinky)
    z = _unit(np.cross(x, y))
    if side.lower().startswith("l"):
        z = -z
        x = _unit(np.cross(y, z))
    else:
        x = _unit(np.cross(y, z))
        z = _unit(np.cross(x, y))
    return x, y, z


def _project_palm(
    v: np.ndarray, x: np.ndarray, y: np.ndarray, z: np.ndarray
) -> tuple[float, float]:
    return float(np.dot(v, x)), float(np.dot(v, y))


def _palm_point(
    p: np.ndarray,
    wrist: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
) -> tuple[float, float]:
    rel = p - wrist
    rel_p = rel - z * float(np.dot(rel, z))
    return _project_palm(rel_p, x, y, z)


_FOUR_PROXIMAL = (
    J.INDEX_PROXIMAL,
    J.MIDDLE_PROXIMAL,
    J.RING_PROXIMAL,
    J.LITTLE_PROXIMAL,
)


def _map_thumb_j1_lateral(
    joints26: np.ndarray,
    *,
    wrist: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    lo1: float,
    hi1: float,
    gains: ThumbTipGains,
) -> float:
    """j1 from thumb tip lateral vs four-finger proximal row (open→lo, oppose→hi)."""
    tip = _pos(joints26, J.THUMB_TIP)
    meta = _pos(joints26, J.THUMB_METACARPAL)
    x_tip, y_tip = _palm_point(tip, wrist, x, y, z)
    x_meta, _ = _palm_point(meta, wrist, x, y, z)

    prox_x: list[float] = []
    prox_y: list[float] = []
    for idx in _FOUR_PROXIMAL:
        xi, yi = _palm_point(_pos(joints26, idx), wrist, x, y, z)
        prox_x.append(xi)
        prox_y.append(yi)

    x_little = prox_x[3]
    x_index = prox_x[0]
    y_row = float(np.mean(prox_y))

    if gains.lateral_open_x_m is not None:
        x_open = float(gains.lateral_open_x_m)
    else:
        # Open-hand anchor: thumb meta + margin (radial); stable when palm spread.
        x_open = max(x_meta + gains.lateral_open_margin_m, x_index)

    x_span = x_open - x_little
    if abs(x_span) < 1e-4:
        x_span = 1e-4

    # Radial (open) → 0; ulnar (at little proximal x) → 1
    s_x = (x_open - x_tip) / x_span
    s_x = float(np.clip(s_x, 0.0, 1.0))

    if s_x <= gains.lateral_open_deadband:
        s_x = 0.0

    fade_y = 1.0
    if gains.lateral_y_span_m > 0.0:
        d_y = y_tip - y_row
        fade_y = float(np.clip(1.0 - abs(d_y) / gains.lateral_y_span_m, 0.0, 1.0))

    j1 = lo1 + s_x * fade_y * (hi1 - lo1)
    return float(np.clip(j1, lo1, hi1))


def _map_thumb_j1_phi(
    joints26: np.ndarray,
    *,
    x: np.ndarray,
    y: np.ndarray,
    lo1: float,
    hi1: float,
    gains: ThumbTipGains,
) -> float:
    meta = _pos(joints26, J.THUMB_METACARPAL)
    tip = _pos(joints26, J.THUMB_TIP)
    bearing = tip - meta
    bu = float(np.dot(bearing, x))
    bv = float(np.dot(bearing, y))
    phi = float(np.arctan2(bv, bu))
    raw = (phi - gains.rest_phi_rad) * gains.opposition_scale + gains.opposition_offset
    if gains.invert_cmc:
        j1 = gains.cmc_span - raw
    else:
        j1 = raw
    return float(np.clip(j1, lo1, hi1))


def _clamp_q(q: np.ndarray, limits: np.ndarray | None = None) -> np.ndarray:
    lim = XHAND1_LIMITS if limits is None else np.asarray(limits, dtype=np.float64)
    return np.clip(q, lim[:, 0], lim[:, 1])


def _map_to_range(norm: float, lo: float, hi: float) -> float:
    n = float(np.clip(norm, 0.0, 1.0))
    return lo + n * (hi - lo)


def _map_finger_chain(
    joints26: np.ndarray,
    meta_i: int,
    prox_i: int,
    inter_i: int,
    dist_i: int,
    tip_i: int,
    lo1: float,
    hi1: float,
    lo2: float,
    hi2: float,
    gains: FingerChainGains,
) -> tuple[float, float]:
    meta = _pos(joints26, meta_i)
    prox = _pos(joints26, prox_i)
    inter = _pos(joints26, inter_i)
    dist = _pos(joints26, dist_i)
    tip = _pos(joints26, tip_i)

    mcp = _flexion(meta, prox, inter)
    pip = _pip_dip(prox, inter, dist, tip)
    mcp_n = float(np.clip(mcp * gains.scale / max(gains.mcp_fist_ref_rad, 1e-6), 0.0, 1.0))
    pip_n = float(np.clip(pip * gains.scale / max(gains.pip_fist_ref_rad, 1e-6), 0.0, 1.0))
    return _map_to_range(mcp_n, lo1, hi1), _map_to_range(pip_n, lo2, hi2)


def _map_index_abduction(
    joints26: np.ndarray,
    *,
    side: str,
    mcp_norm: float,
    lo1: float,
    hi1: float,
    gains: IndexAbductionGains,
) -> float:
    """index_joint1: palm-plane lateral of INDEX_PROXIMAL − INDEX_METACARPAL.

    Uses middle–pinky palm axes so abducting the index does not rotate the
    measurement frame (old index–pinky x made j1 nearly constant).
    """
    x, _y, z = _palm_axes_for_index_abd(joints26, side)
    meta = _pos(joints26, J.INDEX_METACARPAL)
    prox = _pos(joints26, J.INDEX_PROXIMAL)
    bone = prox - meta
    bone_p = bone - z * float(np.dot(bone, z))
    lat = float(np.dot(bone_p, x)) - float(gains.lat_rest_m)
    j1_raw = gains.abduction_sign * lat / max(gains.lateral_span_m, 1e-6) * 0.174
    fade = 1.0
    if gains.abduction_curl_fade > 0.0:
        fade = float(np.clip(1.0 - mcp_norm / gains.abduction_curl_fade, 0.0, 1.0))
    return float(np.clip(j1_raw * fade, lo1, hi1))


def _map_index(
    joints26: np.ndarray,
    *,
    wrist: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    side: str,
    lo: np.ndarray,
    hi: np.ndarray,
    chain: FingerChainGains,
    abd: IndexAbductionGains,
) -> tuple[float, float, float]:
    """Index: same bone chain as other fingers + separate abduction on j1."""
    meta = _pos(joints26, J.INDEX_METACARPAL)
    prox = _pos(joints26, J.INDEX_PROXIMAL)
    inter = _pos(joints26, J.INDEX_INTERMEDIATE)
    mcp = _flexion(meta, prox, inter)
    mcp_n = float(np.clip(mcp * chain.scale / max(chain.mcp_fist_ref_rad, 1e-6), 0.0, 1.0))

    j2, j3 = _map_finger_chain(
        joints26,
        J.INDEX_METACARPAL,
        J.INDEX_PROXIMAL,
        J.INDEX_INTERMEDIATE,
        J.INDEX_DISTAL,
        J.INDEX_TIP,
        lo[4],
        hi[4],
        lo[5],
        hi[5],
        chain,
    )
    j1 = _map_index_abduction(
        joints26,
        side=side,
        mcp_norm=mcp_n,
        lo1=lo[3],
        hi1=hi[3],
        gains=abd,
    )
    return j1, j2, j3


def _map_thumb_tip(
    joints26: np.ndarray,
    *,
    wrist: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    lo: np.ndarray,
    hi: np.ndarray,
    gains: ThumbTipGains,
) -> tuple[float, float, float]:
    """Thumb: j1 lateral anchor or legacy phi; j2 MCP at prox; j3 IP flex."""
    meta = _pos(joints26, J.THUMB_METACARPAL)
    prox = _pos(joints26, J.THUMB_PROXIMAL)
    dist = _pos(joints26, J.THUMB_DISTAL)
    tip = _pos(joints26, J.THUMB_TIP)

    mode = (gains.j1_mode or "lateral_anchor").lower()
    if mode in ("lateral", "lateral_anchor", "anchor"):
        j1 = _map_thumb_j1_lateral(
            joints26,
            wrist=wrist,
            x=x,
            y=y,
            z=z,
            lo1=lo[0],
            hi1=hi[0],
            gains=gains,
        )
    else:
        j1 = _map_thumb_j1_phi(
            joints26, x=x, y=y, lo1=lo[0], hi1=hi[0], gains=gains
        )

    # j2: flexion at 2nd joint (meta–prox–dist); j3: IP to tip.
    mcp = _flexion(meta, prox, dist)
    ip = _flexion(prox, dist, tip)
    fs = gains.flex_scale
    mcp_n = float(np.clip(mcp * fs / max(gains.mcp_fist_ref_rad, 1e-6), 0.0, 1.0))
    ip_n = float(np.clip(ip * fs / max(gains.ip_fist_ref_rad, 1e-6), 0.0, 1.0))

    j2_span = hi[1] - gains.mcp_bias
    j2 = gains.mcp_bias + mcp_n * j2_span
    j2 = float(np.clip(j2, lo[1], hi[1]))
    j3 = _map_to_range(ip_n, lo[2], hi[2])
    return j1, j2, j3


def openxr26_to_xhand1_q(
    joints26: np.ndarray,
    *,
    side: str = "right",
    limits: np.ndarray | None = None,
    finger_chain: FingerChainGains | None = None,
    thumb_tip: ThumbTipGains | None = None,
    index_abduction: IndexAbductionGains | None = None,
    flexion_scale: float = 1.0,
    index_tip: IndexAbductionGains | None = None,
    index_gains: IndexAbductionGains | None = None,
    thumb_gains: ThumbTipGains | None = None,
) -> np.ndarray:
    """Map OpenXR 26×7 (or 26×3) → XHand1 q[12] (rad)."""
    joints26 = np.asarray(joints26, dtype=np.float64)
    if joints26.ndim != 2 or joints26.shape[0] != J.NUM_JOINTS:
        raise ValueError(f"expected (26, ≥3), got {joints26.shape}")

    lim = XHAND1_LIMITS if limits is None else np.asarray(limits, dtype=np.float64)
    lo = lim[:, 0]
    hi = lim[:, 1]

    fc = finger_chain or FingerChainGains()
    if flexion_scale != 1.0:
        fc = FingerChainGains(
            mcp_fist_ref_rad=fc.mcp_fist_ref_rad,
            pip_fist_ref_rad=fc.pip_fist_ref_rad,
            scale=fc.scale * flexion_scale,
        )
    tt = thumb_tip or thumb_gains or ThumbTipGains()
    abd = index_abduction or index_tip or index_gains or IndexAbductionGains()

    wrist, x, y, z = _palm_axes(joints26, side)

    t1, t2, t3 = _map_thumb_tip(
        joints26, wrist=wrist, x=x, y=y, z=z, lo=lo, hi=hi, gains=tt
    )
    i1, i2, i3 = _map_index(
        joints26,
        wrist=wrist,
        x=x,
        y=y,
        z=z,
        side=side,
        lo=lo,
        hi=hi,
        chain=fc,
        abd=abd,
    )
    m1, m2 = _map_finger_chain(
        joints26,
        J.MIDDLE_METACARPAL,
        J.MIDDLE_PROXIMAL,
        J.MIDDLE_INTERMEDIATE,
        J.MIDDLE_DISTAL,
        J.MIDDLE_TIP,
        lo[6],
        hi[6],
        lo[7],
        hi[7],
        fc,
    )
    r1, r2 = _map_finger_chain(
        joints26,
        J.RING_METACARPAL,
        J.RING_PROXIMAL,
        J.RING_INTERMEDIATE,
        J.RING_DISTAL,
        J.RING_TIP,
        lo[8],
        hi[8],
        lo[9],
        hi[9],
        fc,
    )
    p1, p2 = _map_finger_chain(
        joints26,
        J.LITTLE_METACARPAL,
        J.LITTLE_PROXIMAL,
        J.LITTLE_INTERMEDIATE,
        J.LITTLE_DISTAL,
        J.LITTLE_TIP,
        lo[10],
        hi[10],
        lo[11],
        hi[11],
        fc,
    )

    q = np.array(
        [t1, t2, t3, i1, i2, i3, m1, m2, r1, r2, p1, p2],
        dtype=np.float64,
    )
    return _clamp_q(q, lim)


# Back-compat alias
IndexTipGains = IndexAbductionGains

_FINGER_CHAIN_IDX = {
    "index": (
        J.INDEX_METACARPAL,
        J.INDEX_PROXIMAL,
        J.INDEX_INTERMEDIATE,
        J.INDEX_DISTAL,
        J.INDEX_TIP,
    ),
    "middle": (
        J.MIDDLE_METACARPAL,
        J.MIDDLE_PROXIMAL,
        J.MIDDLE_INTERMEDIATE,
        J.MIDDLE_DISTAL,
        J.MIDDLE_TIP,
    ),
    "ring": (
        J.RING_METACARPAL,
        J.RING_PROXIMAL,
        J.RING_INTERMEDIATE,
        J.RING_DISTAL,
        J.RING_TIP,
    ),
    "pinky": (
        J.LITTLE_METACARPAL,
        J.LITTLE_PROXIMAL,
        J.LITTLE_INTERMEDIATE,
        J.LITTLE_DISTAL,
        J.LITTLE_TIP,
    ),
}


def finger_chain_flexions(
    joints26: np.ndarray, finger: str
) -> tuple[float, float, float]:
    """MCP / PIP / DIP bone-chain flexion (rad) for index|middle|ring|pinky."""
    key = finger.lower()
    if key not in _FINGER_CHAIN_IDX:
        raise ValueError(f"unknown finger {finger!r}")
    meta_i, prox_i, inter_i, dist_i, tip_i = _FINGER_CHAIN_IDX[key]
    meta = _pos(joints26, meta_i)
    prox = _pos(joints26, prox_i)
    inter = _pos(joints26, inter_i)
    dist = _pos(joints26, dist_i)
    tip = _pos(joints26, tip_i)
    return (
        _flexion(meta, prox, inter),
        _flexion(prox, inter, dist),
        _flexion(inter, dist, tip),
    )


def finger_chain_flexion(joints26: np.ndarray, finger: str) -> tuple[float, float]:
    """MCP / PIP+DIP bone-chain flexion (rad) for index|middle|ring|pinky."""
    mcp, pip, dip = finger_chain_flexions(joints26, finger)
    return mcp, 0.65 * pip + 0.35 * dip


def finger_curl_whole(joints26: np.ndarray, finger: str) -> tuple[float, float, float, float]:
    """Whole-finger curl from meta→tip chain (OpenXR 26 / VR hand).

    Returns ``(curl, mcp, pip, dip)``. ``curl = min(mcp, pip, dip)`` so robot
    travel needs all three bends (proximal / intermediate / distal→tip), not
    just the largest one or two joints. Open/fist/pinch calib knots should be
    recaptured with this scalar (old ``max(mcp, pip)`` knots are incompatible).
    """
    mcp, pip, dip = finger_chain_flexions(joints26, finger)
    return float(min(mcp, pip, dip)), float(mcp), float(pip), float(dip)


def thumb_chain_flexion(joints26: np.ndarray) -> tuple[float, float]:
    """Thumb MCP (meta–prox–dist) and IP (prox–dist–tip) flexion (rad)."""
    meta = _pos(joints26, J.THUMB_METACARPAL)
    prox = _pos(joints26, J.THUMB_PROXIMAL)
    dist = _pos(joints26, J.THUMB_DISTAL)
    tip = _pos(joints26, J.THUMB_TIP)
    return _flexion(meta, prox, dist), _flexion(prox, dist, tip)


def index_abduction_lateral_m(joints26: np.ndarray, side: str) -> float:
    """Palm-plane lateral (m) of INDEX_PROXIMAL − INDEX_METACARPAL."""
    x, _y, z = _palm_axes_for_index_abd(joints26, side)
    meta = _pos(joints26, J.INDEX_METACARPAL)
    prox = _pos(joints26, J.INDEX_PROXIMAL)
    bone = prox - meta
    bone_p = bone - z * float(np.dot(bone, z))
    return float(np.dot(bone_p, x))


def four_finger_tip_span(joints26: np.ndarray) -> float:
    """||index_tip − pinky_tip|| / palm_width. Together is smaller than spread."""
    idx_p = _pos(joints26, J.INDEX_PROXIMAL)
    pky_p = _pos(joints26, J.LITTLE_PROXIMAL)
    width = max(float(np.linalg.norm(idx_p - pky_p)), 1e-3)
    idx_t = _pos(joints26, J.INDEX_TIP)
    pky_t = _pos(joints26, J.LITTLE_TIP)
    return float(np.linalg.norm(idx_t - pky_t) / width)


def thumb_tip_palm_x_m(joints26: np.ndarray, side: str) -> float:
    """Palm-frame x of thumb tip (m). Ulnar / 最左 typically decreases x."""
    wrist, x, y, z = _palm_axes(joints26, side)
    return _palm_point(_pos(joints26, J.THUMB_TIP), wrist, x, y, z)[0]
