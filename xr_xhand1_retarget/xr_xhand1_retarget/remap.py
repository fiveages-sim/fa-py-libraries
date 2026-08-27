"""Hybrid OpenXR 26 → XHand1 12-DOF remap.

- Index + middle / ring / pinky: bone-chain flexion (URDF U) + index abduction on j1.
- Thumb: tip palm-frame opposition (j1) + bone-chain flex at tip (j2/j3).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import openxr_joints as J

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
    """Index joint1 from INDEX_PROXIMAL − INDEX_METACARPAL in palm plane (no other fingers)."""

    abduction_sign: float = -1.0
    # |bone·x| at palm plane (m) mapped to ±0.174 rad abduction.
    lateral_span_m: float = 0.028
    abduction_curl_fade: float = 0.40


@dataclass
class ThumbTipGains:
    """Thumb: j1 from tip palm bearing; j2/j3 from thumb-chain flex at tip."""

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


def _palm_uvw(
    p: np.ndarray, wrist: np.ndarray, x: np.ndarray, y: np.ndarray, z: np.ndarray
) -> np.ndarray:
    rel = p - wrist
    return np.array(
        [float(np.dot(rel, x)), float(np.dot(rel, y)), float(np.dot(rel, z))],
        dtype=np.float64,
    )


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
    x: np.ndarray,
    z: np.ndarray,
    mcp_norm: float,
    lo1: float,
    hi1: float,
    gains: IndexAbductionGains,
) -> float:
    """index_joint1: palm-plane lateral of INDEX_PROXIMAL − INDEX_METACARPAL."""
    meta = _pos(joints26, J.INDEX_METACARPAL)
    prox = _pos(joints26, J.INDEX_PROXIMAL)
    bone = prox - meta
    bone_p = bone - z * float(np.dot(bone, z))
    lat = float(np.dot(bone_p, x))
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
        x=x,
        z=z,
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
    """Thumb tip inverse: j1 from tip bearing; j2/j3 from thumb bone flex to tip."""
    meta = _pos(joints26, J.THUMB_METACARPAL)
    prox = _pos(joints26, J.THUMB_PROXIMAL)
    dist = _pos(joints26, J.THUMB_DISTAL)
    tip = _pos(joints26, J.THUMB_TIP)

    # j1: palm-plane angle of metacarpal→tip (opposition / CMC).
    bearing = tip - meta
    bu = float(np.dot(bearing, x))
    bv = float(np.dot(bearing, y))
    phi = float(np.arctan2(bv, bu))
    raw = (phi - gains.rest_phi_rad) * gains.opposition_scale + gains.opposition_offset
    if gains.invert_cmc:
        j1 = gains.cmc_span - raw
    else:
        j1 = raw
    j1 = float(np.clip(j1, lo[0], hi[0]))

    # j2/j3: MCP + IP flexion along chain ending at tip (URDF U).
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
