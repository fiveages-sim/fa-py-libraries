"""LinkerHand O6/L6/O7 closed-form map. No fingertip xyh IK.

Human calib (configs/calib/{o6,l6,o7}.yaml) stores extracted VR features:
curl / thumb_az / thumb_h. Robot q lives in configs/{o6,l6,o7}.yaml.

O6/L6 (6-DOF): j2 ← nest_lat / slot_t (tip−巢); thumb_az fallback only.
Pinch/fist thumb along index PIP → middle PIP (``slot_t``). j1 fills
the URDF rectangle: along (j2≈0) uses full curl → j1_hi; pinch contact
only in the j2 corridor. Four fingers: min(MCP, PIP, DIP) chain flexion
to pinch_q (all phalanxes must bend); palmar pack (h/r) to URDF hi only
outside the pinch corridor (fist reclaim).

O7 curl: j2 from az opposition, then a 3D tip−thumb pinch class (index
beats 掌边; fist kills ulnar attract). j3 follows az and may pass the
pinky contact knot; classified pinch nails that slot.
O7 ``veccurl``: ``v = tip − nest`` scalars interpolated to j3/j2/j1 (not IK).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from xr_hand_retarget.algorithms.curl_xhand1 import _interp_knots, _unit_t
from xr_hand_retarget.algorithms.remap import (
    finger_curl_whole,
    thumb_chain_flexion,
)
from xr_hand_retarget.sources import openxr_joints as J

from xr_hand_retarget.kinematics import load_linker_limits

FINGERS = ("index", "middle", "ring", "pinky")
SCHEMA = "linkerhand_o6_palm_tip.v5"

# STL-measured contact (not URDF tip FK). j2 past 1.3 does not add pinch.
J2_INDEX = 1.0
J2_MIDDLE = 1.3

_SEED_OPEN_CURL = 0.15
_SEED_FIST_CURL = 1.25
_SEED_OPEN_THUMB_CURL = 0.10
_SEED_FLEX_THUMB_CURL = 1.05
_SEED_OPEN_AZ = 0.85
_SEED_MIDDLE_AZ = -0.15
_SEED_OPEN_H = 0.04
_SEED_UP_H = 0.0  # 0 disables the ⟂-palm blend until thumb_up is captured
# Pinch curl mid knot: leave a little room before fist so live close can
# pass the contact floor. 0.62 was clipping pinky/index pinch toward
# half-fist; 0.92 keeps the captured contact.
_PINCH_KNOT_MAX_T = 0.92

_FINGER_TIP = {
    "index": J.INDEX_TIP,
    "middle": J.MIDDLE_TIP,
    "ring": J.RING_TIP,
    "pinky": J.LITTLE_TIP,
}
_FINGER_PROX = {
    "index": J.INDEX_PROXIMAL,
    "middle": J.MIDDLE_PROXIMAL,
    "ring": J.RING_PROXIMAL,
    "pinky": J.LITTLE_PROXIMAL,
}
_THUMB_J1 = 0
_THUMB_J2 = 1
_THUMB_J3 = 2
_FINGER_IDX = {"index": 2, "middle": 3, "ring": 4, "pinky": 5}
_FINGER_IDX_7 = {"index": 3, "middle": 4, "ring": 5, "pinky": 6}

_JOINT_IDX = {
    "thumb_joint1": 0,
    "thumb_joint2": 1,
    "index_joint": 2,
    "index": 2,
    "middle_joint": 3,
    "middle": 3,
    "ring_joint": 4,
    "ring": 4,
    "pinky_joint": 5,
    "pinky": 5,
}

_JOINT_IDX_7 = {
    "thumb_joint1": 0,
    "thumb_joint2": 1,
    "thumb_joint3": 2,
    "index_joint": 3,
    "index": 3,
    "middle_joint": 4,
    "middle": 4,
    "ring_joint": 5,
    "ring": 5,
    "pinky_joint": 6,
    "pinky": 6,
}


def _hand_layout(model: str) -> tuple[str, int, dict[str, int], dict[str, int]]:
    m = str(model or "o6").strip().lower()
    if m == "o7":
        return m, 7, dict(_FINGER_IDX_7), dict(_JOINT_IDX_7)
    if m not in ("o6", "l6"):
        m = "o6"
    return m, 6, dict(_FINGER_IDX), dict(_JOINT_IDX)


def _xyz(joints26: np.ndarray, idx: int) -> np.ndarray:
    return np.asarray(joints26, dtype=np.float64)[idx, :3]


@dataclass
class O6Workspace:
    """Robot-side pinch. ``pinch_q`` is attract floor + optional planning ceiling."""

    j2_index: float = J2_INDEX
    j2_middle: float = J2_MIDDLE
    close_j1: float = 0.41
    close_finger: dict[str, float] = field(
        default_factory=lambda: {
            "index": 0.8,
            "middle": 0.83,
            "ring": 0.8,
            "pinky": 0.8,
        }
    )
    attract_near: float = 0.22
    attract_far: float = 0.45
    attract_enabled: bool = True
    slot_near: float = 0.28
    slot_far: float = 1.0
    j2_lock_lo: float | None = None
    j2_lock_hi: float | None = None
    fist_env_enabled: bool = True
    fist_env_fingers: tuple[str, ...] = ("index", "middle")
    fist_env_finger_t: float = 0.92
    fist_env_j2_split: float = 0.6
    fist_env_j1_roof: float = 0.44
    dof: int = 6
    finger_idx: dict[str, int] = field(default_factory=lambda: dict(_FINGER_IDX))
    thumb3_opp: float = 0.65
    thumb3_curl: float = 0.35
    close_j3: float | None = None
    j3_index: float = 0.18
    j3_middle: float = 0.36
    j3_ring: float = 0.50
    j3_pinky: float = 0.70
    j3_ulnar: float = 0.70
    fist_j1: float | None = None
    along_j1: float | None = None
    closed_reach_t: float = 0.88
    pinch_knot_max_t: float = _PINCH_KNOT_MAX_T
    nest_r_near: float = 0.32
    nest_r_far: float = 1.15
    nest_curl_span: float = 0.08
    nest_lat_along: float = 0.42
    nest_lat_opp: float = -0.02
    # j2 ≤ along: 侧掌, full j1. Fade then duck until pinch corridor.
    j2_along: float = 0.22
    j2_along_fade: float = 0.12
    j2_pinch_hw: float = 0.25
    # O7 pinch planning: RViz pinch_q ceilings + optional FK pad projection.
    pinch_q_slots: dict[str, dict[str, float]] = field(default_factory=dict)
    pinch_ceiling_enabled: bool = False
    pinch_ceiling_margin_rad: float = 0.0
    pinch_fk_project_enabled: bool = False
    pinch_fk_clearance_m: float | None = None
    pinch_fk_sphere_radius_m: float | None = None


def _q_overlay(raw, defaults: dict[int, float], *, names=None, dof: int = 6) -> dict[int, float]:
    names = names or _JOINT_IDX
    if not raw:
        return dict(defaults)
    if isinstance(raw, dict):
        out: dict[int, float] = {}
        for key, val in raw.items():
            if val is None:
                continue
            if isinstance(key, int):
                idx = key
            else:
                idx = names.get(str(key).strip().lower())
            if idx is None or int(idx) >= int(dof):
                continue
            out[int(idx)] = float(val)
        return out if out else dict(defaults)
    out = {}
    for i, v in enumerate(raw):
        if v is None or i >= int(dof):
            continue
        out[i] = float(v)
    return out if out else dict(defaults)


def _slot_far_from_gains(gains) -> float:
    if not hasattr(gains, "slot_far"):
        return 1.0
    return float(getattr(gains, "slot_far") or 0.0)


def _slot_near_from_gains(gains) -> float:
    explicit = getattr(gains, "slot_near", None)
    if explicit is not None and float(explicit) > 0.0:
        return float(explicit)
    near = float(getattr(gains, "attract_near", 0.0) or 0.0)
    return near if near > 0.0 else 0.28


def _fist_fingers_from(raw) -> tuple[str, ...]:
    names = []
    if isinstance(raw, str):
        raw = [p.strip() for p in raw.split(",")]
    if raw:
        for item in raw:
            n = str(item).strip().lower()
            if n in FINGERS:
                names.append(n)
    return tuple(names) or ("index", "middle")


def _build_ws_pinch_slots(**kwargs) -> dict[str, dict[str, float]]:
    from xr_hand_retarget.algorithms.pinch_fk import build_pinch_q_slots

    return build_pinch_q_slots(**kwargs)


def workspace_from_gains(gains) -> O6Workspace:
    _model, dof, finger_idx, names = _hand_layout(getattr(gains, "model", "o6"))
    i_idx = finger_idx["index"]
    m_idx = finger_idx["middle"]
    r_idx = finger_idx["ring"]
    p_idx = finger_idx["pinky"]
    raw_idx = getattr(gains, "pinch_index_q", None)
    raw_mid = getattr(gains, "pinch_middle_q", None)
    raw_rng = getattr(gains, "pinch_ring_q", None)
    raw_pky = getattr(gains, "pinch_pinky_q", None)
    has_pinch = bool(raw_idx or raw_mid)
    idx = _q_overlay(raw_idx, {}, names=names, dof=dof)
    mid = _q_overlay(raw_mid, {}, names=names, dof=dof)
    rng = _q_overlay(raw_rng, {}, names=names, dof=dof)
    pky = _q_overlay(raw_pky, {}, names=names, dof=dof)
    env = getattr(gains, "fist_envelope", None) or {}
    fingers = _fist_fingers_from(
        env.get("fingers", getattr(gains, "fist_env_fingers", None))
    )
    j3_close = idx.get(_THUMB_J3, mid.get(_THUMB_J3)) if dof >= 7 else None
    return O6Workspace(
        j2_index=float(getattr(gains, "thumb_j2_index", J2_INDEX)),
        j2_middle=float(getattr(gains, "thumb_j2_middle", J2_MIDDLE)),
        close_j1=float(idx.get(0, mid.get(0, 0.0 if not has_pinch else 0.41))),
        close_finger={
            "index": float(idx.get(i_idx, 0.0 if not has_pinch else 0.8)),
            "middle": float(mid.get(m_idx, 0.0 if not has_pinch else 0.83)),
            "ring": float(rng.get(r_idx, getattr(gains, "close_ring", 0.0))),
            "pinky": float(pky.get(p_idx, getattr(gains, "close_pinky", 0.0))),
        },
        attract_near=float(getattr(gains, "attract_near", 0.22)),
        attract_far=float(getattr(gains, "attract_far", 0.45)),
        attract_enabled=bool(getattr(gains, "attract_enabled", True)),
        slot_near=_slot_near_from_gains(gains),
        slot_far=_slot_far_from_gains(gains),
        j2_lock_lo=(
            None
            if getattr(gains, "j2_lock_lo", None) is None
            else float(gains.j2_lock_lo)
        ),
        j2_lock_hi=(
            None
            if getattr(gains, "j2_lock_hi", None) is None
            else float(gains.j2_lock_hi)
        ),
        fist_env_enabled=bool(
            env.get("enabled", getattr(gains, "fist_env_enabled", True))
        ),
        fist_env_fingers=fingers,
        fist_env_finger_t=float(
            env.get("finger_t", getattr(gains, "fist_env_finger_t", 0.92))
        ),
        fist_env_j2_split=float(
            env.get("j2_split", getattr(gains, "fist_env_j2_split", 0.6))
        ),
        fist_env_j1_roof=float(
            env.get("j1_roof", getattr(gains, "fist_env_j1_roof", 0.44))
        ),
        dof=dof,
        finger_idx=finger_idx,
        thumb3_opp=float(getattr(gains, "thumb3_opp", 0.65)),
        thumb3_curl=float(getattr(gains, "thumb3_curl", 0.35)),
        close_j3=None if j3_close is None else float(j3_close),
        j3_index=float(getattr(gains, "thumb_j3_index", 0.18)),
        j3_middle=float(getattr(gains, "thumb_j3_middle", 0.36)),
        j3_ring=float(getattr(gains, "thumb_j3_ring", 0.50)),
        j3_pinky=float(getattr(gains, "thumb_j3_pinky", 0.70)),
        j3_ulnar=float(
            getattr(gains, "thumb_j3_ulnar", getattr(gains, "thumb_j3_pinky", 0.70))
        ),
        fist_j1=(
            None
            if getattr(gains, "fist_j1", None) is None
            else float(gains.fist_j1)
        ),
        along_j1=(
            None
            if getattr(gains, "along_j1", None) is None
            else float(gains.along_j1)
        ),
        pinch_knot_max_t=float(
            _PINCH_KNOT_MAX_T
            if getattr(gains, "pinch_knot_max_t", None) is None
            else float(gains.pinch_knot_max_t)
        ),
        closed_reach_t=float(getattr(gains, "closed_reach_t", 0.88)),
        nest_r_near=float(getattr(gains, "nest_r_near", 0.32)),
        nest_r_far=float(getattr(gains, "nest_r_far", 1.15)),
        nest_curl_span=float(getattr(gains, "nest_curl_span", 0.08)),
        nest_lat_along=float(getattr(gains, "nest_lat_along", 0.42)),
        nest_lat_opp=float(getattr(gains, "nest_lat_opp", -0.02)),
        j2_along=float(getattr(gains, "thumb_j2_along", 0.22)),
        j2_along_fade=float(getattr(gains, "thumb_j2_along_fade", 0.12)),
        j2_pinch_hw=float(getattr(gains, "thumb_j2_pinch_hw", 0.25)),
        pinch_q_slots=_build_ws_pinch_slots(
            dof=dof,
            finger_idx=finger_idx,
            names=names,
            j2_index=float(getattr(gains, "thumb_j2_index", J2_INDEX)),
            j2_middle=float(getattr(gains, "thumb_j2_middle", J2_MIDDLE)),
            j3_index=float(getattr(gains, "thumb_j3_index", 0.18)),
            j3_middle=float(getattr(gains, "thumb_j3_middle", 0.36)),
            j3_ring=float(getattr(gains, "thumb_j3_ring", 0.50)),
            j3_pinky=float(getattr(gains, "thumb_j3_pinky", 0.70)),
            close_j1=float(idx.get(0, mid.get(0, 0.0 if not has_pinch else 0.41))),
            close_finger={
                "index": float(idx.get(i_idx, 0.0 if not has_pinch else 0.8)),
                "middle": float(mid.get(m_idx, 0.0 if not has_pinch else 0.83)),
                "ring": float(rng.get(r_idx, getattr(gains, "close_ring", 0.0))),
                "pinky": float(pky.get(p_idx, getattr(gains, "close_pinky", 0.0))),
            },
            raw_blocks={
                "index": raw_idx,
                "middle": raw_mid,
                "ring": raw_rng,
                "pinky": raw_pky,
            },
        ),
        pinch_ceiling_enabled=bool(getattr(gains, "pinch_ceiling_enabled", False)),
        pinch_ceiling_margin_rad=float(
            getattr(gains, "pinch_ceiling_margin_rad", 0.0) or 0.0
        ),
        pinch_fk_project_enabled=bool(
            getattr(gains, "pinch_fk_project_enabled", False)
        ),
        pinch_fk_clearance_m=getattr(gains, "pinch_fk_clearance_m", None),
        pinch_fk_sphere_radius_m=getattr(gains, "pinch_fk_sphere_radius_m", None),
    )


@dataclass
class PalmTipFeat:
    finger_curl: dict[str, float]
    thumb_curl: float
    thumb_az: float
    thumb_h: float = 0.0
    pinch_d: dict[str, float] = field(default_factory=dict)
    # Palm-frame (finger_tip − thumb_tip) / palm_width: (vx, vy, vz).
    pinch_xyz: dict[str, tuple[float, float, float]] = field(default_factory=dict)
    # In-plane alignment of that vector with the finger MCP→tip axis, in [0, 1].
    pinch_align: dict[str, float] = field(default_factory=dict)
    nest_r: float = 0.0
    nest_lat: float = 0.0
    nest_r_p: float = 0.0
    slot_t: float = 0.0
    # Tip vs palm: h = normal height / width; r = in-plane distance to nest.
    pack_h: dict[str, float] = field(default_factory=dict)
    pack_r: dict[str, float] = field(default_factory=dict)
    # v1 dump/load aliases
    finger_r: dict[str, float] = field(default_factory=dict)
    thumb_r: float = 0.0
    # Per-phalanx flex (meta→prox, prox→inter, inter→tip) for HUD / debug.
    finger_chain: dict[str, tuple[float, float, float]] = field(
        default_factory=dict
    )


@dataclass
class PalmTipCalib:
    open_finger_curl: dict[str, float]
    fist_finger_curl: dict[str, float]
    open_thumb_curl: float
    flex_thumb_curl: float  # fist thumb curl (j1 upper knot)
    open_thumb_az: float
    along_thumb_az: float
    middle_thumb_az: float
    open_thumb_h: float = 0.0
    up_thumb_h: float = 0.0
    good_finger_curl: dict[str, float] | None = None
    good_thumb_curl: float | None = None
    together_finger_curl: dict[str, float] | None = None
    together_thumb_curl: float | None = None
    together_thumb_az: float | None = None
    together_thumb_h: float | None = None
    together_pack_h: dict[str, float] | None = None
    together_pack_r: dict[str, float] | None = None
    pinch_skipped: bool = True
    pinch: dict[str, dict[str, float] | None] | None = None
    mirrored_from: str | None = None
    along_nest_lat: float | None = None
    along_slot_t: float | None = None
    along_nest_r: float | None = None
    open_pack_h: dict[str, float] | None = None
    open_pack_r: dict[str, float] | None = None
    fist_pack_h: dict[str, float] | None = None
    fist_pack_r: dict[str, float] | None = None

    @property
    def pinky_thumb_az(self) -> float:
        return self.middle_thumb_az

    @property
    def fist_thumb_curl(self) -> float:
        return float(self.flex_thumb_curl)


def _palm_width_m(joints26: np.ndarray) -> float:
    d = _xyz(joints26, J.INDEX_PROXIMAL) - _xyz(joints26, J.LITTLE_PROXIMAL)
    return max(float(np.linalg.norm(d)), 1e-3)


def _unit3(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return np.zeros(3, dtype=np.float64)
    return np.asarray(v, dtype=np.float64) / n


def _thumb_palm_axes(
    joints26: np.ndarray, side: str
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Palm frame for thumb azimuth. Y is not middle MCP (pinch rotates it)."""
    wrist = _xyz(joints26, J.WRIST)
    index = _xyz(joints26, J.INDEX_PROXIMAL) - wrist
    ring = _xyz(joints26, J.RING_PROXIMAL) - wrist
    pinky = _xyz(joints26, J.LITTLE_PROXIMAL) - wrist
    y = _unit3(index + ring + pinky)
    x = _unit3(index - pinky)
    z = _unit3(np.cross(x, y))
    if str(side).lower().startswith("l"):
        z = -z
        x = _unit3(np.cross(y, z))
    else:
        x = _unit3(np.cross(y, z))
        z = _unit3(np.cross(x, y))
    return wrist, x, y, z


def _thumb_palm_frame(
    joints26: np.ndarray, side: str
) -> tuple[float, float]:
    """Metacarpal in-plane azimuth and proximal height / palm width.

    Azimuth is the heading of METACARPAL→PROXIMAL in the palm plane (j2 sweep).
    Height is palm-normal offset of the proximal (abduction cue for j2 blend).
    """
    wrist, x, y, z = _thumb_palm_axes(joints26, side)
    meta = _xyz(joints26, J.THUMB_METACARPAL)
    prox = _xyz(joints26, J.THUMB_PROXIMAL)
    bone = prox - meta
    bn = float(np.dot(bone, z))
    bone_p = bone - z * bn
    px = float(np.dot(bone_p, x))
    py = float(np.dot(bone_p, y))
    az = float(np.arctan2(px, py if abs(py) > 1e-9 else 1e-9))
    width = _palm_width_m(joints26)
    h = float(np.dot(prox - wrist, z)) / width
    return az, h


_NEST_PIPS = (
    J.INDEX_INTERMEDIATE,
    J.MIDDLE_INTERMEDIATE,
    J.RING_INTERMEDIATE,
    J.LITTLE_INTERMEDIATE,
)
# Knuckles barely move in a fist; PIP mean does. Finger pack r uses this.
_PALM_MCPS = (
    J.INDEX_PROXIMAL,
    J.MIDDLE_PROXIMAL,
    J.RING_PROXIMAL,
    J.LITTLE_PROXIMAL,
)


def nest_radius_lat(joints26: np.ndarray, side: str) -> tuple[float, float, float]:
    """Thumb tip vs four-finger PIP mean / palm_width.

    Returns ``(r, lat, r_p)``: 3D radius, radial in-plane offset, in-plane
    radius. +lat is toward the reconstructed palm +X (index−pinky after
    the left/right flip). Sign of along vs pinch is taken from calib knots,
    not assumed positive.
    """
    nest = np.stack([_xyz(joints26, i) for i in _NEST_PIPS], axis=0).mean(axis=0)
    thumb = _xyz(joints26, J.THUMB_TIP)
    width = max(_palm_width_m(joints26), 1e-6)
    _wrist, x, _y, z = _thumb_palm_axes(joints26, side)
    v = thumb - nest
    vn = float(np.dot(v, z))
    v_p = v - z * vn
    r_p = float(np.linalg.norm(v_p) / width)
    return float(np.linalg.norm(v) / width), float(np.dot(v_p, x) / width), r_p


def pinch_slot_t(joints26: np.ndarray, side: str) -> float:
    """Thumb in-plane along index PIP → middle PIP. 0=index pinch, 1=middle.

    This is the j2 x-axis for two-finger slots. The four-finger PIP mean
    ``nest_lat`` collapses index vs middle; the finger-pair axis does not.
    """
    idx = _xyz(joints26, J.INDEX_INTERMEDIATE)
    mid = _xyz(joints26, J.MIDDLE_INTERMEDIATE)
    th = _xyz(joints26, J.THUMB_TIP)
    _wrist, _x, _y, z = _thumb_palm_axes(joints26, side)
    v = mid - idx
    v_p = v - z * float(np.dot(v, z))
    n2 = float(np.dot(v_p, v_p))
    if n2 < 1e-8:
        return 0.0
    w = th - idx
    w_p = w - z * float(np.dot(w, z))
    return float(np.dot(w_p, v_p) / n2)


def extract_palm_tip(joints26: np.ndarray, side: str) -> PalmTipFeat:
    finger_curl: dict[str, float] = {}
    finger_chain: dict[str, tuple[float, float, float]] = {}
    for name in FINGERS:
        curl, mcp, pip, dip = finger_curl_whole(joints26, name)
        finger_curl[name] = curl
        finger_chain[name] = (mcp, pip, dip)
    t_mcp, t_ip = thumb_chain_flexion(joints26)
    thumb_curl = float(max(t_mcp, t_ip))
    thumb_az, thumb_h = _thumb_palm_frame(joints26, side)
    width = _palm_width_m(joints26)
    thumb = _xyz(joints26, J.THUMB_TIP)
    _wrist, ax, ay, az = _thumb_palm_axes(joints26, side)
    pinch_d: dict[str, float] = {}
    pinch_xyz: dict[str, tuple[float, float, float]] = {}
    pinch_align: dict[str, float] = {}
    for name in FINGERS:
        tip = _xyz(joints26, _FINGER_TIP[name])
        rel = tip - thumb
        nrm = float(np.linalg.norm(rel))
        pinch_d[name] = nrm / width
        vx = float(np.dot(rel, ax) / width)
        vy = float(np.dot(rel, ay) / width)
        vz = float(np.dot(rel, az) / width)
        pinch_xyz[name] = (vx, vy, vz)
        bone = tip - _xyz(joints26, _FINGER_PROX[name])
        bone_p = bone - az * float(np.dot(bone, az))
        rel_p = rel - az * float(np.dot(rel, az))
        nb = float(np.linalg.norm(bone_p))
        nv = float(np.linalg.norm(rel_p))
        if nb > 1e-8 and nv > 1e-8:
            pinch_align[name] = float(np.clip(np.dot(rel_p, bone_p) / (nb * nv), 0.0, 1.0))
        else:
            pinch_align[name] = 0.0
    nest_r, nest_lat, nest_r_p = nest_radius_lat(joints26, side)
    pack_h, pack_r = _finger_pack_hr(joints26, side)
    return PalmTipFeat(
        finger_curl=finger_curl,
        finger_chain=finger_chain,
        thumb_curl=thumb_curl,
        thumb_az=thumb_az,
        thumb_h=thumb_h,
        pinch_d=pinch_d,
        pinch_xyz=pinch_xyz,
        pinch_align=pinch_align,
        nest_r=nest_r,
        nest_lat=nest_lat,
        nest_r_p=nest_r_p,
        slot_t=pinch_slot_t(joints26, side),
        pack_h=pack_h,
        pack_r=pack_r,
        finger_r=dict(finger_curl),
        thumb_r=thumb_curl,
    )


def _finger_pack_hr(
    joints26: np.ndarray, side: str
) -> tuple[dict[str, float], dict[str, float]]:
    """Tip vs a stable palm, not vs the PIP nest.

    ``h``: palm-normal height from the wrist. ``r``: in-plane distance to the
    four MCP mean (掌心/指根). PIP nest moves with a fist, so tip−PIP radius
    stays ~one phalanx and cannot tell 悬握 from 扣掌.
    """
    wrist, _x, _y, z = _thumb_palm_axes(joints26, side)
    palm = np.stack([_xyz(joints26, i) for i in _PALM_MCPS], axis=0).mean(axis=0)
    width = max(_palm_width_m(joints26), 1e-6)
    pack_h: dict[str, float] = {}
    pack_r: dict[str, float] = {}
    for name in FINGERS:
        tip = _xyz(joints26, _FINGER_TIP[name])
        pack_h[name] = float(np.dot(tip - wrist, z) / width)
        v = tip - palm
        v_p = v - z * float(np.dot(v, z))
        pack_r[name] = float(np.linalg.norm(v_p) / width)
    return pack_h, pack_r


def seed_palm_calib() -> PalmTipCalib:
    return PalmTipCalib(
        open_finger_curl={n: _SEED_OPEN_CURL for n in FINGERS},
        fist_finger_curl={n: _SEED_FIST_CURL for n in FINGERS},
        open_thumb_curl=_SEED_OPEN_THUMB_CURL,
        flex_thumb_curl=_SEED_FLEX_THUMB_CURL,
        open_thumb_az=_SEED_OPEN_AZ,
        along_thumb_az=_SEED_OPEN_AZ,
        middle_thumb_az=_SEED_MIDDLE_AZ,
        open_thumb_h=_SEED_OPEN_H,
        up_thumb_h=_SEED_UP_H,
        pinch_skipped=True,
        pinch={n: None for n in FINGERS},
        good_finger_curl={n: _SEED_FIST_CURL for n in FINGERS},
        good_thumb_curl=_SEED_OPEN_THUMB_CURL,
        together_finger_curl=None,
        together_thumb_curl=None,
        together_thumb_az=None,
        together_thumb_h=None,
        together_pack_h=None,
        together_pack_r=None,
        along_nest_lat=None,
        along_slot_t=None,
        along_nest_r=None,
        open_pack_h=None,
        open_pack_r=None,
        fist_pack_h=None,
        fist_pack_r=None,
    )


def mirror_palm_calib(
    src: PalmTipCalib, *, from_side: str, az_sign: float = 1.0
) -> PalmTipCalib:
    """Copy one hand's human features onto the other.

    Curl is unsigned bone flexion → copy. ``thumb_az`` lives in a palm frame
    whose +X already points radial (toward the thumb) on both hands, so the
    default is copy as well. Set ``az_sign=-1`` only if j2 direction inverts.
    Robot URDF limits stay per-side; only the human knots are shared.
    """
    pinch = None
    if src.pinch is not None:
        pinch = {}
        for n, knot in src.pinch.items():
            if knot is None:
                pinch[n] = None
                continue
            item = dict(knot)
            if "thumb_az" in item:
                item["thumb_az"] = float(item["thumb_az"]) * float(az_sign)
            pinch[n] = item
    good_f = src.good_finger_curl
    tog_f = src.together_finger_curl
    tog_az = src.together_thumb_az
    return PalmTipCalib(
        open_finger_curl=dict(src.open_finger_curl),
        fist_finger_curl=dict(src.fist_finger_curl),
        open_thumb_curl=float(src.open_thumb_curl),
        flex_thumb_curl=float(src.flex_thumb_curl),
        open_thumb_az=float(src.open_thumb_az) * float(az_sign),
        along_thumb_az=float(src.along_thumb_az) * float(az_sign),
        middle_thumb_az=float(src.middle_thumb_az) * float(az_sign),
        open_thumb_h=float(src.open_thumb_h),
        up_thumb_h=float(src.up_thumb_h),
        good_finger_curl=None if good_f is None else dict(good_f),
        good_thumb_curl=(
            None if src.good_thumb_curl is None else float(src.good_thumb_curl)
        ),
        together_finger_curl=None if tog_f is None else dict(tog_f),
        together_thumb_curl=(
            None if src.together_thumb_curl is None else float(src.together_thumb_curl)
        ),
        together_thumb_az=(
            None if tog_az is None else float(tog_az) * float(az_sign)
        ),
        together_thumb_h=(
            None if src.together_thumb_h is None else float(src.together_thumb_h)
        ),
        together_pack_h=_copy_pack_map(src.together_pack_h),
        together_pack_r=_copy_pack_map(src.together_pack_r),
        pinch_skipped=bool(src.pinch_skipped),
        pinch=pinch,
        mirrored_from=from_side,
        along_nest_lat=(
            None if src.along_nest_lat is None else float(src.along_nest_lat)
        ),
        along_slot_t=(
            None if src.along_slot_t is None else float(src.along_slot_t)
        ),
        along_nest_r=(
            None if src.along_nest_r is None else float(src.along_nest_r)
        ),
        open_pack_h=_copy_pack_map(src.open_pack_h),
        open_pack_r=_copy_pack_map(src.open_pack_r),
        fist_pack_h=_copy_pack_map(src.fist_pack_h),
        fist_pack_r=_copy_pack_map(src.fist_pack_r),
    )


def _copy_pack_map(raw: dict[str, float] | None) -> dict[str, float] | None:
    if not raw:
        return None
    return {n: float(raw[n]) for n in FINGERS if n in raw}


def capture_sides_for_calib(cli_side: str, source_side: str) -> tuple[str, ...]:
    """Which hands to sample. CLI ``left|right`` wins; yaml ``source_side``
    collapses ``full`` to one hand."""
    cli = str(cli_side or "full").strip().lower()
    if cli in ("left", "right"):
        return (cli,)
    src = str(source_side or "both").strip().lower()
    if src in ("left", "right"):
        return (src,)
    return ("left", "right")


def resolve_palm_side(
    loaded: dict[str, PalmTipCalib],
    side: str,
    *,
    source_side: str = "both",
    az_sign: float = 1.0,
) -> tuple[PalmTipCalib, str]:
    """Pick the calib block for ``side``. ``source_side=left|right`` mirrors."""
    src = str(source_side or "both").strip().lower()
    if src not in ("left", "right"):
        cal = loaded.get(side) or seed_palm_calib()
        return cal, side
    origin = loaded.get(src) or seed_palm_calib()
    if side == src:
        return origin, src
    return mirror_palm_calib(origin, from_side=src, az_sign=az_sign), f"{src}→{side}"


def _pinch_curl_t(
    feat: PalmTipFeat, calib: PalmTipCalib | None, name: str
) -> float:
    """0 at open, 1 at that finger's pinch knot (not fist)."""
    if calib is None:
        return 0.0
    cap = _pinch_finger_curl(calib, name)
    if cap is None:
        cap = _finger_closed_curl(calib, name)
    return _unit_t(
        float((feat.finger_curl or {}).get(name, 0.0)),
        _finger_open_curl(calib, name),
        float(cap),
    )


def _thumb_flex_t(feat: PalmTipFeat, calib: PalmTipCalib | None) -> float:
    if calib is None:
        return 0.0
    return _unit_t(
        float(feat.thumb_curl),
        _j1_open_curl(calib),
        float(calib.flex_thumb_curl),
    )


def _iso_slot(
    c_t: dict[str, float], ws: O6Workspace
) -> tuple[float, float, float]:
    """Isolation weight, j3 mix, pad j2. Isolation is 0 when all fingers match."""
    w_radial = max(c_t["index"], c_t["middle"])
    w_ulnar = max(c_t["ring"], c_t["pinky"])
    w_lead = max(w_radial - w_ulnar, w_ulnar - w_radial)
    den = c_t["index"] + c_t["middle"] + c_t["ring"] + c_t["pinky"]
    pad = float(ws.j2_index)
    if w_radial + w_ulnar > 1e-9:
        pad = (
            w_radial * float(ws.j2_index) + w_ulnar * float(ws.j2_middle)
        ) / (w_radial + w_ulnar)
    if den <= 1e-9:
        return 0.0, float(ws.j3_index), pad
    j3_star = (
        c_t["index"] * float(ws.j3_index)
        + c_t["middle"] * float(ws.j3_middle)
        + c_t["ring"] * float(getattr(ws, "j3_ring", 0.50))
        + c_t["pinky"] * float(getattr(ws, "j3_pinky", ws.j3_ulnar))
    ) / den
    return float(np.clip(w_lead, 0.0, 1.0)), float(j3_star), float(pad)


def _lat_j2_monotonic(xs: list[float], qs: list[float]) -> bool:
    """After sorting lat, j2 must be monotonic (either sign of the palm X)."""
    if len(xs) < 2:
        return False
    order = np.argsort(np.asarray(xs, dtype=np.float64))
    qa = np.asarray(qs, dtype=np.float64)[order]
    d = np.diff(qa)
    return bool(np.all(d <= 1e-6) or np.all(d >= -1e-6))


def _j2_lat_knots(
    calib: PalmTipCalib, ws: O6Workspace
) -> tuple[list[float], list[float]] | None:
    """along / index pinch / middle pinch nest_lat → {0, j2_index, j2_middle}.

    Ring/pinky share the middle slot. Falls back to None (caller uses az)
    when lat was not captured or the knots are not monotonic in lat.
    """
    along = calib.along_nest_lat
    pk_i = _pinch_knot(calib, "index")
    pk_m = _pinch_knot(calib, "middle")
    idx_lat = None if not pk_i else pk_i.get("nest_lat")
    mid_lat = None if not pk_m else pk_m.get("nest_lat")
    candidates: list[tuple[list[float], list[float]]] = []
    if along is not None and idx_lat is not None and mid_lat is not None:
        xs = [float(along), float(idx_lat), float(mid_lat)]
        qs = [0.0, float(ws.j2_index), float(ws.j2_middle)]
        for name in ("ring", "pinky"):
            pk = _pinch_knot(calib, name)
            if pk and pk.get("nest_lat") is not None:
                xs.append(float(pk["nest_lat"]))
                qs.append(float(ws.j2_middle))
        candidates.append((xs, qs))
    if idx_lat is not None and mid_lat is not None:
        candidates.append(
            (
                [float(idx_lat), float(mid_lat)],
                [float(ws.j2_index), float(ws.j2_middle)],
            )
        )
    if along is not None and mid_lat is not None:
        candidates.append(
            ([float(along), float(mid_lat)], [0.0, float(ws.j2_middle)])
        )
    for xs, qs in candidates:
        if _lat_j2_monotonic(xs, qs):
            return xs, qs
    return None


def _j2_slot_knots(
    calib: PalmTipCalib | None, ws: O6Workspace
) -> tuple[list[float], list[float]]:
    """index/middle pinch positions on the PIP axis → {j2_index, j2_middle}.

    ``slot_t`` 0 is index PIP, 1 is middle. Canonical knots work before
    recapture; captured along/index/middle ``slot_t`` warp if Pico's pinch
    is not exactly at 0/1.
    """
    j_i = float(ws.j2_index)
    j_m = float(ws.j2_middle)
    along = None if calib is None else calib.along_slot_t
    pk_i = None if calib is None else _pinch_knot(calib, "index")
    pk_m = None if calib is None else _pinch_knot(calib, "middle")
    idx_t = None if not pk_i else pk_i.get("slot_t")
    mid_t = None if not pk_m else pk_m.get("slot_t")
    candidates: list[tuple[list[float], list[float]]] = []
    if along is not None and idx_t is not None and mid_t is not None:
        xs = [float(along), float(idx_t), float(mid_t)]
        qs = [0.0, j_i, j_m]
        for name in ("ring", "pinky"):
            pk = None if calib is None else _pinch_knot(calib, name)
            if pk and pk.get("slot_t") is not None:
                xs.append(float(pk["slot_t"]))
                qs.append(j_m)
        candidates.append((xs, qs))
    if idx_t is not None and mid_t is not None:
        candidates.append(([float(idx_t), float(mid_t)], [j_i, j_m]))
    candidates.append(([-0.5, 0.0, 1.0], [0.0, j_i, j_m]))
    candidates.append(([0.0, 1.0], [j_i, j_m]))
    for xs, qs in candidates:
        if _lat_j2_monotonic(xs, qs):
            return xs, qs
    return [0.0, 1.0], [j_i, j_m]


def _j2_side_park_w(
    feat: PalmTipFeat, calib: PalmTipCalib | None, ws: O6Workspace
) -> float:
    """1 when thumb should park beside extended fingers (侧掌 / open palm)."""
    if calib is None:
        return 0.0
    curl_t = _thumb_flex_t(feat, calib)
    r = float(getattr(feat, "nest_r", 1.0))
    r_p = float(getattr(feat, "nest_r_p", r))
    r_near = float(ws.nest_r_near)
    r_far = float(ws.nest_r_far)
    r_side = 0.5 * (r_near + r_far)
    w_far = _unit_t(r, r_side, r_far)
    w_up = 0.0
    if r_p < 0.18 and r > r_near + 0.15:
        w_up = 1.0
    return float((1.0 - curl_t) * max(w_far, w_up))


def _has_slot_calib(calib: PalmTipCalib) -> bool:
    if calib.along_slot_t is not None:
        return True
    for name in ("index", "middle"):
        pk = _pinch_knot(calib, name)
        if pk and pk.get("slot_t") is not None:
            return True
    return False


def _thumb_j2_from_tip(
    feat: PalmTipFeat, calib: PalmTipCalib | None, ws: O6Workspace
) -> float:
    """Tip-relative j2 opposition: nest_lat → slot_t → thumb_az (last resort).

    Side-park fades j2→0 when the thumb is extended beside an open hand.
    Metacarpal az alone cannot separate along / pinch / fist; tip features can.
    """
    if calib is None:
        return 0.0
    hi = float(ws.j2_middle)
    knots = _j2_lat_knots(calib, ws)
    if knots is not None:
        xs, qs = knots
        j2 = float(
            np.clip(_interp_knots(float(feat.nest_lat), xs, qs), 0.0, hi)
        )
    elif _has_slot_calib(calib):
        xs, qs = _j2_slot_knots(calib, ws)
        j2 = float(
            np.clip(
                _interp_knots(float(getattr(feat, "slot_t", 0.0)), xs, qs),
                0.0,
                hi,
            )
        )
    else:
        j2 = _thumb_j2_from_az(feat, calib, ws)
    w_side = _j2_side_park_w(feat, calib, ws)
    j2 = (1.0 - w_side) * j2
    return float(np.clip(j2, 0.0, hi))


def _thumb_j2_from_az(feat: PalmTipFeat, calib: PalmTipCalib, ws: O6Workspace) -> float:
    """Metacarpal heading → j2 fallback when nest_lat / slot_t knots missing."""
    xs = [float(calib.along_thumb_az)]
    qs = [0.0]
    pk_i = _pinch_knot(calib, "index")
    pk_m = _pinch_knot(calib, "middle")
    if pk_i and pk_i.get("thumb_az") is not None:
        xs.append(float(pk_i["thumb_az"]))
        qs.append(float(ws.j2_index))
    if pk_m and pk_m.get("thumb_az") is not None:
        xs.append(float(pk_m["thumb_az"]))
        qs.append(float(ws.j2_middle))
    else:
        xs.append(float(calib.middle_thumb_az))
        qs.append(float(ws.j2_middle))
    if not _lat_j2_monotonic(xs, qs):
        xs = [float(calib.along_thumb_az), float(calib.middle_thumb_az)]
        qs = [0.0, float(ws.j2_middle)]
        if pk_m and pk_m.get("thumb_az") is not None:
            xs[1] = float(pk_m["thumb_az"])
    return float(
        np.clip(
            _interp_knots(feat.thumb_az, xs, qs),
            0.0,
            float(ws.j2_middle),
        )
    )


def _finger_drive_w(feat: PalmTipFeat, calib: PalmTipCalib, ws: O6Workspace) -> float:
    """0 when fingers are extended (侧掌 / 开掌); 1 when a pinch or fist is on."""
    w = _fist_blend_w(feat, calib, ws)
    for name in FINGERS:
        w = max(w, _pinch_curl_t(feat, calib, name))
    return float(np.clip(w, 0.0, 1.0))


def _thumb_j2_rad(feat: PalmTipFeat, calib: PalmTipCalib, ws: O6Workspace) -> float:
    """O6/L6 map seed j2. O7 returns 0 (``blend_pad_j2`` sets j2 later)."""
    if int(getattr(ws, "dof", 6)) >= 7:
        return 0.0
    return _thumb_j2_from_tip(feat, calib, ws)


def _thumb_j1_from_nest_r(
    feat: PalmTipFeat,
    calib: PalmTipCalib,
    ws: O6Workspace,
    j1_hi: float,
) -> float:
    """Depth from nest radius. Far → 0; near → close_j1. Curl adds curl_span."""
    j1 = float(
        _interp_knots(
            float(getattr(feat, "nest_r", 1.0)),
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


def pinch_alpha(
    feat: PalmTipFeat,
    ws: O6Workspace,
    calib: PalmTipCalib | None = None,
) -> float:
    """Wuji-style pinch weight in [0,1] from tip distance (+ optional curl gate)."""
    near = float(ws.attract_near)
    far = float(ws.attract_far)
    if far <= 0.0 or near < 0.0:
        return 0.0
    a = 0.0
    d = feat.pinch_d or {}
    for name in FINGERS:
        w = _attract_w(d.get(name, 1e9), near, far)
        if calib is not None:
            w *= _pinch_curl_t(feat, calib, name)
        a = max(a, w)
    if calib is not None:
        a *= 1.0 - _fist_blend_w(feat, calib, ws)
    return float(np.clip(a, 0.0, 1.0))


def _reach_curl(c0: float, c2: float, reach_t: float) -> float:
    """Human curl that maps to robot fully-closed. <1 so optical under-read still hits q_hi."""
    r = float(np.clip(reach_t, 0.2, 1.0))
    return float(c0) + r * (float(c2) - float(c0))


def _finger_open_curl(calib: PalmTipCalib, name: str) -> float:
    """Human curl at open far end: min(open, together) when合掌 was captured."""
    c0 = float(calib.open_finger_curl[name])
    tog = calib.together_finger_curl
    if tog is not None and name in tog:
        c0 = min(c0, float(tog[name]))
    return c0


def _finger_closed_curl(calib: PalmTipCalib, name: str) -> float:
    """Human curl at fully closed: max(fist, good) four-finger max."""
    c = float(calib.fist_finger_curl[name])
    good = calib.good_finger_curl
    if good is not None and name in good:
        c = max(c, float(good[name]))
    return c


def _merge_open_pack(
    open_m: dict[str, float] | None,
    together_m: dict[str, float] | None,
    *,
    far: bool,
) -> dict[str, float] | None:
    """Merge open/together pack maps. ``far=True`` → max (open tip); else min."""
    if not open_m and not together_m:
        return None
    out: dict[str, float] = {}
    for n in FINGERS:
        vals: list[float] = []
        if open_m and n in open_m:
            vals.append(float(open_m[n]))
        if together_m and n in together_m:
            vals.append(float(together_m[n]))
        if vals:
            out[n] = max(vals) if far else min(vals)
    return out or None


def _open_pack_r(calib: PalmTipCalib) -> dict[str, float] | None:
    return _merge_open_pack(calib.open_pack_r, calib.together_pack_r, far=True)


def _open_pack_h(calib: PalmTipCalib) -> dict[str, float] | None:
    return _merge_open_pack(calib.open_pack_h, calib.together_pack_h, far=True)


def _axis_pack_t(live: float, x_open: float, x_fist: float, reach_t: float) -> float:
    closed = float(x_open) + float(np.clip(reach_t, 0.2, 1.0)) * (
        float(x_fist) - float(x_open)
    )
    return _unit_t(float(live), float(x_open), closed)


def _finger_has_pack(calib: PalmTipCalib | None, name: str) -> bool:
    """True if open→fist r (tip vs MCP palm) actually shrinks. h is optional."""
    if calib is None:
        return False
    r0 = (_open_pack_r(calib) or {}).get(name)
    r1 = (calib.fist_pack_r or {}).get(name)
    if r0 is None or r1 is None:
        return False
    return float(r0) >= float(r1) + 0.18


def _finger_pack_t(
    feat: PalmTipFeat,
    calib: PalmTipCalib | None,
    name: str,
    *,
    reach_t: float = 1.0,
) -> float:
    """0 at open tips, 1 when that tip is packed (h AND r toward fist)."""
    if not _finger_has_pack(calib, name):
        return 0.0
    assert calib is not None
    open_r = _open_pack_r(calib) or {}
    open_h = _open_pack_h(calib) or {}
    r_live = float((feat.pack_r or {}).get(name, 1e9))
    t_r = _axis_pack_t(
        r_live,
        float(open_r[name]),
        float(calib.fist_pack_r[name]),
        reach_t,
    )
    h0 = open_h.get(name)
    h1 = (calib.fist_pack_h or {}).get(name)
    raw = t_r
    if (
        h0 is not None
        and h1 is not None
        and abs(float(h0) - float(h1)) >= 0.04
        and float(h1) <= float(h0) + 0.02
    ):
        h_live = float((feat.pack_h or {}).get(name, float(h0)))
        t_h = _axis_pack_t(h_live, float(h0), float(h1), reach_t)
        raw = float(min(t_h, t_r))
    # First half of open→fist stays on curl/pinch_q (悬握 / 半钩).
    return _unit_t(raw, 0.48, 0.90)


def _pinch_contact_w(
    feat: PalmTipFeat,
    calib: PalmTipCalib | None,
    name: str,
    *,
    near: float,
    far: float,
) -> float:
    """0..1 thumb–finger tip proximity × curl (legacy helper / diagnostics).

    ``_pack_t_for_map`` no longer uses this; pack reclaim is gated by
    ``_pinch_curl_t`` and the fist curl segment instead of ``pinch_d``.
    """
    w_d = _lead_w((feat.pinch_d or {}).get(name, 1e9), near, far)
    if calib is None:
        return float(np.clip(w_d, 0.0, 1.0))
    w_mid = _pinch_curl_t(feat, calib, name)
    left_open = _unit_t(_finger_curl_t(feat, calib, name), 0.08, 0.30)
    return float(np.clip(w_d * max(w_mid, left_open), 0.0, 1.0))


def _pack_t_for_map(
    feat: PalmTipFeat,
    calib: PalmTipCalib | None,
    name: str,
    ws: O6Workspace,
    *,
    reach_t: float,
    max_t: float = _PINCH_KNOT_MAX_T,
) -> float:
    """Fist reclaim only: blend ``q_hi`` when curl is past the pinch knot.

    Primary four-finger map is ``finger_curl`` piecewise (open→pinch→fist).
    ``pack_r``/``pack_h`` top up closure when optical curl under-reads a
    packed fist. Suppression uses ``_pinch_curl_t`` and the fist segment
    (curl past ``c_pinch``), not ``pinch_d``, so thumb retreat does not
    reopen pack during a held pinch posture.
    """
    if calib is None or not _finger_has_pack(calib, name):
        return 0.0
    pack_t = _finger_pack_t(feat, calib, name, reach_t=reach_t)
    if pack_t <= 1e-9:
        return 0.0
    c0 = _finger_open_curl(calib, name)
    closed = _finger_closed_curl(calib, name)
    c1 = _pinch_finger_curl(calib, name, max_t=max_t)
    c2 = _reach_curl(c0, closed, reach_t)
    if c1 is not None:
        c2 = min(closed, max(c2, float(c1) + 1e-3))
        w_fist_seg = _unit_t(float(feat.finger_curl[name]), float(c1), float(c2))
    else:
        w_fist_seg = _unit_t(float(feat.finger_curl[name]), c0, c2)
    if w_fist_seg <= 1e-9:
        return 0.0
    w_pinch = _pinch_curl_t(feat, calib, name) if c1 is not None else 0.0
    suppress = float(w_pinch * (1.0 - w_fist_seg))
    return float(pack_t * w_fist_seg * (1.0 - float(np.clip(suppress, 0.0, 1.0))))


def _mean_pack_t(
    feat: PalmTipFeat,
    calib: PalmTipCalib | None,
    names: tuple[str, ...] | None = None,
    *,
    reach_t: float = 1.0,
) -> float:
    use = names or FINGERS
    ts = [
        _finger_pack_t(feat, calib, n, reach_t=reach_t)
        for n in use
        if _finger_has_pack(calib, n)
    ]
    return float(np.mean(ts)) if ts else 0.0


def _finger_curl_t(
    feat: PalmTipFeat, calib: PalmTipCalib | None, name: str
) -> float:
    if calib is None:
        return 0.0
    return _unit_t(
        float((feat.finger_curl or {}).get(name, 0.0)),
        _finger_open_curl(calib, name),
        _finger_closed_curl(calib, name),
    )


def blend_pinch_j2(
    j2_az: float,
    feat: PalmTipFeat,
    ws: O6Workspace,
    calib: PalmTipCalib | None = None,
) -> float:
    """Soft pull j2 toward C_index / C_middle when that finger is in its pinch band.

    Base heading is nest_lat (or az fallback). ``pinch_alpha`` scales the pull
    (Wuji-style open↔pinch). Fist fades the lead so 握拳扫掌 is not a pinch slot.
    """
    near = float(ws.slot_near)
    far = float(ws.slot_far)
    if far <= 0.0 or near < 0.0:
        return float(j2_az)
    d = feat.pinch_d or {}
    w_d = {n: _lead_w(d.get(n, 1e9), near, far) for n in FINGERS}
    p_t = {n: _pinch_curl_t(feat, calib, n) for n in FINGERS}
    has_lat = calib is not None and _j2_lat_knots(calib, ws) is not None
    if has_lat:
        w = {n: w_d[n] * p_t[n] for n in FINGERS}
        s_index = 0.25 * w["index"]
        s_middle = 0.25 * w["middle"]
        s_ulnar = 0.25 * (w["ring"] + w["pinky"])
    else:
        w = {n: max(w_d[n], p_t[n]) for n in FINGERS}
        s_index = p_t["index"] + 0.25 * w_d["index"]
        s_middle = p_t["middle"] + 0.25 * w_d["middle"]
        s_ulnar = p_t["ring"] + p_t["pinky"] + 0.25 * (w_d["ring"] + w_d["pinky"])
    w_radial = max(w["index"], w["middle"])
    w_ulnar = max(w["ring"], w["pinky"])
    w_lead_radial = max(0.0, w_radial - w_ulnar)
    w_lead_ulnar = max(0.0, w_ulnar - w_radial)
    w_lead = max(w_lead_radial, w_lead_ulnar)
    if calib is not None:
        w_lead *= 1.0 - _fist_blend_w(feat, calib, ws)
    # Soft Wuji-style gate: keep some pull from slot lead, boost with pinch_alpha.
    pa = pinch_alpha(feat, ws, calib)
    w_lead = float(np.clip(w_lead * (0.35 + 0.65 * max(pa, w_lead)), 0.0, 1.0))
    if w_lead <= 1e-9:
        return float(j2_az)
    den = s_index + s_middle + s_ulnar
    if den <= 1e-9:
        j2_star = float(
            ws.j2_middle if w_lead_ulnar >= w_lead_radial else ws.j2_index
        )
    else:
        j2_star = (
            s_index * float(ws.j2_index)
            + (s_middle + s_ulnar) * float(ws.j2_middle)
        ) / den
    j2 = (1.0 - w_lead) * float(j2_az) + w_lead * j2_star
    return float(np.clip(j2, 0.0, float(ws.j2_middle)))


@dataclass
class O7PinchDecision:
    """Which finger curl should treat as the live pinch. ``slot`` None = none."""

    slot: str | None
    scores: dict[str, float]
    w: float = 0.0


def _pinch_geom_score(feat: PalmTipFeat, name: str, near: float, far: float) -> float:
    """Score from tip−tip Δxyz (3 components) + scalar distance. Not curl."""
    d = float((feat.pinch_d or {}).get(name, 1e9))
    w_d = _lead_w(d, near, far)
    xyz = (feat.pinch_xyz or {}).get(name)
    if xyz is None:
        in_plane = 1.0
    else:
        vx, vy, vz = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
        r = float(np.hypot(np.hypot(vx, vy), abs(vz)))
        in_plane = float(np.hypot(vx, vy) / max(r, 1e-9))
    align = float((feat.pinch_align or {}).get(name, in_plane))
    align = float(np.clip(align, 0.0, 1.0))
    return float(
        w_d * (0.40 + 0.60 * in_plane) * (0.40 + 0.60 * align)
    )


def _o7_pinch_fist_w(feat: PalmTipFeat, calib: PalmTipCalib | None) -> float:
    """Four-finger fist-ness. Prefer palmar pack; curl if pack not captured."""
    if calib is None:
        return 0.0
    if any(_finger_has_pack(calib, n) for n in FINGERS):
        t = _mean_pack_t(feat, calib, FINGERS, reach_t=1.0)
        return _unit_t(t, 0.35, 0.55)
    t = float(np.mean([_finger_curl_t(feat, calib, n) for n in FINGERS]))
    return _unit_t(t, 0.40, 0.60)


def _pinch_others_fist_t(
    feat: PalmTipFeat, calib: PalmTipCalib | None, name: str
) -> float:
    """Fist-ness of the other three fingers (pack if captured, else curl)."""
    if calib is None:
        return 0.0
    others = tuple(n for n in FINGERS if n != name)
    if any(_finger_has_pack(calib, n) for n in others):
        return _mean_pack_t(feat, calib, others, reach_t=1.0)
    ts = [_finger_curl_t(feat, calib, n) for n in others]
    return float(np.mean(ts)) if ts else 0.0


def _pinch_d_gap(feat: PalmTipFeat, name: str) -> float:
    """How much closer this tip is than the next finger (palm widths)."""
    d = float((feat.pinch_d or {}).get(name, 1e9))
    rest = [float((feat.pinch_d or {}).get(n, 1e9)) for n in FINGERS if n != name]
    if not rest:
        return 0.0
    return float(min(rest) - d)


def classify_o7_pinch(
    feat: PalmTipFeat,
    calib: PalmTipCalib | None,
    ws: O6Workspace,
    *,
    sticky: str | None = None,
) -> O7PinchDecision:
    """Pick at most one pinch. Enter is strict; exit is sticky; fist always wins.

    Enter needs all of: that finger's pinch curl, unique VR tip distance, and
    the other three not looking like a fist. Four-finger mean curl kills the
    slot (optical 握拳 never reaches the 0.92 envelope gate). Index can still
    beat 掌边. Sticky only holds a *valid* pinch; fist/isolation drop it.
    """
    near = float(ws.slot_near)
    far = float(ws.slot_far)
    d_enter = near + 0.04
    d_exit = near + 0.22
    fist_w = _o7_pinch_fist_w(feat, calib)
    if calib is not None:
        fist_w = max(fist_w, _fist_blend_w(feat, calib, ws))
    raw: dict[str, float] = {}
    for name in FINGERS:
        geom = _pinch_geom_score(feat, name, near, far)
        p_t = _pinch_curl_t(feat, calib, name)
        others = _pinch_others_fist_t(feat, calib, name)
        gap = _pinch_d_gap(feat, name)
        w_iso = float(np.clip(1.0 - others / 0.62, 0.0, 1.0))
        w_gap = float(np.clip(gap / 0.12, 0.0, 1.0))
        s = geom * p_t * w_iso * (0.25 + 0.75 * w_gap)
        if fist_w >= 0.22 and name in ("ring", "pinky"):
            s = 0.0
        raw[name] = float(s)
    if fist_w >= 0.45:
        return O7PinchDecision(slot=None, scores=raw, w=0.0)
    ranked = dict(raw)
    ranked["index"] = float(ranked["index"] * 1.40)

    def _ok(name: str, *, hold: bool) -> bool:
        if name not in FINGERS:
            return False
        d = float((feat.pinch_d or {}).get(name, 1e9))
        if d > (d_exit if hold else d_enter):
            return False
        if _pinch_d_gap(feat, name) < (0.03 if hold else 0.10):
            return False
        if _pinch_others_fist_t(feat, calib, name) > (0.72 if hold else 0.55):
            return False
        need = 0.12 if hold else 0.30
        if ranked[name] < need:
            return False
        if (not hold) and name in ("ring", "pinky") and fist_w >= 0.22:
            return False
        return True

    order = sorted(FINGERS, key=lambda n: ranked[n], reverse=True)
    best_n = order[0]
    best = ranked[best_n]
    second = ranked[order[1]]
    slot: str | None = None
    if sticky in FINGERS and _ok(sticky, hold=True):
        slot = sticky
    if _ok("index", hold=False) and ranked["index"] >= 0.88 * max(best, 1e-9):
        slot = "index"
    elif slot is None and _ok(best_n, hold=False) and best >= second + 0.08:
        slot = best_n
    w = 0.0
    if slot is not None:
        scale = 0.28 if sticky == slot else 0.40
        w = float(np.clip(ranked[slot] / scale, 0.0, 1.0))
        if sticky == slot and w > 0.0:
            w = max(w, 0.35)
        if w < 0.20:
            slot = None
            w = 0.0
    return O7PinchDecision(slot=slot, scores=raw, w=w)


def _o7_r_span(
    calib: PalmTipCalib | None, ws: O6Workspace
) -> tuple[float, float]:
    """Near/far nest radius for O7 j2. Prefer captured along/pinch nest_r."""
    r_near = float(ws.nest_r_near)
    r_far = float(ws.nest_r_far)
    if calib is not None and calib.along_nest_r is not None:
        r_far = float(calib.along_nest_r)
    pinch_rs: list[float] = []
    if calib is not None:
        for name in FINGERS:
            knot = _pinch_knot(calib, name)
            if knot and knot.get("nest_r") is not None:
                pinch_rs.append(float(knot["nest_r"]))
    if pinch_rs:
        r_near = float(np.median(np.asarray(pinch_rs, dtype=np.float64)))
    if r_near >= r_far - 1e-3:
        return float(ws.nest_r_near), float(ws.nest_r_far)
    return r_near, r_far


def _thumb_j1_curl_hi(ws: O6Workspace, urdf_hi: float) -> float:
    """j1 ceiling for thumb_curl / along (not fist roof or pinch floor)."""
    hi = float(urdf_hi)
    if int(getattr(ws, "dof", 6)) >= 7:
        along = getattr(ws, "along_j1", None)
        if along is not None:
            hi = min(hi, float(along))
        return hi
    if getattr(ws, "fist_j1", None) is not None:
        return float(ws.fist_j1)
    return hi


def _o7_j2_side_w(
    feat: PalmTipFeat, calib: PalmTipCalib | None, ws: O6Workspace
) -> float:
    return _j2_side_park_w(feat, calib, ws)


def _o7_up_w(feat: PalmTipFeat, calib: PalmTipCalib | None) -> float:
    """1 when the thumb stands off the palm (thumb_up). Fades pad facing."""
    if calib is None:
        return 0.0
    up = float(calib.up_thumb_h)
    open_h = float(calib.open_thumb_h)
    if up <= open_h + 1e-4:
        return 0.0
    return _unit_t(float(feat.thumb_h), open_h, up)


def _o7_opp_j2(
    feat: PalmTipFeat, calib: PalmTipCalib | None, ws: O6Workspace
) -> float:
    """O7 curl 对掌: along az → 0; index/middle pinch az → pad (1.2)."""
    if calib is None:
        return 0.0
    xs = [float(calib.along_thumb_az)]
    qs = [0.0]
    pk_i = _pinch_knot(calib, "index")
    if pk_i and pk_i.get("thumb_az") is not None:
        az_i = float(pk_i["thumb_az"])
        # Pico along≈index az; a 3° knot would chatter j2. Classify owns that.
        if abs(az_i - float(calib.along_thumb_az)) >= 0.08:
            xs.append(az_i)
            qs.append(float(ws.j2_index))
    mid_az = float(calib.middle_thumb_az)
    pk_m = _pinch_knot(calib, "middle")
    if pk_m and pk_m.get("thumb_az") is not None:
        mid_az = float(pk_m["thumb_az"])
    xs.append(mid_az)
    qs.append(float(ws.j2_index))
    if not _lat_j2_monotonic(xs, qs):
        xs = [float(calib.along_thumb_az), mid_az]
        qs = [0.0, float(ws.j2_index)]
    return float(
        np.clip(_interp_knots(feat.thumb_az, xs, qs), 0.0, float(ws.j2_index))
    )


def _o7_opp_j2_vec(
    feat: PalmTipFeat, calib: PalmTipCalib | None, ws: O6Workspace
) -> float:
    """veccurl 对掌 from nest_r: far/along → 0; near nest → pad (1.2–1.3)."""
    r_near, r_far = _o7_r_span(calib, ws)
    hi = max(float(ws.j2_index), float(ws.j2_middle))
    j2 = float(
        np.clip(
            _interp_knots(
                float(getattr(feat, "nest_r", 0.0)),
                [r_near, r_far],
                [hi, 0.0],
            ),
            0.0,
            hi,
        )
    )
    j2 *= 1.0 - _o7_up_w(feat, calib)
    if bool(getattr(ws, "attract_enabled", True)):
        d = feat.pinch_d or {}
        near = float(ws.slot_near)
        far = float(ws.slot_far)
        w_tip = 0.0
        if far > 0.0 and near >= 0.0:
            w_tip = max(
                (
                    _lead_w(d.get(n, 1e9), near, far)
                    * _pinch_curl_t(feat, calib, n)
                    for n in FINGERS
                ),
                default=0.0,
            )
        if calib is not None:
            w_tip *= 1.0 - _fist_blend_w(feat, calib, ws)
        j2 = max(j2, float(w_tip) * float(ws.j2_index))
    return float(np.clip(j2, 0.0, hi))


def _thumb_j2_o7_continuous(
    feat: PalmTipFeat, calib: PalmTipCalib | None, ws: O6Workspace
) -> float:
    """O7 j2: slot_t along→index→middle (preferred), else nest_lat; side-park→0."""
    if calib is None:
        return 0.0
    hi = float(ws.j2_middle)
    if _has_slot_calib(calib):
        xs, qs = _j2_slot_knots(calib, ws)
        j2 = float(
            np.clip(
                _interp_knots(float(getattr(feat, "slot_t", 0.0)), xs, qs),
                0.0,
                hi,
            )
        )
    else:
        knots = _j2_lat_knots(calib, ws)
        if knots is not None:
            xs, qs = knots
            j2 = float(
                np.clip(
                    _interp_knots(float(feat.nest_lat), xs, qs),
                    0.0,
                    hi,
                )
            )
        else:
            j2 = _thumb_j2_from_az(feat, calib, ws)
    w_side = _j2_side_park_w(feat, calib, ws)
    j2 = (1.0 - w_side) * j2
    return float(np.clip(j2, 0.0, hi))


def _o7_side_park_w(
    feat: PalmTipFeat, calib: PalmTipCalib | None, ws: O6Workspace
) -> float:
    """1 when thumb should sit beside extended fingers (侧掌 / open)."""
    return float(
        np.clip(
            max(
                _o7_j2_side_w(feat, calib, ws),
                1.0 - _finger_drive_w(feat, calib, ws),
            ),
            0.0,
            1.0,
        )
    )


def _o7_pinch_pad_w(
    feat: PalmTipFeat,
    calib: PalmTipCalib | None,
    ws: O6Workspace,
    slot: str,
    score_w: float,
) -> float:
    """Pinch pad pull fades as slot_t sweeps index → along (侧掌)."""
    w = float(score_w)
    w *= 1.0 - _o7_side_park_w(feat, calib, ws)
    if calib is None or w <= 1e-9:
        return 0.0
    pk = _pinch_knot(calib, slot)
    idx_t = None if not pk else pk.get("slot_t")
    along_t = calib.along_slot_t
    if idx_t is not None and along_t is not None:
        sep = max(abs(float(idx_t) - float(along_t)), 0.12)
        prog = float(np.clip(abs(float(getattr(feat, "slot_t", 0.0)) - float(idx_t)) / sep, 0.0, 1.0))
        w *= 1.0 - prog
    return float(np.clip(w, 0.0, 1.0))


def blend_pad_j2(
    j2_az: float,
    feat: PalmTipFeat,
    ws: O6Workspace,
    calib: PalmTipCalib | None = None,
    *,
    decision: O7PinchDecision | None = None,
) -> float:
    """O7 curl j2: continuous slot_t sweep; soft pinch pad nudge (no mode switch)."""
    del j2_az
    hi = max(float(ws.j2_index), float(ws.j2_middle))
    j2 = _thumb_j2_o7_continuous(feat, calib, ws)
    dec = decision
    if dec is None:
        dec = classify_o7_pinch(feat, calib, ws)
    # With slot_t knots, j2 is already continuous along→index→middle; pad
    # pull from classify fights that and jumps when the slot is released.
    if dec.slot is not None and dec.w > 1e-9 and not _has_slot_calib(calib):
        pad = (
            float(ws.j2_index)
            if dec.slot == "index"
            else float(ws.j2_middle)
        )
        w_pad = _o7_pinch_pad_w(feat, calib, ws, dec.slot, dec.w)
        if w_pad > 1e-6:
            j2 = j2 + w_pad * max(0.0, pad - j2)
    w_fist = 0.0
    if calib is not None:
        w_fist = _fist_blend_w(feat, calib, ws) * _thumb_flex_t(feat, calib)
    if w_fist > 1e-9:
        j2 = max(j2, float(w_fist) * float(ws.j2_index))
    return float(np.clip(j2, 0.0, hi))


def blend_pad_j2_vec(
    j2_az: float,
    feat: PalmTipFeat,
    ws: O6Workspace,
    calib: PalmTipCalib | None = None,
) -> float:
    """veccurl j2 from tip−nest radius. Continuous 0→1.3; no iso/fist max into pad."""
    del j2_az
    return _o7_opp_j2_vec(feat, calib, ws)


def _j3_slot_knots(
    calib: PalmTipCalib | None, ws: O6Workspace
) -> tuple[list[float], list[float]]:
    """along / index / middle ``slot_t`` → {0, j3_index, j3_middle}.

    Ring/pinky collapse on the index–middle PIP axis; do not add them here.
    """
    j_i = float(ws.j3_index)
    j_m = float(ws.j3_middle)
    along = None if calib is None else calib.along_slot_t
    pk_i = None if calib is None else _pinch_knot(calib, "index")
    pk_m = None if calib is None else _pinch_knot(calib, "middle")
    idx_t = None if not pk_i else pk_i.get("slot_t")
    mid_t = None if not pk_m else pk_m.get("slot_t")
    candidates: list[tuple[list[float], list[float]]] = []
    if along is not None and idx_t is not None and mid_t is not None:
        candidates.append(
            ([float(along), float(idx_t), float(mid_t)], [0.0, j_i, j_m])
        )
    if idx_t is not None and mid_t is not None:
        candidates.append(([float(idx_t), float(mid_t)], [j_i, j_m]))
    candidates.append(([-0.5, 0.0, 1.0], [0.0, j_i, j_m]))
    candidates.append(([0.0, 1.0], [j_i, j_m]))
    for xs, qs in candidates:
        if _lat_j2_monotonic(xs, qs):
            return xs, qs
    return [0.0, 1.0], [j_i, j_m]


def _o7_ulnar_w(feat: PalmTipFeat, calib: PalmTipCalib | None) -> float:
    """0 until past middle az; 1 at ring az. slot_t cannot tell ring/pinky."""
    if calib is None:
        return 0.0
    pk_m = _pinch_knot(calib, "middle")
    pk_r = _pinch_knot(calib, "ring")
    if not pk_m or not pk_r:
        return 0.0
    mid = pk_m.get("thumb_az")
    rng = pk_r.get("thumb_az")
    if mid is None or rng is None:
        return 0.0
    return _unit_t(float(feat.thumb_az), float(mid), float(rng))


def _thumb_j3_ulnar_from_az(
    feat: PalmTipFeat, calib: PalmTipCalib, ws: O6Workspace
) -> float:
    """Middle / ring / pinky az → j3 slots. Used once past the middle knot."""
    xs: list[float] = []
    qs: list[float] = []
    slots = (
        ("middle", float(ws.j3_middle)),
        ("ring", float(getattr(ws, "j3_ring", 0.50))),
        ("pinky", float(getattr(ws, "j3_pinky", ws.j3_ulnar))),
    )
    for name, slot in slots:
        pk = _pinch_knot(calib, name)
        if pk and pk.get("thumb_az") is not None:
            xs.append(float(pk["thumb_az"]))
            qs.append(float(slot))
    if len(xs) < 2:
        xs = [float(calib.middle_thumb_az), float(calib.middle_thumb_az) + 0.4]
        qs = [float(ws.j3_middle), float(_j3_hi(ws))]
    return float(np.clip(_interp_knots(feat.thumb_az, xs, qs), 0.0, _j3_hi(ws)))


def _j3_slot_q(ws: O6Workspace, name: str) -> float:
    return {
        "index": float(ws.j3_index),
        "middle": float(ws.j3_middle),
        "ring": float(getattr(ws, "j3_ring", 0.50)),
        "pinky": float(getattr(ws, "j3_pinky", ws.j3_ulnar)),
    }.get(name, float(ws.j3_index))


def blend_slot_j3(
    j3_az: float,
    feat: PalmTipFeat,
    ws: O6Workspace,
    calib: PalmTipCalib | None = None,
    *,
    decision: O7PinchDecision | None = None,
    sweep_hi: float | None = None,
) -> float:
    """O7 curl: j3 follows az (may pass pinky knot); classified pinch nails the slot."""
    cap = max(_j3_hi(ws), float(sweep_hi or 0.0), 1e-9)
    j3 = float(np.clip(j3_az, 0.0, cap))
    if not bool(getattr(ws, "attract_enabled", True)):
        return j3
    dec = decision
    if dec is None:
        dec = classify_o7_pinch(feat, calib, ws)
    if dec.slot is None or dec.w <= 1e-9:
        return j3
    j3_star = _j3_slot_q(ws, dec.slot)
    j3 = (1.0 - dec.w) * j3 + dec.w * float(j3_star)
    return float(np.clip(j3, 0.0, cap))


def blend_slot_j3_vec(
    j3_az: float,
    feat: PalmTipFeat,
    ws: O6Workspace,
    calib: PalmTipCalib | None = None,
) -> float:
    """veccurl: soft pull j3 toward that finger's slot. Isolation is not required."""
    hi = _j3_hi(ws)
    j3 = float(np.clip(j3_az, 0.0, hi))
    if not bool(getattr(ws, "attract_enabled", True)):
        return j3
    near = float(ws.slot_near)
    far = float(ws.slot_far)
    if far <= 0.0 or near < 0.0:
        return j3
    d = feat.pinch_d or {}
    w = {
        n: _lead_w(d.get(n, 1e9), near, far) * _pinch_curl_t(feat, calib, n)
        for n in FINGERS
    }
    w_lead = max(w.values()) if w else 0.0
    if calib is not None:
        w_lead *= 1.0 - _fist_blend_w(feat, calib, ws)
    if w_lead <= 1e-9:
        return j3
    slots = {
        "index": float(ws.j3_index),
        "middle": float(ws.j3_middle),
        "ring": float(getattr(ws, "j3_ring", 0.50)),
        "pinky": float(getattr(ws, "j3_pinky", ws.j3_ulnar)),
    }
    den = sum(w[n] for n in FINGERS)
    if den <= 1e-9:
        j3_star = slots["index"]
    else:
        j3_star = sum(w[n] * slots[n] for n in FINGERS) / den
    j3 = (1.0 - w_lead) * j3 + w_lead * float(j3_star)
    return float(np.clip(j3, 0.0, hi))


def _j3_hi(ws: O6Workspace) -> float:
    return max(
        float(ws.j3_index),
        float(ws.j3_middle),
        float(getattr(ws, "j3_ring", 0.50)),
        float(getattr(ws, "j3_pinky", ws.j3_ulnar)),
        float(ws.j3_ulnar),
        1e-9,
    )


def _thumb_j3_rad(
    feat: PalmTipFeat,
    calib: PalmTipCalib,
    ws: O6Workspace,
    *,
    sweep_hi: float | None = None,
) -> float:
    """O7 curl: along az → 0; pinch az → slots; past pinky continues to URDF hi.

    Pinky ``pinch_q`` j3 is a contact knot, not the travel ceiling.
    """
    cap = max(_j3_hi(ws), float(sweep_hi or 0.0), 1e-9)
    xs = [float(calib.along_thumb_az)]
    qs = [0.0]
    slots = (
        ("index", float(ws.j3_index)),
        ("middle", float(ws.j3_middle)),
        ("ring", float(getattr(ws, "j3_ring", 0.50))),
        ("pinky", float(getattr(ws, "j3_pinky", ws.j3_ulnar))),
    )
    last_az = float(calib.along_thumb_az)
    for name, slot in slots:
        pk = _pinch_knot(calib, name)
        if pk and pk.get("thumb_az") is not None:
            last_az = float(pk["thumb_az"])
            xs.append(last_az)
            qs.append(float(slot))
    if len(xs) < 2:
        xs.append(float(calib.middle_thumb_az))
        qs.append(float(_j3_hi(ws)))
        last_az = float(calib.middle_thumb_az)
    if cap > qs[-1] + 1e-4:
        step = abs(last_az - float(xs[-2])) if len(xs) >= 2 else 0.25
        step = max(step, 0.12)
        sign = 1.0 if last_az >= float(xs[0]) else -1.0
        xs.append(last_az + sign * step)
        qs.append(float(cap))
    return float(np.clip(_interp_knots(feat.thumb_az, xs, qs), 0.0, cap))


def _thumb_j3_rad_vec(feat: PalmTipFeat, calib: PalmTipCalib, ws: O6Workspace) -> float:
    """veccurl: index–middle from slot_t; ring/pinky from az once past middle."""
    xs, qs = _j3_slot_knots(calib, ws)
    j3_slot = float(
        np.clip(
            _interp_knots(float(getattr(feat, "slot_t", 0.0)), xs, qs),
            0.0,
            _j3_hi(ws),
        )
    )
    w_ulnar = _o7_ulnar_w(feat, calib)
    if w_ulnar <= 1e-9:
        return j3_slot
    j3_ulnar = _thumb_j3_ulnar_from_az(feat, calib, ws)
    return float(
        np.clip((1.0 - w_ulnar) * j3_slot + w_ulnar * j3_ulnar, 0.0, _j3_hi(ws))
    )


def _piecewise_q(
    c: float,
    c0: float,
    q0: float,
    c1: float | None,
    q1: float,
    c2: float,
    q2: float,
) -> float:
    """Lerp c0→q0, optional mid c1→q1, c2→q2. Mid skipped if not between ends."""
    if c1 is None or not (min(c0, c2) + 1e-4 < float(c1) < max(c0, c2) - 1e-4):
        return float(q0) + _unit_t(c, c0, c2) * (float(q2) - float(q0))
    if float(c) <= float(c1):
        return float(q0) + _unit_t(c, c0, c1) * (float(q1) - float(q0))
    return float(q1) + _unit_t(c, c1, c2) * (float(q2) - float(q1))


def _pinch_knot(calib: PalmTipCalib, name: str) -> dict[str, float] | None:
    raw = (calib.pinch or {}).get(name)
    if raw is None:
        return None
    if isinstance(raw, (int, float)):
        return {"thumb_az": float(raw)}
    if isinstance(raw, dict):
        return {k: float(v) for k, v in raw.items() if v is not None}
    return None


def _clamp_mid_curl(
    c1: float | None, c0: float, c2: float, *, max_t: float = _PINCH_KNOT_MAX_T
) -> float | None:
    """Keep pinch knot between open and closed. max_t<1 leaves room before fist."""
    if c1 is None:
        return None
    lo = min(float(c0), float(c2))
    hi = max(float(c0), float(c2))
    if not (lo + 1e-4 < float(c1) < hi - 1e-4):
        return None
    cap_t = float(np.clip(max_t, 0.0, 1.0))
    if cap_t >= 1.0 - 1e-9:
        return float(c1)
    if float(c2) >= float(c0):
        cap = float(c0) + cap_t * (float(c2) - float(c0))
        return min(float(c1), cap)
    cap = float(c0) - cap_t * (float(c0) - float(c2))
    return max(float(c1), cap)


def _pinch_finger_curl(
    calib: PalmTipCalib, name: str, *, max_t: float = _PINCH_KNOT_MAX_T
) -> float | None:
    knot = _pinch_knot(calib, name)
    if knot is None or knot.get("finger_curl") is None:
        return None
    return _clamp_mid_curl(
        float(knot["finger_curl"]),
        _finger_open_curl(calib, name),
        _finger_closed_curl(calib, name),
        max_t=max_t,
    )


def _pinch_thumb_curl_mid(
    calib: PalmTipCalib,
    *,
    max_t: float = _PINCH_KNOT_MAX_T,
    lat: float | None = None,
    slot_t: float | None = None,
) -> float | None:
    """Thumb-curl mid knot. Prefer slot_t (index–middle), else nest_lat."""
    fallback: list[float] = []
    slot_xs: list[float] = []
    slot_cs: list[float] = []
    lat_xs: list[float] = []
    lat_cs: list[float] = []
    for name in ("index", "middle"):
        knot = _pinch_knot(calib, name)
        if not knot or knot.get("thumb_curl") is None:
            continue
        fallback.append(float(knot["thumb_curl"]))
        if knot.get("slot_t") is not None:
            slot_xs.append(float(knot["slot_t"]))
            slot_cs.append(float(knot["thumb_curl"]))
        if knot.get("nest_lat") is not None:
            lat_xs.append(float(knot["nest_lat"]))
            lat_cs.append(float(knot["thumb_curl"]))
    raw = None
    if slot_t is not None and len(slot_xs) >= 2:
        raw = float(_interp_knots(float(slot_t), slot_xs, slot_cs))
    elif lat is not None and len(lat_xs) >= 2:
        raw = float(_interp_knots(float(lat), lat_xs, lat_cs))
    elif fallback:
        raw = float(np.median(np.asarray(fallback, dtype=np.float64)))
    if raw is None:
        return None
    return _clamp_mid_curl(
        raw,
        _j1_open_curl(calib),
        float(calib.flex_thumb_curl),
        max_t=max_t,
    )


def _thumb_j1_from_curl(
    feat: PalmTipFeat,
    calib: PalmTipCalib,
    ws: O6Workspace,
    j1_hi: float,
    *,
    pinch_mid: bool,
) -> float:
    """thumb_curl → j1. ``pinch_mid`` inserts close_j1; along skips it."""
    max_t = float(getattr(ws, "pinch_knot_max_t", _PINCH_KNOT_MAX_T))
    c_mid_t = None
    if pinch_mid:
        c_mid_t = _pinch_thumb_curl_mid(
            calib,
            max_t=max_t,
            lat=float(feat.nest_lat),
            slot_t=float(getattr(feat, "slot_t", 0.0)),
        )
    reach_t = float(getattr(ws, "closed_reach_t", 0.88))
    j1_closed = _reach_curl(_j1_open_curl(calib), calib.flex_thumb_curl, reach_t)
    if c_mid_t is not None:
        j1_closed = min(
            float(calib.flex_thumb_curl),
            max(j1_closed, float(c_mid_t) + 1e-3),
        )
    return float(
        np.clip(
            _piecewise_q(
                feat.thumb_curl,
                _j1_open_curl(calib),
                0.0,
                c_mid_t,
                float(ws.close_j1),
                j1_closed,
                float(j1_hi),
            ),
            0.0,
            float(j1_hi),
        )
    )


def _j2_corridor_w(j2: float, center: float, hw: float) -> float:
    span = max(float(hw), 1e-9)
    return float(np.clip(1.0 - abs(float(j2) - float(center)) / span, 0.0, 1.0))


def _j2_along_w(j2: float, ws: O6Workspace) -> float:
    """1 while j2 is parked beside the fingers; fade out into the sweep."""
    along = max(float(getattr(ws, "j2_along", 0.22)), 0.0)
    fade = max(float(getattr(ws, "j2_along_fade", 0.12)), 1e-9)
    x = float(j2)
    if x <= along:
        return 1.0
    return float(np.clip(1.0 - (x - along) / fade, 0.0, 1.0))


def _j2_pinch_slot_w(j2: float, ws: O6Workspace) -> float:
    hw = float(getattr(ws, "j2_pinch_hw", 0.25))
    return max(
        _j2_corridor_w(j2, float(ws.j2_index), hw),
        _j2_corridor_w(j2, float(ws.j2_middle), hw),
    )


def apply_thumb_j1_band(
    q: np.ndarray,
    feat: PalmTipFeat,
    calib: PalmTipCalib,
    ws: O6Workspace,
    j1_hi: float,
    *,
    enabled: bool = False,
) -> np.ndarray:
    """Map j1 after j2 is known. Along: full curl. Else nest_r (+ pinch floor).

    along (j2≈0): full curl → j1_hi (五指同向). Off-along: nest radius depth.
    Pinch corridor / ``pinch_alpha`` raises toward contact piecewise. Fist 扫掌
    uses the envelope cap in the gap.
    """
    if int(getattr(ws, "dof", 6)) >= 7 and not bool(enabled):
        return q
    out = np.asarray(q, dtype=np.float64).copy()
    j2 = float(out[_THUMB_J2])
    hi = float(j1_hi)
    j1_along = _thumb_j1_from_curl(feat, calib, ws, hi, pinch_mid=False)
    j1_pinch = _thumb_j1_from_curl(feat, calib, ws, hi, pinch_mid=True)
    j1_nest = _thumb_j1_from_nest_r(feat, calib, ws, hi)
    w_along = _j2_along_w(j2, ws)
    w_slot = _j2_pinch_slot_w(j2, ws)
    fist_w = _fist_blend_w(feat, calib, ws)
    if fist_w > 1e-9:
        cap = _j1_max_for_j2(
            j2,
            float(ws.fist_env_j1_roof),
            float(ws.fist_env_j2_split),
            hi,
        )
        j1_mid = fist_w * min(max(j1_along, j1_nest), cap)
    else:
        j1_mid = j1_nest
    j1 = w_along * j1_along + (1.0 - w_along) * (
        fist_w * j1_mid + (1.0 - fist_w) * j1_nest
    )
    alpha = pinch_alpha(feat, ws, calib) * w_slot
    if alpha > 1e-9:
        j1 = (1.0 - alpha) * j1 + alpha * max(j1, j1_pinch)
    out[_THUMB_J1] = float(np.clip(j1, 0.0, hi))
    return out


def apply_neighbor_soft(q: np.ndarray, ws: O6Workspace, gain: float) -> np.ndarray:
    """Soft-clamp index↔middle curl gap (reduces fist mid-pose oddities)."""
    g = float(gain)
    if g <= 1e-9:
        return q
    fingers = ws.finger_idx or _FINGER_IDX
    if "index" not in fingers or "middle" not in fingers:
        return q
    out = np.asarray(q, dtype=np.float64).copy()
    i = int(fingers["index"])
    m = int(fingers["middle"])
    diff = float(out[i] - out[m])
    if abs(diff) < 0.20:
        return out
    mid = 0.5 * (float(out[i]) + float(out[m]))
    g = float(np.clip(g, 0.0, 1.0))
    out[i] = (1.0 - g) * float(out[i]) + g * mid
    out[m] = (1.0 - g) * float(out[m]) + g * mid
    return out


def _j1_max_for_j2(j2: float, roof: float, split: float, j1_hi: float) -> float:
    if float(j2) >= float(split):
        return float(roof)
    t = (float(split) - float(j2)) / max(float(split), 1e-9)
    return float(roof) + t * (float(j1_hi) - float(roof))


def four_finger_t(feat: PalmTipFeat, calib: PalmTipCalib) -> float:
    return fist_curl_t(feat, calib, FINGERS)


def fist_curl_t(
    feat: PalmTipFeat,
    calib: PalmTipCalib,
    fingers: tuple[str, ...] | None = None,
) -> float:
    names = fingers or ("index", "middle")
    return min(_finger_curl_t(feat, calib, n) for n in names)


def _fist_blend_w(feat: PalmTipFeat, calib: PalmTipCalib, ws: O6Workspace) -> float:
    """0→1 when fist-env fingers (index/middle) look fisted.

    Uses max(curl, pack): pack alone misses optical fists where tips float;
    curl alone misses tip-pack without full MCP/PIP. Envelope must arm whenever
    index+middle are in fist so j1 cannot punch through the palm at low j2.
    """
    names = ws.fist_env_fingers
    gate = float(ws.fist_env_finger_t)
    enter = max(0.0, gate - 0.20)
    w_curl = _unit_t(fist_curl_t(feat, calib, names), enter, gate)
    w_pack = 0.0
    if any(_finger_has_pack(calib, n) for n in names):
        t = min(
            (
                _finger_pack_t(feat, calib, n, reach_t=1.0)
                for n in names
                if _finger_has_pack(calib, n)
            ),
            default=0.0,
        )
        w_pack = _unit_t(t, 0.45, 0.70)
    return float(max(w_curl, w_pack))


def apply_fist_envelope(
    q: np.ndarray, feat: PalmTipFeat, calib: PalmTipCalib, ws: O6Workspace, j1_hi: float
) -> np.ndarray:
    """Clip j1 onto the roof at the current j2. Does not snap j2.

    w=0 (fingers open): no clip. w=1 (fist): j1 ≤ j1_max(j2). 扫掌 rides
    that curve instead of jumping between park-at-0 and pinch-slot.
    """
    if not ws.fist_env_enabled:
        return q
    w = _fist_blend_w(feat, calib, ws)
    if w <= 1e-9:
        return q
    out = np.asarray(q, dtype=np.float64).copy()
    roof = float(ws.fist_env_j1_roof)
    split = float(ws.fist_env_j2_split)
    j1_hi = float(j1_hi)
    cap = _j1_max_for_j2(float(out[_THUMB_J2]), roof, split, j1_hi)
    lim = (1.0 - w) * j1_hi + w * cap
    out[_THUMB_J1] = min(float(out[_THUMB_J1]), lim)
    return out


def apply_thumb3(
    q: np.ndarray,
    feat: PalmTipFeat,
    calib: PalmTipCalib,
    ws: O6Workspace,
    hi: np.ndarray,
    *,
    solver: str = "curl",
    decision: O7PinchDecision | None = None,
) -> np.ndarray:
    """O7 j3: curl uses az slots; veccurl uses slot_t + ulnar az."""
    if int(getattr(ws, "dof", 6)) < 7:
        return q
    out = np.asarray(q, dtype=np.float64).copy()
    if out.shape[0] < 3:
        return out
    hi = np.asarray(hi, dtype=np.float64)
    if str(solver) == "veccurl":
        j3 = _thumb_j3_rad_vec(feat, calib, ws)
        out[_THUMB_J3] = blend_slot_j3_vec(j3, feat, ws, calib)
    else:
        sweep_hi = float(hi[_THUMB_J3]) if hi.shape[0] > _THUMB_J3 else None
        j3 = _thumb_j3_rad(feat, calib, ws, sweep_hi=sweep_hi)
        out[_THUMB_J3] = blend_slot_j3(
            j3, feat, ws, calib, decision=decision, sweep_hi=sweep_hi
        )
    out[_THUMB_J3] = float(np.clip(out[_THUMB_J3], 0.0, float(hi[_THUMB_J3])))
    return out


def _j1_open_curl(calib: PalmTipCalib) -> float:
    """Human curl that maps to j1=0: open/together/good if more extended."""
    c0 = float(calib.open_thumb_curl)
    if calib.together_thumb_curl is not None:
        c0 = min(c0, float(calib.together_thumb_curl))
    if calib.good_thumb_curl is not None:
        c0 = min(c0, float(calib.good_thumb_curl))
    return c0


def map_palm_tip(
    feat: PalmTipFeat,
    calib: PalmTipCalib,
    limits: np.ndarray,
    ws: O6Workspace | None = None,
) -> np.ndarray:
    ws = ws or O6Workspace()
    lo = limits[:, 0]
    hi = limits[:, 1]
    q = np.zeros(int(lo.shape[0]), dtype=np.float64)
    max_t = float(getattr(ws, "pinch_knot_max_t", _PINCH_KNOT_MAX_T))
    j1_hi = _thumb_j1_curl_hi(ws, float(hi[_THUMB_J1]))
    default_q1 = 0.8  # O6/L6/O7 four-finger contact mid (same curl semantics)
    reach_t = float(getattr(ws, "closed_reach_t", 0.88))
    q[_THUMB_J1] = _thumb_j1_from_curl(
        feat, calib, ws, j1_hi, pinch_mid=True
    )
    q[_THUMB_J2] = _thumb_j2_rad(feat, calib, ws)
    fingers = ws.finger_idx or _FINGER_IDX
    for name, idx in fingers.items():
        c0 = _finger_open_curl(calib, name)
        closed = _finger_closed_curl(calib, name)
        q_mid = float((ws.close_finger or {}).get(name, default_q1))
        q_hi = float(hi[idx])
        c1 = _pinch_finger_curl(calib, name, max_t=max_t)
        c2 = _reach_curl(c0, closed, reach_t)
        if c1 is not None:
            c2 = min(closed, max(c2, float(c1) + 1e-3))
        q_base = _piecewise_q(
            feat.finger_curl[name],
            c0,
            0.0,
            c1,
            q_mid,
            c2,
            q_hi,
        )
        pack_blend = _pack_t_for_map(
            feat, calib, name, ws, reach_t=reach_t, max_t=max_t
        )
        q[idx] = (1.0 - pack_blend) * float(q_base) + pack_blend * q_hi
    return np.clip(q, lo, hi)


def _lead_w(dist: float, near: float, far: float) -> float:
    """Linear proximity in [far, near]. Squared attract_w is too late for j2."""
    span = float(far) - float(near)
    if span <= 1e-9:
        return 1.0 if dist <= near else 0.0
    return float(np.clip((float(far) - float(dist)) / span, 0.0, 1.0))


def _attract_w(dist: float, near: float, far: float) -> float:
    span = float(far) - float(near)
    if span <= 1e-9:
        return 1.0 if dist <= near else 0.0
    w = float(np.clip((float(far) - float(dist)) / span, 0.0, 1.0))
    return w * w


def attract_pinch(
    q: np.ndarray,
    feat: PalmTipFeat,
    ws: O6Workspace,
    calib: PalmTipCalib | None = None,
    *,
    gate_thumb_j1: bool | None = None,
    decision: O7PinchDecision | None = None,
    thumb_j1_only: bool = False,
) -> np.ndarray:
    """Raise joints toward the contact floor only in that finger's pinch band.

    Four fingers always use O6-style proximity × ``_pinch_curl_t`` on every
    model. O7 passes ``decision`` for thumb j1 only (winning pad slot).
    ``thumb_j1_only``: skip four-finger pulls (mp_curl keeps MP21 finger path).
    """
    if not bool(getattr(ws, "attract_enabled", True)):
        return q
    near = float(ws.attract_near)
    far = float(ws.attract_far)
    if far <= 0.0 or near < 0.0:
        return q
    out = np.asarray(q, dtype=np.float64).copy()
    fist_sup = 1.0
    if calib is not None:
        fist_sup = float(np.clip(1.0 - _fist_blend_w(feat, calib, ws), 0.0, 1.0))
    if fist_sup <= 1e-9:
        return q
    w_any = 0.0
    fingers = ws.finger_idx or _FINGER_IDX
    for name in fingers:
        idx = fingers[name]
        w = _attract_w((feat.pinch_d or {}).get(name, 1e9), near, far)
        if calib is not None:
            w *= _pinch_curl_t(feat, calib, name)
        w *= fist_sup
        w_any = max(w_any, w)
        if thumb_j1_only:
            continue
        target = float((ws.close_finger or {}).get(name, 0.8))
        if out[idx] < target:
            out[idx] = out[idx] + w * (target - out[idx])
    j1_t = float(ws.close_j1)
    w_j1 = w_any
    if decision is not None and decision.slot and decision.w > 1e-6:
        slot = str(decision.slot)
        if slot in fingers:
            w_j1 = _attract_w((feat.pinch_d or {}).get(slot, 1e9), near, far)
            if calib is not None:
                w_j1 *= _pinch_curl_t(feat, calib, slot)
            w_j1 *= float(decision.w)
    w_j1 *= fist_sup
    if gate_thumb_j1 is None:
        gate_thumb_j1 = True
    if gate_thumb_j1:
        # Corridor gate: no contact-floor j1 outside pinch j2 slots.
        w_j1 *= _j2_pinch_slot_w(float(out[_THUMB_J2]), ws)
    if out[_THUMB_J1] < j1_t:
        out[_THUMB_J1] = out[_THUMB_J1] + w_j1 * (j1_t - out[_THUMB_J1])
    return out


def snap_pinch(q, feat, latch, ws):
    """Deprecated alias: no latch, same as attract_pinch."""
    return attract_pinch(q, feat, ws), None


def _j2_lock_band(ws: O6Workspace) -> tuple[float, float]:
    lo = getattr(ws, "j2_lock_lo", None)
    hi = getattr(ws, "j2_lock_hi", None)
    default_lo = min(float(ws.j2_index), float(ws.j2_middle))
    default_hi = max(float(ws.j2_index), float(ws.j2_middle))
    lo = default_lo if lo is None else float(lo)
    hi = default_hi if hi is None else float(hi)
    if lo > hi:
        lo, hi = hi, lo
    return lo, hi


def apply_j2_lock(q: np.ndarray, ws: O6Workspace, locked: bool) -> np.ndarray:
    """Clip thumb j2 into the pad-facing band while the lock is on."""
    if not locked:
        return q
    out = np.asarray(q, dtype=np.float64).copy()
    lo, hi = _j2_lock_band(ws)
    out[_THUMB_J2] = float(np.clip(out[_THUMB_J2], lo, hi))
    return out


def _pinch_from_dict(raw: dict | None) -> tuple[bool, dict[str, dict[str, float] | None]]:
    pinch: dict[str, dict[str, float] | None] = {n: None for n in FINGERS}
    if not raw:
        return True, pinch
    skipped = bool(raw.get("skipped", False))
    captured = 0
    for name in FINGERS:
        item = raw.get(name)
        if item is None:
            continue
        if isinstance(item, dict):
            knot = {
                k: float(item[k])
                for k in (
                    "thumb_az",
                    "thumb_curl",
                    "finger_curl",
                    "nest_lat",
                    "slot_t",
                    "nest_r",
                    "pack_h",
                    "pack_r",
                )
                if item.get(k) is not None
            }
            if knot:
                pinch[name] = knot
                captured += 1
        elif isinstance(item, (int, float)):
            pinch[name] = {"thumb_az": float(item)}
            captured += 1
    if captured:
        skipped = False
    return skipped, pinch


def _pack_yaml(pack: dict[str, float] | None, suffix: str) -> dict[str, float]:
    if not pack:
        return {}
    return {f"{n}_{suffix}": float(pack[n]) for n in FINGERS if n in pack}


def calib_to_dict(calib: PalmTipCalib) -> dict:
    pinch_b: dict = {"skipped": bool(calib.pinch_skipped)}
    for name in FINGERS:
        knot = _pinch_knot(calib, name)
        if not knot:
            continue
        pinch_b[name] = dict(knot)
    fist_b = {f"{n}_curl": float(calib.fist_finger_curl[n]) for n in FINGERS}
    fist_b["thumb_curl"] = float(calib.flex_thumb_curl)
    fist_b.update(_pack_yaml(calib.fist_pack_h, "pack_h"))
    fist_b.update(_pack_yaml(calib.fist_pack_r, "pack_r"))
    out = {
        "open": {
            **{f"{n}_curl": float(calib.open_finger_curl[n]) for n in FINGERS},
            "thumb_curl": float(calib.open_thumb_curl),
            "thumb_az": float(calib.open_thumb_az),
            "thumb_h": float(calib.open_thumb_h),
            **_pack_yaml(calib.open_pack_h, "pack_h"),
            **_pack_yaml(calib.open_pack_r, "pack_r"),
        },
        "fist": fist_b,
        "thumb_along": {"thumb_az": float(calib.along_thumb_az)},
        "pinch": pinch_b,
    }
    if calib.together_finger_curl is not None:
        tog_b = {
            f"{n}_curl": float(calib.together_finger_curl[n]) for n in FINGERS
        }
        if calib.together_thumb_curl is not None:
            tog_b["thumb_curl"] = float(calib.together_thumb_curl)
        if calib.together_thumb_az is not None:
            tog_b["thumb_az"] = float(calib.together_thumb_az)
        if calib.together_thumb_h is not None:
            tog_b["thumb_h"] = float(calib.together_thumb_h)
        tog_b.update(_pack_yaml(calib.together_pack_h, "pack_h"))
        tog_b.update(_pack_yaml(calib.together_pack_r, "pack_r"))
        out["together"] = tog_b
    if calib.along_nest_lat is not None:
        out["thumb_along"]["nest_lat"] = float(calib.along_nest_lat)
    if calib.along_slot_t is not None:
        out["thumb_along"]["slot_t"] = float(calib.along_slot_t)
    if calib.along_nest_r is not None:
        out["thumb_along"]["nest_r"] = float(calib.along_nest_r)
    if calib.good_finger_curl is not None:
        good_b = {f"{n}_curl": float(calib.good_finger_curl[n]) for n in FINGERS}
        if calib.good_thumb_curl is not None:
            good_b["thumb_curl"] = float(calib.good_thumb_curl)
        out["good"] = good_b
    if calib.up_thumb_h:
        out["thumb_up"] = {"thumb_h": float(calib.up_thumb_h)}
    if calib.middle_thumb_az != calib.along_thumb_az:
        out["thumb_to_middle"] = {"thumb_az": float(calib.middle_thumb_az)}
    if calib.mirrored_from:
        out["mirrored_from"] = calib.mirrored_from
    return out


def _curl_map(block: dict, suffix: str, seed: dict[str, float]) -> dict[str, float]:
    out = {}
    for n in FINGERS:
        out[n] = float(block.get(f"{n}_{suffix}", block.get(f"{n}_r", seed[n])))
    return out


def calib_from_dict(block: dict) -> PalmTipCalib:
    seed = seed_palm_calib()
    open_b = block.get("open") or {}
    fist_b = block.get("fist") or {}
    tog_b = block.get("together") or {}
    opp_b = block.get("thumb_to_middle") or block.get("thumb_to_pinky") or {}
    skipped, pinch = _pinch_from_dict(block.get("pinch"))
    along_b = block.get("thumb_along") or {}
    up_b = block.get("thumb_up") or {}
    good_b = block.get("good") or {}
    open_az = float(open_b.get("thumb_az", seed.open_thumb_az))
    fist_thumb = fist_b.get("thumb_curl")
    pk_mid = None
    if pinch.get("middle") and pinch["middle"].get("thumb_az") is not None:
        pk_mid = float(pinch["middle"]["thumb_az"])
    return PalmTipCalib(
        open_finger_curl=_curl_map(open_b, "curl", seed.open_finger_curl),
        fist_finger_curl=_curl_map(fist_b, "curl", seed.fist_finger_curl),
        open_thumb_curl=float(
            open_b.get("thumb_curl", open_b.get("thumb_r", seed.open_thumb_curl))
        ),
        flex_thumb_curl=float(
            fist_thumb if fist_thumb is not None else seed.flex_thumb_curl
        ),
        open_thumb_az=open_az,
        along_thumb_az=float(along_b.get("thumb_az", open_az)),
        middle_thumb_az=float(
            opp_b.get("thumb_az", pk_mid if pk_mid is not None else seed.middle_thumb_az)
        ),
        open_thumb_h=float(open_b.get("thumb_h", seed.open_thumb_h)),
        up_thumb_h=float(up_b.get("thumb_h", seed.up_thumb_h)),
        good_finger_curl=(
            _curl_map(good_b, "curl", seed.fist_finger_curl) if good_b else None
        ),
        good_thumb_curl=(
            float(good_b["thumb_curl"]) if good_b.get("thumb_curl") is not None else None
        ),
        together_finger_curl=(
            _curl_map(tog_b, "curl", seed.open_finger_curl) if tog_b else None
        ),
        together_thumb_curl=(
            float(tog_b["thumb_curl"])
            if tog_b.get("thumb_curl") is not None
            else None
        ),
        together_thumb_az=(
            float(tog_b["thumb_az"]) if tog_b.get("thumb_az") is not None else None
        ),
        together_thumb_h=(
            float(tog_b["thumb_h"]) if tog_b.get("thumb_h") is not None else None
        ),
        together_pack_h=_optional_pack_map(tog_b, "pack_h"),
        together_pack_r=_optional_pack_map(tog_b, "pack_r"),
        pinch_skipped=skipped,
        pinch=pinch,
        mirrored_from=(
            str(block.get("mirrored_from")).strip() or None
            if block.get("mirrored_from")
            else None
        ),
        along_nest_lat=(
            None
            if along_b.get("nest_lat") is None
            else float(along_b["nest_lat"])
        ),
        along_slot_t=(
            None
            if along_b.get("slot_t") is None
            else float(along_b["slot_t"])
        ),
        along_nest_r=(
            None
            if along_b.get("nest_r") is None
            else float(along_b["nest_r"])
        ),
        open_pack_h=_optional_pack_map(open_b, "pack_h"),
        open_pack_r=_optional_pack_map(open_b, "pack_r"),
        fist_pack_h=_optional_pack_map(fist_b, "pack_h"),
        fist_pack_r=_optional_pack_map(fist_b, "pack_r"),
    )


def _optional_pack_map(block: dict, suffix: str) -> dict[str, float] | None:
    out: dict[str, float] = {}
    for n in FINGERS:
        key = f"{n}_{suffix}"
        if block.get(key) is None:
            continue
        out[n] = float(block[key])
    return out or None


def dump_palm_calib(
    path: str | Path,
    sides: dict[str, PalmTipCalib],
    *,
    source: str = "xrobotoolkit",
    headset: str = "pico",
    model: str = "o6",
) -> None:
    import yaml

    payload = {
        "schema": SCHEMA,
        "source": source,
        "headset": headset,
        "sides": {
            name: calib_to_dict(sides[name])
            for name in ("left", "right")
            if name in sides
        },
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    key = str(model or "o6").strip().lower() or "o6"
    header = (
        "# Human VR features (curl / pack_h / pack_r / thumb_az / nest_lat / slot_t).\n"
        "# Not raw OpenXR 26, not robot q.\n"
        "# Robot workspace: configs/hands/{o6,l6,o7}.yaml\n"
        "# schema: linkerhand_o6_palm_tip.v5\n"
        "# Poses: open / together(合掌) / fist / good / along / pinch×4.\n"
        f"# Recapture: python -m xr_hand_retarget.calibrate "
        f"--config configs/{key}.yaml\n"
        "# source_side=left|right on the hand index yaml.\n"
    )
    with path.open("w", encoding="utf-8") as f:
        f.write(header)
        yaml.safe_dump(payload, f, sort_keys=False, allow_unicode=True)


def load_palm_calib(path: str | Path) -> dict[str, PalmTipCalib]:
    import yaml

    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    schema = str(raw.get("schema") or "")
    if schema and schema not in (SCHEMA, "linkerhand_o6_palm_tip.v4"):
        print(
            f"[o6] calib {schema!r}: recapture open/together/fist/good/along/pinch×4 "
            "for piecewise curl + pack knots",
            flush=True,
        )
    sides = raw.get("sides") or {}
    out: dict[str, PalmTipCalib] = {}
    for name in ("left", "right"):
        block = sides.get(name)
        if not block:
            out[name] = seed_palm_calib()
            continue
        cal = calib_from_dict(block)
        if schema.endswith(".v1"):
            seed = seed_palm_calib()
            cal.open_finger_curl = dict(seed.open_finger_curl)
            cal.fist_finger_curl = dict(seed.fist_finger_curl)
            cal.open_thumb_curl = seed.open_thumb_curl
            cal.flex_thumb_curl = seed.flex_thumb_curl
        out[name] = cal
    return out


def apply_pose(calib: PalmTipCalib, name: str, feat: PalmTipFeat) -> None:
    if name == "open":
        calib.open_finger_curl = dict(feat.finger_curl)
        calib.open_thumb_curl = feat.thumb_curl
        calib.open_thumb_az = feat.thumb_az
        calib.open_thumb_h = feat.thumb_h
        calib.along_thumb_az = feat.thumb_az
        calib.along_nest_lat = float(feat.nest_lat)
        calib.along_slot_t = float(getattr(feat, "slot_t", 0.0))
        calib.along_nest_r = float(getattr(feat, "nest_r", 0.0))
        calib.open_pack_h = dict(feat.pack_h or {})
        calib.open_pack_r = dict(feat.pack_r or {})
    elif name == "together":
        calib.together_finger_curl = dict(feat.finger_curl)
        calib.together_thumb_curl = float(feat.thumb_curl)
        calib.together_thumb_az = float(feat.thumb_az)
        calib.together_thumb_h = float(feat.thumb_h)
        calib.together_pack_h = dict(feat.pack_h or {})
        calib.together_pack_r = dict(feat.pack_r or {})
    elif name == "fist":
        calib.fist_finger_curl = dict(feat.finger_curl)
        calib.flex_thumb_curl = feat.thumb_curl
        calib.fist_pack_h = dict(feat.pack_h or {})
        calib.fist_pack_r = dict(feat.pack_r or {})
    elif name == "good":
        calib.good_finger_curl = dict(feat.finger_curl)
        calib.good_thumb_curl = feat.thumb_curl
        calib.along_thumb_az = feat.thumb_az
        if calib.along_nest_lat is None:
            calib.along_nest_lat = float(feat.nest_lat)
        if calib.along_slot_t is None:
            calib.along_slot_t = float(getattr(feat, "slot_t", 0.0))
        if calib.along_nest_r is None:
            calib.along_nest_r = float(getattr(feat, "nest_r", 0.0))
    elif name == "thumb_along":
        calib.along_thumb_az = feat.thumb_az
        calib.along_nest_lat = float(feat.nest_lat)
        calib.along_slot_t = float(getattr(feat, "slot_t", 0.0))
        calib.along_nest_r = float(getattr(feat, "nest_r", 0.0))
    elif name == "thumb_flex":
        calib.flex_thumb_curl = feat.thumb_curl
    elif name == "thumb_up":
        calib.up_thumb_h = feat.thumb_h
    elif name in ("thumb_to_pinky", "thumb_to_middle", "thumb_left"):
        calib.middle_thumb_az = feat.thumb_az
    elif name.startswith("pinch_"):
        finger = name[len("pinch_") :]
        if finger not in FINGERS:
            return
        if calib.pinch is None:
            calib.pinch = {n: None for n in FINGERS}
        calib.pinch[finger] = {
            "thumb_az": float(feat.thumb_az),
            "thumb_curl": float(feat.thumb_curl),
            "finger_curl": float(feat.finger_curl[finger]),
            "nest_lat": float(feat.nest_lat),
            "slot_t": float(getattr(feat, "slot_t", 0.0)),
            "nest_r": float(getattr(feat, "nest_r", 0.0)),
            "pack_h": float((feat.pack_h or {}).get(finger, 0.0)),
            "pack_r": float((feat.pack_r or {}).get(finger, 0.0)),
        }
        calib.pinch_skipped = False
        if finger == "middle":
            calib.middle_thumb_az = feat.thumb_az
    else:
        raise ValueError(name)


def evaluate_pose_gate(
    name: str, feat: PalmTipFeat, calib: PalmTipCalib | None = None
) -> tuple[bool, list[str]]:
    """Whether this frame looks like ``name``. Fail = Pico likely mis-tracked."""
    reasons: list[str] = []
    curls = feat.finger_curl or {}
    pr = feat.pack_r or {}
    d = feat.pinch_d or {}
    if name == "open":
        for n in FINGERS:
            if float(curls.get(n, 9.0)) > 0.85:
                reasons.append(f"{n} curl={curls[n]:.2f} 太弯（开掌应伸直）")
            if float(pr.get(n, 0.0)) < 0.40:
                reasons.append(f"{n} pack_r={pr[n]:.2f} 太近巢")
    elif name == "together":
        for n in FINGERS:
            if float(curls.get(n, 9.0)) > 0.85:
                reasons.append(f"{n} curl={curls[n]:.2f} 太弯（合掌应伸直）")
            if float(pr.get(n, 0.0)) < 0.40:
                reasons.append(f"{n} pack_r={pr[n]:.2f} 太近巢")
    elif name == "fist":
        for n in FINGERS:
            if float(curls.get(n, 0.0)) < 0.80:
                reasons.append(f"{n} curl={curls[n]:.2f} 不够弯")
        open_r = None if calib is None else _open_pack_r(calib)
        if open_r:
            for n in FINGERS:
                r0 = open_r.get(n)
                r1 = pr.get(n)
                if r0 is not None and r1 is not None and float(r1) > float(r0) - 0.12:
                    reasons.append(
                        f"{n} pack_r={r1:.2f} 不小于 open {float(r0):.2f} "
                        f"（尖应收到掌心/MCP，不是 PIP 巢）"
                    )
        # Pico often floats fist tips on h; r vs MCP is the gate, h is a hint.
    elif name == "good":
        if float(feat.thumb_curl) > 0.55:
            reasons.append(f"thumb_curl={feat.thumb_curl:.2f} 点赞拇指应伸直")
        for n in FINGERS:
            if float(curls.get(n, 0.0)) < 0.80:
                reasons.append(f"{n} curl={curls[n]:.2f} 不够弯（应与 fist 同程度）")
    elif name == "thumb_along":
        if min(float(curls.get(n, 0.0)) for n in FINGERS) > 0.70:
            reasons.append("四指太弯，along 应伸直")
        if float(getattr(feat, "nest_r", 0.0)) < 0.55:
            reasons.append(f"nest_r={feat.nest_r:.2f} 拇指太靠近巢")
    elif name.startswith("pinch_"):
        finger = name[len("pinch_") :]
        if finger in FINGERS:
            d_self = float(d.get(finger, 9.0))
            if d_self > 0.50:
                reasons.append(f"{finger} pinch_d={d_self:.2f} 两尖不够近")
            c_self = float(curls.get(finger, 0.0))
            if calib is not None:
                c0 = _finger_open_curl(calib, finger)
                if c_self < c0 + 0.12:
                    reasons.append(
                        f"{finger} curl={c_self:.2f} ≤ 开掌 {c0:.2f}（没在捏）"
                    )
            slot = float(getattr(feat, "slot_t", 0.0))
            if finger == "index" and slot > 0.55:
                reasons.append(f"slot_t={slot:.2f} 偏中指/尺侧，不像食指捏合")
            if finger in ("middle", "ring", "pinky") and slot < -0.25:
                reasons.append(f"slot_t={slot:.2f} 偏食指")
    return (not reasons), reasons


def format_feat(feat: PalmTipFeat) -> str:
    parts: list[str] = []
    chains = feat.finger_chain or {}
    for n in FINGERS:
        c = float(feat.finger_curl[n])
        ch = chains.get(n)
        if ch is not None:
            mcp, pip, dip = ch
            parts.append(f"{n[0]}={c:.2f}({mcp:.2f}/{pip:.2f}/{dip:.2f})")
        else:
            parts.append(f"{n[0]}={c:.2f}")
    fr = " ".join(parts)
    d = feat.pinch_d or {}
    pr = feat.pack_r or {}
    ph = feat.pack_h or {}
    pk = " ".join(
        f"{n[0]}={pr.get(n, 0):.2f}/{ph.get(n, 0):.2f}" for n in FINGERS
    )
    return (
        f"curl[{fr}] thumb={feat.thumb_curl:.2f} az={feat.thumb_az:.2f} "
        f"h={feat.thumb_h:.2f} r={getattr(feat, 'nest_r', 0.0):.2f} "
        f"lat={feat.nest_lat:.2f} slot={getattr(feat, 'slot_t', 0.0):.2f} "
        f"pack[r/h {pk}] "
        f"d_i={d.get('index', 0):.2f} d_m={d.get('middle', 0):.2f}"
    )


class PalmTipRetargeter:
    def __init__(
        self, side: str, gains, calib: PalmTipCalib | None = None, *, solver: str = "curl"
    ):
        if side not in ("left", "right"):
            raise ValueError(f"side must be left|right, got {side!r}")
        self.side = side
        self.gains = gains
        self.workspace = workspace_from_gains(gains)
        model = str(getattr(gains, "model", "o6") or "o6").strip().lower()
        if model not in ("o6", "l6", "o7"):
            model = "o6"
        limits, _vel = load_linker_limits(side, model=model, urdf_path=gains.urdf_path)
        self.limits = limits[: int(self.workspace.dof)]
        self.model = model
        raw_solver = str(solver or "curl").strip().lower()
        dof = int(self.workspace.dof)
        if raw_solver in ("veccurl", "vcurl"):
            if dof >= 7:
                raw_solver = "veccurl"
            else:
                print(
                    f"[{side}] veccurl is O7; using nest on {model}",
                    flush=True,
                )
                raw_solver = "nest"
        elif raw_solver in ("nest", "keyvec", "nest_vec"):
            if dof >= 7:
                print(
                    f"[{side}] nest is O6/L6 only; O7 curl (use veccurl for tip−nest)",
                    flush=True,
                )
                raw_solver = "curl"
            else:
                raw_solver = "nest"
        else:
            raw_solver = "curl"
        self.solver = raw_solver
        self._nest_mode = ""
        if calib is not None:
            self.calib = calib
        elif gains.calib_path is not None and Path(gains.calib_path).is_file():
            loaded = load_palm_calib(gains.calib_path)
            source = str(getattr(gains, "calib_source_side", "both") or "both")
            az_sign = float(getattr(gains, "calib_mirror_az", 1.0) or 1.0)
            self.calib, origin = resolve_palm_side(
                loaded, side, source_side=source, az_sign=az_sign
            )
            extra = f" from={origin}" if origin != side else ""
            j3 = ""
            if int(self.workspace.dof) >= 7:
                j3 = (
                    f" j3={self.workspace.j3_index:.2f}/"
                    f"{self.workspace.j3_middle:.2f}/"
                    f"{self.workspace.j3_ring:.2f}/"
                    f"{self.workspace.j3_pinky:.2f}"
                )
            attract_tag = (
                "on"
                if bool(getattr(self.workspace, "attract_enabled", True))
                else "off"
            )
            if int(self.workspace.dof) < 7:
                j2_src = "nest" if self.solver == "curl" else "slot"
            elif self.solver == "veccurl":
                j2_src = "vec"
            else:
                j2_src = "pad"
            mids = []
            max_t = float(getattr(self.workspace, "pinch_knot_max_t", _PINCH_KNOT_MAX_T))
            for n in FINGERS:
                c1 = _pinch_finger_curl(self.calib, n, max_t=max_t)
                q1 = float((self.workspace.close_finger or {}).get(n, 0.0))
                if c1 is not None and q1 > 0.0:
                    mids.append(f"{n[0]}:{c1:.2f}→{q1:.2f}")
            pinch_tag = ",".join(mids) if mids else "off"
            pack_n = sum(1 for n in FINGERS if _finger_has_pack(self.calib, n))
            pack_tag = f"{pack_n}/4" if pack_n else "off"
            print(
                f"[{side}] {self.model} palm-tip calib={gains.calib_path}{extra} "
                f"solver={self.solver} "
                f"j2={j2_src} "
                f"pinch={pinch_tag} "
                f"pack={pack_tag} "
                f"j2_index={self.workspace.j2_index:.2f} "
                f"j2_middle={self.workspace.j2_middle:.2f}{j3} "
                f"attract={attract_tag} "
                f"{self.workspace.attract_near:.2f}/{self.workspace.attract_far:.2f} "
                f"slot={self.workspace.slot_near:.2f}/{self.workspace.slot_far:.2f} "
                f"along={float(getattr(self.workspace, 'j2_along', 0.22)):.2f} "
                f"closed_reach={float(getattr(self.workspace, 'closed_reach_t', 1.0)):.2f}",
                flush=True,
            )
        else:
            if gains.calib_path is not None:
                print(
                    f"[{side}] {self.model} palm-tip calib missing ({gains.calib_path}); using seed",
                    flush=True,
                )
            self.calib = seed_palm_calib()
        self._prev: np.ndarray | None = None
        self._j2_lock = False
        self._pinch_slot: str | None = None
        self._pinch_fk_proj = None
        self._pinch_fk_failed = False

    def _get_pinch_fk(self):
        if self._pinch_fk_failed:
            return None
        if self._pinch_fk_proj is not None:
            return self._pinch_fk_proj
        if not bool(getattr(self.workspace, "pinch_fk_project_enabled", False)):
            return None
        try:
            from xr_hand_retarget.algorithms.pinch_fk import (
                PinchFkProjector,
                pinch_fk_config_from_workspace,
            )
            from xr_hand_retarget.kinematics import make_linker_fk

            fk = make_linker_fk(
                self.side, model=self.model, urdf_path=self.gains.urdf_path
            )
            self._pinch_fk_proj = PinchFkProjector(
                fk,
                self.limits,
                joint_names=None,
                config=pinch_fk_config_from_workspace(self.workspace),
            )
        except Exception as exc:
            print(
                f"[{self.side}] {self.model} pinch FK project unavailable: {exc}",
                flush=True,
            )
            self._pinch_fk_failed = True
            return None
        return self._pinch_fk_proj

    def _apply_o7_pinch_planning(
        self,
        q: np.ndarray,
        *,
        slot: str | None,
        thumb_only: bool = False,
    ) -> np.ndarray:
        if int(self.workspace.dof) < 7:
            return q
        if not (
            bool(getattr(self.workspace, "pinch_ceiling_enabled", False))
            or bool(getattr(self.workspace, "pinch_fk_project_enabled", False))
        ):
            return q
        from xr_hand_retarget.algorithms.pinch_fk import apply_pinch_planning

        return apply_pinch_planning(
            q,
            self.workspace,
            self._get_pinch_fk(),
            slot=slot,
            q_prev=self._prev,
            limits=self.limits,
            thumb_only=thumb_only,
        )

    def reset(self) -> None:
        self._prev = None
        self._pinch_slot = None

    def set_j2_lock(self, on: bool) -> bool:
        on = bool(on)
        if on != self._j2_lock:
            self._j2_lock = on
            lo, hi = _j2_lock_band(self.workspace)
            print(
                f"[{self.side}] j2_lock={'on' if on else 'off'} "
                f"band=[{lo:.2f},{hi:.2f}]",
                flush=True,
            )
        return self._j2_lock

    def toggle_j2_lock(self) -> bool:
        return self.set_j2_lock(not self._j2_lock)

    def retarget(self, joints26: np.ndarray) -> np.ndarray:
        if self.solver == "nest":
            from xr_hand_retarget.algorithms.nest import retarget_nest

            q, decision = retarget_nest(
                joints26,
                self.side,
                self.calib,
                self.limits,
                self.workspace,
                j2_lock=self._j2_lock,
            )
            if decision.mode != self._nest_mode:
                self._nest_mode = decision.mode
                print(
                    f"[{self.side}] nest mode={decision.mode}"
                    f"{'' if decision.slot == 'none' else '/' + decision.slot} "
                    f"fist_w={decision.fist_w:.2f} pinch={decision.w_pinch:.2f} "
                    f"r={decision.nest_r:.2f} "
                    f"lat={getattr(decision, 'nest_lat', 0.0):.2f} "
                    f"j1={q[_THUMB_J1]:.2f} j2={q[_THUMB_J2]:.2f}",
                    flush=True,
                )
            q = np.clip(q, self.limits[:, 0], self.limits[:, 1])
            alpha = float(np.clip(self.gains.output_alpha, 1e-3, 1.0))
            if self._prev is None or alpha >= 1.0 - 1e-12:
                self._prev = q
                return q
            blended = (1.0 - alpha) * self._prev + alpha * q
            self._prev = blended
            return blended
        feat = extract_palm_tip(joints26, self.side)
        q = map_palm_tip(feat, self.calib, self.limits, self.workspace)
        q = np.asarray(q, dtype=np.float64).copy()
        j2_az = float(q[_THUMB_J2])
        use_vec = self.solver == "veccurl"
        o7_curl = int(self.workspace.dof) >= 7 and not use_vec
        dec = None
        if o7_curl:
            dec = classify_o7_pinch(
                feat, self.calib, self.workspace, sticky=self._pinch_slot
            )
            self._pinch_slot = dec.slot
        if int(self.workspace.dof) >= 7:
            if use_vec:
                q[_THUMB_J2] = blend_pad_j2_vec(
                    j2_az, feat, self.workspace, self.calib
                )
            else:
                q[_THUMB_J2] = blend_pad_j2(
                    j2_az, feat, self.workspace, self.calib, decision=dec
                )
        else:
            q[_THUMB_J2] = blend_pinch_j2(j2_az, feat, self.workspace, self.calib)
        q = apply_j2_lock(q, self.workspace, self._j2_lock)
        q = apply_thumb3(
            q,
            feat,
            self.calib,
            self.workspace,
            self.limits[:, 1],
            solver=self.solver,
            decision=dec,
        )
        j1_hi = _thumb_j1_curl_hi(
            self.workspace, float(self.limits[_THUMB_J1, 1])
        )
        q = apply_thumb_j1_band(
            q, feat, self.calib, self.workspace, j1_hi, enabled=True
        )
        q = attract_pinch(
            q,
            feat,
            self.workspace,
            self.calib,
            gate_thumb_j1=True,
            decision=dec,
        )
        q = apply_fist_envelope(
            q, feat, self.calib, self.workspace, float(self.limits[_THUMB_J1, 1])
        )
        if o7_curl:
            q = self._apply_o7_pinch_planning(q, slot=dec.slot if dec else None)
        soft = float(getattr(self.gains, "neighbor_soft", 0.0) or 0.0)
        q = apply_neighbor_soft(q, self.workspace, soft)
        q = np.clip(q, self.limits[:, 0], self.limits[:, 1])
        blended = self._blend_output(q)
        if blended is q or self._prev is None:
            return blended
        blended = apply_j2_lock(blended, self.workspace, self._j2_lock)
        blended = apply_thumb_j1_band(
            blended, feat, self.calib, self.workspace, j1_hi, enabled=True
        )
        # j1_band / lerp can re-raise j1; keep palm roof when index+middle fist.
        blended = apply_fist_envelope(
            blended,
            feat,
            self.calib,
            self.workspace,
            float(self.limits[_THUMB_J1, 1]),
        )
        if o7_curl:
            blended = attract_pinch(
                blended,
                feat,
                self.calib,
                self.workspace,
                gate_thumb_j1=True,
                decision=dec,
            )
            blended = self._apply_o7_pinch_planning(
                blended, slot=dec.slot if dec else None
            )
        blended = np.clip(blended, self.limits[:, 0], self.limits[:, 1])
        self._prev = blended
        return blended

    def _blend_output(self, q: np.ndarray) -> np.ndarray:
        """Layered low-pass: thumb_alpha / finger_alpha (fallback output_alpha)."""
        q = np.asarray(q, dtype=np.float64)
        base = float(np.clip(self.gains.output_alpha, 1e-3, 1.0))
        ta_raw = getattr(self.gains, "thumb_alpha", None)
        fa_raw = getattr(self.gains, "finger_alpha", None)
        ta = float(np.clip(base if ta_raw is None else float(ta_raw), 1e-3, 1.0))
        fa = float(np.clip(base if fa_raw is None else float(fa_raw), 1e-3, 1.0))
        n_thumb = 3 if int(self.workspace.dof) >= 7 else 2
        if self._prev is None or (ta >= 1.0 - 1e-12 and fa >= 1.0 - 1e-12):
            self._prev = q.copy()
            return q
        prev = self._prev
        out = q.copy()
        out[:n_thumb] = (1.0 - ta) * prev[:n_thumb] + ta * q[:n_thumb]
        out[n_thumb:] = (1.0 - fa) * prev[n_thumb:] + fa * q[n_thumb:]
        self._prev = out
        return out
