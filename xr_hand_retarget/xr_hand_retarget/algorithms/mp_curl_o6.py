"""VR26 → MediaPipe21 → LinkerHand O6/L6/O7 curl (method: mp_curl).

Pipeline: OpenXR 26 → MediaPipe 21 bone angles → curl-style knot lerp (same
family as PalmTip ``method: curl``, different features).

Human knots in ``mp_curl.yaml`` (open / fist thumb-in-palm / thumb_along / pinch).
Robot endpoints from RViz ``hands/*.yaml`` ``pinch_q`` + fist envelope roof.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from xr_hand_retarget.algorithms.curl_hand2 import (
    CurlFeatures2,
    SideCurlCalib2,
    load_calib_file,
    seed_side_calib,
    side_calib_to_dict,
)
from xr_hand_retarget.algorithms.curl_o6 import (
    _FINGER_IDX_6,
    _FINGER_IDX_7,
    _THUMB3,
    _THUMB_FLEX,
    _THUMB_YAW,
    _weighted_finger_t,
    _weighted_thumb_t,
    pinch_q_from_urdf,
)
from xr_hand_retarget.algorithms.curl_xhand1 import _interp_knots, _unit_t
from xr_hand_retarget.kinematics import load_linker_limits, make_linker_fk
from xr_hand_retarget.landmarks import openxr26_to_mediapipe21

FINGERS = ("index", "middle", "ring", "pinky")
SCHEMA = "linkerhand_mp_curl.v3"
_DEFAULT_PINCH_T_MID = 0.92
_DEFAULT_J1_ROOF = 0.44

_PACKAGE_ROOT = Path(__file__).resolve().parents[2]
# Shared human MP21 curl knots for all Linker hands (o6/l6/o7).
_DEFAULT_CALIB = _PACKAGE_ROOT / "configs" / "calib" / "mp_curl.yaml"

# MediaPipe 21 indices (Wuji standard hand).
_MP_WRIST = 0
_MP_THUMB_CMC = 1
_MP_THUMB_MCP = 2
_MP_THUMB_IP = 3
_MP_THUMB_TIP = 4
# MCP, PIP, DIP, TIP per finger
_MP_FINGER = {
    "index": (5, 6, 7, 8),
    "middle": (9, 10, 11, 12),
    "ring": (13, 14, 15, 16),
    "pinky": (17, 18, 19, 20),
}

_CURL_SCHEMAS = frozenset(
    {
        SCHEMA,
        "linkerhand_mp_curl.v2",
        "linkerhand_mp_curl.v1",
        "linkerhand_curl_calib.v1",
        "wuji_hand2_curl_calib.v2",
        "wuji_hand2_curl_calib.v1",
    }
)


def _pos(lm21: np.ndarray, idx: int) -> np.ndarray:
    return np.asarray(lm21[idx, :3], dtype=np.float64)


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


def _palm_axes_mp21(
    lm21: np.ndarray, side: str
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Palm frame from wrist / index-MCP / middle-MCP (MediaPipe 0, 5, 9)."""
    wrist = _pos(lm21, _MP_WRIST)
    middle = _pos(lm21, 9) - wrist
    index = _pos(lm21, 5) - wrist
    pinky = _pos(lm21, 17) - wrist
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


def _thumb_lat_mp21(lm21: np.ndarray, side: str) -> float:
    wrist, x, y, z = _palm_axes_mp21(lm21, side)
    tip = _pos(lm21, _MP_THUMB_TIP)
    rel = tip - wrist
    rel_p = rel - z * float(np.dot(rel, z))
    return float(np.dot(rel_p, x))


def _finger_span_mp21(lm21: np.ndarray) -> float:
    idx_p = _pos(lm21, 5)
    pky_p = _pos(lm21, 17)
    width = max(float(np.linalg.norm(idx_p - pky_p)), 1e-3)
    idx_t = _pos(lm21, 8)
    pky_t = _pos(lm21, 20)
    return float(np.linalg.norm(idx_t - pky_t) / width)


def finger_chain_flexions_mp21(
    lm21: np.ndarray, finger: str
) -> tuple[float, float, float]:
    """MCP / PIP / DIP on MediaPipe 21 (MCP uses wrist as proximal)."""
    key = finger.lower()
    if key not in _MP_FINGER:
        raise ValueError(f"unknown finger {finger!r}")
    mcp_i, pip_i, dip_i, tip_i = _MP_FINGER[key]
    wrist = _pos(lm21, _MP_WRIST)
    mcp = _pos(lm21, mcp_i)
    pip = _pos(lm21, pip_i)
    dip = _pos(lm21, dip_i)
    tip = _pos(lm21, tip_i)
    return (
        _flexion(wrist, mcp, pip),
        _flexion(mcp, pip, dip),
        _flexion(pip, dip, tip),
    )


def thumb_chain_flexion_mp21(lm21: np.ndarray) -> tuple[float, float]:
    """Thumb MCP (CMC–MCP–IP) and IP (MCP–IP–TIP)."""
    cmc = _pos(lm21, _MP_THUMB_CMC)
    mcp = _pos(lm21, _MP_THUMB_MCP)
    ip = _pos(lm21, _MP_THUMB_IP)
    tip = _pos(lm21, _MP_THUMB_TIP)
    return _flexion(cmc, mcp, ip), _flexion(mcp, ip, tip)


def extract_curl_features_mp21(lm21: np.ndarray, side: str) -> CurlFeatures2:
    """CurlFeatures2 from MediaPipe-style (21, 3) xyz."""
    arr = np.asarray(lm21, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[0] < 21:
        raise ValueError(f"expected (21, ≥3) MediaPipe landmarks, got {arr.shape}")
    mcp: dict[str, float] = {}
    pip: dict[str, float] = {}
    dip: dict[str, float] = {}
    for name in FINGERS:
        mcp[name], pip[name], dip[name] = finger_chain_flexions_mp21(arr, name)
    t_mcp, t_ip = thumb_chain_flexion_mp21(arr)
    return CurlFeatures2(
        mcp=mcp,
        pip=pip,
        dip=dip,
        thumb_mcp=t_mcp,
        thumb_ip=t_ip,
        thumb_lat=_thumb_lat_mp21(arr, side),
        finger_span=_finger_span_mp21(arr),
    )


def extract_curl_features_mp21_from26(joints26: np.ndarray, side: str) -> CurlFeatures2:
    return extract_curl_features_mp21(openxr26_to_mediapipe21(joints26), side)


def feat2_to_dict(feat: CurlFeatures2) -> dict[str, Any]:
    """Serialize full CurlFeatures2 for multi-take / reuse."""
    out: dict[str, Any] = {
        "thumb_mcp": float(feat.thumb_mcp),
        "thumb_ip": float(feat.thumb_ip),
        "thumb_lat": float(feat.thumb_lat),
        "finger_span": float(feat.finger_span),
    }
    for name in FINGERS:
        out[f"{name}_mcp"] = float(feat.mcp[name])
        out[f"{name}_pip"] = float(feat.pip[name])
        out[f"{name}_dip"] = float(feat.dip[name])
    return out


def feat2_from_dict(raw: dict[str, Any] | None) -> CurlFeatures2 | None:
    if not raw:
        return None
    return CurlFeatures2(
        mcp={n: float(raw.get(f"{n}_mcp", 0.0)) for n in FINGERS},
        pip={n: float(raw.get(f"{n}_pip", 0.0)) for n in FINGERS},
        dip={n: float(raw.get(f"{n}_dip", 0.0)) for n in FINGERS},
        thumb_mcp=float(raw.get("thumb_mcp", 0.0)),
        thumb_ip=float(raw.get("thumb_ip", 0.0)),
        thumb_lat=float(raw.get("thumb_lat", 0.0)),
        finger_span=float(raw.get("finger_span", 0.0)),
    )


def mp21_to_list(mp21: np.ndarray) -> list[list[float]]:
    arr = np.asarray(mp21, dtype=np.float64).reshape(21, 3)
    return [[float(arr[i, 0]), float(arr[i, 1]), float(arr[i, 2])] for i in range(21)]


def _is_curl_style_calib(path: Path) -> bool:
    import yaml

    try:
        with path.open("r", encoding="utf-8") as f:
            raw: dict[str, Any] = yaml.safe_load(f) or {}
    except Exception:
        return False
    schema = str(raw.get("schema") or "").strip()
    if schema in _CURL_SCHEMAS:
        return True
    if "palm_tip" in schema:
        return False
    sides = raw.get("sides") or {}
    for block in sides.values():
        open_b = (block or {}).get("open") or {}
        if "index_mcp" in open_b:
            return True
    return False


def resolve_mp_curl_calib(gains) -> Path | None:
    """Prefer shared ``mp_curl.yaml``; skip palm_tip schema files.

    Human MP21 curl knots are person→standard-hand; O6/L6/O7 share one file.
    Robot-specific ``*_mp_curl.yaml`` remains a legacy fallback.
    """
    model = str(getattr(gains, "model", None) or "o6").strip().lower() or "o6"
    candidates: list[Path] = []
    raw = getattr(gains, "calib_path", None)
    if raw is not None:
        candidates.append(Path(raw))
    candidates.append(_DEFAULT_CALIB)
    candidates.append(_PACKAGE_ROOT / "configs" / "calib" / f"{model}_mp_curl.yaml")
    if model != "o6":
        candidates.append(_PACKAGE_ROOT / "configs" / "calib" / "o6_mp_curl.yaml")
    seen: set[Path] = set()
    for path in candidates:
        key = path.resolve() if path.exists() else path
        if key in seen:
            continue
        seen.add(key)
        if path.is_file() and _is_curl_style_calib(path):
            return path
    return None


def dump_calib_file(
    path: str | Path,
    sides: dict[str, SideCurlCalib2],
    *,
    source: str = "xrobotoolkit",
    headset: str = "pico",
    takes: dict[str, dict[str, list[dict[str, Any]]]] | None = None,
) -> None:
    """Write shared mp_curl calib.

    ``takes[side][pose]`` keeps every capture group (full features + optional
    mp21) so sessions can be re-aggregated without re-wearing the headset.
    """
    import yaml

    side_blocks: dict[str, Any] = {}
    for name in ("left", "right"):
        if name not in sides:
            continue
        block = side_calib_to_dict(sides[name])
        if takes and name in takes and takes[name]:
            block["takes"] = takes[name]
        side_blocks[name] = block
    payload = {
        "schema": SCHEMA,
        "source": source,
        "headset": headset,
        "shared": True,
        "hands": ["o6", "l6", "o7"],
        "sides": side_blocks,
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# Shared LinkerHand mp_curl calibration (OpenXR 26 → MediaPipe 21).\n"
        f"# schema: {SCHEMA}\n"
        "# One human file for o6/l6/o7 (robot q limits stay in hands/*.yaml).\n"
        "# Runtime: VR26→MP21 curl lerp; j1 open→pinch_q→roof; lat gate for pinch.\n"
        "# Pose knots = median of takes[]; fist stores thumb mcp/ip (curl hi).\n"
        "# Recapture:\n"
        "#   python -m xr_hand_retarget.calibrate --config configs/o6.yaml "
        "--method mp_curl\n"
    )
    with path.open("w", encoding="utf-8") as f:
        f.write(header)
        yaml.safe_dump(payload, f, sort_keys=False, allow_unicode=True)


def _joint_from_pinch_block(raw: Any, *keys: str) -> float | None:
    if not isinstance(raw, dict):
        if raw is None:
            return None
        try:
            return float(raw)
        except (TypeError, ValueError):
            return None
    for k in keys:
        if k in raw and raw[k] is not None:
            return float(raw[k])
    return None


def robot_pinch_from_gains(gains) -> dict[str, Any]:
    """RViz T_robot pinch endpoints from ``hands/*.yaml`` (via O6CurlGains)."""
    t_mid = getattr(gains, "pinch_knot_max_t", None)
    t_mid = float(t_mid) if t_mid is not None else _DEFAULT_PINCH_T_MID
    close_finger: dict[str, float] = {}
    close_j1_vals: list[float] = []
    for name, attr in (
        ("index", "pinch_index_q"),
        ("middle", "pinch_middle_q"),
        ("ring", "pinch_ring_q"),
        ("pinky", "pinch_pinky_q"),
    ):
        block = getattr(gains, attr, None)
        fq = _joint_from_pinch_block(
            block, f"{name}_joint", "finger", "q", name
        )
        if fq is not None and fq > 0.0:
            close_finger[name] = float(fq)
        tj1 = _joint_from_pinch_block(block, "thumb_joint1", "thumb_j1", "j1")
        if tj1 is not None and tj1 > 0.0:
            close_j1_vals.append(float(tj1))
    close_j1 = float(np.median(close_j1_vals)) if close_j1_vals else 0.0
    j1_roof = float(getattr(gains, "fist_env_j1_roof", _DEFAULT_J1_ROOF))
    return {
        "t_mid": float(np.clip(t_mid, 0.05, 0.99)),
        "close_j1": close_j1,
        "j1_roof": j1_roof,
        "close_finger": close_finger,
        "j2_index": float(getattr(gains, "thumb_j2_index", 1.0)),
        "j2_middle": float(getattr(gains, "thumb_j2_middle", 1.3)),
    }


def _fit_open_pinch_fist(t: float, q_mid: float, q_hi: float, t_mid: float) -> float:
    """Polyline on unit curl t: 0→0, t_mid→q_mid, 1→q_hi. No mid → linear to hi."""
    t = float(np.clip(t, 0.0, 1.0))
    hi = max(float(q_hi), 0.0)
    mid = float(q_mid)
    if mid <= 1e-9:
        return t * hi
    return float(
        np.clip(
            _interp_knots(t, [0.0, float(t_mid), 1.0], [0.0, mid, hi]),
            0.0,
            hi,
        )
    )


_PINCH_SIGMA_MIN = 0.005
_PINCH_SIGMA_MAX = 0.012
_PINCH_FALL_K = 1.5  # weight → 0 at K*sigma
_PINCH_WIN_MARGIN = 0.12  # best−second lat weight to fully own j1
_PINCH_FINGER_CURL_ON = 0.35  # finger curl t must exceed this to take contact


def _elevate_thumb_hi_from_pinch(calib: SideCurlCalib2) -> None:
    """Raise thumb curl hi knot from pinch flex (contact is often deeper than fist)."""
    if calib.pinch_skipped or not calib.pinch:
        return
    mcp_vals: list[float] = []
    ip_vals: list[float] = []
    for knot in calib.pinch.values():
        if knot is None:
            continue
        if knot.thumb_mcp is not None:
            mcp_vals.append(float(knot.thumb_mcp))
        if knot.thumb_ip is not None:
            ip_vals.append(float(knot.thumb_ip))
    if mcp_vals:
        calib.thumb_flex_mcp = max(float(calib.thumb_flex_mcp), float(np.max(mcp_vals)))
    if ip_vals:
        calib.thumb_flex_ip = max(float(calib.thumb_flex_ip), float(np.max(ip_vals)))


def _warn_degenerate_thumb_span(side: str, calib: SideCurlCalib2) -> None:
    """Open≈fist thumb span makes _unit_t clip early (same as Wuji curl warning)."""
    t_mcp = float(calib.thumb_flex_mcp) - float(calib.open_thumb_mcp)
    t_ip = float(calib.thumb_flex_ip) - float(calib.open_thumb_ip)
    bad: list[str] = []
    if t_mcp < 0.15:
        bad.append(f"thumb_mcp Δ={t_mcp:.3f}")
    if t_ip < 0.15:
        bad.append(f"thumb_ip Δ={t_ip:.3f}")
    if not bad:
        return
    print(
        f"[{side}] mp_curl thumb span narrow ({', '.join(bad)}). "
        "Re-capture fist (thumb in palm) + pinch:\n"
        "  python -m xr_hand_retarget.calibrate --config configs/o6.yaml "
        "--method mp_curl",
        flush=True,
    )


def normalize_mp_curl_calib(
    calib: SideCurlCalib2,
    side: str,
    raw_block: dict[str, Any] | None = None,
) -> None:
    """PalmTip curl semantics on MP21 knots: fist thumb hi, legacy thumb_flex fallback."""
    raw_block = raw_block or {}
    fist_b = raw_block.get("fist") or {}
    flex_b = raw_block.get("thumb_flex") or {}
    has_fist_thumb = fist_b.get("thumb_mcp") is not None or fist_b.get("thumb_ip") is not None
    if has_fist_thumb:
        if fist_b.get("thumb_mcp") is not None:
            calib.thumb_flex_mcp = float(fist_b["thumb_mcp"])
        if fist_b.get("thumb_ip") is not None:
            calib.thumb_flex_ip = float(fist_b["thumb_ip"])
    elif flex_b and (flex_b.get("mcp") is not None or flex_b.get("ip") is not None):
        print(
            f"[{side}] mp_curl calib uses legacy thumb_flex knot; "
            "re-capture fist (thumb in palm) for clearer curl span",
            flush=True,
        )
    _elevate_thumb_hi_from_pinch(calib)
    _warn_degenerate_thumb_span(side, calib)


def _j1_soft_hi(robot: dict[str, Any], urdf_hi: float) -> float:
    close_j1 = float(robot.get("close_j1", 0.0))
    roof = float(robot.get("j1_roof", _DEFAULT_J1_ROOF))
    hi = max(float(urdf_hi), 0.0)
    floor = max(close_j1, roof) if close_j1 > 1e-9 else roof
    return float(min(hi, max(floor, 0.0)))


def _pinch_lat_sigma(calib: SideCurlCalib2) -> float:
    """Narrow kernel (m): half median gap between pinch lats, else 8mm."""
    lats: list[float] = []
    if not calib.pinch_skipped:
        for name in FINGERS:
            knot = (calib.pinch or {}).get(name)
            if knot is not None:
                lats.append(float(knot.thumb_lat))
    if len(lats) >= 2:
        xs = np.sort(np.unique(np.asarray(lats, dtype=np.float64)))
        if xs.size >= 2:
            gaps = np.diff(xs)
            gaps = gaps[gaps > 1e-4]
            if gaps.size:
                return float(
                    np.clip(0.5 * float(np.median(gaps)), _PINCH_SIGMA_MIN, _PINCH_SIGMA_MAX)
                )
    return 0.008


def pinch_lat_weights(
    feat: CurlFeatures2, calib: SideCurlCalib2
) -> dict[str, float]:
    """Soft nearness of thumb_lat to each pinch knot in [0, 1] (narrow kernel)."""
    out = {n: 0.0 for n in FINGERS}
    if calib.pinch_skipped:
        return out
    sigma = _pinch_lat_sigma(calib)
    x = float(feat.thumb_lat)
    for name in FINGERS:
        knot = (calib.pinch or {}).get(name)
        if knot is None:
            continue
        d = abs(x - float(knot.thumb_lat))
        out[name] = float(np.clip(1.0 - d / (_PINCH_FALL_K * sigma), 0.0, 1.0))
    return out


def _mp_fist_w(
    feat: CurlFeatures2,
    calib: SideCurlCalib2,
    *,
    finger_weights: tuple[float, float, float],
    finger_t: float = 0.92,
) -> float:
    """0→1 when index+middle look fisted (suppress lat pinch during fist)."""
    enter = max(0.0, float(finger_t) - 0.20)
    ts = [
        _weighted_finger_t(feat, calib, n, finger_weights)
        for n in ("index", "middle")
    ]
    if not ts:
        return 0.0
    return float(_unit_t(min(ts), enter, float(finger_t)))


def _pinch_winner_weight(w_lat: dict[str, float]) -> tuple[str | None, float]:
    """Winner-take-most: j1 gate only when one finger clearly owns lat."""
    ranked = sorted(w_lat.items(), key=lambda kv: kv[1], reverse=True)
    if not ranked or ranked[0][1] <= 1e-6:
        return None, 0.0
    best_n, best = ranked[0]
    second = ranked[1][1] if len(ranked) > 1 else 0.0
    # Soft: full weight when margin clear; shrink when ambiguous
    margin = float(best - second)
    clarity = float(np.clip(margin / max(_PINCH_WIN_MARGIN, 1e-6), 0.0, 1.0))
    # Also require absolute nearness
    near = float(np.clip((best - 0.35) / 0.45, 0.0, 1.0))
    return best_n, float(best * clarity * near)


def _is_palm_tip_calib(path: Path) -> bool:
    import yaml

    try:
        with path.open("r", encoding="utf-8") as f:
            raw: dict[str, Any] = yaml.safe_load(f) or {}
    except Exception:
        return False
    schema = str(raw.get("schema") or "").strip()
    return "palm_tip" in schema


def _load_palm_calib_for_mp(gains, side: str):
    """PalmTip human knots at ``gains.calib_path`` (linker.user → calib/o6.yaml)."""
    from xr_hand_retarget.algorithms.palm_tip import (
        load_palm_calib,
        resolve_palm_side,
    )

    raw = getattr(gains, "calib_path", None)
    if raw is None:
        return None
    path = Path(raw)
    if not path.is_file() or not _is_palm_tip_calib(path):
        return None
    loaded = load_palm_calib(path)
    src = str(getattr(gains, "calib_source_side", "both") or "both")
    az = float(getattr(gains, "calib_mirror_az", 1.0))
    calib, _label = resolve_palm_side(loaded, side, source_side=src, az_sign=az)
    return calib


def _mp_j2_side_park_w(
    feat: CurlFeatures2,
    calib: SideCurlCalib2,
    *,
    finger_weights: tuple[float, float, float],
) -> float:
    """Fade j2→0 when four fingers are extended (MP21 lat path only)."""
    ts = [
        _weighted_finger_t(feat, calib, n, finger_weights)
        for n in ("index", "middle")
    ]
    if not ts:
        return 0.0
    finger_open = max(float(max(ts)), 0.0)
    thumb_t = _weighted_thumb_t(feat, calib, (0.55, 0.45))
    return float(np.clip((1.0 - thumb_t) * (1.0 - finger_open / 0.35), 0.0, 1.0))


def _thumb_j2_mp(
    feat: CurlFeatures2,
    calib: SideCurlCalib2,
    *,
    hi0: float,
    urdf_q: dict[str, float] | None,
    robot: dict[str, Any],
    finger_weights: tuple[float, float, float],
    palm_feat=None,
    palm_calib=None,
    ws=None,
) -> float:
    """Thumb j2: PalmTip nest_lat/slot_t when palm calib exists; else lat polyline."""
    from xr_hand_retarget.algorithms.palm_tip import _thumb_j2_from_tip

    if palm_feat is not None and palm_calib is not None and ws is not None:
        return _thumb_j2_from_tip(palm_feat, palm_calib, ws)
    j2 = _thumb_yaw_fit(
        feat,
        calib,
        hi0=hi0,
        urdf_q=urdf_q,
        robot=robot,
    )
    w_side = _mp_j2_side_park_w(feat, calib, finger_weights=finger_weights)
    return float(np.clip((1.0 - w_side) * j2, 0.0, hi0))


def _thumb_j3_mp(
    *,
    dof: int,
    hi: np.ndarray,
    feat: CurlFeatures2,
    calib: SideCurlCalib2,
    thumb_weights: tuple[float, float],
    thumb3_opp: float,
    thumb3_curl: float,
    q_j2: float,
    palm_feat=None,
    palm_calib=None,
    ws=None,
) -> float:
    """O7 j3: PalmTip slot/az when palm calib exists; else legacy j2/curl mix."""
    if dof < 7:
        return 0.0
    hi3 = max(float(hi[_THUMB3]), 0.0)
    if palm_feat is not None and palm_calib is not None and ws is not None:
        from xr_hand_retarget.algorithms.palm_tip import _thumb_j3_rad

        j3 = _thumb_j3_rad(palm_feat, palm_calib, ws, sweep_hi=hi3)
        return float(np.clip(j3, 0.0, hi3))
    t_curl = _weighted_thumb_t(feat, calib, thumb_weights)
    t_opp_unit = float(q_j2) / max(float(hi[_THUMB_YAW]), 1e-9)
    mix = float(thumb3_opp) * t_opp_unit + float(thumb3_curl) * t_curl
    return float(np.clip(mix, 0.0, 1.0)) * hi3


def _thumb_yaw_fit(
    feat: CurlFeatures2,
    calib: SideCurlCalib2,
    *,
    hi0: float,
    urdf_q: dict[str, float] | None,
    robot: dict[str, Any],
) -> float:
    """thumb_lat → j2 polyline; along pole → 0; pinch knots → RViz j2 slots."""
    pinky_q = float(hi0)
    if urdf_q and "pinky" in urdf_q:
        pinky_q = float(urdf_q["pinky"])
    xs: list[float] = []
    qs: list[float] = []
    if calib.along_thumb_lat is not None:
        xs.append(float(calib.along_thumb_lat))
        qs.append(0.0)
    else:
        xs.extend([float(calib.open_thumb_lat), float(calib.thumb_to_pinky_lat)])
        qs.extend([0.0, pinky_q])
    j2_slot = {
        "index": float(robot["j2_index"]),
        "middle": float(robot["j2_middle"]),
        "ring": float(robot["j2_middle"]),
        "pinky": pinky_q,
    }
    if not calib.pinch_skipped:
        for name in FINGERS:
            knot = (calib.pinch or {}).get(name)
            if knot is None:
                continue
            if knot.q is not None:
                qk = float(knot.q)
            elif name in j2_slot:
                qk = float(j2_slot[name])
            elif urdf_q and name in urdf_q:
                qk = float(urdf_q[name])
            else:
                continue
            xs.append(float(knot.thumb_lat))
            qs.append(float(np.clip(qk, 0.0, hi0)))
    if xs:
        pairs = sorted(zip(xs, qs), key=lambda p: p[0])
        xs = [p[0] for p in pairs]
        qs = [p[1] for p in pairs]
    return float(np.clip(_interp_knots(feat.thumb_lat, xs, qs), 0.0, hi0))


def _use_o7_thumb_curl(
    dof: int,
    palm_calib,
    ws,
    *,
    gains=None,
) -> bool:
    """O7 mp_curl thumb uses PalmTip curl when palm calib is loaded."""
    mode = str(getattr(gains, "mp_o7_thumb", "curl") or "curl").strip().lower()
    if mode in ("legacy", "mp21", "lat"):
        return False
    return int(dof) >= 7 and palm_calib is not None and ws is not None


def _apply_o7_thumb_curl(
    q: np.ndarray,
    palm_feat,
    palm_calib,
    ws,
    limits: np.ndarray,
    *,
    decision=None,
    j2_locked: bool = False,
    pinch_fk=None,
    q_prev: np.ndarray | None = None,
) -> np.ndarray:
    """O7 thumb j1/j2/j3 — PalmTip curl chain; thumb j1 attract like method curl."""
    from xr_hand_retarget.algorithms.palm_tip import (
        _THUMB_J1,
        _THUMB_J2,
        _thumb_j1_curl_hi,
        apply_fist_envelope,
        apply_j2_lock,
        apply_thumb3,
        apply_thumb_j1_band,
        attract_pinch,
        blend_pad_j2,
        map_palm_tip,
    )
    from xr_hand_retarget.algorithms.pinch_fk import apply_pinch_planning

    q = np.asarray(q, dtype=np.float64).copy()
    hi = limits[:, 1]
    q_seed = map_palm_tip(palm_feat, palm_calib, limits, ws)
    q[_THUMB_J1] = float(q_seed[_THUMB_J1])
    q[_THUMB_J2] = blend_pad_j2(
        0.0, palm_feat, ws, palm_calib, decision=decision
    )
    q = apply_j2_lock(q, ws, j2_locked)
    q = apply_thumb3(
        q,
        palm_feat,
        palm_calib,
        ws,
        hi,
        solver="curl",
        decision=decision,
    )
    j1_hi = _thumb_j1_curl_hi(ws, float(hi[_THUMB_J1]))
    q = apply_thumb_j1_band(
        q, palm_feat, palm_calib, ws, j1_hi, enabled=True
    )
    q = attract_pinch(
        q,
        palm_feat,
        ws,
        palm_calib,
        gate_thumb_j1=True,
        decision=decision,
        thumb_j1_only=True,
    )
    q = apply_fist_envelope(
        q, palm_feat, palm_calib, ws, float(hi[_THUMB_J1])
    )
    slot = decision.slot if decision is not None else None
    return apply_pinch_planning(
        q,
        ws,
        pinch_fk,
        slot=slot,
        q_prev=q_prev,
        limits=limits,
        thumb_only=False,
    )


def map_curl_features_mp_o6(
    feat: CurlFeatures2,
    calib: SideCurlCalib2,
    *,
    limits: np.ndarray,
    finger_weights: tuple[float, float, float],
    thumb_weights: tuple[float, float],
    robot: dict[str, Any],
    urdf_q: dict[str, float] | None = None,
    thumb3_opp: float = 0.65,
    thumb3_curl: float = 0.35,
    palm_feat=None,
    palm_calib=None,
    ws=None,
    o7_thumb_curl: bool = False,
) -> np.ndarray:
    """Human curl → O6 q; MP21 fingers; O7 thumb optionally from PalmTip curl."""
    lo = limits[:, 0]
    hi = limits[:, 1]
    dof = int(lo.shape[0])
    q = np.zeros(dof, dtype=np.float64)
    close_j1 = float(robot.get("close_j1", 0.0))
    t_mid = float(robot.get("t_mid", _DEFAULT_PINCH_T_MID))
    j1_soft = _j1_soft_hi(robot, float(hi[_THUMB_FLEX]))
    t_curl = _weighted_thumb_t(feat, calib, thumb_weights)
    j1_curl = _fit_open_pinch_fist(t_curl, close_j1, j1_soft, t_mid)
    w_fist = _mp_fist_w(feat, calib, finger_weights=finger_weights)
    w_lat = pinch_lat_weights(feat, calib)
    winner, pinch_w = _pinch_winner_weight(w_lat)
    pinch_w *= 1.0 - w_fist
    use_o7_thumb = bool(o7_thumb_curl)
    if not use_o7_thumb:
        if close_j1 > 1e-9 and pinch_w > 1e-6:
            q[_THUMB_FLEX] = (1.0 - pinch_w) * j1_curl + pinch_w * close_j1
        else:
            q[_THUMB_FLEX] = j1_curl
        q[_THUMB_YAW] = _thumb_j2_mp(
            feat,
            calib,
            hi0=max(float(hi[_THUMB_YAW]), 0.0),
            urdf_q=urdf_q,
            robot=robot,
            finger_weights=finger_weights,
            palm_feat=palm_feat,
            palm_calib=palm_calib,
            ws=ws,
        )
    fingers = _FINGER_IDX_7 if dof >= 7 else _FINGER_IDX_6
    if dof >= 7 and not use_o7_thumb:
        q[_THUMB3] = _thumb_j3_mp(
            dof=dof,
            hi=hi,
            feat=feat,
            calib=calib,
            thumb_weights=thumb_weights,
            thumb3_opp=thumb3_opp,
            thumb3_curl=thumb3_curl,
            q_j2=float(q[_THUMB_YAW]),
            palm_feat=palm_feat,
            palm_calib=palm_calib,
            ws=ws,
        )
    close_finger = robot.get("close_finger") or {}
    for name, idx in fingers.items():
        t = _weighted_finger_t(feat, calib, name, finger_weights)
        # Curl channel: open→fist only (no shared pinch mid that couples all fingers).
        q_curl = float(t) * max(float(hi[idx]), 0.0)
        q_contact = float(close_finger.get(name, 0.0))
        # Lat near this finger × finger must be curling × (boost if winner).
        curl_gate = float(
            np.clip(
                (t - _PINCH_FINGER_CURL_ON) / max(1.0 - _PINCH_FINGER_CURL_ON, 1e-3),
                0.0,
                1.0,
            )
        )
        w_f = float(w_lat.get(name, 0.0)) * curl_gate * (1.0 - w_fist)
        if winner is not None and name == winner:
            w_f = max(w_f, float(pinch_w) * curl_gate)
        else:
            # Non-winners: strongly suppress contact blend so fingers don't lock together.
            w_f *= 0.15
        if q_contact > 1e-9 and w_f > 1e-6:
            q[idx] = (1.0 - w_f) * q_curl + w_f * q_contact
        else:
            q[idx] = q_curl
    return np.clip(q, lo, hi)


def enrich_pinch_from_takes(
    calib: SideCurlCalib2, takes: dict[str, list] | None
) -> int:
    """Fill pinch thumb/finger flex from ``takes[pinch_*]`` when knots lack them."""
    if not takes or calib.pinch_skipped:
        return 0
    filled = 0
    if calib.pinch is None:
        calib.pinch = {}
    for name in FINGERS:
        key = f"pinch_{name}"
        rows = takes.get(key) or takes.get(name)
        if not rows:
            continue
        feats = []
        for row in rows:
            if not isinstance(row, dict):
                continue
            feat_d = row.get("features") or row
            if not isinstance(feat_d, dict):
                continue
            f = feat2_from_dict(feat_d)
            if f is not None:
                feats.append(f)
        if not feats:
            continue
        thumb_mcp = float(np.median([f.thumb_mcp for f in feats]))
        thumb_ip = float(np.median([f.thumb_ip for f in feats]))
        finger_mcp = float(np.median([f.mcp[name] for f in feats]))
        finger_pip = float(np.median([f.pip[name] for f in feats]))
        finger_dip = float(np.median([f.dip[name] for f in feats]))
        thumb_lat = float(np.median([f.thumb_lat for f in feats]))
        knot = (calib.pinch or {}).get(name)
        if knot is None:
            from xr_hand_retarget.algorithms.curl_xhand1 import PinchKnot

            calib.pinch[name] = PinchKnot(
                thumb_lat=thumb_lat,
                thumb_mcp=thumb_mcp,
                thumb_ip=thumb_ip,
                finger_mcp=finger_mcp,
                finger_pip=finger_pip,
                finger_dip=finger_dip,
            )
            calib.pinch_skipped = False
            filled += 1
            continue
        if knot.thumb_mcp is None:
            knot.thumb_mcp = thumb_mcp
            filled += 1
        if knot.thumb_ip is None:
            knot.thumb_ip = thumb_ip
            filled += 1
        if knot.finger_mcp is None:
            knot.finger_mcp = finger_mcp
            filled += 1
        if knot.finger_pip is None:
            knot.finger_pip = finger_pip
            filled += 1
        if knot.finger_dip is None:
            knot.finger_dip = finger_dip
            filled += 1
    return filled


def load_mp_curl_side(
    path: Path, side: str
) -> tuple[SideCurlCalib2, dict[str, list] | None]:
    """Load SideCurlCalib2 and optional takes; enrich pinch flex from takes."""
    import yaml

    loaded = load_calib_file(path)
    calib = loaded[side]
    takes = None
    with path.open("r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}
    block = (raw.get("sides") or {}).get(side) or {}
    if isinstance(block.get("takes"), dict):
        takes = block["takes"]
        n = enrich_pinch_from_takes(calib, takes)
        if n:
            print(
                f"[{side}] mp_curl enriched pinch flex from takes ({n} fields)",
                flush=True,
            )
    normalize_mp_curl_calib(calib, side, block if isinstance(block, dict) else None)
    return calib, takes


class MpCurlO6Retargeter:
    """OpenXR 26 → MP21 curl features → O6/L6/O7 q."""

    def __init__(self, side: str, gains, calib: SideCurlCalib2 | None = None):
        if side not in ("left", "right"):
            raise ValueError(f"side must be left|right, got {side!r}")
        self.side = side
        self.gains = gains
        self.model = str(getattr(gains, "model", None) or "o6").strip().lower()
        limits, _vel = load_linker_limits(
            side, model=self.model, urdf_path=self.gains.urdf_path
        )
        self.limits = limits
        calib_path = resolve_mp_curl_calib(gains)
        if calib is not None:
            self.calib = calib
            self.calib_path = None
        elif calib_path is not None:
            self.calib, _takes = load_mp_curl_side(calib_path, side)
            self.calib_path = calib_path
            extra = "" if self.calib.pinch_skipped else " pinch=on"
            print(
                f"[{side}] {self.model} mp_curl calib={calib_path}{extra}",
                flush=True,
            )
        else:
            missing = getattr(gains, "calib_path", None)
            print(
                f"[{side}] {self.model} mp_curl calib missing "
                f"({missing or _DEFAULT_CALIB}); using seed",
                flush=True,
            )
            self.calib = seed_side_calib()
            self.calib_path = None
        self._fk = None
        self._q_cache: dict[float, dict[str, float]] = {}
        self._robot_pinch = robot_pinch_from_gains(gains)
        rp = self._robot_pinch
        fingers = ",".join(
            f"{n}={float((rp.get('close_finger') or {}).get(n, 0)):.2f}"
            for n in FINGERS
            if float((rp.get("close_finger") or {}).get(n, 0)) > 0
        ) or "off"
        print(
            f"[{side}] {self.model} mp_curl pinch_q fit "
            f"t_mid={rp['t_mid']:.2f} j1_mid={rp['close_j1']:.2f} "
            f"j1_roof={rp.get('j1_roof', _DEFAULT_J1_ROOF):.2f} "
            f"j2={rp['j2_index']:.2f}/{rp['j2_middle']:.2f} fingers={fingers}",
            flush=True,
        )
        try:
            self._fk = make_linker_fk(
                side, model=self.model, urdf_path=self.gains.urdf_path
            )
            print(
                f"[{side}] {self.model} mp_curl pinch URDF={self._fk.urdf_path.name} "
                f"axis={self.gains.pinch_axis} target={self.gains.pinch_target_link}",
                flush=True,
            )
        except Exception as exc:
            print(
                f"[{side}] {self.model} mp_curl URDF pinch table unavailable: {exc}",
                flush=True,
            )
        self._prev: np.ndarray | None = None
        self._pinch_slot: str | None = None
        self._j2_lock = False
        self._pinch_fk_proj = None
        self._pinch_fk_failed = False
        self._palm_calib = _load_palm_calib_for_mp(gains, side)
        self._ws = None
        if self._palm_calib is not None:
            from xr_hand_retarget.algorithms.palm_tip import workspace_from_gains

            self._ws = workspace_from_gains(gains)
            palm_path = getattr(gains, "calib_path", None)
            if int(self.limits.shape[0]) >= 7:
                print(
                    f"[{side}] {self.model} mp_curl: fingers=MP21 "
                    f"thumb=curl PalmTip ({palm_path})",
                    flush=True,
                )
            else:
                print(
                    f"[{side}] {self.model} mp_curl thumb j2 ← PalmTip "
                    f"({palm_path})",
                    flush=True,
                )
        elif int(self.limits.shape[0]) >= 7:
            print(
                f"[{side}] {self.model} mp_curl O7: no PalmTip calib "
                f"({getattr(gains, 'calib_path', None)}); thumb uses MP21 legacy",
                flush=True,
            )

    def set_j2_lock(self, on: bool) -> bool:
        from xr_hand_retarget.algorithms.palm_tip import _j2_lock_band

        if on != self._j2_lock and self._ws is not None:
            lo, hi = _j2_lock_band(self._ws)
            print(
                f"[{self.side}] mp_curl j2_lock={'on' if on else 'off'} "
                f"band=[{lo:.2f}, {hi:.2f}]",
                flush=True,
            )
        self._j2_lock = bool(on)
        return self._j2_lock

    def toggle_j2_lock(self) -> bool:
        return self.set_j2_lock(not self._j2_lock)

    def _get_pinch_fk(self):
        if self._pinch_fk_failed or self._ws is None:
            return None
        if self._pinch_fk_proj is not None:
            return self._pinch_fk_proj
        if not bool(getattr(self._ws, "pinch_fk_project_enabled", False)):
            return None
        if self._fk is None:
            try:
                self._fk = make_linker_fk(
                    self.side, model=self.model, urdf_path=self.gains.urdf_path
                )
            except Exception as exc:
                print(
                    f"[{self.side}] {self.model} mp_curl pinch FK unavailable: {exc}",
                    flush=True,
                )
                self._pinch_fk_failed = True
                return None
        try:
            from xr_hand_retarget.algorithms.pinch_fk import (
                PinchFkProjector,
                pinch_fk_config_from_workspace,
            )

            self._pinch_fk_proj = PinchFkProjector(
                self._fk,
                self.limits,
                joint_names=None,
                config=pinch_fk_config_from_workspace(self._ws),
            )
        except Exception as exc:
            print(
                f"[{self.side}] {self.model} mp_curl pinch FK project failed: {exc}",
                flush=True,
            )
            self._pinch_fk_failed = True
            return None
        return self._pinch_fk_proj

    def _blend_output(self, q: np.ndarray) -> np.ndarray:
        """Layered EMA: slower thumb (j2 jitter), faster fingers."""
        q = np.asarray(q, dtype=np.float64)
        base = float(np.clip(self.gains.output_alpha, 1e-3, 1.0))
        ta_raw = getattr(self.gains, "thumb_alpha", None)
        fa_raw = getattr(self.gains, "finger_alpha", None)
        ta = float(np.clip(base if ta_raw is None else float(ta_raw), 1e-3, 1.0))
        fa = float(np.clip(base if fa_raw is None else float(fa_raw), 1e-3, 1.0))
        n_thumb = 3 if int(self.limits.shape[0]) >= 7 else 2
        if self._prev is None or (ta >= 1.0 - 1e-12 and fa >= 1.0 - 1e-12):
            self._prev = q.copy()
            return q
        prev = self._prev
        out = q.copy()
        out[:n_thumb] = (1.0 - ta) * prev[:n_thumb] + ta * q[:n_thumb]
        out[n_thumb:] = (1.0 - fa) * prev[n_thumb:] + fa * q[n_thumb:]
        self._prev = out
        return out

    def reset(self) -> None:
        self._prev = None
        self._pinch_slot = None
        self._q_cache.clear()

    def _urdf_q(self, q_flex: float) -> dict[str, float] | None:
        if self._fk is None:
            return None
        key = round(float(q_flex), 2)
        hit = self._q_cache.get(key)
        if hit is not None:
            return hit
        q_rest = np.zeros(int(self.limits.shape[0]), dtype=np.float64)
        q_rest[_THUMB_FLEX] = float(q_flex)
        hi = max(float(self.limits[_THUMB_YAW, 1]), 0.0)
        table = pinch_q_from_urdf(
            self._fk,
            q_rest,
            target=self.gains.pinch_target_link,
            axis=int(self.gains.pinch_axis),
            opp_index=int(self.gains.opposition_index),
            q_lo=0.0,
            q_hi=hi,
        )
        self._q_cache[key] = table
        return table

    def retarget(self, joints26: np.ndarray) -> np.ndarray:
        from xr_hand_retarget.algorithms.palm_tip import (
            _thumb_j1_curl_hi,
            apply_fist_envelope,
            apply_j2_lock,
            apply_thumb_j1_band,
            attract_pinch,
            classify_o7_pinch,
            extract_palm_tip,
        )
        from xr_hand_retarget.algorithms.pinch_fk import (
            apply_pinch_planning,
            clamp_pinch_ceiling,
        )

        feat = extract_curl_features_mp21_from26(joints26, self.side)
        palm_feat = None
        if self._palm_calib is not None and self._ws is not None:
            palm_feat = extract_palm_tip(joints26, self.side)
        dof = int(self.limits.shape[0])
        o7_thumb = _use_o7_thumb_curl(
            dof, self._palm_calib, self._ws, gains=self.gains
        )
        t_curl = _weighted_thumb_t(
            feat, self.calib, self.gains.thumb_flex_weights
        )
        q_flex_seed = float(self._robot_pinch.get("close_j1") or 0.0)
        if q_flex_seed <= 1e-9:
            j1_soft = _j1_soft_hi(
                self._robot_pinch, float(self.limits[_THUMB_FLEX, 1])
            )
            q_flex_seed = t_curl * j1_soft
        q = map_curl_features_mp_o6(
            feat,
            self.calib,
            limits=self.limits,
            finger_weights=self.gains.finger_flex_weights,
            thumb_weights=self.gains.thumb_flex_weights,
            robot=self._robot_pinch,
            urdf_q=self._urdf_q(q_flex_seed),
            thumb3_opp=float(getattr(self.gains, "thumb3_opp", 0.65)),
            thumb3_curl=float(getattr(self.gains, "thumb3_curl", 0.35)),
            palm_feat=palm_feat,
            palm_calib=self._palm_calib,
            ws=self._ws,
            o7_thumb_curl=o7_thumb,
        )
        dec = None
        if o7_thumb and palm_feat is not None:
            dec = classify_o7_pinch(
                palm_feat,
                self._palm_calib,
                self._ws,
                sticky=self._pinch_slot,
            )
            self._pinch_slot = dec.slot
            if dec.slot and bool(getattr(self._ws, "pinch_ceiling_enabled", False)):
                margin = float(
                    getattr(self._ws, "pinch_ceiling_margin_rad", 0.0) or 0.0
                )
                q = clamp_pinch_ceiling(
                    q,
                    self._ws,
                    slot=dec.slot,
                    margin=margin,
                    fingers_only=True,
                )
            q = _apply_o7_thumb_curl(
                q,
                palm_feat,
                self._palm_calib,
                self._ws,
                self.limits,
                decision=dec,
                j2_locked=self._j2_lock,
                pinch_fk=self._get_pinch_fk(),
                q_prev=self._prev,
            )
        q = np.clip(q, self.limits[:, 0], self.limits[:, 1])
        blended = self._blend_output(q)
        if not (o7_thumb and palm_feat is not None):
            return blended
        if blended is q:
            return blended
        j1_hi = _thumb_j1_curl_hi(
            self._ws, float(self.limits[_THUMB_FLEX, 1])
        )
        blended = apply_j2_lock(blended, self._ws, self._j2_lock)
        blended = apply_thumb_j1_band(
            blended,
            palm_feat,
            self._palm_calib,
            self._ws,
            j1_hi,
            enabled=True,
        )
        blended = apply_fist_envelope(
            blended,
            palm_feat,
            self._palm_calib,
            self._ws,
            float(self.limits[_THUMB_FLEX, 1]),
        )
        blended = attract_pinch(
            blended,
            palm_feat,
            self._ws,
            self._palm_calib,
            gate_thumb_j1=True,
            decision=dec,
            thumb_j1_only=True,
        )
        blended = apply_pinch_planning(
            blended,
            self._ws,
            self._get_pinch_fk(),
            slot=dec.slot if dec else None,
            q_prev=self._prev,
            limits=self.limits,
            thumb_only=False,
        )
        blended = np.clip(
            blended, self.limits[:, 0], self.limits[:, 1]
        )
        self._prev = blended
        return blended


def _self_test_o7_mp_curl() -> None:
    """Synthetic checks: O6/O7 four-finger parity + O7 j2 slot_t sweep smoothness."""
    from xr_hand_retarget.algorithms.palm_tip import (
        FINGERS as PT_FINGERS,
        PalmTipCalib,
        PalmTipFeat,
        blend_pad_j2,
        classify_o7_pinch,
        workspace_from_gains,
    )

    hi6 = np.array([1.0, 1.3, 1.0, 1.0, 1.0, 1.0])
    hi7 = np.array([1.0, 1.3, 0.7, 1.0, 1.0, 1.0, 1.0])
    limits6 = np.stack([np.zeros(6), hi6], axis=1)
    limits7 = np.stack([np.zeros(7), hi7], axis=1)
    calib = seed_side_calib()
    robot = {
        "close_j1": 0.41,
        "t_mid": 0.92,
        "j1_roof": 0.44,
        "j2_index": 1.0,
        "j2_middle": 1.3,
        "close_finger": {"index": 0.8, "middle": 0.83},
    }
    feat = CurlFeatures2(
        mcp={n: 0.5 for n in FINGERS},
        pip={n: 0.4 for n in FINGERS},
        dip={n: 0.3 for n in FINGERS},
        thumb_mcp=0.4,
        thumb_ip=0.3,
        thumb_lat=0.05,
        finger_span=0.8,
    )
    fw, tw = (0.45, 0.35, 0.20), (0.55, 0.45)
    q6 = map_curl_features_mp_o6(
        feat, calib, limits=limits6, finger_weights=fw, thumb_weights=tw, robot=robot
    )
    q7 = map_curl_features_mp_o6(
        feat,
        calib,
        limits=limits7,
        finger_weights=fw,
        thumb_weights=tw,
        robot=robot,
        o7_thumb_curl=False,
    )
    if not np.allclose(q6[2:6], q7[3:7]):
        raise AssertionError(f"four-finger parity failed: {q6[2:6]} vs {q7[3:7]}")

    class _G:
        model = "o7"
        thumb_j2_index = 1.0
        thumb_j2_middle = 1.3
        pinch_index_q = {"thumb_joint1": 0.41, "index_joint": 0.8}
        pinch_middle_q = {"thumb_joint1": 0.41, "middle_joint": 0.83}
        fist_envelope = {"enabled": True, "j1_roof": 0.44}
        attract_near = 0.22
        attract_far = 0.45
        attract_enabled = True

    ws = workspace_from_gains(_G())
    palm_calib = PalmTipCalib(
        open_finger_curl={n: 0.05 for n in PT_FINGERS},
        fist_finger_curl={n: 1.0 for n in PT_FINGERS},
        open_thumb_curl=0.1,
        flex_thumb_curl=0.9,
        open_thumb_az=0.05,
        along_thumb_az=0.05,
        middle_thumb_az=0.55,
        along_slot_t=-0.20,
        pinch_skipped=False,
        pinch={
            "index": {"slot_t": 0.08, "nest_lat": 0.50},
            "middle": {"slot_t": 0.92, "nest_lat": 0.62},
            "ring": None,
            "pinky": None,
        },
    )
    j2s: list[float] = []
    for t in np.linspace(0.08, -0.20, 40):
        pf = PalmTipFeat(
            finger_curl={n: 0.08 for n in PT_FINGERS},
            thumb_curl=0.45,
            thumb_az=0.38,
            slot_t=float(t),
            nest_lat=0.5,
            nest_r=0.32,
            pinch_d={"index": 0.3},
        )
        dec = classify_o7_pinch(pf, palm_calib, ws)
        j2s.append(blend_pad_j2(0.0, pf, ws, palm_calib, decision=dec))
    step = float(np.max(np.abs(np.diff(j2s))))
    if step >= 0.05:
        raise AssertionError(f"j2 slot_t sweep max step {step:.4f} >= 0.05")


if __name__ == "__main__":
    _self_test_o7_mp_curl()
    from xr_hand_retarget.algorithms.pinch_fk import _self_test_pinch_planning

    _self_test_pinch_planning()
    print("mp_curl O7 self-tests passed")
