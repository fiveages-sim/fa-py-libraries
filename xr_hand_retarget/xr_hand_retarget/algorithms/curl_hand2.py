"""Open / fist / thumb-to-pinky / optional pinch → Wuji Hand 2 lerp.

Same human features as XHand1 curl. Robot side is 20-DOF:

  thumb:  cmc_flex (URDF palm-X slot) / cmc_abd=0 / mcp / ip
  fingers: mcp_flex / abd=0 / pip / dip (dip default-coupled to pip)

Pinch: scan thumb_cmc_flex so thumb_tip.x matches that finger's PIP/MCP/tip
x in hand_base (finger row is along +X index → −X pinky).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from xr_hand_retarget.algorithms.curl_xhand1 import PinchKnot, _interp_knots, _unit_t
from xr_hand_retarget.algorithms.remap import (
    finger_chain_flexions,
    four_finger_tip_span,
    thumb_chain_flexion,
    thumb_tip_palm_x_m,
)

from xr_hand_retarget.config import WujiCurlGains
from xr_hand_retarget.kinematics import load_hand2_limits, make_hand2_fk
from xr_hand_retarget.landmarks import openxr26_to_mediapipe21, palm_triangle_area_m2
from xr_hand_retarget.pipeline import SideStep
from xr_hand_retarget.algorithms.safety_xhand1 import pose_is_zero

FINGERS = ("index", "middle", "ring", "pinky")
SCHEMA = "wuji_hand2_curl_calib.v2"

# q indices in fa_w2 hand2.yaml order.
_THUMB_FLEX = 0
_THUMB_ABD = 1
_THUMB_MCP = 2
_THUMB_IP = 3
_FINGER_IDX = {
    "index": (4, 5, 6, 7),
    "middle": (8, 9, 10, 11),
    "ring": (12, 13, 14, 15),
    "pinky": (16, 17, 18, 19),
}

_PINCH_LINKS = {
    "mcp": {
        "index": "index_mcp",
        "middle": "middle_mcp",
        "ring": "ring_mcp",
        "pinky": "pinky_mcp",
    },
    "pip": {
        "index": "index_pip",
        "middle": "middle_pip",
        "ring": "ring_pip",
        "pinky": "pinky_pip",
    },
    "tip": {
        "index": "index_tip",
        "middle": "middle_tip",
        "ring": "ring_tip",
        "pinky": "pinky_tip",
    },
}

_SEED_FIST_MCP = 1.15
_SEED_FIST_PIP = 1.05
_SEED_FIST_DIP = 0.90
_SEED_THUMB_MCP = 1.10
_SEED_THUMB_IP = 1.05
_SEED_OPEN_THUMB_LAT = 0.045
_SEED_PINKY_THUMB_LAT = -0.025
_Q_SCAN = 65


@dataclass
class CurlFeatures2:
    mcp: dict[str, float]
    pip: dict[str, float]
    dip: dict[str, float]
    thumb_mcp: float
    thumb_ip: float
    thumb_lat: float
    finger_span: float = 0.0


@dataclass
class SideCurlCalib2:
    open_mcp: dict[str, float]
    open_pip: dict[str, float]
    open_dip: dict[str, float]
    fist_mcp: dict[str, float]
    fist_pip: dict[str, float]
    fist_dip: dict[str, float]
    open_thumb_mcp: float
    open_thumb_ip: float
    thumb_flex_mcp: float
    thumb_flex_ip: float
    open_thumb_lat: float
    thumb_to_pinky_lat: float
    along_thumb_lat: float | None = None
    pinch_skipped: bool = True
    pinch: dict[str, PinchKnot | None] = field(default_factory=dict)
    together_finger_span: float | None = None
    spread_finger_span: float | None = None


def extract_curl_features2(joints26: np.ndarray, side: str) -> CurlFeatures2:
    mcp: dict[str, float] = {}
    pip: dict[str, float] = {}
    dip: dict[str, float] = {}
    for name in FINGERS:
        mcp[name], pip[name], dip[name] = finger_chain_flexions(joints26, name)
    t_mcp, t_ip = thumb_chain_flexion(joints26)
    return CurlFeatures2(
        mcp=mcp,
        pip=pip,
        dip=dip,
        thumb_mcp=t_mcp,
        thumb_ip=t_ip,
        thumb_lat=thumb_tip_palm_x_m(joints26, side),
        finger_span=four_finger_tip_span(joints26),
    )


def seed_side_calib() -> SideCurlCalib2:
    zeros = {n: 0.0 for n in FINGERS}
    return SideCurlCalib2(
        open_mcp=dict(zeros),
        open_pip=dict(zeros),
        open_dip=dict(zeros),
        fist_mcp={n: _SEED_FIST_MCP for n in FINGERS},
        fist_pip={n: _SEED_FIST_PIP for n in FINGERS},
        fist_dip={n: _SEED_FIST_DIP for n in FINGERS},
        open_thumb_mcp=0.0,
        open_thumb_ip=0.0,
        thumb_flex_mcp=_SEED_THUMB_MCP,
        thumb_flex_ip=_SEED_THUMB_IP,
        open_thumb_lat=_SEED_OPEN_THUMB_LAT,
        thumb_to_pinky_lat=_SEED_PINKY_THUMB_LAT,
        pinch_skipped=True,
        pinch={n: None for n in FINGERS},
        together_finger_span=None,
        spread_finger_span=None,
    )


def _floats(block: dict[str, Any], suffix: str, default: float) -> dict[str, float]:
    return {n: float(block.get(f"{n}_{suffix}", default)) for n in FINGERS}


def _opt_float(block: dict[str, Any], *keys: str) -> float | None:
    for k in keys:
        if k in block and block[k] is not None:
            return float(block[k])
    return None


def _parse_pinch(raw: dict[str, Any] | None) -> tuple[bool, dict[str, PinchKnot | None]]:
    out: dict[str, PinchKnot | None] = {n: None for n in FINGERS}
    if not raw:
        return True, out
    skipped = bool(raw.get("skipped", False))
    captured = 0
    for name in FINGERS:
        item = raw.get(name)
        if not isinstance(item, dict):
            continue
        lat = item.get("thumb_lat")
        if lat is None:
            continue
        out[name] = PinchKnot(
            thumb_lat=float(lat),
            q=_opt_float(item, "q"),
            thumb_mcp=_opt_float(item, "thumb_mcp"),
            thumb_ip=_opt_float(item, "thumb_ip"),
            finger_mcp=_opt_float(item, f"{name}_mcp", "finger_mcp", "mcp"),
            finger_pip=_opt_float(item, f"{name}_pip", "finger_pip", "pip"),
            finger_dip=_opt_float(item, f"{name}_dip", "finger_dip", "dip"),
        )
        captured += 1
    if captured == 0:
        skipped = True
    return skipped, out


def side_calib_from_dict(raw: dict[str, Any] | None) -> SideCurlCalib2:
    seed = seed_side_calib()
    if not raw:
        return seed
    open_b = raw.get("open") or {}
    fist_b = raw.get("fist") or {}
    pinky_b = raw.get("thumb_to_pinky") or raw.get("thumb_left") or {}
    along_b = raw.get("thumb_along") or {}
    flex_b = raw.get("thumb_flex") or {}
    fist_thumb_mcp = fist_b.get("thumb_mcp")
    fist_thumb_ip = fist_b.get("thumb_ip")
    pinch_skipped, pinch = _parse_pinch(raw.get("pinch"))
    ft_b = raw.get("fingers_together") or {}
    fs_b = raw.get("fingers_spread") or {}
    return SideCurlCalib2(
        open_mcp=_floats(open_b, "mcp", 0.0),
        open_pip=_floats(open_b, "pip", 0.0),
        open_dip=_floats(open_b, "dip", 0.0),
        fist_mcp=_floats(fist_b, "mcp", _SEED_FIST_MCP),
        fist_pip=_floats(fist_b, "pip", _SEED_FIST_PIP),
        fist_dip=_floats(fist_b, "dip", _SEED_FIST_DIP),
        open_thumb_mcp=float(open_b.get("thumb_mcp", 0.0)),
        open_thumb_ip=float(open_b.get("thumb_ip", 0.0)),
        thumb_flex_mcp=float(
            fist_thumb_mcp
            if fist_thumb_mcp is not None
            else flex_b.get("mcp", _SEED_THUMB_MCP)
        ),
        thumb_flex_ip=float(
            fist_thumb_ip
            if fist_thumb_ip is not None
            else flex_b.get("ip", _SEED_THUMB_IP)
        ),
        open_thumb_lat=float(open_b.get("thumb_lat", seed.open_thumb_lat)),
        thumb_to_pinky_lat=float(pinky_b.get("thumb_lat", seed.thumb_to_pinky_lat)),
        along_thumb_lat=_opt_float(along_b, "thumb_lat", "lat"),
        pinch_skipped=pinch_skipped,
        pinch=pinch,
        together_finger_span=_opt_float(ft_b, "finger_span", "span"),
        spread_finger_span=_opt_float(fs_b, "finger_span", "span"),
    )


def side_calib_to_dict(calib: SideCurlCalib2) -> dict[str, Any]:
    open_b: dict[str, float] = {}
    fist_b: dict[str, float] = {}
    for name in FINGERS:
        open_b[f"{name}_mcp"] = float(calib.open_mcp[name])
        open_b[f"{name}_pip"] = float(calib.open_pip[name])
        open_b[f"{name}_dip"] = float(calib.open_dip[name])
        fist_b[f"{name}_mcp"] = float(calib.fist_mcp[name])
        fist_b[f"{name}_pip"] = float(calib.fist_pip[name])
        fist_b[f"{name}_dip"] = float(calib.fist_dip[name])
    open_b["thumb_mcp"] = float(calib.open_thumb_mcp)
    open_b["thumb_ip"] = float(calib.open_thumb_ip)
    open_b["thumb_lat"] = float(calib.open_thumb_lat)
    fist_b["thumb_mcp"] = float(calib.thumb_flex_mcp)
    fist_b["thumb_ip"] = float(calib.thumb_flex_ip)
    pinch: dict[str, Any] = {"skipped": bool(calib.pinch_skipped)}
    for name in FINGERS:
        knot = (calib.pinch or {}).get(name)
        if knot is None:
            continue
        from xr_hand_retarget.algorithms.curl_xhand1 import _pinch_knot_to_item

        pinch[name] = _pinch_knot_to_item(name, knot)
    out: dict[str, Any] = {
        "open": open_b,
        "fist": fist_b,
        "thumb_to_pinky": {"thumb_lat": float(calib.thumb_to_pinky_lat)},
        "thumb_flex": {
            "mcp": float(calib.thumb_flex_mcp),
            "ip": float(calib.thumb_flex_ip),
        },
        "pinch": pinch,
    }
    if calib.along_thumb_lat is not None:
        out["thumb_along"] = {"thumb_lat": float(calib.along_thumb_lat)}
    if calib.together_finger_span is not None:
        out["fingers_together"] = {"finger_span": float(calib.together_finger_span)}
    if calib.spread_finger_span is not None:
        out["fingers_spread"] = {"finger_span": float(calib.spread_finger_span)}
    return out


def load_calib_file(path: str | Path) -> dict[str, SideCurlCalib2]:
    import yaml

    cfg_path = Path(path)
    if not cfg_path.is_file():
        raise FileNotFoundError(f"curl calib not found: {cfg_path}")
    with cfg_path.open("r", encoding="utf-8") as f:
        raw: dict[str, Any] = yaml.safe_load(f) or {}
    sides = raw.get("sides") or {}
    return {
        "left": side_calib_from_dict(sides.get("left")),
        "right": side_calib_from_dict(sides.get("right")),
    }


def dump_calib_file(
    path: str | Path,
    sides: dict[str, SideCurlCalib2],
    *,
    source: str = "xrobotoolkit",
    headset: str = "pico",
) -> None:
    import yaml

    payload: dict[str, Any] = {
        "schema": SCHEMA,
        "source": source,
        "headset": headset,
        "sides": {
            name: side_calib_to_dict(sides[name])
            for name in ("left", "right")
            if name in sides
        },
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# Wuji Hand 2 curl calibration (OpenXR 26 xyz).\n"
        "# schema: wuji_hand2_curl_calib.v1\n"
        "# open→q=0; fist four-finger flex→URDF upper; thumb_to_pinky→cmc_flex;\n"
        "# pinch: robot thumb_cmc_flex from URDF palm-X slot (hand_base +X).\n"
        "# Recapture: python -m xr_hand_retarget.calibrate --config configs/wuji_hand2.yaml\n"
    )
    with path.open("w", encoding="utf-8") as f:
        f.write(header)
        yaml.safe_dump(payload, f, sort_keys=False, allow_unicode=True)


def _flex_to_upper(t: float, hi: float) -> float:
    """Open at 0, fist toward URDF upper (Hand 2 flexion is +)."""
    return float(t) * max(float(hi), 0.0)


def _warn_degenerate_curl_span(side: str, calib: SideCurlCalib2) -> None:
    """Open≈fist makes _unit_t clip to {0,1}; robot only ever hits 0 or URDF upper."""
    bad: list[str] = []
    for name in FINGERS:
        for joint, open_v, fist_v in (
            ("mcp", calib.open_mcp[name], calib.fist_mcp[name]),
            ("pip", calib.open_pip[name], calib.fist_pip[name]),
            ("dip", calib.open_dip[name], calib.fist_dip[name]),
        ):
            span = float(fist_v) - float(open_v)
            if span < 0.15:
                bad.append(f"{name}_{joint} Δ={span:.3f}")
    t_mcp = float(calib.thumb_flex_mcp) - float(calib.open_thumb_mcp)
    t_ip = float(calib.thumb_flex_ip) - float(calib.open_thumb_ip)
    if t_mcp < 0.15:
        bad.append(f"thumb_mcp Δ={t_mcp:.3f}")
    if t_ip < 0.15:
        bad.append(f"thumb_ip Δ={t_ip:.3f}")
    if not bad:
        return
    print(
        f"[{side}] curl calib open≈fist ({', '.join(bad)}). "
        "t clips to 0 or 1 → joints only 0 or URDF upper. "
        "Re-capture a real fist (stop vr-xrt first):\n"
        "  python -m xr_hand_retarget.calibrate "
        "--config xr_hand_retarget/configs/wuji_hand2.yaml "
        f"--side {side}",
        flush=True,
    )


def pinch_q_from_urdf(
    fk,
    q_rest: np.ndarray,
    *,
    target: str = "pip",
    axis: int = 0,
    opp_index: int = 0,
    q_lo: float = 0.0,
    q_hi: float = 1.291,
    nscan: int = _Q_SCAN,
) -> dict[str, float]:
    """Opposition joint that puts thumb_tip[axis] on each finger slot."""
    key = (target or "pip").lower()
    links = _PINCH_LINKS.get(key) or _PINCH_LINKS["pip"]
    q_slots = np.zeros(20, dtype=np.float64)
    targets = {
        name: float(fk.link_position(q_slots, link)[axis])
        for name, link in links.items()
    }
    q = np.asarray(q_rest, dtype=np.float64).copy()
    qs = np.linspace(float(q_lo), float(q_hi), int(max(nscan, 9)))
    tip_x = np.empty(qs.size, dtype=np.float64)
    for i, t in enumerate(qs):
        q[opp_index] = float(t)
        tip_x[i] = float(fk.link_position(q, "thumb_tip")[axis])
    out: dict[str, float] = {}
    for name, tx in targets.items():
        i = int(np.argmin(np.abs(tip_x - tx)))
        out[name] = float(qs[i])
    return out


def _thumb_opp(
    feat: CurlFeatures2,
    calib: SideCurlCalib2,
    *,
    hi0: float,
    urdf_q: dict[str, float] | None,
) -> float:
    pinky_q = float(hi0)
    if urdf_q and "pinky" in urdf_q:
        pinky_q = float(urdf_q["pinky"])
    xs = [calib.open_thumb_lat, calib.thumb_to_pinky_lat]
    qs = [0.0, pinky_q]
    if not calib.pinch_skipped:
        for name in FINGERS:
            knot = (calib.pinch or {}).get(name)
            if knot is None:
                continue
            if knot.q is not None:
                qk = float(knot.q)
            elif urdf_q and name in urdf_q:
                qk = float(urdf_q[name])
            else:
                continue
            xs.append(float(knot.thumb_lat))
            qs.append(float(np.clip(qk, 0.0, hi0)))
    return float(np.clip(_interp_knots(feat.thumb_lat, xs, qs), 0.0, hi0))


def map_curl_features2(
    feat: CurlFeatures2,
    calib: SideCurlCalib2,
    *,
    limits: np.ndarray,
    gains: WujiCurlGains,
    urdf_q: dict[str, float] | None = None,
) -> np.ndarray:
    lo = limits[:, 0]
    hi = limits[:, 1]
    q = np.zeros(20, dtype=np.float64)

    q[_THUMB_MCP] = _flex_to_upper(
        _unit_t(feat.thumb_mcp, calib.open_thumb_mcp, calib.thumb_flex_mcp),
        float(hi[_THUMB_MCP]),
    )
    q[_THUMB_IP] = _flex_to_upper(
        _unit_t(feat.thumb_ip, calib.open_thumb_ip, calib.thumb_flex_ip),
        float(hi[_THUMB_IP]),
    )
    q[_THUMB_FLEX] = _thumb_opp(
        feat, calib, hi0=max(float(hi[_THUMB_FLEX]), 0.0), urdf_q=urdf_q
    )
    q[_THUMB_ABD] = 0.0

    dip_mode = (gains.dip_mode or "couple").lower()
    ratio = float(gains.dip_couple_ratio)
    for name, (i_flex, i_abd, i_pip, i_dip) in _FINGER_IDX.items():
        q[i_flex] = _flex_to_upper(
            _unit_t(feat.mcp[name], calib.open_mcp[name], calib.fist_mcp[name]),
            float(hi[i_flex]),
        )
        q[i_abd] = 0.0
        q[i_pip] = _flex_to_upper(
            _unit_t(feat.pip[name], calib.open_pip[name], calib.fist_pip[name]),
            float(hi[i_pip]),
        )
        if dip_mode == "calib":
            q[i_dip] = _flex_to_upper(
                _unit_t(feat.dip[name], calib.open_dip[name], calib.fist_dip[name]),
                float(hi[i_dip]),
            )
        else:
            q[i_dip] = float(np.clip(ratio * q[i_pip], lo[i_dip], hi[i_dip]))

    return np.clip(q, lo, hi)


class CurlHand2Retargeter:
    def __init__(
        self,
        side: str,
        gains: WujiCurlGains | None = None,
        calib: SideCurlCalib2 | None = None,
    ):
        if side not in ("left", "right"):
            raise ValueError(f"side must be left|right, got {side!r}")
        self.side = side
        self.gains = gains or WujiCurlGains()
        limits, _vel = load_hand2_limits(side, urdf_path=self.gains.urdf_path)
        self.limits = limits
        if calib is not None:
            self.calib = calib
        elif self.gains.calib_path is not None and Path(self.gains.calib_path).is_file():
            self.calib = load_calib_file(self.gains.calib_path)[side]
            extra = "" if self.calib.pinch_skipped else " pinch=on"
            print(f"[{side}] curl calib={self.gains.calib_path}{extra}", flush=True)
            _warn_degenerate_curl_span(side, self.calib)
        else:
            missing = self.gains.calib_path
            if missing is not None:
                print(f"[{side}] curl calib missing ({missing}); using seed", flush=True)
            self.calib = seed_side_calib()
        self._fk = None
        self._q_cache: dict[tuple[float, float], dict[str, float]] = {}
        try:
            self._fk = make_hand2_fk(side, urdf_path=self.gains.urdf_path)
            print(
                f"[{side}] curl pinch URDF={self._fk.urdf_path.name} "
                f"axis={self.gains.pinch_axis} target={self.gains.pinch_target_link}",
                flush=True,
            )
        except Exception as exc:
            print(f"[{side}] curl URDF pinch table unavailable: {exc}", flush=True)
        self._prev: np.ndarray | None = None

    def reset(self) -> None:
        self._prev = None
        self._q_cache.clear()

    def _urdf_q(self, q_mcp: float, q_ip: float) -> dict[str, float] | None:
        if self._fk is None:
            return None
        key = (round(float(q_mcp), 2), round(float(q_ip), 2))
        hit = self._q_cache.get(key)
        if hit is not None:
            return hit
        q_rest = np.zeros(20, dtype=np.float64)
        q_rest[_THUMB_MCP] = float(q_mcp)
        q_rest[_THUMB_IP] = float(q_ip)
        hi = max(float(self.limits[_THUMB_FLEX, 1]), 0.0)
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
        feat = extract_curl_features2(joints26, self.side)
        hi = self.limits[:, 1]
        q_mcp = _flex_to_upper(
            _unit_t(feat.thumb_mcp, self.calib.open_thumb_mcp, self.calib.thumb_flex_mcp),
            float(hi[_THUMB_MCP]),
        )
        q_ip = _flex_to_upper(
            _unit_t(feat.thumb_ip, self.calib.open_thumb_ip, self.calib.thumb_flex_ip),
            float(hi[_THUMB_IP]),
        )
        q = map_curl_features2(
            feat,
            self.calib,
            limits=self.limits,
            gains=self.gains,
            urdf_q=self._urdf_q(q_mcp, q_ip),
        )
        alpha = float(np.clip(self.gains.output_alpha, 1e-3, 1.0))
        if self._prev is None or alpha >= 1.0 - 1e-12:
            self._prev = q
            return q
        blended = (1.0 - alpha) * self._prev + alpha * q
        self._prev = blended
        return blended


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


class WujiCurlBackend:
    """OpenXR 26 → curl 20-DOF. No official wuji-retargeting / Pinocchio."""

    def __init__(self, side: str, cfg):
        if side not in ("left", "right"):
            raise ValueError(f"side must be left|right, got {side!r}")
        if cfg.wuji_curl is None:
            raise ValueError("wuji curl backend requires retargeting.curl")
        self.side = side
        self.dof = int(cfg.dof)
        self.retargeting_type = "curl"
        self.vector_profile = ""
        self._cfg = cfg
        self._inner = CurlHand2Retargeter(side, cfg.wuji_curl)
        if self._inner.limits.shape[0] != self.dof:
            self.dof = int(self._inner.limits.shape[0])
        dt = 1.0 / max(float(cfg.rate_hz), 1.0)
        vel = max(float(cfg.max_joint_vel_rad_s), 0.0)
        self._max_delta = vel * dt if vel > 0.0 else 0.0
        self._q_hold: np.ndarray | None = None
        self._was_held = True
        self._unlock = 0
        print(
            f"[retarget] side={side} type=curl engine=CurlHand2Retargeter "
            f"dof={self.dof} dip={cfg.wuji_curl.dip_mode} "
            f"max_dq={self._max_delta:.4f}rad/step",
            flush=True,
        )

    def _hold(self, active: int, xyz_abs: float, reason: str, *, reset_unlock: bool = True) -> SideStep:
        if reset_unlock:
            self._unlock = 0
        self._was_held = True
        q = (
            np.zeros(self.dof, dtype=np.float64)
            if self._q_hold is None
            else self._q_hold.copy()
        )
        return SideStep(q=q, active=active, held=True, xyz_abs=xyz_abs, reason=reason)

    def _shape_q(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=np.float64).reshape(-1)
        if q.shape[0] != self.dof:
            pad = np.zeros(self.dof, dtype=np.float64)
            n = min(q.shape[0], self.dof)
            pad[:n] = q[:n]
            q = pad
        q = np.clip(q, self._inner.limits[:, 0], self._inner.limits[:, 1])
        if self._max_delta > 0.0 and self._q_hold is not None:
            q = self._q_hold + np.clip(
                q - self._q_hold, -self._max_delta, self._max_delta
            )
        return q

    def step(self, frame) -> SideStep:
        from xr_hand_retarget.sources.frame import HandFrame

        if not isinstance(frame, HandFrame):
            raise TypeError(f"expected HandFrame, got {type(frame)!r}")
        active = int(frame.active)
        raw = frame.ensure_joints26()
        xyz_abs = float(frame.xyz_abs)
        if self._cfg.hold_on_inactive and active == 0:
            return self._hold(active, xyz_abs, "inactive")
        if self._cfg.hold_on_zero_pose and pose_is_zero(raw):
            return self._hold(active, xyz_abs, "zero")

        mp21 = frame.ensure_xyz21()
        area = palm_triangle_area_m2(mp21)
        if area < float(self._cfg.min_palm_area_m2):
            return self._hold(active, xyz_abs, f"palm={area:.2e}")

        need = max(int(self._cfg.unlock_frames), 1)
        if self._was_held:
            self._unlock += 1
            if self._unlock < need:
                return self._hold(
                    active, xyz_abs, f"unlock {self._unlock}/{need}", reset_unlock=False
                )
            self._was_held = False
            self._unlock = need
            self._inner.reset()

        q = self._shape_q(self._inner.retarget(raw))
        self._q_hold = q.copy()
        return SideStep(q=q, active=active, held=False, xyz_abs=xyz_abs, reason="ok")
