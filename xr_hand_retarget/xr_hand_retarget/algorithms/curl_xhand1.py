"""Open / fist / thumb-to-pinky / optional pinch → XHand1 lerp.

Open palm (thumb perpendicular) → q=0.
Four-finger fist → URDF upper (thumb not sampled).
Thumb to pinky → opposition toward pinky.
Optional pinch knots: human thumb_lat → URDF thumb_joint1 that places
thumb_tip on that finger's palm-lateral slot (PIP/MCP/tip Y in hand_base).
Index abduction is calibrated separately on Pico: fingers-together → 0,
index opened to the tracking-good limit → URDF positive limit.
If those two poses are missing, index_joint1 stays 0.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from .kinematics_xhand1 import XHandFK, make_hand_fk
from .remap import (
    XHAND1_LIMITS,
    finger_chain_flexion,
    index_abduction_lateral_m,
    thumb_chain_flexion,
    thumb_tip_palm_x_m,
)

FINGERS = ("index", "middle", "ring", "pinky")
SCHEMA = "xhand1_curl_calib.v4"
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
_SEED_THUMB_MCP = 1.10
_SEED_THUMB_IP = 1.05
_SEED_OPEN_THUMB_LAT = 0.045
_SEED_PINKY_THUMB_LAT = -0.025
_Q1_SCAN = 65


@dataclass
class CurlFeatures:
    mcp: dict[str, float]
    pip: dict[str, float]
    thumb_mcp: float
    thumb_ip: float
    thumb_lat: float
    index_lat: float


@dataclass
class PinchKnot:
    thumb_lat: float
    q: float | None = None
    # Optional human flex at pinch (mp_curl / richer curl dumps).
    thumb_mcp: float | None = None
    thumb_ip: float | None = None
    finger_mcp: float | None = None
    finger_pip: float | None = None
    finger_dip: float | None = None


@dataclass
class SideCurlCalib:
    open_mcp: dict[str, float]
    open_pip: dict[str, float]
    fist_mcp: dict[str, float]
    fist_pip: dict[str, float]
    open_thumb_mcp: float
    open_thumb_ip: float
    thumb_flex_mcp: float
    thumb_flex_ip: float
    open_thumb_lat: float
    thumb_to_pinky_lat: float
    pinch_skipped: bool = True
    pinch: dict[str, PinchKnot | None] = field(default_factory=dict)
    together_index_lat: float | None = None
    abd_index_lat: float | None = None

    @property
    def thumb_left_lat(self) -> float:
        return self.thumb_to_pinky_lat


@dataclass
class CurlCalibGains:
    calib_path: Path | None = None
    output_alpha: float = 0.85
    pinch_target_link: str = "pip"
    urdf_path: str | None = None
    finger_scale: dict[str, float] | None = None


def extract_curl_features(joints26: np.ndarray, side: str) -> CurlFeatures:
    joints26 = np.asarray(joints26, dtype=np.float64)
    mcp: dict[str, float] = {}
    pip: dict[str, float] = {}
    for name in FINGERS:
        mcp[name], pip[name] = finger_chain_flexion(joints26, name)
    t_mcp, t_ip = thumb_chain_flexion(joints26)
    return CurlFeatures(
        mcp=mcp,
        pip=pip,
        thumb_mcp=t_mcp,
        thumb_ip=t_ip,
        thumb_lat=thumb_tip_palm_x_m(joints26, side),
        index_lat=index_abduction_lateral_m(joints26, side),
    )


def seed_side_calib() -> SideCurlCalib:
    zeros = {n: 0.0 for n in FINGERS}
    return SideCurlCalib(
        open_mcp=dict(zeros),
        open_pip=dict(zeros),
        fist_mcp={n: _SEED_FIST_MCP for n in FINGERS},
        fist_pip={n: _SEED_FIST_PIP for n in FINGERS},
        open_thumb_mcp=0.0,
        open_thumb_ip=0.0,
        thumb_flex_mcp=_SEED_THUMB_MCP,
        thumb_flex_ip=_SEED_THUMB_IP,
        open_thumb_lat=_SEED_OPEN_THUMB_LAT,
        thumb_to_pinky_lat=_SEED_PINKY_THUMB_LAT,
        pinch_skipped=True,
        pinch={n: None for n in FINGERS},
        together_index_lat=None,
        abd_index_lat=None,
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


def side_calib_from_dict(raw: dict[str, Any] | None) -> SideCurlCalib:
    seed = seed_side_calib()
    if not raw:
        return seed
    open_b = raw.get("open") or {}
    fist_b = raw.get("fist") or {}
    pinky_b = raw.get("thumb_to_pinky") or raw.get("thumb_left") or {}
    flex_b = raw.get("thumb_flex") or {}
    pinch_skipped, pinch = _parse_pinch(raw.get("pinch"))
    together_b = raw.get("index_together") or {}
    abd_b = raw.get("index_abd") or {}
    return SideCurlCalib(
        open_mcp=_floats(open_b, "mcp", 0.0),
        open_pip=_floats(open_b, "pip", 0.0),
        fist_mcp=_floats(fist_b, "mcp", _SEED_FIST_MCP),
        fist_pip=_floats(fist_b, "pip", _SEED_FIST_PIP),
        open_thumb_mcp=float(open_b.get("thumb_mcp", 0.0)),
        open_thumb_ip=float(open_b.get("thumb_ip", 0.0)),
        thumb_flex_mcp=float(flex_b.get("mcp", _SEED_THUMB_MCP)),
        thumb_flex_ip=float(flex_b.get("ip", _SEED_THUMB_IP)),
        open_thumb_lat=float(open_b.get("thumb_lat", seed.open_thumb_lat)),
        thumb_to_pinky_lat=float(
            pinky_b.get("thumb_lat", seed.thumb_to_pinky_lat)
        ),
        pinch_skipped=pinch_skipped,
        pinch=pinch,
        together_index_lat=_opt_float(together_b, "index_lat", "lat"),
        abd_index_lat=_opt_float(abd_b, "index_lat", "lat"),
    )


def _pinch_knot_to_item(name: str, knot: PinchKnot) -> dict[str, float]:
    item: dict[str, float] = {"thumb_lat": float(knot.thumb_lat)}
    if knot.q is not None:
        item["q"] = float(knot.q)
    if knot.thumb_mcp is not None:
        item["thumb_mcp"] = float(knot.thumb_mcp)
    if knot.thumb_ip is not None:
        item["thumb_ip"] = float(knot.thumb_ip)
    if knot.finger_mcp is not None:
        item[f"{name}_mcp"] = float(knot.finger_mcp)
    if knot.finger_pip is not None:
        item[f"{name}_pip"] = float(knot.finger_pip)
    if knot.finger_dip is not None:
        item[f"{name}_dip"] = float(knot.finger_dip)
    return item


def _pinch_to_dict(calib: SideCurlCalib) -> dict[str, Any]:
    block: dict[str, Any] = {"skipped": bool(calib.pinch_skipped)}
    for name in FINGERS:
        knot = (calib.pinch or {}).get(name)
        if knot is None:
            continue
        block[name] = _pinch_knot_to_item(name, knot)
    return block


def side_calib_to_dict(calib: SideCurlCalib) -> dict[str, Any]:
    open_b: dict[str, float] = {}
    fist_b: dict[str, float] = {}
    for name in FINGERS:
        open_b[f"{name}_mcp"] = float(calib.open_mcp[name])
        open_b[f"{name}_pip"] = float(calib.open_pip[name])
        fist_b[f"{name}_mcp"] = float(calib.fist_mcp[name])
        fist_b[f"{name}_pip"] = float(calib.fist_pip[name])
    open_b["thumb_mcp"] = float(calib.open_thumb_mcp)
    open_b["thumb_ip"] = float(calib.open_thumb_ip)
    open_b["thumb_lat"] = float(calib.open_thumb_lat)
    out: dict[str, Any] = {
        "open": open_b,
        "fist": fist_b,
        "thumb_to_pinky": {"thumb_lat": float(calib.thumb_to_pinky_lat)},
        "thumb_flex": {
            "mcp": float(calib.thumb_flex_mcp),
            "ip": float(calib.thumb_flex_ip),
        },
        "pinch": _pinch_to_dict(calib),
    }
    if calib.together_index_lat is not None:
        out["index_together"] = {"index_lat": float(calib.together_index_lat)}
    if calib.abd_index_lat is not None:
        out["index_abd"] = {"index_lat": float(calib.abd_index_lat)}
    return out


def load_calib_file(path: str | Path) -> dict[str, SideCurlCalib]:
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
    sides: dict[str, SideCurlCalib],
    *,
    source: str = "xrobotoolkit",
    headset: str = "pico",
    extra: dict[str, Any] | None = None,
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
    if extra:
        payload.update(extra)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# XHand1 curl calibration (OpenXR 26 xyz).\n"
        "# schema: xhand1_curl_calib.v4\n"
        "# open=伸直(拇指与掌垂直)→q0; fist=握拳四指→限位;\n"
        "# thumb_to_pinky=对掌到小指侧;\n"
        "# index_together=五指并拢→index_joint1=0;\n"
        "# index_abd=食指外开到极限(Pico 追踪好时)→正极限;\n"
        "# pinch=可选，拇指到各指下方；robot thumb_joint1 由 URDF 指槽反求.\n"
        "# Recapture: python -m xr_hand_retarget.calibrate --config xr_hand_retarget/configs/xhand1.yaml\n"
    )
    with path.open("w", encoding="utf-8") as f:
        f.write(header)
        yaml.safe_dump(payload, f, sort_keys=False, allow_unicode=True)


def _unit_t(x: float, x0: float, x1: float) -> float:
    span = float(x1) - float(x0)
    if abs(span) < 1e-6:
        return 0.0
    return float(np.clip((float(x) - float(x0)) / span, 0.0, 1.0))


def _interp_knots(x: float, xs: list[float], qs: list[float]) -> float:
    if not xs:
        return 0.0
    if len(xs) == 1:
        return float(qs[0])
    order = np.argsort(np.asarray(xs, dtype=np.float64))
    xa = np.asarray(xs, dtype=np.float64)[order]
    qa = np.asarray(qs, dtype=np.float64)[order]
    uniq_x, uniq_idx = np.unique(xa, return_index=True)
    uniq_q = qa[uniq_idx]
    if uniq_x.size == 1:
        return float(uniq_q[0])
    return float(np.interp(float(x), uniq_x, uniq_q))


def pinch_q1_from_urdf(
    fk: XHandFK,
    q2: float,
    q3: float,
    *,
    target: str = "pip",
    q1_lo: float = 0.0,
    q1_hi: float = 1.832,
    nscan: int = _Q1_SCAN,
) -> dict[str, float]:
    """thumb_joint1 that puts thumb_tip on each finger's palm-lateral slot (URDF Y)."""
    key = (target or "pip").lower()
    links = _PINCH_LINKS.get(key) or _PINCH_LINKS["pip"]
    q0 = np.zeros(12, dtype=np.float64)
    targets_y = {
        name: float(fk.link_position(q0, link)[1]) for name, link in links.items()
    }
    q = np.zeros(12, dtype=np.float64)
    q[1] = float(q2)
    q[2] = float(q3)
    qs = np.linspace(float(q1_lo), float(q1_hi), int(max(nscan, 9)))
    tip_y = np.empty(qs.size, dtype=np.float64)
    for i, t in enumerate(qs):
        q[0] = float(t)
        tip_y[i] = float(fk.link_position(q, "thumb_tip")[1])
    out: dict[str, float] = {}
    for name, ty in targets_y.items():
        i = int(np.argmin(np.abs(tip_y - ty)))
        out[name] = float(qs[i])
    return out


def _thumb_j1(
    feat: CurlFeatures,
    calib: SideCurlCalib,
    *,
    hi0: float,
    urdf_q1: dict[str, float] | None,
) -> float:
    pinky_q = float(hi0)
    if urdf_q1 and "pinky" in urdf_q1:
        pinky_q = float(urdf_q1["pinky"])
    xs = [calib.open_thumb_lat, calib.thumb_to_pinky_lat]
    qs = [0.0, pinky_q]
    if not calib.pinch_skipped:
        for name in FINGERS:
            knot = (calib.pinch or {}).get(name)
            if knot is None:
                continue
            if knot.q is not None:
                qk = float(knot.q)
            elif urdf_q1 and name in urdf_q1:
                qk = float(urdf_q1[name])
            else:
                continue
            xs.append(float(knot.thumb_lat))
            qs.append(float(np.clip(qk, 0.0, hi0)))
    return float(np.clip(_interp_knots(feat.thumb_lat, xs, qs), 0.0, hi0))


def map_curl_features(
    feat: CurlFeatures,
    calib: SideCurlCalib,
    *,
    limits: np.ndarray | None = None,
    gains: CurlCalibGains | None = None,
    fk: XHandFK | None = None,
    urdf_q1: dict[str, float] | None = None,
) -> np.ndarray:
    """Map live features → q[12]. index_joint1 from together/abd endpoints, else 0."""
    lim = XHAND1_LIMITS if limits is None else np.asarray(limits, dtype=np.float64)
    lo = lim[:, 0]
    hi = lim[:, 1]
    q = np.zeros(12, dtype=np.float64)

    q[1] = _unit_t(feat.thumb_mcp, calib.open_thumb_mcp, calib.thumb_flex_mcp) * hi[1]
    q[2] = _unit_t(feat.thumb_ip, calib.open_thumb_ip, calib.thumb_flex_ip) * hi[2]
    table = urdf_q1
    if table is None and fk is not None:
        table = pinch_q1_from_urdf(
            fk,
            q[1],
            q[2],
            target=(gains.pinch_target_link if gains else "pip"),
            q1_lo=float(lo[0]),
            q1_hi=float(hi[0]),
        )
    q[0] = _thumb_j1(feat, calib, hi0=float(hi[0]), urdf_q1=table)
    q[3] = 0.0
    if (
        calib.together_index_lat is not None
        and calib.abd_index_lat is not None
        and abs(float(calib.abd_index_lat) - float(calib.together_index_lat)) > 1e-4
    ):
        q[3] = _unit_t(
            feat.index_lat, calib.together_index_lat, calib.abd_index_lat
        ) * float(hi[3])

    flex_slots = (
        ("index", 4, 5),
        ("middle", 6, 7),
        ("ring", 8, 9),
        ("pinky", 10, 11),
    )
    for name, i_mcp, i_pip in flex_slots:
        q[i_mcp] = (
            _unit_t(feat.mcp[name], calib.open_mcp[name], calib.fist_mcp[name])
            * hi[i_mcp]
        )
        q[i_pip] = (
            _unit_t(feat.pip[name], calib.open_pip[name], calib.fist_pip[name])
            * hi[i_pip]
        )
        # T_xhand per-finger scale (e.g. long pinky) — not a user/calib knob.
        if gains is not None and gains.finger_scale:
            s = float(gains.finger_scale.get(name, 1.0))
            if abs(s - 1.0) > 1e-9:
                q[i_mcp] = float(np.clip(q[i_mcp] * s, lo[i_mcp], hi[i_mcp]))
                q[i_pip] = float(np.clip(q[i_pip] * s, lo[i_pip], hi[i_pip]))

    return np.clip(q, lo, hi)


class CurlCalibRetargeter:
    def __init__(
        self,
        side: str,
        limits: np.ndarray,
        gains: CurlCalibGains | None = None,
        calib: SideCurlCalib | None = None,
    ):
        if side not in ("left", "right"):
            raise ValueError(f"side must be left|right, got {side!r}")
        self.side = side
        self.limits = np.asarray(limits, dtype=np.float64)
        self.gains = gains or CurlCalibGains()
        if calib is not None:
            self.calib = calib
        elif self.gains.calib_path is not None and Path(self.gains.calib_path).is_file():
            self.calib = load_calib_file(self.gains.calib_path)[side]
            extra = "" if self.calib.pinch_skipped else " pinch=on"
            if (
                self.calib.together_index_lat is not None
                and self.calib.abd_index_lat is not None
            ):
                extra += " index_abd=on"
            print(f"[{side}] curl calib={self.gains.calib_path}{extra}", flush=True)
        else:
            missing = self.gains.calib_path
            if missing is not None:
                print(f"[{side}] curl calib missing ({missing}); using seed", flush=True)
            self.calib = seed_side_calib()
        self._fk: XHandFK | None = None
        self._q1_cache: dict[tuple[float, float], dict[str, float]] = {}
        try:
            self._fk = make_hand_fk(side, urdf_path=self.gains.urdf_path)
            print(
                f"[{side}] curl pinch URDF={self._fk.urdf_path.name} "
                f"target={self.gains.pinch_target_link}",
                flush=True,
            )
        except Exception as exc:
            print(f"[{side}] curl URDF pinch table unavailable: {exc}", flush=True)
        self._prev: np.ndarray | None = None

    def reset(self) -> None:
        self._prev = None
        self._q1_cache.clear()

    def _urdf_q1(self, q2: float, q3: float) -> dict[str, float] | None:
        if self._fk is None:
            return None
        key = (round(float(q2), 2), round(float(q3), 2))
        hit = self._q1_cache.get(key)
        if hit is not None:
            return hit
        lo, hi = float(self.limits[0, 0]), float(self.limits[0, 1])
        table = pinch_q1_from_urdf(
            self._fk,
            q2,
            q3,
            target=self.gains.pinch_target_link,
            q1_lo=lo,
            q1_hi=hi,
        )
        self._q1_cache[key] = table
        return table

    def retarget(self, joints26: np.ndarray) -> np.ndarray:
        feat = extract_curl_features(joints26, self.side)
        lim = self.limits
        hi = lim[:, 1]
        q2 = _unit_t(feat.thumb_mcp, self.calib.open_thumb_mcp, self.calib.thumb_flex_mcp) * hi[1]
        q3 = _unit_t(feat.thumb_ip, self.calib.open_thumb_ip, self.calib.thumb_flex_ip) * hi[2]
        q = map_curl_features(
            feat,
            self.calib,
            limits=lim,
            gains=self.gains,
            fk=None,
            urdf_q1=self._urdf_q1(q2, q3),
        )
        alpha = float(np.clip(self.gains.output_alpha, 1e-3, 1.0))
        if self._prev is None or alpha >= 1.0 - 1e-12:
            self._prev = q
            return q
        blended = (1.0 - alpha) * self._prev + alpha * q
        self._prev = blended
        return blended
