"""Open / fist / thumb-to-pinky / optional pinch → LinkerHand O6/L6/O7.

Same human OpenXR features as Wuji/XHand1 curl.

  O6/L6 q = [thumb_joint1, thumb_joint2, index, middle, ring, pinky]
  O7    q = [thumb_joint1, thumb_joint2, thumb_joint3, index, middle, ring, pinky]

Runtime for o6/l6/o7 is PalmTipRetargeter (palm_tip.py). CurlO6Retargeter
is the unused angle-synergy fallback (O7 mix of opposition+curl is only
in that path).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from xr_hand_retarget.algorithms.curl_xhand1 import _interp_knots, _unit_t
from xr_hand_retarget.algorithms.safety_xhand1 import pose_is_zero

from xr_hand_retarget.algorithms.curl_hand2 import (
    CurlFeatures2,
    SideCurlCalib2,
    extract_curl_features2,
    load_calib_file,
    seed_side_calib,
    side_calib_to_dict,
)
from xr_hand_retarget.kinematics import (
    default_linker_urdf,
    load_linker_limits,
    make_linker_fk,
)
from xr_hand_retarget.safety import JointMotionFilter, MotionSafetyGains
from xr_hand_retarget.landmarks import openxr26_to_mediapipe21, palm_triangle_area_m2
from xr_hand_retarget.pipeline import SideStep

FINGERS = ("index", "middle", "ring", "pinky")
SCHEMA = "linkerhand_curl_calib.v1"

_THUMB_FLEX = 0
_THUMB_YAW = 1
_THUMB3 = 2
_FINGER_IDX_6 = {"index": 2, "middle": 3, "ring": 4, "pinky": 5}
_FINGER_IDX_7 = {"index": 3, "middle": 4, "ring": 5, "pinky": 6}

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

_Q_SCAN = 65


def dump_calib_file(
    path: str | Path,
    sides: dict[str, SideCurlCalib2],
    *,
    source: str = "xrobotoolkit",
    headset: str = "pico",
) -> None:
    import yaml

    payload = {
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
        "# LinkerHand curl calibration (OpenXR 26 xyz).\n"
        "# schema: linkerhand_curl_calib.v1\n"
        "# open→q=0; fist four-finger flex→URDF upper; thumb_flex→thumb_joint1;\n"
        "# thumb_to_pinky→thumb_joint2; pinch: robot opposition from URDF palm-Y.\n"
        "# Recapture: python -m xr_hand_retarget.calibrate --config configs/{o6,l6,o7}.yaml\n"
    )
    with path.open("w", encoding="utf-8") as f:
        f.write(header)
        yaml.safe_dump(payload, f, sort_keys=False, allow_unicode=True)


def _flex_to_upper(t: float, hi: float) -> float:
    return float(t) * max(float(hi), 0.0)


def _weighted_finger_t(
    feat: CurlFeatures2,
    calib: SideCurlCalib2,
    name: str,
    weights: tuple[float, float, float],
) -> float:
    w = np.asarray(weights, dtype=np.float64)
    s = float(w.sum())
    if s <= 1e-9:
        w = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        s = 1.0
    w = w / s
    t_mcp = _unit_t(feat.mcp[name], calib.open_mcp[name], calib.fist_mcp[name])
    t_pip = _unit_t(feat.pip[name], calib.open_pip[name], calib.fist_pip[name])
    t_dip = _unit_t(feat.dip[name], calib.open_dip[name], calib.fist_dip[name])
    return float(np.clip(w[0] * t_mcp + w[1] * t_pip + w[2] * t_dip, 0.0, 1.0))


def _weighted_thumb_t(
    feat: CurlFeatures2,
    calib: SideCurlCalib2,
    weights: tuple[float, float],
) -> float:
    w = np.asarray(weights, dtype=np.float64)
    s = float(w.sum())
    if s <= 1e-9:
        w = np.array([1.0, 0.0], dtype=np.float64)
        s = 1.0
    w = w / s
    t_mcp = _unit_t(feat.thumb_mcp, calib.open_thumb_mcp, calib.thumb_flex_mcp)
    t_ip = _unit_t(feat.thumb_ip, calib.open_thumb_ip, calib.thumb_flex_ip)
    return float(np.clip(w[0] * t_mcp + w[1] * t_ip, 0.0, 1.0))


def pinch_q_from_urdf(
    fk,
    q_rest: np.ndarray,
    *,
    target: str = "pip",
    axis: int = 1,
    opp_index: int = 1,
    q_lo: float = 0.0,
    q_hi: float = 1.36,
    nscan: int = _Q_SCAN,
) -> dict[str, float]:
    """Opposition joint that puts thumb_tip[axis] on each finger slot."""
    key = (target or "pip").lower()
    links = _PINCH_LINKS.get(key) or _PINCH_LINKS["pip"]
    q_slots = np.zeros(int(np.asarray(q_rest).reshape(-1).shape[0]), dtype=np.float64)
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


def _thumb_yaw(
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


def map_curl_features_o6(
    feat: CurlFeatures2,
    calib: SideCurlCalib2,
    *,
    limits: np.ndarray,
    finger_weights: tuple[float, float, float],
    thumb_weights: tuple[float, float],
    urdf_q: dict[str, float] | None = None,
    model: str = "o6",
    thumb3_opp: float = 0.65,
    thumb3_curl: float = 0.35,
) -> np.ndarray:
    lo = limits[:, 0]
    hi = limits[:, 1]
    dof = int(lo.shape[0])
    q = np.zeros(dof, dtype=np.float64)
    t_curl = _weighted_thumb_t(feat, calib, thumb_weights)
    q[_THUMB_FLEX] = _flex_to_upper(t_curl, float(hi[_THUMB_FLEX]))
    q[_THUMB_YAW] = _thumb_yaw(
        feat, calib, hi0=max(float(hi[_THUMB_YAW]), 0.0), urdf_q=urdf_q
    )
    fingers = _FINGER_IDX_7 if dof >= 7 else _FINGER_IDX_6
    if dof >= 7:
        t_opp_unit = q[_THUMB_YAW] / max(float(hi[_THUMB_YAW]), 1e-9)
        mix = float(thumb3_opp) * t_opp_unit + float(thumb3_curl) * t_curl
        q[_THUMB3] = _flex_to_upper(float(np.clip(mix, 0.0, 1.0)), float(hi[_THUMB3]))
    for name, idx in fingers.items():
        q[idx] = _flex_to_upper(
            _weighted_finger_t(feat, calib, name, finger_weights),
            float(hi[idx]),
        )
    return np.clip(q, lo, hi)


class CurlO6Retargeter:
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
        if calib is not None:
            self.calib = calib
        elif self.gains.calib_path is not None and Path(self.gains.calib_path).is_file():
            self.calib = load_calib_file(self.gains.calib_path)[side]
            extra = "" if self.calib.pinch_skipped else " pinch=on"
            print(
                f"[{side}] {self.model} curl calib={self.gains.calib_path}{extra}",
                flush=True,
            )
        else:
            missing = self.gains.calib_path
            if missing is not None:
                print(
                    f"[{side}] {self.model} curl calib missing ({missing}); using seed",
                    flush=True,
                )
            self.calib = seed_side_calib()
        self._fk = None
        self._q_cache: dict[float, dict[str, float]] = {}
        try:
            self._fk = make_linker_fk(
                side, model=self.model, urdf_path=self.gains.urdf_path
            )
            print(
                f"[{side}] {self.model} pinch URDF={self._fk.urdf_path.name} "
                f"axis={self.gains.pinch_axis} target={self.gains.pinch_target_link}",
                flush=True,
            )
        except Exception as exc:
            print(
                f"[{side}] {self.model} URDF pinch table unavailable: {exc}",
                flush=True,
            )
        self._prev: np.ndarray | None = None

    def reset(self) -> None:
        self._prev = None
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
        feat = extract_curl_features2(joints26, self.side)
        q_flex = _flex_to_upper(
            _weighted_thumb_t(
                feat, self.calib, self.gains.thumb_flex_weights
            ),
            float(self.limits[_THUMB_FLEX, 1]),
        )
        q = map_curl_features_o6(
            feat,
            self.calib,
            limits=self.limits,
            finger_weights=self.gains.finger_flex_weights,
            thumb_weights=self.gains.thumb_flex_weights,
            urdf_q=self._urdf_q(q_flex),
            model=self.model,
            thumb3_opp=float(getattr(self.gains, "thumb3_opp", 0.65)),
            thumb3_curl=float(getattr(self.gains, "thumb3_curl", 0.35)),
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


class O6Backend:
    """OpenXR 26 → LinkerHand O6/L6/O7 curl."""

    def __init__(self, side: str, cfg, retargeting_type: str | None = None):
        if side not in ("left", "right"):
            raise ValueError(f"side must be left|right, got {side!r}")
        if cfg.o6_curl is None:
            raise ValueError(f"{cfg.backend} backend requires retargeting.curl")
        self.side = side
        self.dof = int(cfg.dof)
        rtype = (retargeting_type or cfg.retargeting_type or "curl").strip().lower()
        if rtype in ("mp_curl", "mediapipe", "mp21_curl"):
            rtype = "mp_curl"
        elif rtype in ("nest", "keyvec", "nest_vec"):
            rtype = "nest"
        elif rtype in ("veccurl", "vcurl"):
            rtype = "veccurl"
        elif rtype in ("curl", "curl_calib", "calib", "lerp", ""):
            rtype = "curl"
        else:
            print(
                f"[retarget] backend={cfg.backend} unknown type {rtype!r}; using curl",
                flush=True,
            )
            rtype = "curl"
        self.retargeting_type = rtype
        self.vector_profile = ""
        self._cfg = cfg
        use_palm = cfg.backend in ("o6", "l6", "o7")
        if rtype == "mp_curl":
            from xr_hand_retarget.algorithms.mp_curl_o6 import MpCurlO6Retargeter

            self._inner = MpCurlO6Retargeter(side, cfg.o6_curl)
            engine = "MpCurlO6Retargeter"
            model = cfg.backend
        elif use_palm:
            from xr_hand_retarget.algorithms.palm_tip import PalmTipRetargeter

            self._inner = PalmTipRetargeter(side, cfg.o6_curl, solver=rtype)
            engine = "PalmTipRetargeter"
            model = cfg.backend
            self.retargeting_type = str(
                getattr(self._inner, "solver", rtype) or rtype
            )
        else:
            self._inner = CurlO6Retargeter(side, cfg.o6_curl)
            engine = "CurlO6Retargeter"
            model = self._inner.model
        if self._inner.limits.shape[0] != self.dof:
            self.dof = int(self._inner.limits.shape[0])
        dt = 1.0 / max(float(cfg.rate_hz), 1.0)
        _lim, urdf_vel = load_linker_limits(
            side, model=cfg.backend, urdf_path=cfg.o6_curl.urdf_path
        )
        urdf_path = cfg.o6_curl.urdf_path or default_linker_urdf(cfg.backend, side)
        motion = cfg.motion or MotionSafetyGains()
        fk = None
        if motion.fk_enabled:
            fk = make_linker_fk(
                side, model=cfg.backend, urdf_path=cfg.o6_curl.urdf_path
            )
        self._motion = JointMotionFilter(
            self._inner.limits,
            np.asarray(urdf_vel, dtype=np.float64).reshape(-1)[: self.dof],
            motion,
            dt,
            urdf_path=urdf_path,
            fk=fk,
            joint_names=list(cfg.joint_names),
            side=side,
        )
        self._q_hold: np.ndarray | None = None
        self._was_held = True
        self._unlock = 0
        self._j2_btn_prev = False
        self._dbg_n = 0
        self._dbg_fk0 = 0
        self._dbg_vel0 = 0
        self._dbg_cspace0 = 0
        btn = str(getattr(cfg.o6_curl, "j2_lock_button", "off") or "off")
        attract_tag = (
            "on" if bool(getattr(cfg.o6_curl, "attract_enabled", True)) else "off"
        )
        print(
            f"[retarget] side={side} type={self.retargeting_type} engine={engine} "
            f"model={model} dof={self.dof} "
            f"attract={attract_tag} j2_lock_btn={btn} "
            f"safety={self._motion.summary()}",
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
        self._motion.sync(q)
        return SideStep(q=q, active=active, held=True, xyz_abs=xyz_abs, reason=reason)

    def _shape_q(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=np.float64).reshape(-1)
        if q.shape[0] != self.dof:
            pad = np.zeros(self.dof, dtype=np.float64)
            n = min(q.shape[0], self.dof)
            pad[:n] = q[:n]
            q = pad
        q_raw = q
        q_cmd = self._motion.step(q)
        self._dbg_n += 1
        if self._dbg_n % 50 == 0:
            fk_n = int(self._motion.fk_blocks)
            vel_n = int(self._motion.velocity_clips)
            cs_n = int(getattr(self._motion, "cspace_hits", 0))
            dfk = fk_n - self._dbg_fk0
            dvel = vel_n - self._dbg_vel0
            dcs = cs_n - self._dbg_cspace0
            self._dbg_fk0 = fk_n
            self._dbg_vel0 = vel_n
            self._dbg_cspace0 = cs_n
            j2 = 1 if self.dof > 1 else 0
            g = self._motion.gains
            in_pinch = False
            if g.fk_enabled and self.dof > 1:
                j2v = float(q_cmd[j2])
                hw = float(g.fk_pinch_j2_halfwidth)
                in_pinch = abs(j2v - float(g.fk_pinch_j2_index)) <= hw or abs(
                    j2v - float(g.fk_pinch_j2_middle)
                ) <= hw
            print(
                f"[{self.side}] safety dbg "
                f"mode={getattr(self._motion, 'mode', '?')} "
                f"j2_raw={float(q_raw[j2]):.3f} j2_cmd={float(q_cmd[j2]):.3f} "
                f"dfk={dfk} dvel={dvel} cspace_hit={dcs} "
                f"pinch_slot={int(in_pinch)}",
                flush=True,
            )
        return q_cmd

    def set_j2_lock(self, on: bool) -> None:
        fn = getattr(self._inner, "set_j2_lock", None)
        if callable(fn):
            fn(bool(on))

    def toggle_j2_lock(self) -> None:
        fn = getattr(self._inner, "toggle_j2_lock", None)
        if callable(fn):
            fn()

    def _xrt_lock_pressed(self, xrt) -> bool:
        btn = str(getattr(self._cfg.o6_curl, "j2_lock_button", "off") or "off")
        from xr_hand_retarget.sources.xrt import controller_button_pressed

        return controller_button_pressed(xrt, self.side, btn)

    def _poll_j2_lock_button(self, xrt) -> None:
        if xrt is None:
            return
        pressed = self._xrt_lock_pressed(xrt)
        if pressed and not self._j2_btn_prev:
            self.toggle_j2_lock()
        self._j2_btn_prev = pressed

    def step(self, frame, *, xrt=None) -> SideStep:
        from xr_hand_retarget.sources.frame import HandFrame

        if not isinstance(frame, HandFrame):
            raise TypeError(f"expected HandFrame, got {type(frame)!r}")
        self._poll_j2_lock_button(xrt)
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
