"""Wuji-style Adaptive thumb IK for XHand1.

Reference pose ``ref_q_thumb`` fixes robot thumb length / open tip.
Far from pinch → FullHand bone vectors; near pinch → TipDir (tip pos + tip dir).
Alpha blends like Wuji AdaptiveOptimizerAnalytical.

Other fingers still come from geometric palm-frame remap (`remap.openxr26_to_xhand1_q`).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .kinematics_xhand1 import XHandFK, make_hand_fk
from xr_hand_retarget.sources.landmarks import TemporalFilter, openxr26_to_mediapipe21, preprocess_landmarks
from .remap import openxr26_to_xhand1_q

# MediaPipe indices (after OpenXR→21).
_MP_WRIST = 0
_MP_THUMB_CMC = 1
_MP_THUMB_MCP = 2
_MP_THUMB_IP = 3
_MP_THUMB_TIP = 4
_MP_INDEX_TIP = 8
_MP_INDEX_MCP = 5
_MP_MIDDLE_MCP = 9
_MP_PINKY_MCP = 17


@dataclass
class ThumbIkGains:
    """Adaptive TipDir / FullHand thumb IK (reference pose + pinch alpha)."""

    tip_link: str = "thumb_tip"
    dir_link: str = "thumb_ip"  # tip direction = tip - dir_link
    human_tip_index: int = _MP_THUMB_TIP
    human_root_index: int = _MP_THUMB_CMC
    # Fixed robot reference for length / reach (open or calibrated pose).
    ref_q_thumb: tuple[float, float, float] = (0.0, 0.0, 0.0)
    auto_scale: bool = True
    tip_scale: float = 1.0
    # Palm-frame anisotropic tip scale (after preprocess, before tip_rotation).
    # lateral = index↔pinky, longitudinal = wrist→middle, normal = out of palm.
    scale_lateral: float = 1.0
    scale_longitudinal: float = 1.0
    scale_normal: float = 1.0
    tip_offset_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    tip_rotation: tuple[float, ...] = (
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    )
    # Thumb segment scales [CMC→MCP, MCP→IP, IP→TIP] for FullHand targets.
    segment_scaling: tuple[float, float, float] = (1.0, 1.0, 1.0)
    reach_margin: float = 1.05
    max_iterations: int = 35
    # TipDir weights (pinch mode).
    w_pos: float = 1.0
    w_dir: float = 8.0
    # FullHand bone-direction weight (open / mid).
    w_full: float = 1.0
    w_reg: float = 0.02
    output_alpha: float = 0.85
    temporal_filter_alpha: float = 0.65
    # Pinch alpha: d < d1 → TipDir, d > d2 → FullHand (meters, human wrist frame).
    pinch_d1_m: float = 0.022
    pinch_d2_m: float = 0.040
    pinch_alpha_max: float = 0.85
    # EMA on pinch alpha (TipDir↔FullHand); lower = smoother mode transitions.
    pinch_alpha_smooth: float = 0.35
    # EMA on tip position target (VR noise / reach clamp jumps).
    target_smooth_alpha: float = 0.55
    # Soft FK clearance: penalize thumb_tip closer than clearance_m to clearance_link.
    w_clearance: float = 2.0
    clearance_m: float = 0.018
    clearance_link: str = "index_pip"
    # Per-frame thumb joint rate limit (rad); 0 = off.
    thumb_max_delta_rad: float = 0.08
    # Tip lateral → thumb_joint1 prior: outside palm → low j1; inside → high j1.
    w_rot_pref: float = 0.12
    rot_pref_span_m: float = 0.028
    # Legacy soft pull of tip target toward index (extra to TipDir); usually 0 with Adaptive.
    pinch_blend: float = 0.0
    # Temporary post-IK: q2' = clip(q2 * joint2_scale + joint2_bias).
    # Amplify thumb_joint2 to lift tip away from four fingers (URDF rota_joint1).
    joint2_scale: float = 1.0
    joint2_bias: float = 0.0
    urdf_path: str | None = None


def _unit(v: np.ndarray) -> np.ndarray | None:
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return None
    return v / n


def _huber_sq(err: np.ndarray, delta: float = 0.02) -> float:
    """Scalar Huber on ||err|| then square-ish: use 0.5 r^2 / delta*r."""
    r = float(np.linalg.norm(err))
    if r <= delta:
        return 0.5 * r * r
    return delta * (r - 0.5 * delta)


class ThumbIkRetargeter:
    """Hybrid fingers + Adaptive TipDir/FullHand thumb IK."""

    def __init__(
        self,
        *,
        side: str,
        limits: np.ndarray,
        gains: ThumbIkGains | None = None,
        fk: XHandFK | None = None,
    ):
        self.side = "left" if side.lower().startswith("l") else "right"
        self.limits = np.asarray(limits, dtype=np.float64)
        self.gains = gains or ThumbIkGains()
        self.fk = fk or make_hand_fk(self.side, urdf_path=self.gains.urdf_path)
        self.landmark_filter = TemporalFilter(self.gains.temporal_filter_alpha)
        self._q_prev: np.ndarray | None = None
        self._rot = np.asarray(self.gains.tip_rotation, dtype=np.float64).reshape(3, 3)
        self._offset = np.asarray(self.gains.tip_offset_m, dtype=np.float64)
        self._seg = np.asarray(self.gains.segment_scaling, dtype=np.float64)

        # ---- Fixed reference pose (robot) ----
        q_ref = np.zeros(12, dtype=np.float64)
        q_ref[0:3] = np.asarray(self.gains.ref_q_thumb, dtype=np.float64)
        tip_ref = self.fk.link_position(q_ref, self.gains.tip_link)
        cmc_ref = self.fk.link_position(q_ref, "thumb_cmc")
        mcp_ref = self.fk.link_position(q_ref, "thumb_mcp")
        ip_ref = self.fk.link_position(q_ref, self.gains.dir_link)
        self._robot_thumb_len = max(float(np.linalg.norm(tip_ref - cmc_ref)), 1e-3)
        self._robot_reach = max(float(np.linalg.norm(tip_ref)), self._robot_thumb_len)
        self._robot_seg_len = np.array(
            [
                max(float(np.linalg.norm(mcp_ref - cmc_ref)), 1e-4),
                max(float(np.linalg.norm(ip_ref - mcp_ref)), 1e-4),
                max(float(np.linalg.norm(tip_ref - ip_ref)), 1e-4),
            ],
            dtype=np.float64,
        )
        self._ref_tip = tip_ref.copy()
        self._last_alpha = 0.0
        self._last_scale = 1.0
        self._alpha_prev: float | None = None
        self._tip_pos_prev: np.ndarray | None = None

    def reset(self) -> None:
        self._q_prev = None
        self._last_alpha = 0.0
        self._alpha_prev = None
        self._tip_pos_prev = None
        self.landmark_filter.reset()

    def _map_point(self, p: np.ndarray, scale: float) -> np.ndarray:
        return self._rot @ (p * scale) + self._offset

    def _palm_basis(self, lm: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        """Orthonormal palm basis from preprocessed landmarks (wrist at origin)."""
        lat = _unit(lm[_MP_INDEX_MCP] - lm[_MP_PINKY_MCP])
        lng = _unit(lm[_MP_MIDDLE_MCP].copy())
        if lat is None or lng is None:
            return None
        if self.side == "left":
            lat = -lat
        # Remove lateral component from longitudinal, then normal = lat × long.
        lng = lng - float(np.dot(lng, lat)) * lat
        lng = _unit(lng)
        if lng is None:
            return None
        nrm = _unit(np.cross(lat, lng))
        if nrm is None:
            return None
        # Re-orthogonalize longitudinal against normal.
        lng = _unit(np.cross(nrm, lat))
        if lng is None:
            return None
        return lat, lng, nrm

    def _aniso_scale_point(self, lm: np.ndarray, p: np.ndarray) -> np.ndarray:
        """Scale point in palm basis: lateral / longitudinal / normal."""
        sl = float(self.gains.scale_lateral)
        sg = float(self.gains.scale_longitudinal)
        sn = float(self.gains.scale_normal)
        if abs(sl - 1.0) < 1e-12 and abs(sg - 1.0) < 1e-12 and abs(sn - 1.0) < 1e-12:
            return np.asarray(p, dtype=np.float64)
        basis = self._palm_basis(lm)
        if basis is None:
            return np.asarray(p, dtype=np.float64)
        lat, lng, nrm = basis
        p = np.asarray(p, dtype=np.float64)
        return (sl * float(np.dot(p, lat)) * lat
                + sg * float(np.dot(p, lng)) * lng
                + sn * float(np.dot(p, nrm)) * nrm)

    def _pinch_alpha(self, lm: np.ndarray) -> float:
        tip_i = int(np.clip(self.gains.human_tip_index, 0, 20))
        d = float(np.linalg.norm(lm[tip_i] - lm[_MP_INDEX_TIP]))
        d1, d2 = float(self.gains.pinch_d1_m), float(self.gains.pinch_d2_m)
        if d2 <= d1 + 1e-9:
            return float(self.gains.pinch_alpha_max) if d <= d1 else 0.0
        a = (d2 - d) / (d2 - d1)
        return float(np.clip(a, 0.0, 1.0) * self.gains.pinch_alpha_max)

    def _human_scale(self, lm: np.ndarray) -> float:
        chain = 0.0
        for a, b in (
            (_MP_THUMB_CMC, _MP_THUMB_MCP),
            (_MP_THUMB_MCP, _MP_THUMB_IP),
            (_MP_THUMB_IP, _MP_THUMB_TIP),
        ):
            chain += float(np.linalg.norm(lm[b] - lm[a]))
        chain = max(chain, 1e-4)
        scale = float(self.gains.tip_scale)
        if self.gains.auto_scale:
            scale *= self._robot_thumb_len / chain
        return scale

    def _tip_rot_pref(self, lm: np.ndarray, tip_i: int) -> float:
        """Map tip palm-lateral position → preferred thumb_joint1 in [0, 1].

        Outside (toward index / palm edge) → 0 (external / open bend).
        Inside (toward pinky / palm) → 1 (internal / opposition).
        """
        lat_axis = _unit(lm[_MP_INDEX_MCP] - lm[_MP_PINKY_MCP])
        if lat_axis is None:
            return 0.5
        if self.side == "left":
            lat_axis = -lat_axis
        center = lm[_MP_MIDDLE_MCP]
        tip = lm[tip_i]
        lat = float(np.dot(tip - center, lat_axis))
        span = max(float(self.gains.rot_pref_span_m), 1e-4)
        # lat > 0 (index/outside) → outside≈1 → pref≈0; lat < 0 → inside → pref≈1
        outside = float(np.clip(0.5 + lat / span, 0.0, 1.0))
        return 1.0 - outside

    def build_targets(self, joints26: np.ndarray) -> dict:
        """Build TipDir + FullHand targets in hand_base from OpenXR frame."""
        lm21 = openxr26_to_mediapipe21(joints26)
        lm = preprocess_landmarks(lm21, hand_side=self.side)
        lm = self.landmark_filter.filter(lm)

        tip_i = int(np.clip(self.gains.human_tip_index, 0, 20))
        scale = self._human_scale(lm)
        raw_alpha = self._pinch_alpha(lm)
        aa = float(np.clip(self.gains.pinch_alpha_smooth, 1e-3, 1.0))
        if self._alpha_prev is not None:
            alpha = aa * raw_alpha + (1.0 - aa) * self._alpha_prev
        else:
            alpha = raw_alpha
        self._alpha_prev = alpha
        self._last_alpha = alpha
        self._last_scale = scale

        j1_pref_n = self._tip_rot_pref(lm, tip_i)

        # TipDir: wrist→tip position + IP→tip direction
        tip_h = self._aniso_scale_point(lm, lm[tip_i])
        tip_pos = self._map_point(tip_h, scale)
        if self.gains.pinch_blend > 0.0 and alpha > 0.0:
            idx_h = self._aniso_scale_point(lm, lm[_MP_INDEX_TIP])
            idx = self._map_point(idx_h, scale)
            tip_pos = (1.0 - self.gains.pinch_blend * alpha) * tip_pos + (
                self.gains.pinch_blend * alpha
            ) * idx

        r = float(np.linalg.norm(tip_pos))
        r_max = self._robot_reach * float(self.gains.reach_margin)
        if r > r_max and r > 1e-9:
            tip_pos = tip_pos * (r_max / r)

        ta = float(np.clip(self.gains.target_smooth_alpha, 1e-3, 1.0))
        if self._tip_pos_prev is not None:
            tip_pos = ta * tip_pos + (1.0 - ta) * self._tip_pos_prev
        self._tip_pos_prev = tip_pos.copy()

        tip_dir = _unit(lm[tip_i] - lm[_MP_THUMB_IP])
        if tip_dir is not None:
            tip_dir = self._rot @ tip_dir
            tip_dir = _unit(tip_dir)

        # FullHand: three bone directions (unit) + optional length targets
        bone_pairs = (
            (_MP_THUMB_CMC, _MP_THUMB_MCP),
            (_MP_THUMB_MCP, _MP_THUMB_IP),
            (_MP_THUMB_IP, tip_i),
        )
        bone_dirs = []
        for i, (a, b) in enumerate(bone_pairs):
            d = _unit(lm[b] - lm[a])
            if d is None:
                bone_dirs.append(None)
            else:
                bone_dirs.append(_unit(self._rot @ d))

        return {
            "tip_pos": tip_pos,
            "tip_dir": tip_dir,
            "bone_dirs": bone_dirs,
            "alpha": alpha,
            "scale": scale,
            "j1_pref_n": j1_pref_n,
        }

    def _ik_thumb(self, targets: dict, q_seed: np.ndarray) -> np.ndarray:
        lo = self.limits[0:3, 0]
        hi = self.limits[0:3, 1]
        q_full = np.asarray(q_seed, dtype=np.float64).copy()
        q_t = np.clip(q_full[0:3], lo, hi)
        q_prev = (
            self._q_prev[0:3].copy() if self._q_prev is not None else q_t.copy()
        )

        tip_link = self.gains.tip_link
        dir_link = self.gains.dir_link
        alpha = float(targets["alpha"])
        tip_tgt = targets["tip_pos"]
        tip_dir_h = targets["tip_dir"]
        bone_dirs = targets["bone_dirs"]
        j1_pref = float(lo[0] + float(targets["j1_pref_n"]) * (hi[0] - lo[0]))
        robot_bones = ("thumb_cmc", "thumb_mcp", dir_link, tip_link)

        w_pos = float(self.gains.w_pos)
        w_dir = float(self.gains.w_dir)
        w_full = float(self.gains.w_full)
        w_reg = float(self.gains.w_reg)
        w_rot = float(self.gains.w_rot_pref)
        w_clear = float(self.gains.w_clearance)
        clearance_m = float(self.gains.clearance_m)
        clear_link = self.gains.clearance_link
        seg_w = self._seg

        def pack(qt: np.ndarray) -> np.ndarray:
            q = q_full.copy()
            q[0:3] = qt
            return q

        def objective(qt: np.ndarray) -> float:
            q = pack(np.clip(qt, lo, hi))
            names = list(robot_bones)
            pos = self.fk.link_positions(q, names)
            tip = pos[3]
            ip = pos[2]

            loss = 0.0
            # TipDir
            if alpha > 1e-6:
                loss += alpha * w_pos * _huber_sq(tip - tip_tgt)
                rd = _unit(tip - ip)
                if rd is not None and tip_dir_h is not None:
                    loss += alpha * w_dir * (1.0 - float(np.dot(rd, tip_dir_h)))

            # FullHand bone directions
            beta = 1.0 - alpha
            if beta > 1e-6:
                for i in range(3):
                    hd = bone_dirs[i]
                    if hd is None:
                        continue
                    rd = _unit(pos[i + 1] - pos[i])
                    if rd is None:
                        continue
                    loss += beta * w_full * float(seg_w[i]) * (1.0 - float(np.dot(rd, hd)))

            # Thumb–finger clearance (relaxed during pinch when alpha is high).
            if w_clear > 0.0 and clearance_m > 0.0:
                try:
                    obs = self.fk.link_position(q, clear_link)
                    d_clear = float(np.linalg.norm(tip - obs))
                    if d_clear < clearance_m:
                        gap = clearance_m - d_clear
                        w_eff = w_clear * (1.0 - 0.85 * alpha)
                        loss += w_eff * gap * gap
                except ValueError:
                    pass

            # Tip inside/outside → thumb_joint1 rotation prior (null-space bias).
            if w_rot > 0.0:
                loss += w_rot * (float(qt[0]) - j1_pref) ** 2

            loss += w_reg * float(np.sum((qt - q_prev) ** 2))
            return loss

        try:
            from scipy.optimize import minimize

            res = minimize(
                objective,
                q_t,
                method="L-BFGS-B",
                bounds=[(float(lo[i]), float(hi[i])) for i in range(3)],
                options={"maxiter": self.gains.max_iterations, "ftol": 1e-10},
            )
            if res.success or res.fun <= objective(q_t):
                q_t = np.clip(res.x, lo, hi)
            return q_t
        except ImportError:
            pass

        step = 0.3
        best = objective(q_t)
        for _ in range(self.gains.max_iterations):
            grad = np.zeros(3)
            base = objective(q_t)
            eps = 1e-4
            for i in range(3):
                qp = q_t.copy()
                qp[i] = np.clip(qp[i] + eps, lo[i], hi[i])
                grad[i] = (objective(qp) - base) / eps
            q_try = np.clip(q_t - step * grad, lo, hi)
            loss = objective(q_try)
            if loss < best - 1e-12:
                best = loss
                q_t = q_try
                step = min(step * 1.1, 0.5)
            else:
                step *= 0.5
                if step < 0.01:
                    break
        return q_t

    def retarget(
        self,
        joints26: np.ndarray,
        *,
        flexion_scale: float = 1.0,
        finger_chain=None,
        thumb_tip=None,
        index_abduction=None,
    ) -> np.ndarray:
        q = openxr26_to_xhand1_q(
            joints26,
            side=self.side,
            limits=self.limits,
            flexion_scale=flexion_scale,
            finger_chain=finger_chain,
            thumb_tip=thumb_tip,
            index_abduction=index_abduction,
        )
        targets = self.build_targets(joints26)
        seed = q.copy()
        if self._q_prev is not None:
            seed[0:3] = 0.35 * q[0:3] + 0.65 * self._q_prev[0:3]
        q[0:3] = self._ik_thumb(targets, seed)

        alpha_out = float(self.gains.output_alpha)
        if self._q_prev is not None and alpha_out < 1.0 - 1e-12:
            q[0:3] = alpha_out * q[0:3] + (1.0 - alpha_out) * self._q_prev[0:3]
            q[0:3] = np.clip(q[0:3], self.limits[0:3, 0], self.limits[0:3, 1])

        max_d = float(self.gains.thumb_max_delta_rad)
        if max_d > 0.0 and self._q_prev is not None:
            lo3, hi3 = self.limits[0:3, 0], self.limits[0:3, 1]
            delta = np.clip(q[0:3] - self._q_prev[0:3], -max_d, max_d)
            q[0:3] = np.clip(self._q_prev[0:3] + delta, lo3, hi3)

        # Temporary: amplify thumb_joint2 (rota1) to reduce tip–finger collision.
        s2 = float(self.gains.joint2_scale)
        b2 = float(self.gains.joint2_bias)
        if abs(s2 - 1.0) > 1e-12 or abs(b2) > 1e-12:
            q[1] = float(np.clip(q[1] * s2 + b2, self.limits[1, 0], self.limits[1, 1]))

        self._q_prev = q.copy()
        return q
