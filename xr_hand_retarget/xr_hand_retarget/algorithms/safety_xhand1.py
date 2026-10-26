"""Safety filters for commanded joint vectors.

Layers (strict motion block is optional / switchable):
1. Hard position clamp from **URDF** (optional inward margin) — not YAML alone.
2. Optional soft thumb-corridor heuristic.
3. Strict motion (``motion.enabled``):
   - velocity from URDF → |Δq| ≤ v·dt
   - optional acceleration limit
   - URDF FK clearance (thumb prioritized) — never publish colliding q
4. Hold last command when tracking is inactive / zero.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .collision import CollisionGains, soft_collision_project
from .kinematics_xhand1 import XHandFK, load_urdf_joint_limits_velocity, make_hand_fk

# Thumb DOF indices in XHand1 command vector.
_THUMB = slice(0, 3)

# Links treated as "thumb chain" for priority retract.
_THUMB_LINKS = frozenset(
    {"thumb_tip", "thumb_ip", "thumb_mcp", "thumb_cmc"}
)


def clamp_q(q: np.ndarray, limits: np.ndarray) -> np.ndarray:
    """Clip q to [lower, upper] per joint. limits shape (N, 2)."""
    q = np.asarray(q, dtype=np.float64).reshape(-1)
    return np.clip(q, limits[:, 0], limits[:, 1])


def pose_is_zero(joints26: np.ndarray, eps: float = 1e-6) -> bool:
    xyz = np.asarray(joints26, dtype=np.float64)[:, :3]
    return not bool(np.any(np.abs(xyz) > eps))


def _default_clearance_pairs() -> list[tuple[str, str]]:
    """Thumb-heavy pairs (URDF link aliases)."""
    return [
        ("thumb_tip", "index_tip"),
        ("thumb_tip", "index_pip"),
        ("thumb_tip", "index_mcp"),
        ("thumb_tip", "middle_tip"),
        ("thumb_tip", "middle_pip"),
        ("thumb_tip", "middle_mcp"),
        ("thumb_ip", "index_mcp"),
        ("thumb_ip", "index_pip"),
        ("thumb_mcp", "index_mcp"),
        ("index_tip", "middle_tip"),
    ]


@dataclass
class MotionSafetyGains:
    """Strict motion / collision gate (can be fully disabled)."""

    enabled: bool = False  # opt-in; xhand1.yaml sets true
    # Force joint position limits from URDF (ignore YAML hand.limits for clamp).
    use_urdf_limits: bool = True
    position_margin_rad: float = 0.02
    # Velocity: |Δq|/dt ≤ scale * urdf_velocity (or uniform override).
    use_urdf_velocity: bool = True
    velocity_scale: float = 0.5
    velocity_limit_rad_s: float | None = None
    velocity_limit_per_joint: tuple[float, ...] | None = None
    accel_limit_rad_s2: float = 40.0
    # URDF FK pairwise clearance.
    strict_collision: bool = True
    min_clearance_m: float = 0.014
    # Signed distance of thumb_tip above palm plane (index/middle/pinky MCP).
    thumb_palm_clearance_m: float = 0.008
    on_violation: str = "scale"  # scale | hold
    scale_iters: int = 12
    # When a thumb-involved pair fails, retract thumb DOFs first.
    thumb_priority: bool = True
    clearance_pairs: list[tuple[str, str]] = field(
        default_factory=_default_clearance_pairs
    )
    urdf_path: str | None = None


class CommandSafety:
    """URDF clamp → soft collision → strict motion (vel/FK thumb) → hold/reseed."""

    def __init__(
        self,
        limits: np.ndarray,
        *,
        side: str = "right",
        alpha: float = 1.0,
        soft_collision: bool = True,
        collision_gains: CollisionGains | None = None,
        max_delta_rad: float = 0.0,
        motion: MotionSafetyGains | None = None,
        fk: XHandFK | None = None,
        urdf_velocity: np.ndarray | None = None,
        dt_default: float = 0.02,
    ):
        self.limits = np.asarray(limits, dtype=np.float64)
        self.side = "left" if str(side).lower().startswith("l") else "right"
        self.alpha = float(np.clip(alpha, 1e-3, 1.0))
        self.soft_collision = bool(soft_collision)
        self.collision_gains = collision_gains or CollisionGains()
        self.max_delta_rad = float(max_delta_rad)
        self.motion = motion or MotionSafetyGains(enabled=False)
        self.dt_default = float(max(dt_default, 1e-4))

        self._q_cmd: np.ndarray | None = None
        self._q_vel: np.ndarray | None = None
        self._needs_reseed = True
        self.collision_blocks = 0
        self.velocity_clips = 0
        self.urdf_limits_applied = False

        self._fk: XHandFK | None = None
        self._vel_limits = np.zeros(self.limits.shape[0], dtype=np.float64)
        self._eff_limits = self.limits.copy()
        self._clearance_links: list[str] = []
        self._pair_idx: list[tuple[int, int]] = []
        self._pair_thumb: list[bool] = []
        self._palm_link_idx: tuple[int, int, int] | None = None
        self._thumb_tip_idx: int | None = None
        self._palm_n_ref: np.ndarray | None = None  # unit normal; open tip has +height

        if self.motion.enabled:
            self._init_motion(fk=fk, urdf_velocity=urdf_velocity)

    def _init_motion(
        self,
        *,
        fk: XHandFK | None,
        urdf_velocity: np.ndarray | None,
    ) -> None:
        m = self.motion
        urdf_lim, urdf_vel = load_urdf_joint_limits_velocity(
            self.side, urdf_path=m.urdf_path
        )
        if urdf_lim.shape != self.limits.shape:
            raise ValueError(
                f"URDF limits shape {urdf_lim.shape} != yaml limits {self.limits.shape}"
            )

        # --- Position: force URDF ---
        if m.use_urdf_limits:
            self.limits = urdf_lim.copy()
            self.urdf_limits_applied = True
        base = self.limits
        margin = max(float(m.position_margin_rad), 0.0)
        lo = base[:, 0] + margin
        hi = base[:, 1] - margin
        bad = hi < lo
        lo = np.where(bad, base[:, 0], lo)
        hi = np.where(bad, base[:, 1], hi)
        self._eff_limits = np.stack([lo, hi], axis=1)

        # --- Velocity: force URDF unless overridden ---
        n = self.limits.shape[0]
        if m.velocity_limit_per_joint is not None:
            v = np.asarray(m.velocity_limit_per_joint, dtype=np.float64).reshape(-1)
            if v.shape[0] != n:
                raise ValueError(
                    f"velocity_limit_per_joint length {v.shape[0]} != dof {n}"
                )
            self._vel_limits = np.maximum(v, 0.0)
        elif m.velocity_limit_rad_s is not None:
            self._vel_limits = np.full(n, max(float(m.velocity_limit_rad_s), 0.0))
        elif m.use_urdf_velocity:
            src = (
                np.asarray(urdf_velocity, dtype=np.float64).reshape(-1)
                if urdf_velocity is not None
                else urdf_vel
            )
            self._vel_limits = np.maximum(src * float(m.velocity_scale), 0.0)
        else:
            self._vel_limits = np.zeros(n)

        # --- FK clearance (always build FK when motion on; needed for thumb palm) ---
        need_fk = m.strict_collision or float(m.thumb_palm_clearance_m) > 0.0
        if need_fk:
            self._fk = fk or make_hand_fk(self.side, urdf_path=m.urdf_path)
            links: list[str] = []
            seen: set[str] = set()

            def _add(name: str) -> int:
                if name not in seen:
                    seen.add(name)
                    links.append(name)
                return links.index(name)

            pairs: list[tuple[int, int]] = []
            pair_thumb: list[bool] = []
            if m.strict_collision:
                for a, b in m.clearance_pairs:
                    ia, ib = _add(a), _add(b)
                    pairs.append((ia, ib))
                    pair_thumb.append(a in _THUMB_LINKS or b in _THUMB_LINKS)

            # Palm plane + thumb tip for vertical clearance.
            if float(m.thumb_palm_clearance_m) > 0.0:
                self._thumb_tip_idx = _add("thumb_tip")
                self._palm_link_idx = (
                    _add("index_mcp"),
                    _add("middle_mcp"),
                    _add("pinky_mcp"),
                )

            self._clearance_links = links
            self._pair_idx = pairs
            self._pair_thumb = pair_thumb

            # Calibrate palm normal so open-hand thumb_tip has positive height.
            if self._palm_link_idx is not None and self._thumb_tip_idx is not None:
                q0 = clamp_q(np.zeros(n), self._eff_limits)
                pos0 = self._fk.link_positions(q0, self._clearance_links)
                i0, i1, i2 = self._palm_link_idx
                nrm = np.cross(pos0[i1] - pos0[i0], pos0[i2] - pos0[i0])
                nn = float(np.linalg.norm(nrm))
                if nn > 1e-9:
                    nrm = nrm / nn
                    h0 = float(np.dot(pos0[self._thumb_tip_idx] - pos0[i0], nrm))
                    if h0 < 0.0:
                        nrm = -nrm
                    self._palm_n_ref = nrm
                else:
                    self._palm_n_ref = None

    @property
    def last_command(self) -> np.ndarray | None:
        return None if self._q_cmd is None else self._q_cmd.copy()

    @property
    def effective_limits(self) -> np.ndarray:
        return self._eff_limits.copy() if self.motion.enabled else self.limits.copy()

    def hold(self) -> np.ndarray:
        """Keep last command; do not seed zeros."""
        self._needs_reseed = True
        if self._q_cmd is None:
            return np.zeros(self.limits.shape[0], dtype=np.float64)
        return self._q_cmd.copy()

    def _limits_now(self) -> np.ndarray:
        return self._eff_limits if self.motion.enabled else self.limits

    def _thumb_palm_ok(self, pos: np.ndarray) -> bool:
        """Require thumb_tip on the open-hand side of palm plane, height ≥ margin."""
        thr = float(self.motion.thumb_palm_clearance_m)
        if (
            thr <= 0.0
            or self._palm_link_idx is None
            or self._thumb_tip_idx is None
            or self._palm_n_ref is None
        ):
            return True
        i0, i1, i2 = self._palm_link_idx
        p0, p1, p2 = pos[i0], pos[i1], pos[i2]
        n = np.cross(p1 - p0, p2 - p0)
        nn = float(np.linalg.norm(n))
        if nn < 1e-9:
            return True
        n = n / nn
        # Align with calibrated open-hand normal hemisphere.
        if float(np.dot(n, self._palm_n_ref)) < 0.0:
            n = -n
        height = float(np.dot(pos[self._thumb_tip_idx] - p0, n))
        return height >= thr

    def _clearance_ok(self, q: np.ndarray) -> bool:
        if self._fk is None or not self._clearance_links:
            return True
        pos = self._fk.link_positions(q, self._clearance_links)
        thr = float(self.motion.min_clearance_m)
        if self.motion.strict_collision:
            for ia, ib in self._pair_idx:
                if float(np.linalg.norm(pos[ia] - pos[ib])) < thr:
                    return False
        if not self._thumb_palm_ok(pos):
            return False
        return True

    def _thumb_pair_violated(self, q: np.ndarray) -> bool:
        """True if a thumb-involved pair or palm height fails."""
        if self._fk is None:
            return False
        pos = self._fk.link_positions(q, self._clearance_links)
        if not self._thumb_palm_ok(pos):
            return True
        thr = float(self.motion.min_clearance_m)
        for (ia, ib), is_thumb in zip(self._pair_idx, self._pair_thumb):
            if is_thumb and float(np.linalg.norm(pos[ia] - pos[ib])) < thr:
                return True
        return False

    def _apply_velocity(
        self, q: np.ndarray, q_prev: np.ndarray, dt: float
    ) -> np.ndarray:
        m = self.motion
        out = q.copy()
        if self.max_delta_rad > 0.0:
            d = np.clip(out - q_prev, -self.max_delta_rad, self.max_delta_rad)
            out = q_prev + d

        if m.enabled and np.any(self._vel_limits > 0.0):
            max_step = self._vel_limits * dt
            d = out - q_prev
            clipped = np.clip(d, -max_step, max_step)
            if bool(np.any(np.abs(clipped - d) > 1e-12)):
                self.velocity_clips += 1
            out = q_prev + clipped

        if m.enabled and m.accel_limit_rad_s2 > 0.0 and self._q_vel is not None:
            v_des = (out - q_prev) / dt
            max_dv = float(m.accel_limit_rad_s2) * dt
            v_new = self._q_vel + np.clip(v_des - self._q_vel, -max_dv, max_dv)
            if np.any(self._vel_limits > 0.0):
                v_new = np.clip(v_new, -self._vel_limits, self._vel_limits)
            out = q_prev + v_new * dt

        return out

    def _binary_scale(
        self,
        q: np.ndarray,
        q_prev: np.ndarray,
        *,
        thumb_only: bool,
    ) -> np.ndarray:
        lim = self._limits_now()
        best = q_prev.copy()
        lo_a, hi_a = 0.0, 1.0
        for _ in range(int(max(self.motion.scale_iters, 1))):
            mid = 0.5 * (lo_a + hi_a)
            q_try = q_prev.copy()
            if thumb_only:
                q_try[_THUMB] = q_prev[_THUMB] + mid * (q[_THUMB] - q_prev[_THUMB])
            else:
                q_try = q_prev + mid * (q - q_prev)
            q_try = clamp_q(q_try, lim)
            if self._clearance_ok(q_try):
                best = q_try
                lo_a = mid
            else:
                hi_a = mid
        return best

    def _enforce_clearance(self, q: np.ndarray, q_prev: np.ndarray) -> np.ndarray:
        m = self.motion
        if not m.enabled or self._fk is None:
            return q
        if self._clearance_ok(q):
            return q

        self.collision_blocks += 1
        mode = (m.on_violation or "scale").lower()
        if mode == "hold" or not self._clearance_ok(q_prev):
            return q_prev.copy()

        # 1) Thumb-priority: freeze fingers, retract only thumb_joint1..3.
        if m.thumb_priority and self._thumb_pair_violated(q):
            q_thumb = self._binary_scale(q, q_prev, thumb_only=True)
            if self._clearance_ok(q_thumb):
                return q_thumb
            # Pull thumb joints toward open (lower) while freezing fingers.
            q_open = q.copy()
            lim3 = self._limits_now()
            q_open[_THUMB] = lim3[_THUMB, 0].copy()
            q_thumb2 = self._binary_scale(q_open, q_prev, thumb_only=True)
            if self._clearance_ok(q_thumb2):
                return q_thumb2

        # 2) Full-hand scale fallback.
        return self._binary_scale(q, q_prev, thumb_only=False)

    def step(self, q_raw: np.ndarray, *, dt: float | None = None) -> np.ndarray:
        dt_use = float(dt) if dt is not None and dt > 1e-6 else self.dt_default
        lim = self._limits_now()
        q = clamp_q(q_raw, lim)
        q = soft_collision_project(
            q,
            lim,
            enabled=self.soft_collision,
            gains=self.collision_gains,
        )

        if self._needs_reseed or self._q_cmd is None:
            q = clamp_q(q, lim)
            if self.motion.enabled and self._fk is not None and not self._clearance_ok(q):
                q0 = clamp_q(np.zeros_like(q), lim)
                q = self._enforce_clearance(q, q0)
            self._q_cmd = q.copy()
            self._q_vel = np.zeros_like(q)
            self._needs_reseed = False
            return self._q_cmd.copy()

        q_prev = self._q_cmd

        if self.alpha < 1.0 - 1e-12:
            q = self.alpha * q + (1.0 - self.alpha) * q_prev
            q = clamp_q(q, lim)
            q = soft_collision_project(
                q,
                lim,
                enabled=self.soft_collision,
                gains=self.collision_gains,
            )

        q = self._apply_velocity(q, q_prev, dt_use)
        q = clamp_q(q, lim)
        q = self._enforce_clearance(q, q_prev)
        q = clamp_q(q, lim)

        if self.motion.enabled and self._fk is not None and not self._clearance_ok(q):
            self.collision_blocks += 1
            q = q_prev.copy()

        self._q_vel = (q - q_prev) / dt_use
        self._q_cmd = q
        return self._q_cmd.copy()
