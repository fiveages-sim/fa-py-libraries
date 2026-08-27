"""Safety filters for commanded joint vectors.

Design (teleop with position + limited torque):
- Hard: URDF joint limits.
- Soft: optional light EMA; URDF-informed self-collision projection.
- Hold last command when tracking is inactive / zero.
- No rate limit / jump reject by default (controller torque limits the plant).
"""

from __future__ import annotations

import numpy as np

from .collision import soft_collision_project


def clamp_q(q: np.ndarray, limits: np.ndarray) -> np.ndarray:
    """Clip q to [lower, upper] per joint. limits shape (N, 2)."""
    q = np.asarray(q, dtype=np.float64).reshape(-1)
    lo = limits[:, 0]
    hi = limits[:, 1]
    return np.clip(q, lo, hi)


def pose_is_zero(joints26: np.ndarray, eps: float = 1e-6) -> bool:
    xyz = np.asarray(joints26, dtype=np.float64)[:, :3]
    return not bool(np.any(np.abs(xyz) > eps))


class CommandSafety:
    """clamp → soft collision → optional EMA → hold/reseed."""

    def __init__(
        self,
        limits: np.ndarray,
        *,
        alpha: float = 1.0,
        soft_collision: bool = True,
        max_delta_rad: float = 0.0,
        jump_reject_rad: float = 0.0,
    ):
        self.limits = np.asarray(limits, dtype=np.float64)
        # alpha=1 → passthrough; kept for optional smoothing only.
        self.alpha = float(np.clip(alpha, 1e-3, 1.0))
        self.soft_collision = bool(soft_collision)
        # Retained for CLI/config compatibility; 0 disables.
        self.max_delta_rad = float(max_delta_rad)
        self.jump_reject_rad = float(jump_reject_rad)
        self._q_cmd: np.ndarray | None = None
        self._needs_reseed = True
        self.rejected_jumps = 0  # legacy counter (unused when jump disabled)

    @property
    def last_command(self) -> np.ndarray | None:
        return None if self._q_cmd is None else self._q_cmd.copy()

    def hold(self) -> np.ndarray:
        """Keep last command; do not seed zeros."""
        self._needs_reseed = True
        if self._q_cmd is None:
            return np.zeros(self.limits.shape[0], dtype=np.float64)
        return self._q_cmd.copy()

    def step(self, q_raw: np.ndarray) -> np.ndarray:
        q = clamp_q(q_raw, self.limits)
        q = soft_collision_project(q, self.limits, enabled=self.soft_collision)

        if self._needs_reseed or self._q_cmd is None:
            self._q_cmd = q.copy()
            self._needs_reseed = False
            return self._q_cmd.copy()

        if self.alpha < 1.0 - 1e-12:
            q = self.alpha * q + (1.0 - self.alpha) * self._q_cmd
            q = clamp_q(q, self.limits)
            q = soft_collision_project(q, self.limits, enabled=self.soft_collision)

        # Optional legacy rate limit (off unless max_delta_rad > 0).
        if self.max_delta_rad > 0.0:
            delta = np.clip(q - self._q_cmd, -self.max_delta_rad, self.max_delta_rad)
            q = self._q_cmd + delta
            q = clamp_q(q, self.limits)

        # Optional legacy jump reject (off unless jump_reject_rad > 0).
        if self.jump_reject_rad > 0.0:
            if bool(np.any(np.abs(q - self._q_cmd) > self.jump_reject_rad)):
                self.rejected_jumps += 1
                return self._q_cmd.copy()

        self._q_cmd = q
        return self._q_cmd.copy()
