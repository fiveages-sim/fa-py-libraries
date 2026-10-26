"""Stateful filtering for online robot joint commands (teleop_ws OnlineCommandFilter)."""

from __future__ import annotations

import math

import numpy as np


class OnlineCommandFilter:
    """Low-pass filter + deadband + velocity limit + joint limits."""

    def __init__(self, lower, upper, cutoff_hz, max_velocity, deadband):
        self.lower = np.asarray(lower, dtype=float)
        self.upper = np.asarray(upper, dtype=float)
        self.cutoff_hz = float(cutoff_hz)
        self.max_velocity = float(max_velocity)
        self.deadband = float(deadband)
        if self.lower.shape != self.upper.shape:
            raise ValueError("Filter lower/upper limits must have equal shapes")
        if self.cutoff_hz <= 0.0 or not np.isfinite(self.cutoff_hz):
            raise ValueError("filter_cutoff_hz must be positive")
        if self.max_velocity <= 0.0 or not np.isfinite(self.max_velocity):
            raise ValueError("max_velocity must be positive")
        if self.deadband < 0.0 or not np.isfinite(self.deadband):
            raise ValueError("command_epsilon must be non-negative")
        self.position = None

    def reset(self, position):
        position = np.asarray(position, dtype=float)
        if position.shape != self.lower.shape:
            raise ValueError("Initial filter position has the wrong shape")
        self.position = np.clip(position, self.lower, self.upper).copy()
        return self.position.copy()

    def update(self, target, dt):
        target = np.asarray(target, dtype=float)
        if target.shape != self.lower.shape:
            raise ValueError("Filter target has the wrong shape")
        if self.position is None:
            raise RuntimeError("Filter must be reset from a seed position first")
        dt = float(dt)
        if dt <= 0.0 or not np.isfinite(dt):
            return self.position.copy()

        target = np.clip(target, self.lower, self.upper)
        error = target - self.position
        error[np.abs(error) < self.deadband] = 0.0
        alpha = 1.0 - math.exp(-2.0 * math.pi * self.cutoff_hz * dt)
        desired_step = alpha * error
        max_step = self.max_velocity * dt
        step = np.clip(desired_step, -max_step, max_step)
        self.position = np.clip(self.position + step, self.lower, self.upper)
        return self.position.copy()
