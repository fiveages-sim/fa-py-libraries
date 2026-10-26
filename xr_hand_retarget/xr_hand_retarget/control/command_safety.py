"""Publish-path command safety matching teleop_ws DexCap hand retarget."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from xr_hand_retarget.control.command_filter import OnlineCommandFilter
from xr_hand_retarget.control.command_gate import FsmCommandGate


def default_max_velocity(backend: str) -> float:
    """teleop_ws launch defaults: xhand1=6, wuji_hand2=1.5; Linker ≈ finger vmax."""
    key = (backend or "").lower()
    if key == "xhand1":
        return 6.0
    if key == "wuji":
        return 1.5
    if key in ("o6", "l6", "o7", "linker"):
        return 4.0
    return 4.0


@dataclass
class CommandSafetyConfig:
    """YAML ``safety.command`` — DexCap OnlineCommandFilter + FsmCommandGate."""

    enabled: bool = True
    publish_command: bool = True
    allowed_fsm_states: tuple[int, ...] = (3, 4)  # OCS2, MOVEJ
    max_velocity: float | None = None  # None → default_max_velocity(backend)
    filter_cutoff_hz: float = 8.0
    command_epsilon: float = 0.001
    target_timeout: float = 0.25
    # When True, gate stays closed until filter.reset() after open (DexCap).
    require_resync: bool = True


# Keys shared with teleop_ws dexcap_hand_retarget.launch.py (flat operator names).
_TELEOP_COMMAND_KEYS = (
    "enabled",
    "publish_command",
    "allowed_fsm_states",
    "max_velocity",
    "max_joint_vel_rad_s",
    "filter_cutoff_hz",
    "command_epsilon",
    "target_timeout",
    "require_resync",
    "command_publish_rate",  # accepted, unused (rate comes from publish.rate_hz)
)


def _command_mapping_from_block(raw: dict[str, Any]) -> dict[str, Any]:
    """Normalize a flat teleop-style mapping into safety.command fields."""
    out: dict[str, Any] = {}
    for key in _TELEOP_COMMAND_KEYS:
        if key in raw and raw[key] is not None:
            out[key] = raw[key]
    return out


def parse_command_block(
    raw: dict[str, Any] | None,
    *,
    backend: str,
) -> CommandSafetyConfig:
    """Parse a teleop_ws-style flat command mapping (or nested safety.command)."""
    block = raw if isinstance(raw, dict) else {}
    states = block.get("allowed_fsm_states") or (3, 4)
    vmax = block.get("max_velocity", block.get("max_joint_vel_rad_s"))
    return CommandSafetyConfig(
        enabled=bool(block.get("enabled", True)),
        publish_command=bool(block.get("publish_command", True)),
        allowed_fsm_states=tuple(int(s) for s in states),
        max_velocity=None if vmax in (None, "") else float(vmax),
        filter_cutoff_hz=float(block.get("filter_cutoff_hz", 8.0)),
        command_epsilon=float(block.get("command_epsilon", 0.001)),
        target_timeout=float(block.get("target_timeout", 0.25)),
        require_resync=bool(block.get("require_resync", True)),
    )


def parse_command_safety(
    safety: dict[str, Any] | None,
    *,
    backend: str,
) -> CommandSafetyConfig:
    raw_root = safety or {}
    raw = raw_root.get("command")
    if raw is None:
        # Legacy: top-level max_joint_vel_rad_s still maps into command vmax.
        enabled = bool(raw_root.get("command_enabled", True))
        vmax = raw_root.get("max_joint_vel_rad_s")
        return CommandSafetyConfig(
            enabled=enabled,
            max_velocity=(
                None
                if vmax in (None, 0, 0.0)
                else float(vmax)
            ),
        )
    if not isinstance(raw, dict):
        raise ValueError("safety.command must be a mapping")
    return parse_command_block(raw, backend=backend)


def merge_command_override(
    base: CommandSafetyConfig | None,
    override: dict[str, Any] | None,
    *,
    backend: str,
) -> CommandSafetyConfig:
    """Overlay teleop-style keys onto an existing CommandSafetyConfig."""
    cur = base or CommandSafetyConfig()
    patch = _command_mapping_from_block(override or {})
    if not patch:
        return cur
    merged = {
        "enabled": cur.enabled,
        "publish_command": cur.publish_command,
        "allowed_fsm_states": list(cur.allowed_fsm_states),
        "max_velocity": cur.max_velocity,
        "filter_cutoff_hz": cur.filter_cutoff_hz,
        "command_epsilon": cur.command_epsilon,
        "target_timeout": cur.target_timeout,
        "require_resync": cur.require_resync,
    }
    merged.update(patch)
    return parse_command_block(merged, backend=backend)


def extract_local_command_block(raw: dict[str, Any] | None) -> dict[str, Any] | None:
    """Prefer top-level ``command:`` (teleop_ws names); else ``hands.command`` / ``safety.command``."""
    if not isinstance(raw, dict):
        return None
    for key_path in (
        ("command",),
        ("hands", "command"),
        ("safety", "command"),
    ):
        cur: Any = raw
        ok = True
        for key in key_path:
            if not isinstance(cur, dict) or key not in cur:
                ok = False
                break
            cur = cur[key]
        if ok and isinstance(cur, dict):
            return dict(cur)
    return None


@dataclass
class SideCommandSafety:
    """Per-side filter + shared gate consumer helpers."""

    filter: OnlineCommandFilter
    cfg: CommandSafetyConfig
    last_target_time: float | None = None
    stale: bool = False

    @classmethod
    def create(
        cls,
        *,
        limits: np.ndarray,
        cfg: CommandSafetyConfig,
        backend: str,
    ) -> "SideCommandSafety":
        lim = np.asarray(limits, dtype=float)
        if lim.ndim != 2 or lim.shape[1] != 2:
            raise ValueError(f"limits must be (N,2), got {lim.shape}")
        vmax = float(cfg.max_velocity or default_max_velocity(backend))
        filt = OnlineCommandFilter(
            lower=lim[:, 0],
            upper=lim[:, 1],
            cutoff_hz=cfg.filter_cutoff_hz,
            max_velocity=vmax,
            deadband=cfg.command_epsilon,
        )
        # Seed at mid-range so first update works before explicit reset.
        seed = 0.5 * (lim[:, 0] + lim[:, 1])
        filt.reset(seed)
        return cls(filter=filt, cfg=cfg)

    def reset(self, position: np.ndarray) -> np.ndarray:
        return self.filter.reset(position)

    def process(
        self,
        target: np.ndarray,
        *,
        now: float,
        dt: float,
        target_fresh: bool,
    ) -> np.ndarray:
        """Filter a fresh target, or hold last filtered q when stale/timeout."""
        if self.filter.position is None:
            self.filter.reset(target)
        if target_fresh:
            self.last_target_time = now
            self.stale = False
            return self.filter.update(target, dt)
        age = (
            float("inf")
            if self.last_target_time is None
            else now - self.last_target_time
        )
        if age <= float(self.cfg.target_timeout):
            return self.filter.update(target, dt)
        self.stale = True
        return self.filter.position.copy()


def make_fsm_gate(cfg: CommandSafetyConfig) -> FsmCommandGate:
    return FsmCommandGate(cfg.allowed_fsm_states)
