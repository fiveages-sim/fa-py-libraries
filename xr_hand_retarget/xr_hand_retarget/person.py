"""Standard hand + T_person helpers (no ROS)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
_DEFAULT_STANDARD = _PACKAGE_ROOT / "configs" / "standard_hand.yaml"

_FINGER_ORDER = ("thumb", "index", "middle", "ring", "pinky")


@dataclass(frozen=True)
class StandardHand:
    """Fixed MediaPipe-21 proportions (T_standard)."""

    scale_m: float
    bones: dict[str, tuple[float, float, float]]
    path: Path

    def bone_lengths_m(self, finger: str) -> np.ndarray:
        fracs = np.asarray(self.bones[finger], dtype=np.float64)
        return fracs * float(self.scale_m)


@dataclass(frozen=True)
class PersonCalib:
    """T_person: operator → standard hand (optional overlays)."""

    path: Path | None
    bone_scale: float = 1.0
    segment_scaling: dict[str, list[float]] | None = None
    raw: dict[str, Any] | None = None


def default_standard_path() -> Path:
    return _DEFAULT_STANDARD


def load_standard_hand(path: str | Path | None = None) -> StandardHand:
    cfg = Path(path) if path else _DEFAULT_STANDARD
    raw = yaml.safe_load(cfg.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"{cfg} must be a mapping")
    bones_raw = raw.get("bones") or {}
    bones: dict[str, tuple[float, float, float]] = {}
    for name in _FINGER_ORDER:
        seq = bones_raw.get(name)
        if seq is None:
            raise ValueError(f"{cfg}: bones.{name} required")
        bones[name] = (float(seq[0]), float(seq[1]), float(seq[2]))
    return StandardHand(
        scale_m=float(raw.get("scale_m") or 0.095),
        bones=bones,
        path=cfg.resolve(),
    )


def load_person_calib(path: str | Path | None) -> PersonCalib:
    """Load a users/*.yaml or curl calib file as T_person metadata.

    Curl pose endpoints stay in the file for solvers; optional ``bone_scale`` /
    ``segment_scaling`` are person overlays for official Wuji / future HandFrame.
    """
    if path in (None, ""):
        return PersonCalib(path=None)
    cfg = Path(path).expanduser()
    if not cfg.is_file():
        raise FileNotFoundError(f"user calib not found: {cfg}")
    raw = yaml.safe_load(cfg.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"{cfg} must be a mapping")
    person = raw.get("person") if isinstance(raw.get("person"), dict) else {}
    seg = person.get("segment_scaling") or raw.get("segment_scaling")
    scale = float(person.get("bone_scale") or raw.get("bone_scale") or 1.0)
    return PersonCalib(
        path=cfg.resolve(),
        bone_scale=scale,
        segment_scaling=dict(seg) if isinstance(seg, dict) else None,
        raw=raw,
    )


def merge_segment_scaling(
    robot: dict[str, Any] | None,
    person: dict[str, Any] | None,
) -> dict[str, Any]:
    """T_robot segment_scaling, then optional T_person override (same keys)."""
    out: dict[str, Any] = {}
    if robot:
        out.update(robot)
    if person:
        out.update(person)
    return out
