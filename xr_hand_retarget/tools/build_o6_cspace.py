#!/usr/bin/env python3
"""Build O6 joint-space safety masks (pad/tip spheres → safe grid).

Usage (from xr_hand_retarget package root)::

    python tools/build_o6_cspace.py --side both
    python -m xr_hand_retarget.tools.build_o6_cspace --side left

Requires the package installed (or PYTHONPATH including this tree).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

_HERE = Path(__file__).resolve().parent
_PKG_ROOT = _HERE.parent
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

from xr_hand_retarget.cspace import (  # noqa: E402
    DEFAULT_BINS,
    DEFAULT_WEIGHTS,
    build_cspace_table,
    default_build_meta,
    resolve_cspace_path,
)
from xr_hand_retarget.kinematics import (  # noqa: E402
    load_o6_limits,
    make_o6_fk,
)
from xr_hand_retarget.safety import (  # noqa: E402
    JointMotionFilter,
    MotionSafetyGains,
    parse_motion_safety,
)

_O6_JOINTS = [
    "thumb_joint1",
    "thumb_joint2",
    "index_joint",
    "middle_joint",
    "ring_joint",
    "pinky_joint",
]


def _load_yaml(path: Path) -> dict:
    if not path.is_file():
        return {}
    with path.open("r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}
    return raw if isinstance(raw, dict) else {}


def _deep_merge(base: dict, over: dict) -> dict:
    out = dict(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def _motion_gains() -> MotionSafetyGains:
    defaults = _load_yaml(_PKG_ROOT / "configs" / "linker_defaults.yaml")
    hands = _load_yaml(_PKG_ROOT / "configs" / "hands" / "o6.yaml")
    merged = _deep_merge(defaults, hands)
    safety = merged.get("safety") or {}
    # Builder always needs FK labeling even if runtime safety.enabled is false.
    safety = dict(safety)
    safety["enabled"] = True
    pos = dict(safety.get("position") or {})
    fk = dict(pos.get("fk") or {})
    fk["enabled"] = True
    pos["fk"] = fk
    safety["position"] = pos
    return parse_motion_safety(safety)


def _parse_bins(raw: str | None) -> tuple[int, ...]:
    if not raw:
        return DEFAULT_BINS
    parts = [int(x.strip()) for x in raw.split(",") if x.strip()]
    if len(parts) != 6:
        raise SystemExit(f"--bins needs 6 ints, got {parts!r}")
    return tuple(parts)


def _parse_weights(raw: str | None) -> tuple[float, ...]:
    if not raw:
        return DEFAULT_WEIGHTS
    parts = [float(x.strip()) for x in raw.split(",") if x.strip()]
    if len(parts) != 6:
        raise SystemExit(f"--weights needs 6 floats, got {parts!r}")
    return tuple(parts)


def build_side(
    side: str,
    *,
    bins: tuple[int, ...],
    weights: tuple[float, ...],
    out: Path | None,
    progress_every: int,
) -> Path:
    side = str(side).strip().lower()
    gains = _motion_gains()
    # Force FK for labeling.
    gains.fk_enabled = True
    limits, urdf_vel = load_o6_limits(side)
    fk = make_o6_fk(side)
    filt = JointMotionFilter(
        np.asarray(limits, dtype=np.float64),
        np.asarray(urdf_vel, dtype=np.float64).reshape(-1)[:6],
        gains,
        dt=0.02,
        fk=fk,
        joint_names=list(_O6_JOINTS),
        side=side,
        load_cspace=False,
    )
    if filt._fk is None:
        raise RuntimeError("FK not initialized; cannot label cspace")
    table = build_cspace_table(
        filt, bins=bins, weights=weights, progress_every=progress_every
    )
    path = out or resolve_cspace_path(None, side)
    meta = default_build_meta(gains, side=side, bins=table.bins)
    table.save(path, meta=meta)
    print(f"[cspace] wrote {path} safe_frac={table.safe_fraction:.3f}", flush=True)
    return path


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--side",
        default="both",
        choices=("left", "right", "both"),
        help="Which hand table to build",
    )
    p.add_argument(
        "--bins",
        default=None,
        help="Comma bins, default 12,16,8,8,8,8",
    )
    p.add_argument(
        "--weights",
        default=None,
        help="Comma projection weights, default 4,10,0.2,0.2,0.2,0.2",
    )
    p.add_argument(
        "--out",
        default=None,
        help="Output npz (single side only); default assets/cspace/o6_{side}.npz",
    )
    p.add_argument(
        "--progress-every",
        type=int,
        default=50000,
        help="Print progress every N cells (0=quiet)",
    )
    args = p.parse_args(argv)
    bins = _parse_bins(args.bins)
    weights = _parse_weights(args.weights)
    sides = ("left", "right") if args.side == "both" else (args.side,)
    if args.out and len(sides) > 1:
        raise SystemExit("--out requires a single --side")
    for side in sides:
        build_side(
            side,
            bins=bins,
            weights=weights,
            out=Path(args.out) if args.out else None,
            progress_every=int(args.progress_every),
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
