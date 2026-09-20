"""Dump / verify T_xhand from XHand1 URDF (device once, not per operator).

Limits and pinch lateral slots come from assets/xhand_{left,right}.urdf.
Person curl endpoints stay in calib/xrt_default.yaml (T_person).

  python -m xr_hand_retarget.txhand --side right
  python -m xr_hand_retarget.txhand --side both --json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


def _dump_side(side: str, *, urdf_path: str | None, pinch_target: str) -> dict:
    from xr_hand_retarget.algorithms.curl_xhand1 import pinch_q1_from_urdf
    from xr_hand_retarget.algorithms.kinematics_xhand1 import (
        load_urdf_joint_limits_velocity,
        make_hand_fk,
    )
    from xr_hand_retarget.algorithms.remap import XHAND1_JOINT_NAMES

    limits, vel = load_urdf_joint_limits_velocity(side, urdf_path=urdf_path)
    fk = make_hand_fk(side, urdf_path=urdf_path)
    # Rest thumb flex for slot table (open-ish j2/j3).
    pinch = pinch_q1_from_urdf(
        fk,
        0.0,
        0.0,
        target=pinch_target,
        q1_lo=float(limits[0, 0]),
        q1_hi=float(limits[0, 1]),
    )
    return {
        "side": side,
        "urdf": str(fk.urdf_path),
        "pinch_target_link": pinch_target,
        "limits": {
            name: [float(limits[i, 0]), float(limits[i, 1])]
            for i, name in enumerate(XHAND1_JOINT_NAMES)
        },
        "velocity": {
            name: float(vel[i]) for i, name in enumerate(XHAND1_JOINT_NAMES)
        },
        "pinch_q1_urdf": {k: float(v) for k, v in pinch.items()},
    }


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--side", choices=("left", "right", "both"), default="right")
    p.add_argument("--urdf", default=None, help="Override URDF path")
    p.add_argument(
        "--pinch-target",
        default="pip",
        choices=("mcp", "pip", "tip"),
        help="Finger link used for thumb_joint1 lateral slot",
    )
    p.add_argument("--json", action="store_true")
    p.add_argument(
        "--compare-config",
        default=None,
        help="hands/xhand1.yaml path; print delta vs yaml hand.limits",
    )
    args = p.parse_args(argv)

    sides = ("left", "right") if args.side == "both" else (args.side,)
    reports = [
        _dump_side(s, urdf_path=args.urdf, pinch_target=args.pinch_target) for s in sides
    ]

    if args.compare_config:
        from xr_hand_retarget.backends.xhand1_config import load_hand_config
        import yaml

        cfg = load_hand_config(args.compare_config)
        raw = yaml.safe_load(Path(args.compare_config).read_text(encoding="utf-8")) or {}
        yaml_limits = np.asarray((raw.get("hand") or {}).get("limits"), dtype=np.float64)
        print(
            f"[compare] config limits_source={cfg.limits_source} "
            f"use_urdf_limits={cfg.use_urdf_limits}",
            flush=True,
        )
        for rep in reports:
            urdf_lim = np.asarray(
                list(rep["limits"].values()), dtype=np.float64
            )
            if yaml_limits.shape == urdf_lim.shape:
                delta = float(np.max(np.abs(urdf_lim - yaml_limits)))
                print(f"  {rep['side']}: max|URDF−yaml.limits|={delta:.6f}", flush=True)
            else:
                print(
                    f"  {rep['side']}: shape mismatch yaml={yaml_limits.shape} "
                    f"urdf={urdf_lim.shape}",
                    flush=True,
                )

    if args.json:
        print(json.dumps(reports if len(reports) > 1 else reports[0], indent=2))
        return 0

    for rep in reports:
        print(f"=== T_xhand {rep['side']} ===", flush=True)
        print(f"urdf: {rep['urdf']}", flush=True)
        print(f"pinch_target: {rep['pinch_target_link']}", flush=True)
        print("limits (lo, hi):", flush=True)
        for name, (lo, hi) in rep["limits"].items():
            print(f"  {name:16s}  [{lo:8.4f}, {hi:8.4f}]", flush=True)
        print("pinch thumb_joint1 from URDF Y-slot:", flush=True)
        for name, q1 in rep["pinch_q1_urdf"].items():
            print(f"  {name:8s}  q1={q1:.4f}", flush=True)
        print(flush=True)
    print(
        "Write finger_scale / pinch_target under robot: in hands/xhand1.yaml.\n"
        "Do not put these in calib/xrt_default.yaml (that is T_person).",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
