"""One VR session → PalmTip curl exports for LinkerHand O6/L6/O7."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from xr_hand_retarget.algorithms.palm_tip import (
    FINGERS,
    PalmTipCalib,
    PalmTipFeat,
    apply_pose,
    dump_palm_calib,
    mirror_palm_calib,
    seed_palm_calib,
)

SCHEMA = "linkerhand_session.v1"
_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SESSION = _PACKAGE_ROOT / "configs" / "calib" / "linker_session.yaml"
PALM_TARGETS = ("o6", "l6", "o7")


def palm_feat_to_dict(feat: PalmTipFeat) -> dict[str, Any]:
    pinch_xyz = {
        n: [float(v[0]), float(v[1]), float(v[2])]
        for n, v in (feat.pinch_xyz or {}).items()
    }
    finger_chain = {
        n: [float(v[0]), float(v[1]), float(v[2])]
        for n, v in (feat.finger_chain or {}).items()
    }
    return {
        "thumb_az": float(feat.thumb_az),
        "thumb_curl": float(feat.thumb_curl),
        "finger_curl": float(feat.finger_curl),
        "pinch_d": float(feat.pinch_d),
        "nest_r": float(getattr(feat, "nest_r", 0.0)),
        "nest_lat": float(getattr(feat, "nest_lat", 0.0)),
        "nest_r_p": float(getattr(feat, "nest_r_p", 0.0)),
        "slot_t": float(getattr(feat, "slot_t", 0.0)),
        "pack_r": float(getattr(feat, "pack_r", 0.0)),
        "pack_h": float(getattr(feat, "pack_h", 0.0)),
        "pinch_xyz": pinch_xyz,
        "pinch_align": {
            n: float(v) for n, v in (feat.pinch_align or {}).items()
        },
        "finger_chain": finger_chain,
    }


def palm_feat_from_dict(raw: dict[str, Any] | None) -> PalmTipFeat | None:
    if not isinstance(raw, dict):
        return None
    pinch_xyz = {}
    for n, v in (raw.get("pinch_xyz") or {}).items():
        if isinstance(v, (list, tuple)) and len(v) >= 3:
            pinch_xyz[n] = (float(v[0]), float(v[1]), float(v[2]))
    finger_chain = {}
    for n, v in (raw.get("finger_chain") or {}).items():
        if isinstance(v, (list, tuple)) and len(v) >= 3:
            finger_chain[n] = (float(v[0]), float(v[1]), float(v[2]))
    return PalmTipFeat(
        thumb_az=float(raw.get("thumb_az", 0.0)),
        thumb_curl=float(raw.get("thumb_curl", 0.0)),
        finger_curl=float(raw.get("finger_curl", 0.0)),
        pinch_d=float(raw.get("pinch_d", 0.0)),
        nest_r=float(raw.get("nest_r", 0.0)),
        nest_lat=float(raw.get("nest_lat", 0.0)),
        nest_r_p=float(raw.get("nest_r_p", 0.0)),
        slot_t=float(raw.get("slot_t", 0.0)),
        pack_r=float(raw.get("pack_r", 0.0)),
        pack_h=float(raw.get("pack_h", 0.0)),
        pinch_xyz=pinch_xyz,
        pinch_align={
            n: float(v) for n, v in (raw.get("pinch_align") or {}).items()
        },
        finger_chain=finger_chain,
    )


def median_palm_feat(samples: list[PalmTipFeat]) -> PalmTipFeat:
    def med(vals):
        return float(np.median(np.asarray(vals, dtype=np.float64)))

    return PalmTipFeat(
        thumb_az=med([s.thumb_az for s in samples]),
        thumb_curl=med([s.thumb_curl for s in samples]),
        finger_curl=med([s.finger_curl for s in samples]),
        pinch_d=med([s.pinch_d for s in samples]),
        nest_r=med([getattr(s, "nest_r", 0.0) for s in samples]),
        nest_lat=med([getattr(s, "nest_lat", 0.0) for s in samples]),
        nest_r_p=med([getattr(s, "nest_r_p", 0.0) for s in samples]),
        slot_t=med([getattr(s, "slot_t", 0.0) for s in samples]),
        pack_r=med([getattr(s, "pack_r", 0.0) for s in samples]),
        pack_h=med([getattr(s, "pack_h", 0.0) for s in samples]),
        pinch_xyz={
            n: tuple(
                med(
                    [
                        ((s.pinch_xyz or {}).get(n) or (0.0, 0.0, 0.0))[i]
                        for s in samples
                    ]
                )
                for i in range(3)
            )
            for n in FINGERS
        },
        pinch_align={
            n: med([(s.pinch_align or {}).get(n, 0.0) for s in samples])
            for n in FINGERS
        },
        finger_chain={
            n: tuple(
                med(
                    [
                        ((s.finger_chain or {}).get(n) or (0.0, 0.0, 0.0))[i]
                        for s in samples
                    ]
                )
                for i in range(3)
            )
            for n in FINGERS
        },
    )


def build_palm_calib_from_takes(takes: dict[str, list[dict[str, Any]]]) -> PalmTipCalib:
    cal = seed_palm_calib()
    for pose_name, take_list in takes.items():
        feats = [
            palm_feat_from_dict(t.get("palm"))
            for t in take_list
            if isinstance(t, dict) and t.get("palm")
        ]
        feats = [f for f in feats if f is not None]
        if not feats:
            continue
        apply_pose(cal, pose_name, median_palm_feat(feats))
    pinch_poses = [n for n in takes if str(n).startswith("pinch_")]
    if pinch_poses:
        cal.pinch_skipped = not any(takes.get(n) for n in pinch_poses)
    return cal


def dump_session(
    path: str | Path,
    sides: dict[str, dict[str, list[dict[str, Any]]]],
    *,
    source: str = "xrobotoolkit",
    headset: str = "pico",
) -> None:
    import yaml

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema": SCHEMA,
        "source": source,
        "headset": headset,
        "hands": list(PALM_TARGETS),
        "sides": {name: {"takes": takes} for name, takes in sides.items()},
    }
    header = (
        "# Unified LinkerHand calibration session (O6/L6/O7).\n"
        f"# schema: {SCHEMA}\n"
        "# One VR pass stores PalmTip features per pose.\n"
        "# Export without headset:\n"
        "#   python -m xr_hand_retarget.calibrate --config configs/o6.yaml "
        "--export-only\n"
        "# Recapture:\n"
        "#   python -m xr_hand_retarget.calibrate --config configs/o6.yaml\n"
    )
    with path.open("w", encoding="utf-8") as f:
        f.write(header)
        yaml.safe_dump(payload, f, sort_keys=False, allow_unicode=True)


def load_session(path: str | Path) -> dict[str, Any]:
    import yaml

    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        return {}
    return raw


def _side_takes(session: dict[str, Any], side: str) -> dict[str, list[dict[str, Any]]]:
    sides = session.get("sides") or {}
    block = sides.get(side) or {}
    takes = block.get("takes") or {}
    return takes if isinstance(takes, dict) else {}


def export_from_session(
    session: dict[str, Any],
    *,
    mirror_from: str | None = None,
    mirror_to: str | None = None,
    az_sign: float = -1.0,
    lat_sign: float = -1.0,
    palm_dir: Path | None = None,
    source: str | None = None,
    headset: str | None = None,
    **_ignored: Any,
) -> list[Path]:
    del lat_sign  # kept for call-site compat; PalmTip mirror uses az_sign only
    src = str(source or session.get("source") or "xrobotoolkit")
    hs = str(headset or session.get("headset") or "pico")
    palm_root = palm_dir or (_PACKAGE_ROOT / "configs" / "calib")

    palm_blocks: dict[str, PalmTipCalib] = {}
    for side in ("left", "right"):
        takes = _side_takes(session, side)
        if not takes:
            continue
        palm_blocks[side] = build_palm_calib_from_takes(takes)

    if mirror_from and mirror_to and mirror_from in palm_blocks:
        palm_blocks[mirror_to] = mirror_palm_calib(
            palm_blocks[mirror_from], from_side=mirror_from, az_sign=az_sign
        )

    written: list[Path] = []
    for model in PALM_TARGETS:
        out = palm_root / f"{model}.yaml"
        dump_palm_calib(out, palm_blocks, source=src, headset=hs, model=model)
        written.append(out)
    return written
