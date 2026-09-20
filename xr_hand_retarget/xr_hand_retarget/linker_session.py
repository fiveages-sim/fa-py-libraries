"""One VR session → PalmTip curl + mp_curl exports for LinkerHand O6/L6/O7."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from xr_hand_retarget.algorithms.curl_hand2 import SideCurlCalib2, seed_side_calib
from xr_hand_retarget.algorithms.curl_xhand1 import PinchKnot
from xr_hand_retarget.algorithms.mp_curl_o6 import (
    _DEFAULT_CALIB as _DEFAULT_MP_CALIB,
    dump_calib_file as dump_mp_curl_calib,
    feat2_from_dict,
    feat2_to_dict,
)
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


def _mp_pose_names() -> frozenset[str]:
    pinch = {f"pinch_{n}" for n in FINGERS}
    return frozenset({"open", "fist", "thumb_along", "along", *pinch})


MP_CURL_POSES = _mp_pose_names()


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
        "finger_curl": {n: float(feat.finger_curl[n]) for n in FINGERS},
        "thumb_curl": float(feat.thumb_curl),
        "thumb_az": float(feat.thumb_az),
        "thumb_h": float(getattr(feat, "thumb_h", 0.0)),
        "pinch_d": {n: float((feat.pinch_d or {}).get(n, 0.0)) for n in FINGERS},
        "pinch_xyz": pinch_xyz,
        "pinch_align": {
            n: float((feat.pinch_align or {}).get(n, 0.0)) for n in FINGERS
        },
        "nest_r": float(getattr(feat, "nest_r", 0.0)),
        "nest_lat": float(getattr(feat, "nest_lat", 0.0)),
        "nest_r_p": float(getattr(feat, "nest_r_p", 0.0)),
        "slot_t": float(getattr(feat, "slot_t", 0.0)),
        "pack_h": {n: float((feat.pack_h or {}).get(n, 0.0)) for n in FINGERS},
        "pack_r": {n: float((feat.pack_r or {}).get(n, 0.0)) for n in FINGERS},
        "finger_chain": finger_chain,
    }


def palm_feat_from_dict(raw: dict[str, Any] | None) -> PalmTipFeat | None:
    if not raw:
        return None
    finger_curl = raw.get("finger_curl") or {}
    pinch_xyz_raw = raw.get("pinch_xyz") or {}
    pinch_xyz = {
        n: (float(v[0]), float(v[1]), float(v[2]))
        for n, v in pinch_xyz_raw.items()
        if isinstance(v, (list, tuple)) and len(v) >= 3
    }
    chain_raw = raw.get("finger_chain") or {}
    finger_chain = {
        n: (float(v[0]), float(v[1]), float(v[2]))
        for n, v in chain_raw.items()
        if isinstance(v, (list, tuple)) and len(v) >= 3
    }
    return PalmTipFeat(
        finger_curl={n: float(finger_curl.get(n, 0.0)) for n in FINGERS},
        thumb_curl=float(raw.get("thumb_curl", 0.0)),
        thumb_az=float(raw.get("thumb_az", 0.0)),
        thumb_h=float(raw.get("thumb_h", 0.0)),
        pinch_d={n: float((raw.get("pinch_d") or {}).get(n, 0.0)) for n in FINGERS},
        pinch_xyz=pinch_xyz,
        pinch_align={
            n: float((raw.get("pinch_align") or {}).get(n, 0.0)) for n in FINGERS
        },
        nest_r=float(raw.get("nest_r", 0.0)),
        nest_lat=float(raw.get("nest_lat", 0.0)),
        nest_r_p=float(raw.get("nest_r_p", 0.0)),
        slot_t=float(raw.get("slot_t", 0.0)),
        pack_h={n: float((raw.get("pack_h") or {}).get(n, 0.0)) for n in FINGERS},
        pack_r={n: float((raw.get("pack_r") or {}).get(n, 0.0)) for n in FINGERS},
        finger_chain=finger_chain,
    )


def median_palm_feat(samples: list[PalmTipFeat]) -> PalmTipFeat:
    def med(vals):
        return float(np.median(np.asarray(vals, dtype=np.float64)))

    return PalmTipFeat(
        finger_curl={n: med([s.finger_curl[n] for s in samples]) for n in FINGERS},
        thumb_curl=med([s.thumb_curl for s in samples]),
        thumb_az=med([s.thumb_az for s in samples]),
        thumb_h=med([getattr(s, "thumb_h", 0.0) for s in samples]),
        pinch_d={
            n: med([(s.pinch_d or {}).get(n, 0.0) for s in samples])
            for n in FINGERS
        },
        nest_r=med([getattr(s, "nest_r", 0.0) for s in samples]),
        nest_lat=med([getattr(s, "nest_lat", 0.0) for s in samples]),
        nest_r_p=med([getattr(s, "nest_r_p", 0.0) for s in samples]),
        slot_t=med([getattr(s, "slot_t", 0.0) for s in samples]),
        pack_h={n: med([(s.pack_h or {}).get(n, 0.0) for s in samples]) for n in FINGERS},
        pack_r={n: med([(s.pack_r or {}).get(n, 0.0) for s in samples]) for n in FINGERS},
        pinch_xyz={
            n: tuple(
                med([((s.pinch_xyz or {}).get(n) or (0.0, 0.0, 0.0))[i] for s in samples])
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
                med([((s.finger_chain or {}).get(n) or (0.0, 0.0, 0.0))[i] for s in samples])
                for i in range(3)
            )
            for n in FINGERS
        },
    )


def median_feat2(samples):
    from xr_hand_retarget.algorithms.curl_hand2 import CurlFeatures2

    def med(vals):
        return float(np.median(np.asarray(vals, dtype=np.float64)))

    names = ("index", "middle", "ring", "pinky")
    return CurlFeatures2(
        mcp={n: med([s.mcp[n] for s in samples]) for n in names},
        pip={n: med([s.pip[n] for s in samples]) for n in names},
        dip={n: med([s.dip[n] for s in samples]) for n in names},
        thumb_mcp=med([s.thumb_mcp for s in samples]),
        thumb_ip=med([s.thumb_ip for s in samples]),
        thumb_lat=med([s.thumb_lat for s in samples]),
    )


def apply_mp_pose(calib: SideCurlCalib2, name: str, feat) -> None:
    pose = "thumb_along" if name == "along" else name
    if pose == "open":
        calib.open_mcp = dict(feat.mcp)
        calib.open_pip = dict(feat.pip)
        calib.open_dip = dict(feat.dip)
        calib.open_thumb_mcp = feat.thumb_mcp
        calib.open_thumb_ip = feat.thumb_ip
        calib.open_thumb_lat = feat.thumb_lat
    elif pose == "fist":
        calib.fist_mcp = dict(feat.mcp)
        calib.fist_pip = dict(feat.pip)
        calib.fist_dip = dict(feat.dip)
        calib.thumb_flex_mcp = float(feat.thumb_mcp)
        calib.thumb_flex_ip = float(feat.thumb_ip)
    elif pose == "thumb_along":
        calib.along_thumb_lat = float(feat.thumb_lat)
    elif pose.startswith("pinch_"):
        finger = pose[len("pinch_") :]
        if calib.pinch is None:
            calib.pinch = {}
        finger_mcp = feat.mcp.get(finger)
        finger_pip = feat.pip.get(finger)
        finger_dip = feat.dip.get(finger) if feat.dip else None
        calib.pinch[finger] = PinchKnot(
            thumb_lat=feat.thumb_lat,
            thumb_mcp=float(feat.thumb_mcp),
            thumb_ip=float(feat.thumb_ip),
            finger_mcp=None if finger_mcp is None else float(finger_mcp),
            finger_pip=None if finger_pip is None else float(finger_pip),
            finger_dip=None if finger_dip is None else float(finger_dip),
        )
        calib.pinch_skipped = False


def mirror_mp_side_calib(
    src: SideCurlCalib2, *, from_side: str, lat_sign: float = -1.0
) -> SideCurlCalib2:
    def lat(v: float) -> float:
        return float(v) * float(lat_sign)

    pinch = None
    if src.pinch is not None:
        pinch = {}
        for n, knot in src.pinch.items():
            if knot is None:
                pinch[n] = None
                continue
            pinch[n] = PinchKnot(
                thumb_lat=lat(knot.thumb_lat),
                thumb_mcp=float(knot.thumb_mcp),
                thumb_ip=float(knot.thumb_ip),
                finger_mcp=knot.finger_mcp,
                finger_pip=knot.finger_pip,
                finger_dip=knot.finger_dip,
            )
    return SideCurlCalib2(
        open_mcp=dict(src.open_mcp),
        open_pip=dict(src.open_pip),
        open_dip=dict(src.open_dip),
        fist_mcp=dict(src.fist_mcp),
        fist_pip=dict(src.fist_pip),
        fist_dip=dict(src.fist_dip),
        open_thumb_mcp=float(src.open_thumb_mcp),
        open_thumb_ip=float(src.open_thumb_ip),
        open_thumb_lat=lat(src.open_thumb_lat),
        thumb_flex_mcp=float(src.thumb_flex_mcp),
        thumb_flex_ip=float(src.thumb_flex_ip),
        along_thumb_lat=(
            None if src.along_thumb_lat is None else lat(src.along_thumb_lat)
        ),
        thumb_to_pinky_lat=lat(src.thumb_to_pinky_lat),
        pinch_skipped=bool(src.pinch_skipped),
        pinch=pinch,
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


def build_mp_calib_from_takes(takes: dict[str, list[dict[str, Any]]]) -> SideCurlCalib2:
    cal = seed_side_calib()
    for pose_name, take_list in takes.items():
        if pose_name not in MP_CURL_POSES and not str(pose_name).startswith("pinch_"):
            continue
        feats = [
            feat2_from_dict(t.get("mp"))
            for t in take_list
            if isinstance(t, dict) and t.get("mp")
        ]
        feats = [f for f in feats if f is not None]
        if not feats:
            continue
        apply_mp_pose(cal, str(pose_name), median_feat2(feats))
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
        "# One VR pass stores PalmTip + MP21 features per pose.\n"
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
    mp_path: Path | None = None,
    source: str | None = None,
    headset: str | None = None,
) -> list[Path]:
    src = str(source or session.get("source") or "xrobotoolkit")
    hs = str(headset or session.get("headset") or "pico")
    palm_root = palm_dir or (_PACKAGE_ROOT / "configs" / "calib")
    mp_out = mp_path or _DEFAULT_MP_CALIB

    palm_blocks: dict[str, PalmTipCalib] = {}
    mp_blocks: dict[str, SideCurlCalib2] = {}
    mp_takes: dict[str, dict[str, list[dict[str, Any]]]] = {}

    for side in ("left", "right"):
        takes = _side_takes(session, side)
        if not takes:
            continue
        palm_blocks[side] = build_palm_calib_from_takes(takes)
        mp_blocks[side] = build_mp_calib_from_takes(takes)
        mp_takes[side] = {
            pose: take_list
            for pose, take_list in takes.items()
            if pose in MP_CURL_POSES or str(pose).startswith("pinch_")
        }

    if mirror_from and mirror_to and mirror_from in palm_blocks:
        palm_blocks[mirror_to] = mirror_palm_calib(
            palm_blocks[mirror_from], from_side=mirror_from, az_sign=az_sign
        )
        mp_blocks[mirror_to] = mirror_mp_side_calib(
            mp_blocks[mirror_from], from_side=mirror_from, lat_sign=lat_sign
        )
        src_takes = _side_takes(session, mirror_from)
        mp_takes[mirror_to] = {
            pose: list(take_list)
            for pose, take_list in src_takes.items()
            if pose in MP_CURL_POSES or str(pose).startswith("pinch_")
        }

    written: list[Path] = []
    for model in PALM_TARGETS:
        out = palm_root / f"{model}.yaml"
        dump_palm_calib(out, palm_blocks, source=src, headset=hs, model=model)
        written.append(out)

    dump_mp_curl_calib(mp_out, mp_blocks, source=src, headset=hs, takes=mp_takes)
    written.append(Path(mp_out))
    return written
