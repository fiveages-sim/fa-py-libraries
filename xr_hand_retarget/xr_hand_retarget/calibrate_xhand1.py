"""Capture OpenXR curl-calib poses from xrobotoolkit_sdk.

Core (per side):
  open             手掌伸直，拇指与掌垂直 → 各关节 0 位
  fist             握拳（拇指不测）→ 四指 URDF 上限
  thumb_to_pinky   拇指收到小指侧（对掌）

Index abduction (Pico 追踪好时采；可整组跳过):
  index_together   五指并拢 → index_joint1 = 0
  index_abd        食指向外打开到极限 → 正极限

Fine (集体可跳过):
  pinch_index/middle/ring/pinky  拇指收到该指下方
  robot thumb_joint1 按 URDF 该指 PIP 横向槽反求（随当前拇指屈伸）

PC Service 同时只接受一个 SDK 客户端：采集时不要跑 ./run.sh vr-xrt。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

from xr_hand_retarget.algorithms.curl_xhand1 import (
    CurlFeatures,
    PinchKnot,
    SideCurlCalib,
    dump_calib_file,
    extract_curl_features,
    load_calib_file,
    seed_side_calib,
)
from xr_hand_retarget.algorithms.safety_xhand1 import pose_is_zero

_CORE = (
    (
        "open",
        "手掌伸直：五指自然张开、不要刻意外展，拇指伸直并与手掌垂直",
    ),
    (
        "fist",
        "握拳：四指握紧；拇指随意（本姿态不测拇指）",
    ),
    (
        "thumb_to_pinky",
        "对掌：其余指伸直，拇指收到掌面最靠小指一侧",
    ),
)

_ABD = (
    (
        "index_together",
        "五指并拢：食指贴中指，不要外展（Pico 追踪稳定时采）→ index_joint1=0",
    ),
    (
        "index_abd",
        "其余指并拢，食指向外打开到极限（Pico 追踪好时）→ 正极限",
    ),
)

_PINCH = (
    ("pinch_index", "食指", "拇指与食指捏合：拇指收到食指下方（对掌；robot 侧按 URDF 食指槽）"),
    ("pinch_middle", "中指", "拇指与中指捏合：拇指收到中指下方"),
    ("pinch_ring", "无名指", "拇指与无名指捏合：拇指收到无名指下方"),
    ("pinch_pinky", "小指", "拇指与小指捏合：拇指收到小指下方"),
)

_DEFAULT_OUT = (
    Path(__file__).resolve().parents[2] / "configs" / "calib" / "xrt_default.yaml"
)


def _parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--side", choices=("left", "right", "full"), default="full")
    p.add_argument(
        "--out",
        default=str(_DEFAULT_OUT),
        help="Calib YAML path (default: package configs/calib/xrt_default.yaml)",
    )
    p.add_argument(
        "--seconds",
        type=float,
        default=1.0,
        help="Median window per pose (seconds)",
    )
    p.add_argument("--rate", type=float, default=50.0)
    p.add_argument(
        "--preview-only",
        action="store_true",
        help="Print live features; do not write a file",
    )
    p.add_argument(
        "--skip-pinch",
        action="store_true",
        help="Do not prompt for pinch poses (keep previous pinch block)",
    )
    p.add_argument(
        "--skip-abd",
        action="store_true",
        help="Do not prompt for index together/abd (keep previous side-angle block)",
    )
    return p.parse_args(argv)


def _normalize_hand26(raw) -> np.ndarray:
    arr = np.asarray(raw, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[0] != 26:
        arr = arr.reshape(26, -1)
    return arr


def _as_active(flag) -> bool:
    try:
        v = np.asarray(flag).reshape(-1)
        return bool(v.size) and float(v[0]) > 0.5
    except Exception:
        return False


def _read_frame(xrt, side: str) -> tuple[bool, np.ndarray]:
    get_active = (
        xrt.get_left_hand_is_active if side == "left" else xrt.get_right_hand_is_active
    )
    get_hand = (
        xrt.get_left_hand_tracking_state
        if side == "left"
        else xrt.get_right_hand_tracking_state
    )
    raw = _normalize_hand26(get_hand())
    ok = _as_active(get_active()) and not pose_is_zero(raw)
    return ok, raw


def _fmt_feat(feat: CurlFeatures) -> str:
    mcp = " ".join(
        f"{n[0]}={feat.mcp[n]:.2f}" for n in ("index", "middle", "ring", "pinky")
    )
    return (
        f"mcp[{mcp}] "
        f"t_mcp={feat.thumb_mcp:.2f} t_ip={feat.thumb_ip:.2f} "
        f"t_lat={feat.thumb_lat:.3f} i_lat={feat.index_lat:.3f}"
    )


def _median_feat(samples: list[CurlFeatures]) -> CurlFeatures:
    def med(vals: list[float]) -> float:
        return float(np.median(np.asarray(vals, dtype=np.float64)))

    names = ("index", "middle", "ring", "pinky")
    return CurlFeatures(
        mcp={n: med([s.mcp[n] for s in samples]) for n in names},
        pip={n: med([s.pip[n] for s in samples]) for n in names},
        thumb_mcp=med([s.thumb_mcp for s in samples]),
        thumb_ip=med([s.thumb_ip for s in samples]),
        thumb_lat=med([s.thumb_lat for s in samples]),
        index_lat=med([s.index_lat for s in samples]),
    )


def _apply_pose(calib: SideCurlCalib, name: str, feat: CurlFeatures) -> None:
    if name == "open":
        calib.open_mcp = dict(feat.mcp)
        calib.open_pip = dict(feat.pip)
        calib.open_thumb_mcp = feat.thumb_mcp
        calib.open_thumb_ip = feat.thumb_ip
        calib.open_thumb_lat = feat.thumb_lat
    elif name == "index_together":
        calib.together_index_lat = feat.index_lat
    elif name == "index_abd":
        calib.abd_index_lat = feat.index_lat
    elif name == "fist":
        calib.fist_mcp = dict(feat.mcp)
        calib.fist_pip = dict(feat.pip)
    elif name in ("thumb_to_pinky", "thumb_left"):
        calib.thumb_to_pinky_lat = feat.thumb_lat
    elif name.startswith("pinch_"):
        finger = name[len("pinch_") :]
        if calib.pinch is None:
            calib.pinch = {}
        calib.pinch[finger] = PinchKnot(thumb_lat=feat.thumb_lat)
        calib.pinch_skipped = False
    else:
        raise ValueError(name)


def _wait_enter(prompt: str, extra: str = "") -> str:
    print(prompt, flush=True)
    hint = "  Enter=采集  s+Enter=跳过  q+Enter=放弃"
    if extra:
        hint += f"  {extra}"
    print(hint, flush=True)
    line = sys.stdin.readline()
    if not line:
        return "q"
    return line.strip().lower()


def _sample(
    xrt,
    side: str,
    seconds: float,
    rate: float,
) -> CurlFeatures | None:
    period = 1.0 / max(rate, 1.0)
    deadline = time.time() + max(seconds, 0.2)
    samples: list[CurlFeatures] = []
    rejected = 0
    while time.time() < deadline:
        ok, raw = _read_frame(xrt, side)
        if ok:
            samples.append(extract_curl_features(raw, side))
        else:
            rejected += 1
        time.sleep(period)
    if not samples:
        print(f"  无有效帧 (rejected={rejected})", flush=True)
        return None
    feat = _median_feat(samples)
    print(
        f"  已采 n={len(samples)} reject={rejected}  {_fmt_feat(feat)}",
        flush=True,
    )
    return feat


def _preview_loop(xrt, side: str) -> None:
    print(f"live features [{side}]  Ctrl+C 退出", flush=True)
    try:
        while True:
            ok, raw = _read_frame(xrt, side)
            if not ok:
                print("  (inactive / zero)          ", end="\r", flush=True)
            else:
                feat = extract_curl_features(raw, side)
                print(f"  {_fmt_feat(feat)}     ", end="\r", flush=True)
            time.sleep(0.08)
    except KeyboardInterrupt:
        print("", flush=True)


def _load_or_seed(path: Path) -> dict[str, SideCurlCalib]:
    if path.is_file():
        return load_calib_file(path)
    return {"left": seed_side_calib(), "right": seed_side_calib()}


def _capture_one(
    xrt,
    side: str,
    calib: SideCurlCalib,
    name: str,
    hint: str,
    seconds: float,
    rate: float,
    *,
    skip_keys: tuple[str, ...] = ("s", "skip"),
) -> str:
    """Return ok | skip | quit."""
    while True:
        ok, raw = _read_frame(xrt, side)
        live = _fmt_feat(extract_curl_features(raw, side)) if ok else "inactive/zero"
        print(f"\n[{side}] {name}: {hint}", flush=True)
        print(f"  now: {live}", flush=True)
        cmd = _wait_enter("  保持姿势后确认")
        if cmd in ("q", "quit"):
            return "quit"
        if cmd in skip_keys:
            print(f"  skip {name}", flush=True)
            return "skip"
        feat = _sample(xrt, side, seconds, rate)
        if feat is None:
            print("  采集失败，重试", flush=True)
            continue
        _apply_pose(calib, name, feat)
        return "ok"


def _capture_abd_group(
    xrt,
    side: str,
    calib: SideCurlCalib,
    seconds: float,
    rate: float,
) -> str:
    print(
        f"\n[{side}] 食指侧角（Pico 追踪好时采）：并拢→0，外开到极限→正极限",
        flush=True,
    )
    cmd = _wait_enter("  开始侧角标定？", extra="k+Enter=集体跳过（保留已有 together/abd）")
    if cmd in ("q", "quit"):
        return "quit"
    if cmd in ("k", "skip", "s"):
        print("  skip index abduction group", flush=True)
        return "skip"
    for name, hint in _ABD:
        st = _capture_one(xrt, side, calib, name, hint, seconds, rate)
        if st == "quit":
            return "quit"
    print(
        f"  index_lat together={calib.together_index_lat} abd={calib.abd_index_lat}",
        flush=True,
    )
    return "ok"


def _capture_pinch_group(
    xrt,
    side: str,
    calib: SideCurlCalib,
    seconds: float,
    rate: float,
) -> str:
    print(f"\n[{side}] 精细化：拇指依次收到食/中/无/小指下方（只标定对掌位置，不管高度和贴掌）", flush=True)
    cmd = _wait_enter("  开始捏合标定？", extra="k+Enter=集体跳过（保留已有 pinch）")
    if cmd in ("q", "quit"):
        return "quit"
    if cmd in ("k", "skip", "s"):
        print("  skip pinch group", flush=True)
        return "skip"
    captured = 0
    for name, _label, hint in _PINCH:
        st = _capture_one(xrt, side, calib, name, hint, seconds, rate)
        if st == "quit":
            return "quit"
        if st == "ok":
            captured += 1
    if captured == 0:
        calib.pinch_skipped = True
        print("  pinch 全部跳过", flush=True)
    else:
        calib.pinch_skipped = False
        print(f"  pinch 已写入 {captured}/4 指", flush=True)
    return "ok"


def _capture_side(
    xrt,
    side: str,
    calib: SideCurlCalib,
    seconds: float,
    rate: float,
    *,
    skip_pinch: bool,
    skip_abd: bool = False,
) -> bool:
    print(f"\n======== {side.upper()} ========", flush=True)
    for name, hint in _CORE:
        st = _capture_one(xrt, side, calib, name, hint, seconds, rate)
        if st == "quit":
            return False
    if skip_abd:
        print(f"[{side}] index abduction skipped (--skip-abd)", flush=True)
    else:
        st = _capture_abd_group(xrt, side, calib, seconds, rate)
        if st == "quit":
            return False
    if skip_pinch:
        print(f"[{side}] pinch skipped (--skip-pinch)", flush=True)
        return True
    st = _capture_pinch_group(xrt, side, calib, seconds, rate)
    return st != "quit"


def main(argv=None) -> int:
    args = _parse_args(argv)
    try:
        import xrobotoolkit_sdk as xrt
    except ImportError as exc:
        print(
            "xrobotoolkit_sdk not found. ./init.sh install-xrobotoolkit",
            file=sys.stderr,
        )
        raise SystemExit(1) from exc

    sides = ("left", "right") if args.side == "full" else (args.side,)
    out = Path(args.out)
    print("init xrobotoolkit_sdk...", flush=True)
    xrt.init()
    time.sleep(0.8)
    print("sdk ok  (采集期间不要同时跑 vr-xrt)", flush=True)

    try:
        if args.preview_only:
            for side in sides:
                _preview_loop(xrt, side)
            return 0

        store = _load_or_seed(out)
        for side in sides:
            if not _capture_side(
                xrt,
                side,
                store[side],
                args.seconds,
                args.rate,
                skip_pinch=args.skip_pinch,
                skip_abd=args.skip_abd,
            ):
                print("aborted, file not written", flush=True)
                return 1
        dump_calib_file(out, store)
        print(f"\nwrote {out}", flush=True)
        print(
            "然后: configs/local.yaml 设 method: curl 后 ./run.sh vr-xrt wrist",
            flush=True,
        )
        return 0
    finally:
        try:
            xrt.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
