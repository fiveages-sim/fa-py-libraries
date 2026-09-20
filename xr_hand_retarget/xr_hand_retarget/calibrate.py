"""Calibration entry: XHand1 curl, Wuji official tools, or LinkerHand curl.

LinkerHand O6/L6/O7: one VR session captures PalmTip + MP21 features, then
exports ``calib/o6.yaml``, ``calib/l6.yaml``, ``calib/o7.yaml`` (curl) and
``calib/mp_curl.yaml``. Master session: ``calib/linker_session.yaml``.
Robot q (pinch_q, j2, j3) stays in configs/hands/{hand}.yaml.
"""

from __future__ import annotations

import argparse
import pickle
import sys
import time
from pathlib import Path

import numpy as np

from xr_hand_retarget.config import find_wuji_retargeting_root, load_runtime_config
from xr_hand_retarget.landmarks import openxr26_to_mediapipe21, pose_is_zero


def _parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--config",
        default=None,
        help="Same YAML as teleop (FA_HAND_CONFIG). Backend is read from it.",
    )
    p.add_argument("--side", choices=("left", "right", "full"), default="full")
    p.add_argument(
        "--seconds",
        type=float,
        default=2.0,
        help="Capture window for --dump-pkl (Wuji optical)",
    )
    p.add_argument("--rate", type=float, default=50.0)
    p.add_argument(
        "--dump-pkl",
        default=None,
        help="Wuji: write official-style {left,right}_fingers pkl for tuning_tool / calibrate_offset --play",
    )
    p.add_argument(
        "--preview-only",
        action="store_true",
        help="XHand1: preview curl features; Wuji: print MP21 stats",
    )
    p.add_argument(
        "--out",
        default=None,
        help="Curl calib YAML path (default: retargeting.curl.calib)",
    )
    p.add_argument(
        "--skip-pinch",
        action="store_true",
        help="Curl: skip pinch poses (keep previous pinch block)",
    )
    p.add_argument(
        "--skip-abd",
        action="store_true",
        help="XHand1 curl: skip index together/abd (keep previous side-angle block)",
    )
    p.add_argument(
        "--preview-mp",
        action="store_true",
        help="Wuji: live OpenCV window of OpenXR→MP21 (and official MANO+rotation)",
    )
    p.add_argument(
        "--no-preview-mp",
        action="store_true",
        help="mp_curl: disable the live MediaPipe-21 OpenCV window during capture",
    )
    p.add_argument(
        "--method",
        default=None,
        help="Linker: ignored for o6/l6/o7 (always unified session); Wuji/XHand1 only",
    )
    p.add_argument(
        "--session",
        default=None,
        help="Linker unified session YAML (default: configs/calib/linker_session.yaml)",
    )
    p.add_argument(
        "--export-only",
        action="store_true",
        help="Linker: re-export curl + mp_curl from an existing session file",
    )
    return p.parse_args(argv)


def _wuji_help(cfg) -> int:
    root = find_wuji_retargeting_root(None if cfg.wuji is None else cfg.wuji.root)
    print("Wuji Hand 2 uses the official wuji-retargeting calibration, not XHand1 curl.")
    print()
    print("1) Install official source + models:")
    print("     ./init.sh install-wuji-retargeting")
    print("2) Official parameter order (docs.wuji.tech → Parameter Tuning):")
    print("     segment_scaling → w_pos/w_dir/w_full_hand → pinch_thresholds → lp_alpha")
    if root is None:
        print()
        print("Official checkout not found. After install, re-run this command.")
        print("  ./init.sh install-wuji-retargeting")
        print()
        print("Optical capture (after SDK is up, stop vr-xrt):")
        print(
            "  python -m xr_hand_retarget.calibrate --config <this yaml> "
            "--side full --dump-pkl /tmp/xrt_mp21.pkl"
        )
        return 1
    example = root / "example"
    print()
    print("Official tools (from the cloned repo):")
    print(f"  cd {example}")
    for side in ("left", "right"):
        try:
            yml = cfg.wuji.yaml_for(side) if cfg.wuji is not None else None
        except FileNotFoundError:
            yml = None
        rel = yml if yml is not None else f"<official yaml for {side}>"
        print(f"  python tuning_tool.py --hand {side} --config {rel}")
        print(
            f"  python calibrate_offset.py --hand {side} --config {rel}   # glove live"
        )
    print()
    print("Optical Pico (no glove):")
    print("  - Stop ./run.sh vr-xrt (PC Service allows one SDK client).")
    print("  - Capture MediaPipe-21 pkl:")
    print(
        "      python -m xr_hand_retarget.calibrate --config <this yaml> "
        "--side full --dump-pkl /tmp/xrt_mp21.pkl"
    )
    print("  - Replay in official tools:")
    print("      python tuning_tool.py --play /tmp/xrt_mp21.pkl --hand right --config <official yaml>")
    print(
        "      python calibrate_offset.py --play /tmp/xrt_mp21.pkl --trust-pkl "
        "--hand right --config <official yaml>"
    )
    print()
    print("  calibrate_offset.py link names are gen-1 (finger*_link1). If Hand 2")
    print("  FK fails, use tuning_tool.py and write wrist_offset_cm / thumb_offset_cm")
    print("  into wuji.overlay in this YAML (or edit the official yaml).")
    print()
    print("Docs: https://docs.wuji.tech/docs/en/wuji-retargeting/latest/tuning/")
    return 0


def _capture_pkl(cfg, *, sides: tuple[str, ...], seconds: float, rate: float, out: Path) -> int:
    try:
        import xrobotoolkit_sdk as xrt
    except ImportError as exc:
        print("xrobotoolkit_sdk not found", file=sys.stderr)
        raise SystemExit(1) from exc

    getters = {
        "left": (
            xrt.get_left_hand_is_active,
            xrt.get_left_hand_tracking_state,
        ),
        "right": (
            xrt.get_right_hand_is_active,
            xrt.get_right_hand_tracking_state,
        ),
    }
    print("init xrobotoolkit_sdk (stop vr-xrt first)...", flush=True)
    xrt.init()
    time.sleep(0.8)
    print(
        f"Recording {seconds:.1f}s at {rate:.0f} Hz for sides={sides}. "
        "Hold a relaxed open palm.",
        flush=True,
    )
    period = 1.0 / max(rate, 1.0)
    n_frames = max(1, int(seconds * rate))
    frames: list[dict] = []
    try:
        for i in range(n_frames):
            t0 = time.time()
            rec: dict = {}
            for side in sides:
                _active, get_hand = getters[side]
                raw = np.asarray(get_hand(), dtype=np.float64)
                if raw.ndim != 2 or raw.shape[0] != 26:
                    raw = raw.reshape(26, -1)
                if pose_is_zero(raw):
                    rec[f"{side}_fingers"] = np.zeros((21, 3), dtype=np.float32)
                else:
                    rec[f"{side}_fingers"] = openxr26_to_mediapipe21(raw).astype(
                        np.float32
                    )
            frames.append(rec)
            if (i + 1) % max(1, n_frames // 5) == 0:
                print(f"  {i + 1}/{n_frames}", flush=True)
            dt = time.time() - t0
            time.sleep(max(0.0, period - dt))
    finally:
        try:
            xrt.close()
        except Exception:
            pass

    out = out.expanduser().resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("wb") as f:
        pickle.dump(frames, f)
    print(f"Wrote {len(frames)} frames → {out}")
    print("Replay with official example/tuning_tool.py --play …")
    return 0


_CORE = (
    ("open", "手掌伸直：五指自然张开、不要刻意外展，拇指伸直并与手掌垂直"),
    ("fist", "握拳：四指握紧；拇指随意（本姿态不测拇指）"),
    ("thumb_to_pinky", "对掌：其余指伸直，拇指收到掌面最靠小指一侧"),
)
_O6_CORE = (
    ("open", "手掌伸直：五指自然张开、不要刻意外展，拇指伸直并与手掌垂直"),
    ("fist", "握拳：四指握紧；拇指保持放松（本姿态不测拇指）"),
    ("thumb_flex", "四指展开，拇指充分弯曲（收缩 thumb_joint1），不要横移"),
    ("thumb_to_pinky", "横移：其余指伸直，拇指扫到能碰上中指的位置（无名指/小指碰不上）"),
)
_MP_CURL_CORE = (
    ("open", "开掌：四指自然张开；拇指垂直于掌面"),
    (
        "fist",
        "握拳：四指握紧，大拇指收在掌中（明显入掌；与 PalmTip curl 同语义）",
    ),
    (
        "thumb_along",
        "四指伸直，拇指伸直并与四指同向、贴掌外侧（侧掌 j2≈0）",
    ),
)
_PALM_CORE = (
    ("open", "开掌：四指自然张开；拇指垂直"),
    ("together", "合掌：四指并拢，拇指垂直于四指"),
    ("fist", "握拳：四指握紧，大拇指收在掌中（掌侧由 thumb_along 标定）"),
    ("good", "点赞：四指握紧程度与 fist 相同；拇指伸出"),
    ("thumb_along", "四指伸直，拇指伸直并与四指同向、贴掌外侧"),
)
_O7_PALM_CORE = (
    ("open", "开掌：四指自然张开；拇指垂直"),
    ("together", "合掌：四指并拢，拇指垂直于四指"),
    ("fist", "握拳：四指握紧，大拇指收在掌中（掌侧由 thumb_along 标定）"),
    ("good", "点赞：四指握紧程度与 fist 相同；拇指伸出"),
    ("thumb_along", "四指伸直，拇指伸直并与四指同向、贴掌外侧"),
)
_PALM_PINCH = (
    ("pinch_index", "食指捏合：拇指与食指指腹对上即可；其余指不管"),
    ("pinch_middle", "中指捏合：拇指与中指指腹对上；其余指不管"),
    ("pinch_ring", "无名指捏合：拇指与无名指指腹对上；其余指不管"),
    ("pinch_pinky", "小指捏合：拇指与小指指腹对上；其余指不管"),
)
_PINCH = (
    ("pinch_index", "拇指与食指捏合：拇指收到食指下方"),
    ("pinch_middle", "拇指与中指捏合：拇指收到中指下方"),
    ("pinch_ring", "拇指与无名指捏合：拇指收到无名指下方"),
    ("pinch_pinky", "拇指与小指捏合：拇指收到小指下方"),
)


def _fmt_feat2(feat) -> str:
    mcp = " ".join(f"{n[0]}={feat.mcp[n]:.2f}" for n in ("index", "middle", "ring", "pinky"))
    return (
        f"mcp[{mcp}] t_mcp={feat.thumb_mcp:.2f} t_ip={feat.thumb_ip:.2f} "
        f"t_lat={feat.thumb_lat:.3f}"
    )


def _stdin_line_ready() -> bool:
    import select

    try:
        return bool(select.select([sys.stdin], [], [], 0)[0])
    except Exception:
        return False


def _wait_with_mp_preview(
    xrt,
    side: str,
    prompt: str,
    *,
    extra: str = "",
    win: str = "mp_curl MP21",
    pose_name: str = "",
    enabled: bool = True,
) -> str:
    """Like ``_wait_enter``, but keep refreshing a MediaPipe-21 stick-figure window.

    Terminal: Enter / s / q / a / r (same as before).
    OpenCV window: Space/Enter=confirm, s=skip, q=quit, a=again, r=redo.
    """
    from xr_hand_retarget.calibrate_xhand1 import _read_frame, _wait_enter
    from xr_hand_retarget.landmarks import openxr26_to_mediapipe21

    if not enabled:
        return _wait_enter(prompt, extra=extra)

    try:
        from xr_hand_retarget.preview import close_mp21_windows, show_mp21_window
    except Exception:
        return _wait_enter(prompt, extra=extra)

    print(prompt, flush=True)
    hint = "  Enter=确认  s=跳过  q=放弃"
    if extra:
        hint += f"  {extra}"
    hint += "  | 视窗: Space/Enter 同确认"
    print(hint, flush=True)

    title = f"{side} {pose_name}".strip() or f"{side} MP21"
    while True:
        ok, raw = _read_frame(xrt, side)
        if ok:
            mp21 = openxr26_to_mediapipe21(raw)
            try:
                feat = None
                from xr_hand_retarget.algorithms.mp_curl_o6 import (
                    extract_curl_features_mp21,
                )

                feat = extract_curl_features_mp21(mp21, side)
                hud = _fmt_feat2(feat)
            except Exception:
                hud = "TRACK"
        else:
            mp21 = np.zeros((21, 3), dtype=np.float64)
            hud = "inactive/zero"
        key = show_mp21_window(win, mp21, title=title, hud=hud)
        if key in (13, 10, ord(" ")):
            return ""
        if key == ord("s"):
            return "s"
        if key in (ord("q"), 27):
            return "q"
        if key == ord("a"):
            return "a"
        if key == ord("r"):
            return "r"
        if _stdin_line_ready():
            line = sys.stdin.readline()
            if not line:
                return "q"
            return line.strip().lower()
        time.sleep(0.02)



def _apply_pose2(calib, name: str, feat) -> None:
    from xr_hand_retarget.algorithms.curl_xhand1 import PinchKnot

    if name == "open":
        calib.open_mcp = dict(feat.mcp)
        calib.open_pip = dict(feat.pip)
        calib.open_dip = dict(feat.dip)
        calib.open_thumb_mcp = feat.thumb_mcp
        calib.open_thumb_ip = feat.thumb_ip
        calib.open_thumb_lat = feat.thumb_lat
    elif name == "fist":
        calib.fist_mcp = dict(feat.mcp)
        calib.fist_pip = dict(feat.pip)
        calib.fist_dip = dict(feat.dip)
        # mp_curl: fist thumb-in-palm is the curl high knot (replaces thumb_flex pose).
        calib.thumb_flex_mcp = float(feat.thumb_mcp)
        calib.thumb_flex_ip = float(feat.thumb_ip)
    elif name == "thumb_flex":
        calib.thumb_flex_mcp = feat.thumb_mcp
        calib.thumb_flex_ip = feat.thumb_ip
    elif name in ("thumb_along", "along"):
        calib.along_thumb_lat = float(feat.thumb_lat)
    elif name in ("thumb_to_pinky", "thumb_left"):
        calib.thumb_to_pinky_lat = feat.thumb_lat
    elif name.startswith("pinch_"):
        finger = name[len("pinch_") :]
        if calib.pinch is None:
            calib.pinch = {}
        finger_mcp = feat.mcp.get(finger) if hasattr(feat, "mcp") else None
        finger_pip = feat.pip.get(finger) if hasattr(feat, "pip") else None
        finger_dip = (
            feat.dip.get(finger) if hasattr(feat, "dip") and feat.dip else None
        )
        calib.pinch[finger] = PinchKnot(
            thumb_lat=feat.thumb_lat,
            thumb_mcp=float(feat.thumb_mcp),
            thumb_ip=float(feat.thumb_ip),
            finger_mcp=None if finger_mcp is None else float(finger_mcp),
            finger_pip=None if finger_pip is None else float(finger_pip),
            finger_dip=None if finger_dip is None else float(finger_dip),
        )
        calib.pinch_skipped = False
    else:
        raise ValueError(name)


def _median_feat2(samples):
    import numpy as np
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


def _run_curl_capture(
    _cfg,
    args,
    *,
    dump_calib_file,
    default_out: Path,
    calib_path: Path | None,
    core_poses: tuple[tuple[str, str], ...],
    run_hint: str,
    extract_features=None,
) -> int:
    import time

    from xr_hand_retarget.algorithms.curl_hand2 import (
        extract_curl_features2,
        load_calib_file,
        seed_side_calib,
    )
    from xr_hand_retarget.calibrate_xhand1 import _read_frame, _wait_enter

    extract = extract_features or extract_curl_features2

    try:
        import xrobotoolkit_sdk as xrt
    except ImportError as exc:
        print("xrobotoolkit_sdk not found", file=sys.stderr)
        raise SystemExit(1) from exc

    out = Path(args.out) if args.out else (calib_path or default_out)
    sides = ("left", "right") if args.side == "full" else (args.side,)
    print("init xrobotoolkit_sdk...", flush=True)
    xrt.init()
    time.sleep(0.8)
    print("sdk ok  (采集期间不要同时跑 vr-xrt)", flush=True)

    def sample(side: str):
        period = 1.0 / max(args.rate, 1.0)
        deadline = time.time() + max(args.seconds, 0.2)
        samples = []
        rejected = 0
        while time.time() < deadline:
            ok, raw = _read_frame(xrt, side)
            if ok:
                samples.append(extract(raw, side))
            else:
                rejected += 1
            time.sleep(period)
        if not samples:
            print(f"  无有效帧 (rejected={rejected})", flush=True)
            return None
        feat = _median_feat2(samples)
        print(f"  已采 n={len(samples)} reject={rejected}  {_fmt_feat2(feat)}", flush=True)
        return feat

    def capture_one(side, calib, name, hint):
        while True:
            ok, raw = _read_frame(xrt, side)
            live = (
                _fmt_feat2(extract(raw, side)) if ok else "inactive/zero"
            )
            print(f"\n[{side}] {name}: {hint}", flush=True)
            print(f"  now: {live}", flush=True)
            cmd = _wait_enter("  保持姿势后确认")
            if cmd in ("q", "quit"):
                return "quit"
            if cmd in ("s", "skip"):
                print(f"  skip {name}", flush=True)
                return "skip"
            feat = sample(side)
            if feat is None:
                print("  采集失败，重试", flush=True)
                continue
            _apply_pose2(calib, name, feat)
            return "ok"

    try:
        if args.preview_only:
            for side in sides:
                print(f"live features [{side}]  Ctrl+C 退出", flush=True)
                try:
                    while True:
                        ok, raw = _read_frame(xrt, side)
                        if not ok:
                            print("  (inactive / zero)          ", end="\r", flush=True)
                        else:
                            print(
                                f"  {_fmt_feat2(extract(raw, side))}     ",
                                end="\r",
                                flush=True,
                            )
                        time.sleep(0.08)
                except KeyboardInterrupt:
                    print("", flush=True)
            return 0

        store = (
            load_calib_file(out)
            if out.is_file()
            else {"left": seed_side_calib(), "right": seed_side_calib()}
        )
        for side in sides:
            print(f"\n======== {side.upper()} ========", flush=True)
            for name, hint in core_poses:
                st = capture_one(side, store[side], name, hint)
                if st == "quit":
                    print("aborted, file not written", flush=True)
                    return 1
            if args.skip_pinch:
                print(f"[{side}] pinch skipped (--skip-pinch)", flush=True)
                continue
            print(
                f"\n[{side}] 精细化：拇指依次收到食/中/无/小指下方",
                flush=True,
            )
            cmd = _wait_enter(
                "  开始捏合标定？", extra="k+Enter=集体跳过（保留已有 pinch）"
            )
            if cmd in ("q", "quit"):
                print("aborted, file not written", flush=True)
                return 1
            if cmd in ("k", "skip", "s"):
                print("  skip pinch group", flush=True)
                continue
            captured = 0
            for name, hint in _PINCH:
                st = capture_one(side, store[side], name, hint)
                if st == "quit":
                    print("aborted, file not written", flush=True)
                    return 1
                if st == "ok":
                    captured += 1
            store[side].pinch_skipped = captured == 0
            print(
                f"  pinch {'全部跳过' if captured == 0 else f'已写入 {captured}/4 指'}",
                flush=True,
            )
        dump_calib_file(out, store)
        print(f"\nwrote {out}", flush=True)
        print(f"然后: {run_hint}", flush=True)
        return 0
    finally:
        try:
            xrt.close()
        except Exception:
            pass


def _run_hand2_curl_capture(cfg, args) -> int:
    from xr_hand_retarget.algorithms.curl_hand2 import dump_calib_file

    return _run_curl_capture(
        cfg,
        args,
        dump_calib_file=dump_calib_file,
        default_out=Path("configs/calib/wuji_hand2.yaml"),
        calib_path=cfg.wuji_curl.calib_path if cfg.wuji_curl else None,
        core_poses=_CORE,
        run_hint=(
            "FA_HAND_CONFIG=xr_hand_retarget/configs/wuji_hand2.yaml "
            "./run.sh vr-xrt wrist"
        ),
    )


def _export_linker_session(args) -> int:
    from xr_hand_retarget.linker_session import (
        DEFAULT_SESSION,
        export_from_session,
        load_session,
    )

    path = Path(args.session).expanduser() if args.session else DEFAULT_SESSION
    if not path.is_file():
        print(f"session not found: {path}", file=sys.stderr)
        return 1
    session = load_session(path)
    paths = export_from_session(session)
    print("\nExported from session:", flush=True)
    for p in paths:
        print(f"  {p}", flush=True)
    print(
        "\nRun: FA_HAND=o6|l6|o7 ./run.sh vr-xrt wrist\n"
        "  mp_curl: FA_HAND_RETARGETING=mp_curl ./run.sh vr-xrt wrist",
        flush=True,
    )
    return 0


def _run_linker_unified_capture(cfg, args) -> int:
    """One VR pass → linker_session.yaml + curl (o6/l6/o7) + mp_curl."""
    import time

    from xr_hand_retarget.algorithms.mp_curl_o6 import (
        extract_curl_features_mp21,
        feat2_to_dict,
        mp21_to_list,
    )
    from xr_hand_retarget.algorithms.palm_tip import (
        capture_sides_for_calib,
        evaluate_pose_gate,
        extract_palm_tip,
        format_feat,
        seed_palm_calib,
    )
    from xr_hand_retarget.calibrate_xhand1 import _read_frame, _wait_enter
    from xr_hand_retarget.linker_session import (
        DEFAULT_SESSION,
        dump_session,
        export_from_session,
        load_session,
        median_feat2,
        median_palm_feat,
        palm_feat_to_dict,
    )
    from xr_hand_retarget.poses import poses_for_layout

    try:
        import xrobotoolkit_sdk as xrt
    except ImportError as exc:
        print("xrobotoolkit_sdk not found", file=sys.stderr)
        raise SystemExit(1) from exc

    preview_on = not bool(getattr(args, "no_preview_mp", False))
    win = "linker unified MP21"
    show_mp21_window = None
    close_mp21_windows = None
    if preview_on:
        try:
            import cv2  # noqa: F401

            from xr_hand_retarget.preview import close_mp21_windows, show_mp21_window
        except Exception as exc:
            print(f"OpenCV preview unavailable ({exc}); text-only capture", flush=True)
            preview_on = False
            show_mp21_window = None
            close_mp21_windows = None

    session_path = (
        Path(args.session).expanduser()
        if args.session
        else DEFAULT_SESSION
    )
    if args.session and not session_path.is_absolute():
        pkg_out = Path(__file__).resolve().parents[1] / session_path
        if str(session_path).startswith("configs/") or not session_path.parent.exists():
            session_path = pkg_out

    source_side = str(cfg.o6_curl.calib_source_side) if cfg.o6_curl else "both"
    az_sign = float(cfg.o6_curl.calib_mirror_az) if cfg.o6_curl else -1.0
    sides = capture_sides_for_calib(args.side, source_side)
    model = str(cfg.backend or "o6").strip().lower() or "o6"

    print("init xrobotoolkit_sdk...", flush=True)
    xrt.init()
    time.sleep(0.8)
    print("sdk ok  (采集期间不要同时跑 vr-xrt)", flush=True)
    print(
        f"LinkerHand 统一标定 ({model.upper()} 触发，O6/L6/O7 共用)\n"
        f"  会话 → {session_path}\n"
        "  导出 → calib/o6.yaml, l6.yaml, o7.yaml (curl) + calib/mp_curl.yaml\n"
        "  姿态: open / together / fist / good / thumb_along / pinch×4\n"
        "  每组同时采 PalmTip(VR26) + MP21；gate 以 PalmTip 为准\n"
        f"  preview={'ON ' + win if preview_on else 'off (--no-preview-mp)'}",
        flush=True,
    )
    if len(sides) == 1:
        other = "right" if sides[0] == "left" else "left"
        print(
            f"source_side={source_side}: 只采 {sides[0]}，导出时镜像到 {other}",
            flush=True,
        )

    def pump_preview(side: str, pose_name: str = "", hud: str = "") -> None:
        if not preview_on or show_mp21_window is None:
            return
        ok, raw = _read_frame(xrt, side)
        if ok:
            mp21 = openxr26_to_mediapipe21(raw)
            if not hud:
                try:
                    mp_feat = extract_curl_features_mp21(mp21, side)
                    palm = extract_palm_tip(raw, side)
                    hud = f"{format_feat(palm)} | {_fmt_feat2(mp_feat)}"
                except Exception:
                    hud = "TRACK"
        else:
            mp21 = np.zeros((21, 3), dtype=np.float64)
            hud = hud or "inactive/zero"
        title = f"{side} {pose_name}".strip() or f"{side} unified"
        show_mp21_window(win, mp21, title=title, hud=hud[:220])

    def sample(side: str, name: str, calib, *, force: bool):
        period = 1.0 / max(args.rate, 1.0)
        deadline = time.time() + max(args.seconds, 0.2)
        palm_samples = []
        mp_samples = []
        mp21s = []
        rejected = 0
        gated = 0
        while time.time() < deadline:
            ok, raw = _read_frame(xrt, side)
            if ok:
                palm_feat = extract_palm_tip(raw, side)
                mp21 = openxr26_to_mediapipe21(raw)
                mp_feat = extract_curl_features_mp21(mp21, side)
                gate_ok, _why = evaluate_pose_gate(name, palm_feat, calib)
                if force or gate_ok:
                    palm_samples.append(palm_feat)
                    mp_samples.append(mp_feat)
                    mp21s.append(mp21)
                else:
                    gated += 1
                if preview_on and show_mp21_window is not None:
                    show_mp21_window(
                        win,
                        mp21,
                        title=f"{side} {name} CAPTURE",
                        hud=f"{format_feat(palm_feat)} | {_fmt_feat2(mp_feat)}"[:220],
                    )
            else:
                rejected += 1
                pump_preview(side, name, hud="reject")
            time.sleep(period)
        if not palm_samples:
            print(
                f"  无有效帧 (rejected={rejected} gated={gated})",
                flush=True,
            )
            return None
        if not force and gated and len(palm_samples) < 8:
            print(
                f"  合格帧太少 n={len(palm_samples)} gated={gated}，换正对再采",
                flush=True,
            )
            return None
        palm_med = median_palm_feat(palm_samples)
        mp_med = median_feat2(mp_samples)
        mp21_med = np.median(np.stack(mp21s, axis=0), axis=0)
        print(
            f"  已采 n={len(palm_samples)} reject={rejected} gated={gated}\n"
            f"    palm: {format_feat(palm_med)}\n"
            f"    mp:   {_fmt_feat2(mp_med)}",
            flush=True,
        )
        return {
            "palm": palm_feat_to_dict(palm_med),
            "mp": feat2_to_dict(mp_med),
            "mp21": mp21_to_list(mp21_med),
        }

    def capture_one(side, calib, name, hint, takes_side: dict):
        pose_takes: list[dict] = []
        while True:
            ok, raw = _read_frame(xrt, side)
            if ok:
                palm_feat = extract_palm_tip(raw, side)
                mp_feat = extract_curl_features_mp21(openxr26_to_mediapipe21(raw), side)
                live = (
                    f"palm: {format_feat(palm_feat)} | mp: {_fmt_feat2(mp_feat)}"
                )
                gate_ok, why = evaluate_pose_gate(name, palm_feat, calib)
                gate = "OK" if gate_ok else "FAIL " + "; ".join(why[:3])
            else:
                live, gate_ok, gate = "inactive/zero", False, "FAIL inactive"
            n = len(pose_takes)
            print(f"\n[{side}] {name}: {hint}", flush=True)
            print(f"  now: {live}", flush=True)
            print(f"  gate: {gate}  (已采组数={n})", flush=True)
            cmd = _wait_with_mp_preview(
                xrt,
                side,
                "  保持姿势后确认（gate=OK 再 Enter）",
                extra="f+Enter=强制  a+Enter=再采一组  r+Enter=清空  s=跳过  q=放弃",
                win=win,
                pose_name=name,
                enabled=preview_on,
            )
            if cmd in ("q", "quit"):
                return "quit"
            if cmd in ("s", "skip"):
                print(f"  skip {name}", flush=True)
                return "skip"
            if cmd in ("r", "redo") and not pose_takes:
                continue
            force = cmd in ("f", "force")
            if not force and not gate_ok:
                print("  特征不合格，未写入。摆对后再 Enter，或 f 强制。", flush=True)
                continue
            take = sample(side, name, calib, force=force)
            if take is None:
                print("  采集失败，重试", flush=True)
                continue
            from xr_hand_retarget.linker_session import palm_feat_from_dict

            palm_med = palm_feat_from_dict(take["palm"])
            ok_med, why_med = evaluate_pose_gate(name, palm_med, calib)
            if not force and not ok_med:
                print(
                    "  窗口中位数仍不合格：" + "; ".join(why_med[:4]),
                    flush=True,
                )
                continue
            pose_takes.append(take)
            print(f"  本组已写入；合计 {len(pose_takes)} 组", flush=True)
            cmd2 = _wait_with_mp_preview(
                xrt,
                side,
                "  继续？",
                extra="Enter=下一姿态  a+Enter=再采一组  r+Enter=清空本组",
                win=win,
                pose_name=name,
                enabled=preview_on,
            )
            if cmd2 in ("q", "quit"):
                return "quit"
            if cmd2 in ("a", "again", "+"):
                continue
            if cmd2 in ("r", "redo"):
                pose_takes.clear()
                print("  已清空本组，重新采集", flush=True)
                continue
            if pose_takes:
                takes_side[name] = pose_takes
            return "ok"

    try:
        if args.preview_only:
            for side in sides:
                print(f"live unified [{side}]  Ctrl+C 退出", flush=True)
                try:
                    while True:
                        ok, raw = _read_frame(xrt, side)
                        if not ok:
                            print("  (inactive / zero)          ", end="\r", flush=True)
                        else:
                            palm = extract_palm_tip(raw, side)
                            mp = extract_curl_features_mp21(
                                openxr26_to_mediapipe21(raw), side
                            )
                            print(
                                f"  {format_feat(palm)} | {_fmt_feat2(mp)}     ",
                                end="\r",
                                flush=True,
                            )
                        time.sleep(0.08)
                except KeyboardInterrupt:
                    print("", flush=True)
            return 0

        all_takes: dict[str, dict[str, list]] = {"left": {}, "right": {}}
        if session_path.is_file():
            prev = load_session(session_path)
            for sname, sblock in (prev.get("sides") or {}).items():
                if sname not in sides and isinstance(sblock, dict):
                    old = sblock.get("takes")
                    if old:
                        all_takes.setdefault(sname, {}).update(old)

        layout = "o6"
        for side in sides:
            calib = seed_palm_calib()
            print(f"\n======== {side.upper()} ========", flush=True)
            takes_side: dict[str, list] = {}
            for name, hint in poses_for_layout(layout, group="core"):
                st = capture_one(side, calib, name, hint, takes_side)
                if st == "quit":
                    print("aborted, session not written", flush=True)
                    return 1
            if args.skip_pinch:
                print(f"[{side}] pinch skipped (--skip-pinch)", flush=True)
            else:
                print(
                    f"\n[{side}] 捏合标定：食/中/无名/小指",
                    flush=True,
                )
                cmd = _wait_with_mp_preview(
                    xrt,
                    side,
                    "  开始捏合标定？",
                    extra="k+Enter=集体跳过（保留已有 pinch）",
                    win=win,
                    pose_name="pinch?",
                    enabled=preview_on,
                )
                if cmd in ("q", "quit"):
                    print("aborted, session not written", flush=True)
                    return 1
                if cmd not in ("k", "skip", "s"):
                    captured = 0
                    for name, hint in poses_for_layout(layout, group="pinch"):
                        st = capture_one(side, calib, name, hint, takes_side)
                        if st == "quit":
                            print("aborted, session not written", flush=True)
                            return 1
                        if st == "ok":
                            captured += 1
                    print(
                        f"  pinch "
                        f"{'全部跳过' if captured == 0 else f'已写入 {captured}/4 指'}",
                        flush=True,
                    )
            all_takes[side] = takes_side

        dump_session(session_path, all_takes)
        print(f"\nWrote session {session_path}", flush=True)

        mirror_from = mirror_to = None
        if len(sides) == 1:
            mirror_from = sides[0]
            mirror_to = "right" if mirror_from == "left" else "left"
            print(
                f"export: mirror {mirror_from} → {mirror_to} "
                f"(palm az×{az_sign}, mp lat×-1)",
                flush=True,
            )

        paths = export_from_session(
            load_session(session_path),
            mirror_from=mirror_from,
            mirror_to=mirror_to,
            az_sign=az_sign,
            lat_sign=-1.0,
        )
        print("\nExported:", flush=True)
        for p in paths:
            print(f"  {p}", flush=True)
        print(
            f"\nRun curl:  FA_HAND={model} ./run.sh vr-xrt wrist\n"
            f"Run mp_curl: FA_HAND={model} FA_HAND_RETARGETING=mp_curl ./run.sh vr-xrt wrist",
            flush=True,
        )
        return 0
    finally:
        if close_mp21_windows is not None:
            close_mp21_windows(win)
        try:
            xrt.close()
        except Exception:
            pass


def _run_o6_palm_capture(cfg, args) -> int:
    """O6: same poses as Wuji/XHand1 curl (core + optional pinch×4)."""
    import time

    from xr_hand_retarget.algorithms.palm_tip import (
        apply_pose,
        capture_sides_for_calib,
        dump_palm_calib,
        evaluate_pose_gate,
        extract_palm_tip,
        format_feat,
        load_palm_calib,
        mirror_palm_calib,
        seed_palm_calib,
    )
    from xr_hand_retarget.poses import poses_for_layout
    from xr_hand_retarget.calibrate_xhand1 import _read_frame, _wait_enter
    from xr_hand_retarget.linker_session import median_palm_feat

    try:
        import xrobotoolkit_sdk as xrt
    except ImportError as exc:
        print("xrobotoolkit_sdk not found", file=sys.stderr)
        raise SystemExit(1) from exc

    out = Path(args.out) if args.out else (
        cfg.o6_curl.calib_path if cfg.o6_curl and cfg.o6_curl.calib_path
        else Path("configs/users/pico.yaml")
    )
    source_side = (
        str(cfg.o6_curl.calib_source_side) if cfg.o6_curl else "both"
    )
    az_sign = float(cfg.o6_curl.calib_mirror_az) if cfg.o6_curl else 1.0
    sides = capture_sides_for_calib(args.side, source_side)
    print("init xrobotoolkit_sdk...", flush=True)
    xrt.init()
    time.sleep(0.8)
    model = str(cfg.backend or "o6").strip().lower() or "o6"
    print("sdk ok  (采集期间不要同时跑 vr-xrt)", flush=True)
    print(
        f"{model.upper()}: 写入人手特征 curl + pack(h,r) → {out}\n"
        f"  （不是 OpenXR 26，也不是机器人 q；工作空间在 configs/hands/{model}.yaml）\n"
        "  姿态: open / together(合掌) / fist(掌中握拳) / good(同四指·拇指伸出) / along / pinch×4\n"
        "  四指闭合端点=max(fist,good)；pinch 只盯捏合指。live 看 pack r/h：open/together 的 r 应明显大于 fist\n"
        "  按手分文件：o6→calib/o6.yaml  l6→calib/l6.yaml  o7→users/pico.yaml，不要混会话",
        flush=True,
    )
    if len(sides) == 1:
        other = "right" if sides[0] == "left" else "left"
        print(
            f"source_side={source_side}: 只采 {sides[0]}，写文件时镜像到 {other}",
            flush=True,
        )

    def sample(side: str, name: str, calib, *, force: bool):
        period = 1.0 / max(args.rate, 1.0)
        deadline = time.time() + max(args.seconds, 0.2)
        samples = []
        rejected = 0
        gated = 0
        while time.time() < deadline:
            ok, raw = _read_frame(xrt, side)
            if ok:
                feat = extract_palm_tip(raw, side)
                gate_ok, _why = evaluate_pose_gate(name, feat, calib)
                if force or gate_ok:
                    samples.append(feat)
                else:
                    gated += 1
            else:
                rejected += 1
            time.sleep(period)
        if not samples:
            print(
                f"  无有效帧 (rejected={rejected} gated={gated})",
                flush=True,
            )
            return None
        if not force and gated and len(samples) < 8:
            print(
                f"  合格帧太少 n={len(samples)} gated={gated}，换正对再采",
                flush=True,
            )
            return None
        feat = median_palm_feat(samples)
        print(
            f"  已采 n={len(samples)} reject={rejected} gated={gated}  "
            f"{format_feat(feat)}",
            flush=True,
        )
        return feat

    def capture_one(side, calib, name, hint):
        while True:
            ok, raw = _read_frame(xrt, side)
            if ok:
                live_feat = extract_palm_tip(raw, side)
                live = format_feat(live_feat)
                gate_ok, why = evaluate_pose_gate(name, live_feat, calib)
                gate = "OK" if gate_ok else "FAIL " + "; ".join(why[:3])
            else:
                live, gate_ok, gate = "inactive/zero", False, "FAIL inactive"
            print(f"\n[{side}] {name}: {hint}", flush=True)
            print(f"  now: {live}", flush=True)
            print(f"  gate: {gate}", flush=True)
            cmd = _wait_enter(
                "  保持姿势后确认（gate=OK 再 Enter）",
                extra="f+Enter=强制写入",
            )
            if cmd in ("q", "quit"):
                return "quit"
            if cmd in ("s", "skip"):
                print(f"  skip {name}", flush=True)
                return "skip"
            force = cmd in ("f", "force")
            if not force and not gate_ok:
                print("  特征不合格，未写入。摆对后再 Enter，或 f 强制。", flush=True)
                continue
            feat = sample(side, name, calib, force=force)
            if feat is None:
                print("  采集失败，重试", flush=True)
                continue
            ok_med, why_med = evaluate_pose_gate(name, feat, calib)
            if not force and not ok_med:
                print(
                    "  窗口中位数仍不合格：" + "; ".join(why_med[:4]),
                    flush=True,
                )
                continue
            apply_pose(calib, name, feat)
            return "ok"

    try:
        if args.preview_only:
            for side in sides:
                print(f"live palm-tip [{side}]  Ctrl+C 退出", flush=True)
                try:
                    while True:
                        ok, raw = _read_frame(xrt, side)
                        if not ok:
                            print("  (inactive / zero)          ", end="\r", flush=True)
                        else:
                            print(
                                f"  {format_feat(extract_palm_tip(raw, side))}     ",
                                end="\r",
                                flush=True,
                            )
                        time.sleep(0.08)
                except KeyboardInterrupt:
                    print("", flush=True)
            return 0

        store = (
            load_palm_calib(out)
            if out.is_file()
            else {"left": seed_palm_calib(), "right": seed_palm_calib()}
        )
        for side in sides:
            print(f"\n======== {side.upper()} ========", flush=True)
            core = poses_for_layout(model, group="core")
            pinch_poses = poses_for_layout(model, group="pinch")
            for name, hint in core:
                st = capture_one(side, store[side], name, hint)
                if st == "quit":
                    print("aborted, file not written", flush=True)
                    return 1
            if args.skip_pinch:
                print(f"[{side}] pinch skipped (--skip-pinch)", flush=True)
                continue
            print(
                f"\n[{side}] 捏合标定：食/中/无名/小指"
                f"（人手意图；机器人槽位在 configs/{model}.yaml）",
                flush=True,
            )
            cmd = _wait_enter(
                "  开始捏合标定？", extra="k+Enter=集体跳过（保留已有 pinch）"
            )
            if cmd in ("q", "quit"):
                print("aborted, file not written", flush=True)
                return 1
            if cmd in ("k", "skip", "s"):
                print("  skip pinch group", flush=True)
                continue
            captured = 0
            for name, hint in pinch_poses:
                st = capture_one(side, store[side], name, hint)
                if st == "quit":
                    print("aborted, file not written", flush=True)
                    return 1
                if st == "ok":
                    captured += 1
            store[side].pinch_skipped = captured == 0
            print(
                f"  pinch {'全部跳过' if captured == 0 else f'已写入 {captured}/4 指'}",
                flush=True,
            )
        if len(sides) == 1:
            src = sides[0]
            other = "right" if src == "left" else "left"
            store[other] = mirror_palm_calib(
                store[src], from_side=src, az_sign=az_sign
            )
            print(
                f"\nmirrored {src} → {other} "
                f"(curl copy, az {'negate' if az_sign < 0 else 'copy'})",
                flush=True,
            )
        dump_palm_calib(out, store, model=model)
        print(f"\nWrote {out}", flush=True)
        print(f"FA_HAND={cfg.backend} ./run.sh vr-xrt wrist", flush=True)
        return 0
    finally:
        try:
            xrt.close()
        except Exception:
            pass


def _run_o6_curl_capture(cfg, args) -> int:
    if cfg.backend in ("o6", "l6", "o7"):
        return _run_linker_unified_capture(cfg, args)

    from xr_hand_retarget.algorithms.curl_o6 import dump_calib_file

    model = cfg.backend if cfg.backend in ("l6", "o7") else "o6"
    return _run_curl_capture(
        cfg,
        args,
        dump_calib_file=dump_calib_file,
        default_out=Path(f"configs/calib/{model}.yaml"),
        calib_path=cfg.o6_curl.calib_path if cfg.o6_curl else None,
        core_poses=_O6_CORE,
        run_hint=(
            f"FA_HAND={model} ./run.sh vr-xrt wrist"
        ),
    )


def _run_mp_curl_capture(cfg, args) -> int:
    """Shared MP21 curl calib for o6/l6/o7; VR curl poses; multi-take per gesture."""
    import time

    from xr_hand_retarget.algorithms.curl_hand2 import load_calib_file, seed_side_calib
    from xr_hand_retarget.algorithms.mp_curl_o6 import (
        _DEFAULT_CALIB,
        dump_calib_file,
        extract_curl_features_mp21,
        feat2_from_dict,
        feat2_to_dict,
        mp21_to_list,
    )
    from xr_hand_retarget.calibrate_xhand1 import _read_frame
    from xr_hand_retarget.landmarks import openxr26_to_mediapipe21

    try:
        import xrobotoolkit_sdk as xrt
    except ImportError as exc:
        print("xrobotoolkit_sdk not found", file=sys.stderr)
        raise SystemExit(1) from exc

    preview_on = not bool(getattr(args, "no_preview_mp", False))
    win = "mp_curl MP21"
    show_mp21_window = None
    close_mp21_windows = None
    if preview_on:
        try:
            import cv2  # noqa: F401 — probe before enabling window

            from xr_hand_retarget.preview import close_mp21_windows, show_mp21_window
        except Exception as exc:
            print(f"OpenCV preview unavailable ({exc}); text-only capture", flush=True)
            preview_on = False
            show_mp21_window = None
            close_mp21_windows = None
    # Shared human file (not per-robot).
    out = _DEFAULT_CALIB if not args.out else Path(args.out).expanduser()
    if args.out and not out.is_absolute():
        # Relative --out: prefer package configs/ when writing calib/*
        pkg_out = Path(__file__).resolve().parents[1] / out
        if str(out).startswith("configs/") or not out.parent.exists():
            out = pkg_out
    sides = ("left", "right") if args.side == "full" else (args.side,)
    print(
        f"mp_curl: shared MediaPipe21 calib → {out}\n"
        f"  poses=open/fist(拇入掌)/thumb_along+pinch; multi-take: a=再采一组 Enter=下一姿态\n"
        f"  preview={'ON ' + win if preview_on else 'off (--no-preview-mp)'}",
        flush=True,
    )
    print("init xrobotoolkit_sdk...", flush=True)
    xrt.init()
    time.sleep(0.8)
    print("sdk ok  (采集期间不要同时跑 vr-xrt)", flush=True)

    def pump_preview(side: str, pose_name: str = "", hud: str = "") -> None:
        if not preview_on or show_mp21_window is None:
            return
        ok, raw = _read_frame(xrt, side)
        if ok:
            mp21 = openxr26_to_mediapipe21(raw)
            if not hud:
                try:
                    hud = _fmt_feat2(extract_curl_features_mp21(mp21, side))
                except Exception:
                    hud = "TRACK"
        else:
            mp21 = np.zeros((21, 3), dtype=np.float64)
            hud = hud or "inactive/zero"
        title = f"{side} {pose_name}".strip() or f"{side} MP21"
        show_mp21_window(win, mp21, title=title, hud=hud)

    def sample(side: str, pose_name: str = ""):
        period = 1.0 / max(args.rate, 1.0)
        deadline = time.time() + max(args.seconds, 0.2)
        feats = []
        mp21s = []
        rejected = 0
        while time.time() < deadline:
            ok, raw = _read_frame(xrt, side)
            if ok:
                mp21 = openxr26_to_mediapipe21(raw)
                feats.append(extract_curl_features_mp21(mp21, side))
                mp21s.append(mp21)
                if preview_on and show_mp21_window is not None:
                    show_mp21_window(
                        win,
                        mp21,
                        title=f"{side} {pose_name} CAPTURE",
                        hud=_fmt_feat2(feats[-1]),
                    )
            else:
                rejected += 1
                pump_preview(side, pose_name, hud="reject")
            time.sleep(period)
        if not feats:
            print(f"  无有效帧 (rejected={rejected})", flush=True)
            return None
        feat = _median_feat2(feats)
        mp21_med = np.median(np.stack(mp21s, axis=0), axis=0)
        print(
            f"  已采 n={len(feats)} reject={rejected}  {_fmt_feat2(feat)}",
            flush=True,
        )
        if preview_on and show_mp21_window is not None:
            show_mp21_window(
                win,
                mp21_med,
                title=f"{side} {pose_name} median",
                hud=_fmt_feat2(feat),
            )
        return feat, mp21_med

    def capture_one(side, calib, name, hint, takes_side: dict):
        pose_takes: list[dict] = []
        while True:
            ok, raw = _read_frame(xrt, side)
            if ok:
                live = _fmt_feat2(
                    extract_curl_features_mp21(openxr26_to_mediapipe21(raw), side)
                )
            else:
                live = "inactive/zero"
            n = len(pose_takes)
            print(f"\n[{side}] {name}: {hint}", flush=True)
            print(f"  now: {live}  (已采组数={n})", flush=True)
            cmd = _wait_with_mp_preview(
                xrt,
                side,
                "  保持姿势后确认",
                extra="a+Enter=再采一组(采完后)  r+Enter=清空重来  s=跳过  q=放弃",
                win=win,
                pose_name=name,
                enabled=preview_on,
            )
            if cmd in ("q", "quit"):
                return "quit"
            if cmd in ("s", "skip"):
                print(f"  skip {name}", flush=True)
                return "skip"
            if cmd in ("r", "redo") and not pose_takes:
                continue
            got = sample(side, pose_name=name)
            if got is None:
                print("  采集失败，重试", flush=True)
                continue
            feat, mp21_med = got
            pose_takes.append(
                {
                    "features": feat2_to_dict(feat),
                    "mp21": mp21_to_list(mp21_med),
                }
            )
            # Aggregate so far → knot used at runtime
            agg_feats = [feat2_from_dict(t["features"]) for t in pose_takes]
            agg_feats = [f for f in agg_feats if f is not None]
            agg = _median_feat2(agg_feats)
            _apply_pose2(calib, name, agg)
            print(f"  本组已写入；合计 {len(pose_takes)} 组 → knot 已用中位数", flush=True)
            cmd2 = _wait_with_mp_preview(
                xrt,
                side,
                "  继续？",
                extra="Enter=下一姿态  a+Enter=再采一组  r+Enter=清空本组重来",
                win=win,
                pose_name=name,
                enabled=preview_on,
            )
            if cmd2 in ("q", "quit"):
                return "quit"
            if cmd2 in ("a", "again", "+"):
                continue
            if cmd2 in ("r", "redo"):
                pose_takes.clear()
                print("  已清空本组，重新采集", flush=True)
                continue
            takes_side[name] = pose_takes
            return "ok"

    try:
        if args.preview_only:
            if preview_on and show_mp21_window is not None:
                print(
                    f"live mp_curl OpenCV [{'/'.join(sides)}]  "
                    "视窗 q 或 Ctrl+C 退出",
                    flush=True,
                )
                side0 = sides[0]
                try:
                    while True:
                        ok, raw = _read_frame(xrt, side0)
                        if not ok:
                            show_mp21_window(
                                win,
                                np.zeros((21, 3)),
                                title=f"{side0} MP21",
                                hud="inactive/zero",
                            )
                        else:
                            mp21 = openxr26_to_mediapipe21(raw)
                            feat = extract_curl_features_mp21(mp21, side0)
                            key = show_mp21_window(
                                win,
                                mp21,
                                title=f"{side0} mp_curl",
                                hud=_fmt_feat2(feat),
                            )
                            if key in (ord("q"), 27):
                                break
                        time.sleep(0.03)
                except KeyboardInterrupt:
                    print("", flush=True)
                return 0
            for side in sides:
                print(f"live mp_curl features [{side}]  Ctrl+C 退出", flush=True)
                try:
                    while True:
                        ok, raw = _read_frame(xrt, side)
                        if not ok:
                            print("  (inactive / zero)          ", end="\r", flush=True)
                        else:
                            feat = extract_curl_features_mp21(
                                openxr26_to_mediapipe21(raw), side
                            )
                            print(f"  {_fmt_feat2(feat)}     ", end="\r", flush=True)
                        time.sleep(0.08)
                except KeyboardInterrupt:
                    print("", flush=True)
            return 0

        store = (
            load_calib_file(out)
            if out.is_file()
            else {"left": seed_side_calib(), "right": seed_side_calib()}
        )
        all_takes: dict[str, dict[str, list]] = {
            s: {} for s in ("left", "right") if s in store
        }
        # Preserve prior takes for sides not being recaptured
        if out.is_file():
            import yaml

            with out.open("r", encoding="utf-8") as f:
                prev = yaml.safe_load(f) or {}
            for sname, sblock in (prev.get("sides") or {}).items():
                if sname not in sides and isinstance(sblock, dict):
                    old = sblock.get("takes")
                    if old:
                        all_takes.setdefault(sname, {}).update(old)

        for side in sides:
            print(f"\n======== {side.upper()} (mp_curl) ========", flush=True)
            takes_side: dict[str, list] = {}
            for name, hint in _MP_CURL_CORE:
                st = capture_one(side, store[side], name, hint, takes_side)
                if st == "quit":
                    print("aborted, file not written", flush=True)
                    return 1
            if args.skip_pinch:
                print(f"[{side}] pinch skipped (--skip-pinch)", flush=True)
            else:
                print(
                    f"\n[{side}] 精细化：拇指依次收到食/中/无/小指下方",
                    flush=True,
                )
                cmd = _wait_with_mp_preview(
                    xrt,
                    side,
                    "  开始捏合标定？",
                    extra="k+Enter=集体跳过（保留已有 pinch）",
                    win=win,
                    pose_name="pinch?",
                    enabled=preview_on,
                )
                if cmd in ("q", "quit"):
                    print("aborted, file not written", flush=True)
                    return 1
                if cmd not in ("k", "skip", "s"):
                    captured = 0
                    for name, hint in _PINCH:
                        st = capture_one(side, store[side], name, hint, takes_side)
                        if st == "quit":
                            print("aborted, file not written", flush=True)
                            return 1
                        if st == "ok":
                            captured += 1
                    store[side].pinch_skipped = captured == 0
                    print(
                        f"  pinch "
                        f"{'全部跳过' if captured == 0 else f'已写入 {captured}/4 指'}",
                        flush=True,
                    )
            all_takes[side] = takes_side

        dump_calib_file(out, store, takes=all_takes)
        print(f"\nwrote shared {out}", flush=True)
        model = getattr(cfg, "backend", "o6")
        if model not in ("o6", "l6", "o7"):
            model = "o6"
        print(
            f"然后: FA_HAND={model} FA_HAND_RETARGETING=mp_curl ./run.sh vr-xrt wrist",
            flush=True,
        )
        print(
            "  (o6/l6/o7 共用 mp_curl.yaml；机器人 pinch_q 各读 hands/{o6,l6,o7}.yaml)",
            flush=True,
        )
        return 0
    finally:
        if close_mp21_windows is not None:
            close_mp21_windows(win)
        try:
            xrt.close()
        except Exception:
            pass


def _linker_method(cfg, args) -> str:
    from xr_hand_retarget.algorithms import normalize

    raw = getattr(args, "method", None) or cfg.retargeting_type or "curl"
    return normalize(str(raw))


def main(argv=None):
    args = _parse_args(argv)
    cfg = load_runtime_config(args.config)
    sides = ("left", "right") if args.side == "full" else (args.side,)

    if cfg.backend == "xhand1":
        from xr_hand_retarget.calibrate_xhand1 import main as xhand1_calibrate

        forwarded = ["--side", args.side]
        if args.preview_only:
            forwarded.append("--preview-only")
        if args.seconds:
            forwarded.extend(["--seconds", str(args.seconds)])
        if args.rate:
            forwarded.extend(["--rate", str(args.rate)])
        if args.skip_pinch:
            forwarded.append("--skip-pinch")
        if args.skip_abd:
            forwarded.append("--skip-abd")
        return xhand1_calibrate(forwarded)

    if cfg.backend in ("o6", "l6", "o7"):
        if args.export_only:
            return _export_linker_session(args)
        return _run_linker_unified_capture(cfg, args)

    if cfg.retargeting_type == "curl":
        if args.dump_pkl:
            return _capture_pkl(
                cfg,
                sides=sides,
                seconds=args.seconds,
                rate=args.rate,
                out=Path(args.dump_pkl),
            )
        if args.preview_mp:
            from xr_hand_retarget.preview import run_preview

            vis_side = "left" if args.side == "left" else "right"
            return run_preview(cfg, side=vis_side, rate=args.rate)
        return _run_hand2_curl_capture(cfg, args)

    if args.dump_pkl:
        return _capture_pkl(
            cfg,
            sides=sides,
            seconds=args.seconds,
            rate=args.rate,
            out=Path(args.dump_pkl),
        )
    if args.preview_mp or args.preview_only:
        from xr_hand_retarget.preview import run_preview

        vis_side = "left" if args.side == "left" else "right"
        if args.side == "full":
            print("preview-mp shows one hand; using right (pass --side left to switch)", flush=True)
        return run_preview(cfg, side=vis_side, rate=args.rate)
    return _wuji_help(cfg)


if __name__ == "__main__":
    raise SystemExit(main() or 0)
