"""Live OpenXR → MediaPipe-21 skeleton preview (no IK, no ROS).

Use this to see whether Pico tracking / 26→21 mapping looks like a hand
before blaming the official Retargeter.

    python -m xr_hand_retarget.preview --side right
    python -m xr_hand_retarget.preview --side right --dump /tmp/xrt_mp21.pkl

Dump format matches official wuji-retargeting ``example/input_devices/mediapipe_replay.py``
(list of ``{t, left_fingers, right_fingers}``, each (21, 3) meters, raw MP21).
Replay with official ``example/tuning_tool.py --play``.
"""

from __future__ import annotations

import argparse
import pickle
import sys
import time
from pathlib import Path

import numpy as np

from xr_hand_retarget.landmarks import (
    openxr26_to_mediapipe21,
    palm_triangle_area_m2,
    pose_is_zero,
)

_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (0, 9), (9, 10), (10, 11), (11, 12),
    (0, 13), (13, 14), (14, 15), (15, 16),
    (0, 17), (17, 18), (18, 19), (19, 20),
    (5, 9), (9, 13), (13, 17),
]

# BGR
_FINGER_BGR = {
    range(0, 1): (200, 200, 200),
    range(1, 5): (0, 140, 255),
    range(5, 9): (0, 220, 0),
    range(9, 13): (0, 220, 220),
    range(13, 17): (255, 120, 0),
    range(17, 21): (255, 0, 180),
}


def _bgr_for(idx: int) -> tuple[int, int, int]:
    for rng, color in _FINGER_BGR.items():
        if idx in rng:
            return color
    return (180, 180, 180)


def _as_active(flag) -> int:
    try:
        v = np.asarray(flag).reshape(-1)
        if v.size == 0:
            return 0
        return 1 if float(v[0]) > 0.5 else 0
    except Exception:
        return 0


def _normalize_hand26(raw) -> np.ndarray:
    arr = np.asarray(raw, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[0] != 26:
        arr = arr.reshape(26, -1)
    return arr


def _palm_xy(mp21: np.ndarray) -> np.ndarray:
    p = np.asarray(mp21, dtype=np.float64)[:, :3]
    p = p - p[0:1]
    x = p[5].copy()
    xn = float(np.linalg.norm(x))
    x = np.array([1.0, 0.0, 0.0]) if xn < 1e-6 else x / xn
    n = np.cross(p[5], p[9])
    nn = float(np.linalg.norm(n))
    n = np.array([0.0, 0.0, 1.0]) if nn < 1e-8 else n / nn
    y = np.cross(n, x)
    yn = float(np.linalg.norm(y))
    if yn > 1e-8:
        y = y / yn
    return np.stack([p @ x, p @ y], axis=1)


def _draw_panel(mp21: np.ndarray, title: str, size: int = 420) -> "np.ndarray":
    import cv2

    img = np.zeros((size, size, 3), dtype=np.uint8)
    img[:] = (28, 28, 28)
    if mp21 is None or np.asarray(mp21).size == 0:
        cv2.putText(img, title, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)
        cv2.putText(img, "no points", (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 80, 220), 1)
        return img
    mp21 = np.asarray(mp21, dtype=np.float64)
    if not np.any(np.abs(mp21[:, :3]) > 1e-6):
        cv2.putText(img, title, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)
        cv2.putText(img, "no points", (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 80, 220), 1)
        return img
    xy = _palm_xy(mp21)
    span = float(np.max(np.abs(xy))) + 0.04
    span = max(span, 0.08)

    def to_px(pt):
        u = int((pt[0] / span * 0.45 + 0.5) * (size - 1))
        v = int((0.5 - pt[1] / span * 0.45) * (size - 1))
        return u, v

    pts = [to_px(xy[i]) for i in range(21)]
    for a, b in _CONNECTIONS:
        cv2.line(img, pts[a], pts[b], _bgr_for(b), 2, cv2.LINE_AA)
    for i, px in enumerate(pts):
        cv2.circle(img, px, 5, _bgr_for(i), -1, cv2.LINE_AA)
        cv2.putText(
            img, str(i), (px[0] + 6, px[1] - 6),
            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (230, 230, 230), 1, cv2.LINE_AA,
        )
    cv2.putText(img, title, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (240, 240, 240), 1)
    return img


def _apply_mano(mp21: np.ndarray, side: str, rotation_xyz: dict) -> np.ndarray:
    from wuji_retargeting.mediapipe import apply_mediapipe_transformations

    kp = apply_mediapipe_transformations(mp21, side)
    x_deg = float(rotation_xyz.get("x", 0.0) or 0.0)
    y_deg = float(rotation_xyz.get("y", 0.0) or 0.0)
    z_deg = float(rotation_xyz.get("z", 0.0) or 0.0)
    if x_deg == 0.0 and y_deg == 0.0 and z_deg == 0.0:
        return kp
    from scipy.spatial.transform import Rotation

    rot = Rotation.from_euler("xyz", [x_deg, y_deg, z_deg], degrees=True)
    return kp @ rot.as_matrix().T


def _rotation_from_cfg(cfg, side: str) -> dict:
    if cfg.wuji is None:
        return {}
    from xr_hand_retarget.backends.wuji import load_official_config

    yaml_path = cfg.wuji.yaml_for(side)
    official = load_official_config(yaml_path, cfg.wuji.retarget_overlay())
    return dict((official.get("retarget") or {}).get("mediapipe_rotation") or {})


def _empty21() -> np.ndarray:
    return np.zeros((21, 3), dtype=np.float32)


def _dump_frame(t: float, side: str, mp21: np.ndarray) -> dict:
    left = _empty21()
    right = _empty21()
    kp = np.asarray(mp21, dtype=np.float32).reshape(21, 3)
    if side == "left":
        left = kp
    else:
        right = kp
    return {"t": float(t), "left_fingers": left, "right_fingers": right}


def draw_mp21_panel(mp21: np.ndarray, title: str, size: int = 420) -> "np.ndarray":
    """Public wrapper: palm-plane MediaPipe-21 stick figure (BGR image)."""
    return _draw_panel(mp21, title, size=size)


def show_mp21_window(
    win: str,
    mp21: np.ndarray | None,
    *,
    title: str = "MP21",
    hud: str = "",
    size: int = 420,
) -> int:
    """``imshow`` one MP21 panel; returns ``waitKey`` code (low 8 bits), or -1."""
    try:
        import cv2
    except ImportError:
        return -1
    img = _draw_panel(
        np.zeros((21, 3)) if mp21 is None else mp21,
        title,
        size=size,
    )
    if hud:
        cv2.putText(
            img,
            hud[:72],
            (12, img.shape[0] - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (220, 220, 220),
            1,
            cv2.LINE_AA,
        )
    cv2.imshow(win, img)
    return int(cv2.waitKey(1) & 0xFF)


def close_mp21_windows(*names: str) -> None:
    try:
        import cv2

        if names:
            for n in names:
                try:
                    cv2.destroyWindow(n)
                except Exception:
                    pass
        else:
            cv2.destroyAllWindows()
    except Exception:
        pass


def run_preview(cfg, *, side: str, rate: float, dump: str | None = None) -> int:
    try:
        import cv2
    except ImportError as exc:
        print("Need OpenCV for the preview window:  uv pip install opencv-python", file=sys.stderr)
        raise SystemExit(1) from exc
    try:
        import xrobotoolkit_sdk as xrt
    except ImportError as exc:
        print("xrobotoolkit_sdk not found (stop vr-xrt; one PC Service client)", file=sys.stderr)
        raise SystemExit(1) from exc

    if side == "left":
        get_active, get_hand = xrt.get_left_hand_is_active, xrt.get_left_hand_tracking_state
    else:
        get_active, get_hand = xrt.get_right_hand_is_active, xrt.get_right_hand_tracking_state

    rot = {}
    try:
        rot = _rotation_from_cfg(cfg, side)
    except Exception as exc:
        print(f"official yaml rotation skipped: {exc}", flush=True)

    print("init xrobotoolkit_sdk (stop ./run.sh vr-xrt first)...", flush=True)
    xrt.init()
    time.sleep(0.8)
    print(
        f"preview {side}  rotation={rot or '0,0,0'}\n"
        "  LEFT  = OpenXR 26→21 in palm plane  (Pico tracking)\n"
        "  RIGHT = official MANO + mediapipe_rotation  (what IK sees)\n"
        "  q quit.  A good Pico hand looks like a 5-finger stick figure on BOTH panels.",
        flush=True,
    )
    period = 1.0 / max(rate, 1.0)
    win = "xr_hand_retarget MP21"
    frames: list[dict] = []
    t0_rec = time.time()
    if dump:
        print(f"dumping official pkl → {dump}  (q to save and quit)", flush=True)
    try:
        while True:
            t0 = time.time()
            active = _as_active(get_active())
            raw = _normalize_hand26(get_hand())
            xyz = float(np.abs(raw[:, :3]).max()) if raw.size else 0.0
            if pose_is_zero(raw) or active == 0:
                mp21 = np.zeros((21, 3), dtype=np.float64)
                mano = mp21
                area = 0.0
                span = 0.0
            else:
                mp21 = openxr26_to_mediapipe21(raw)
                area = palm_triangle_area_m2(mp21)
                span = float(np.linalg.norm(mp21 - mp21[0:1], axis=1).max())
                try:
                    mano = _apply_mano(mp21, side, rot)
                except Exception:
                    mano = mp21
            if dump:
                frames.append(_dump_frame(time.time() - t0_rec, side, mp21))
            left = _draw_panel(mp21, f"raw MP21  active={active}")
            right = _draw_panel(mano, f"MANO+rot {rot or '{}'}")
            canvas = np.hstack([left, right])
            hud = (
                f"{side} xyz={xyz:.3f}m  palm={area:.2e}m2  span={span:.3f}m  "
                f"{'TRACK' if active else 'HOLD/inactive'}"
            )
            import cv2 as _cv2

            _cv2.putText(
                canvas, hud, (12, canvas.shape[0] - 12),
                _cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1, _cv2.LINE_AA,
            )
            _cv2.imshow(win, canvas)
            key = _cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            dt = time.time() - t0
            time.sleep(max(0.0, period - dt))
    finally:
        if dump and frames:
            out = Path(dump).expanduser()
            out.parent.mkdir(parents=True, exist_ok=True)
            with open(out, "wb") as f:
                pickle.dump(frames, f)
            n_live = sum(
                1
                for fr in frames
                if float(np.abs(fr[f"{side}_fingers"]).max()) > 1e-6
            )
            print(f"wrote {len(frames)} frames ({n_live} tracked) → {out}", flush=True)
        try:
            xrt.close()
        except Exception:
            pass
        try:
            import cv2 as _cv2

            _cv2.destroyAllWindows()
        except Exception:
            pass
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", default=None)
    p.add_argument("--side", choices=("left", "right"), default="right")
    p.add_argument("--rate", type=float, default=30.0)
    p.add_argument(
        "--dump",
        default=None,
        metavar="PKL",
        help="Write official MediaPipe-replay pkl for wuji-retargeting tuning_tool.py --play",
    )
    args = p.parse_args(argv)
    from xr_hand_retarget.config import load_runtime_config

    cfg = load_runtime_config(args.config)
    return run_preview(cfg, side=args.side, rate=args.rate, dump=args.dump)


if __name__ == "__main__":
    raise SystemExit(main() or 0)
