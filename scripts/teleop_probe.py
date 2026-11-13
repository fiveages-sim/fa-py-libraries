#!/usr/bin/env python3
"""One-shot teleop probe (read-only; never touches xrobotoolkit_sdk).

Optical Wrist Tracking Quality v1 — subscriber-side, monotonic clock.

Per topic: arrival / unique / rep% / dt / stall.
EE quality: Position/Orientation RMS (static-ish), v/ω/a/α with real Δt,
            Hold Duration, Effective Update Interval (tolerance equality).
Stutter localize: pico unique interval vs EE effective interval vs isActive.

See: vr_pose_publisher/docs/EE_TRACKING_QUALITY.md

Usage
    python3 scripts/teleop_probe.py --sec 10
    python3 scripts/teleop_probe.py --sec 10 --settle-sec 1 --scene center_static \\
        --csv /tmp/ee.csv
    # With --csv: companion .txt (tee) + .npz pose timeseries for spectrum plots.
    # Guided Quality: scripts/teleop_probe_session.py
    # Guided Spectrum: scripts/teleop_spectrum_session.py + teleop_spectrum_plot.py

Settle discards the first N seconds (keyboard press / hand settle), then prints
RECORDING so static RMS is not polluted by the start transient.

Run WITH the teleop stack up. Never start a second XRT client.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int64MultiArray


class _TeeStdout:
    """Mirror stdout to a UTF-8 log file (built-in tee; no shell pipe needed)."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fp = self.path.open("w", encoding="utf-8")
        self._stdout = sys.stdout

    def write(self, data: str) -> int:
        self._stdout.write(data)
        self._fp.write(data)
        return len(data)

    def flush(self) -> None:
        self._stdout.flush()
        self._fp.flush()

    def isatty(self) -> bool:
        return bool(getattr(self._stdout, "isatty", lambda: False)())

    def fileno(self) -> int:
        return self._stdout.fileno()

    def close(self) -> None:
        try:
            self._fp.flush()
            self._fp.close()
        except Exception:
            pass

    def __enter__(self) -> "_TeeStdout":
        sys.stdout = self  # type: ignore[assignment]
        return self

    def __exit__(self, *exc) -> None:
        try:
            self.write(f"log -> {self.path}\n")
            self.flush()
        except Exception:
            pass
        sys.stdout = self._stdout
        self.close()


def _resolve_log_path(csv_path: str, log_path: str, no_log: bool) -> str | None:
    if no_log:
        return None
    if log_path:
        return log_path
    if csv_path:
        p = Path(csv_path)
        return str(p.with_suffix(".txt"))
    return None

POSE_TOPICS = ("/teleop/head_pose", "/teleop/left_ee_pose", "/teleop/right_ee_pose")
EE_TOPICS = ("/teleop/left_ee_pose", "/teleop/right_ee_pose")
Q_TOPICS = (
    "/left_hand_controller/target_joint_position",
    "/right_hand_controller/target_joint_position",
)
JOINT_TOPICS = (
    "/joint_states",
    "/left_hand_controller/joint_states",
    "/right_hand_controller/joint_states",
)
DIAG = "/teleop/xrt_diag"

# Scene IDs from EE_TRACKING_QUALITY.md (documentation / CSV tag only).
SCENE_IDS = (
    "center_static",
    "center_translate",
    "center_rotate",
    "edge_static",
    "edge_translate",
    "edge_rotate",
    "optional_flick",
    # Spectrum-session long sequence labels
    "square_path",
    "forward_reach",
    "xyz_rotate",
    "unspecified",
)


def _pct(a: np.ndarray, q: float) -> float:
    return float(np.percentile(a, q)) if a.size else float("nan")


def _quat_to_R(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    n = float(np.linalg.norm(q))
    if n < 1e-12 or not np.isfinite(n):
        return np.eye(3)
    x, y, z, w = q / n
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def _rel_angle(R_prev: np.ndarray, R_cur: np.ndarray) -> float:
    R = R_prev.T @ R_cur
    c = float(np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0))
    return float(np.arccos(c))


def _pose_close(
    a: tuple[float, ...],
    b: tuple[float, ...],
    *,
    eps_m: float,
    eps_rad: float,
) -> bool:
    pa = np.asarray(a[:3], dtype=np.float64)
    pb = np.asarray(b[:3], dtype=np.float64)
    if float(np.linalg.norm(pa - pb)) > eps_m:
        return False
    Ra = _quat_to_R(np.asarray(a[3:7], dtype=np.float64))
    Rb = _quat_to_R(np.asarray(b[3:7], dtype=np.float64))
    return _rel_angle(Ra, Rb) <= eps_rad


class Rec:
    __slots__ = ("t", "sig", "data")

    def __init__(self) -> None:
        self.t = []
        self.sig = []
        self.data = []

    def add(self, sig, data=None) -> None:
        self.t.append(time.monotonic())
        self.sig.append(sig)
        if data is not None:
            self.data.append(data)

    def clear(self) -> None:
        self.t.clear()
        self.sig.clear()
        self.data.clear()


class Probe(Node):
    def __init__(self, hand_filter: str) -> None:
        super().__init__("teleop_probe")
        self.recs: dict[str, Rec] = {}
        self.hand_filter = hand_filter
        qos = QoSProfile(
            depth=4000,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        for n in POSE_TOPICS + Q_TOPICS + JOINT_TOPICS + (DIAG,):
            self.recs[n] = Rec()

        for n in POSE_TOPICS:
            self.create_subscription(Pose, n, self._pose_cb(n), qos)
        for n in Q_TOPICS:
            self.create_subscription(Float64MultiArray, n, self._q_cb(n), qos)
        for n in JOINT_TOPICS:
            self.create_subscription(JointState, n, self._joint_cb(n), qos)
        self.create_subscription(Int64MultiArray, DIAG, self._diag_cb, qos)
        self._recording = True  # False during settle: callbacks no-op after warm-up clear

    def clear_recs(self) -> None:
        for r in self.recs.values():
            r.clear()

    def _pose_cb(self, name: str):
        def cb(msg: Pose) -> None:
            if not self._recording:
                return
            p, o = msg.position, msg.orientation
            self.recs[name].add((p.x, p.y, p.z, o.x, o.y, o.z, o.w))

        return cb

    def _q_cb(self, name: str):
        def cb(msg: Float64MultiArray) -> None:
            if not self._recording:
                return
            self.recs[name].add(tuple(msg.data))

        return cb

    def _joint_cb(self, name: str):
        def cb(msg: JointState) -> None:
            if not self._recording:
                return
            idx = [i for i, jn in enumerate(msg.name) if self.hand_filter in jn]
            if not idx:
                return
            self.recs[name].add(
                tuple(round(float(msg.position[i]), 9) for i in idx)
            )

        return cb

    def _diag_cb(self, msg: Int64MultiArray) -> None:
        if not self._recording:
            return
        d = tuple(int(v) for v in msg.data)
        self.recs[DIAG].add((d[0],), d)


def _hold_and_eff(
    t: np.ndarray,
    poses: list,
    *,
    eps_m: float,
    eps_rad: float,
) -> tuple[np.ndarray, np.ndarray, float]:
    """Return (hold_durations_ms, eff_update_intervals_ms, max_hold_ms)."""
    if len(poses) < 2:
        return np.array([]), np.array([]), float("nan")
    holds: list[float] = []
    eff: list[float] = []
    run_t0 = float(t[0])
    last_eff_t = float(t[0])
    for i in range(1, len(poses)):
        same = _pose_close(poses[i - 1], poses[i], eps_m=eps_m, eps_rad=eps_rad)
        if same:
            continue
        # end of a hold run (may be zero-length if every frame unique)
        holds.append((float(t[i - 1]) - run_t0) * 1e3)
        eff.append((float(t[i]) - last_eff_t) * 1e3)
        last_eff_t = float(t[i])
        run_t0 = float(t[i])
    # trailing hold through last sample
    holds.append((float(t[-1]) - run_t0) * 1e3)
    h = np.asarray(holds, dtype=np.float64)
    e = np.asarray(eff, dtype=np.float64)
    return h, e, float(h.max()) if h.size else float("nan")


def _ee_motion_stats(
    t: np.ndarray,
    poses: list,
) -> dict[str, float]:
    """v/ω/a/α from real Δt; RMS relative to mean pose (static proxy)."""
    n = len(poses)
    out = {
        "pos_rms_m": float("nan"),
        "ori_rms_rad": float("nan"),
        "v_p99": float("nan"),
        "w_p99": float("nan"),
        "a_p99": float("nan"),
        "alpha_p99": float("nan"),
    }
    if n < 3:
        return out
    P = np.array([p[:3] for p in poses], dtype=np.float64)
    Q = np.array([p[3:7] for p in poses], dtype=np.float64)
    R = np.stack([_quat_to_R(q) for q in Q], axis=0)
    mean_p = P.mean(axis=0)
    out["pos_rms_m"] = float(np.sqrt(np.mean(np.sum((P - mean_p) ** 2, axis=1))))
    R0 = R[0]
    ang = np.array([_rel_angle(R0, R[i]) for i in range(n)], dtype=np.float64)
    out["ori_rms_rad"] = float(np.sqrt(np.mean(ang**2)))

    dt = np.diff(t)
    dt = np.maximum(dt, 1e-6)
    dP = np.diff(P, axis=0)
    v = np.linalg.norm(dP, axis=1) / dt
    w = np.array(
        [_rel_angle(R[i], R[i + 1]) / dt[i] for i in range(n - 1)],
        dtype=np.float64,
    )
    out["v_p99"] = _pct(v, 99)
    out["w_p99"] = _pct(w, 99)
    if v.size >= 2:
        dt2 = np.maximum(np.diff(t[1:]), 1e-6)
        a = np.abs(np.diff(v)) / dt2
        alpha = np.abs(np.diff(w)) / dt2
        out["a_p99"] = _pct(a, 99)
        out["alpha_p99"] = _pct(alpha, 99)
    return out


def _resample_poses(
    t: np.ndarray,
    poses: list,
    hz: float,
) -> tuple[np.ndarray, list]:
    """Uniform time grid; xyz linear, R via successive SLERP-ish quat lerp."""
    if len(poses) < 2 or hz <= 0:
        return t, poses
    t0, t1 = float(t[0]), float(t[-1])
    if t1 <= t0:
        return t, poses
    grid = np.arange(t0, t1, 1.0 / hz)
    if grid.size < 2:
        return t, poses
    P = np.array([p[:3] for p in poses], dtype=np.float64)
    Q = np.array([p[3:7] for p in poses], dtype=np.float64)
    # Fix double-cover for lerp
    for i in range(1, len(Q)):
        if float(np.dot(Q[i], Q[i - 1])) < 0.0:
            Q[i] = -Q[i]
    out_poses = []
    for tg in grid:
        j = int(np.searchsorted(t, tg, side="right") - 1)
        j = max(0, min(j, len(t) - 2))
        u = (tg - float(t[j])) / max(float(t[j + 1]) - float(t[j]), 1e-9)
        u = float(np.clip(u, 0.0, 1.0))
        p = (1.0 - u) * P[j] + u * P[j + 1]
        q = (1.0 - u) * Q[j] + u * Q[j + 1]
        qn = float(np.linalg.norm(q))
        if qn > 1e-12:
            q = q / qn
        out_poses.append(
            (float(p[0]), float(p[1]), float(p[2]), float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        )
    return grid, out_poses


def report(
    recs: dict[str, Rec],
    dur: float,
    csv_path: str,
    *,
    eps_m: float,
    eps_rad: float,
    scene: str,
    resample_hz: float,
    settle_sec: float = 0.0,
    tag: str = "",
) -> None:
    W = 44
    print()
    print("=" * 100)
    print(
        f"TELEOP PROBE   dur={dur:.2f}s   scene={scene}   "
        f"settle={settle_sec:.2f}s   tag={tag or '-'}   "
        f"(subscriber-side, monotonic clock)"
    )
    print("=" * 100)
    print(
        f"{'topic':<{W}}{'arriv':>8}{'unique':>8}{'rep%':>7}"
        f"{'p50':>8}{'p95':>8}{'p99':>8}{'max':>8}{'stall':>7}"
    )
    print("-" * 100)

    ee_uniq_hz: dict[str, float] = {}
    ee_arr_hz: dict[str, float] = {}
    for name, r in recs.items():
        if not r.t:
            print(f"{name:<{W}}{'-- no data --':>47}")
            continue
        t = np.asarray(r.t)
        dts = np.diff(t) * 1e3
        arr = max(len(t) - 1, 0) / max(dur, 1e-9)
        uniq = sum(1 for i in range(1, len(r.sig)) if r.sig[i] != r.sig[i - 1])
        uniq_hz = uniq / max(dur, 1e-9)
        rep = 100.0 * (1.0 - uniq / max(len(r.sig) - 1, 1))
        med = float(np.median(dts)) if dts.size else float("nan")
        stall = int(np.sum(dts > 3.0 * med)) if dts.size and med > 0 else 0
        print(
            f"{name:<{W}}{arr:>8.1f}{uniq_hz:>8.1f}{rep:>6.1f}%"
            f"{med:>8.2f}{_pct(dts, 95):>8.2f}{_pct(dts, 99):>8.2f}"
            f"{dts.max():>8.2f}{stall:>7}"
        )
        if name in EE_TOPICS:
            side = "left" if "left" in name else "right"
            ee_uniq_hz[side] = uniq_hz
            ee_arr_hz[side] = arr

    # Always print L/R unique/arriv (even without diag) — primary asymmetry signal
    print("-" * 100)
    for side in ("left", "right"):
        uh = ee_uniq_hz.get(side, float("nan"))
        ah = ee_arr_hz.get(side, float("nan"))
        u_over_a = uh / max(ah, 1e-9) if ah == ah else float("nan")
        print(
            f"{side}_ee unique/arriv : {u_over_a:8.3f}   "
            f"arriv={ah:6.1f}Hz unique={uh:6.1f}Hz   "
            f"(≈1.0=每帧新位姿; <<1=同值重发/hold)"
        )

    d = recs.get(DIAG)
    pico_iv = np.array([])
    la = ra = np.array([])
    pico_hz = float("nan")
    if d is not None and d.data:
        pico = np.array([x[0] for x in d.data], dtype=np.int64)
        pc = np.array([x[1] for x in d.data], dtype=np.int64)
        ros = np.array([x[2] for x in d.data], dtype=np.int64)
        la = np.array([x[3] for x in d.data], dtype=np.int64)
        ra = np.array([x[4] for x in d.data], dtype=np.int64)
        t_diag = np.asarray(d.t)
        uniq_pico = int(np.sum(pico[1:] != pico[:-1])) if pico.size > 1 else 0
        pico_hz = uniq_pico / max(dur, 1e-9)
        lat = (ros - pc) / 1e6
        # pico unique intervals (when frame id changes)
        pico_iv_list = []
        last_t = None
        for i in range(1, pico.size):
            if pico[i] != pico[i - 1]:
                if last_t is not None:
                    pico_iv_list.append((float(t_diag[i]) - last_t) * 1e3)
                last_t = float(t_diag[i])
            elif last_t is None:
                last_t = float(t_diag[i])
        pico_iv = np.asarray(pico_iv_list, dtype=np.float64)
        print("-" * 100)
        print(f"pico unique frames     : {pico_hz:8.1f} Hz   <- Sampling / 源端新鲜度")
        if pico_iv.size:
            print(
                f"pico unique interval   : p50 {_pct(pico_iv, 50):5.2f} ms  "
                f"p99 {_pct(pico_iv, 99):5.2f} ms  max {pico_iv.max():5.2f} ms"
            )
        print(
            f"PC->ROS latency        : p50 {_pct(lat, 50):5.2f} ms  "
            f"p99 {_pct(lat, 99):5.2f} ms  max {lat.max():5.2f} ms"
        )
        for side in ("left", "right"):
            uh = ee_uniq_hz.get(side, float("nan"))
            u_over_p = uh / max(pico_hz, 1e-9)
            print(
                f"{side}_ee unique/pico  : {u_over_p:8.3f}   "
                f"(≈1.0 = 1:1 vs pico Sampling)"
            )
        for tag, a in (("left", la), ("right", ra)):
            tr = int(np.sum(a[1:] != a[:-1])) if a.size > 1 else 0
            print(
                f"isActive {tag:<12}: duty={a.mean():.2f}  "
                f"transitions={tr:3d} / {dur:.0f}s   <- 冻结/恢复"
            )
        # Correlate EE bit-exact holds with isActive==0 (L/R asymmetry check)
        if d is not None and d.t and la.size and ra.size:
            t_diag = np.asarray(d.t, dtype=np.float64)
            for side, active in (("left", la), ("right", ra)):
                r = recs.get(f"/teleop/{side}_ee_pose")
                if r is None or len(r.sig) < 2 or t_diag.size < 2:
                    continue
                t_ee = np.asarray(r.t, dtype=np.float64)
                # sample isActive at each EE msg (nearest diag)
                idx = np.searchsorted(t_diag, t_ee, side="left")
                idx = np.clip(idx, 0, active.size - 1)
                act_at = active[idx].astype(bool)
                same = np.array(
                    [r.sig[i] == r.sig[i - 1] for i in range(1, len(r.sig))],
                    dtype=bool,
                )
                # hold sample uses activity at current frame
                act_cur = act_at[1:]
                n_hold = int(same.sum())
                n_hold_inactive = int(np.sum(same & ~act_cur)) if same.size else 0
                n_hold_active = int(np.sum(same & act_cur)) if same.size else 0
                pct_in = (
                    100.0 * n_hold_inactive / max(n_hold, 1) if n_hold else float("nan")
                )
                print(
                    f"hold↔isActive {side:5}: holds={n_hold:5d}  "
                    f"on_inactive={n_hold_inactive:5d} ({pct_in:5.1f}%)  "
                    f"on_active={n_hold_active:5d}  "
                    f"<- 高%on_inactive=hold_on_inactive放大源端掉沿"
                )
        if d.data and len(d.data[0]) >= 9:
            lp = np.array([x[5] for x in d.data], dtype=np.float64)
            rp = np.array([x[6] for x in d.data], dtype=np.float64)
            lo = np.array([x[7] for x in d.data], dtype=np.int64)
            ro = np.array([x[8] for x in d.data], dtype=np.int64)
            _ORIENT = {
                0: "none",
                1: "sdk_quat",
                2: "xyz_hold_r",
                3: "xyz_seed",
                4: "reject",
                5: "error",
                6: "position_mode",
            }

            def _orient_hist(codes: np.ndarray) -> str:
                if codes.size == 0:
                    return "-"
                parts = []
                for k, name in _ORIENT.items():
                    n = int(np.sum(codes == k))
                    if n:
                        parts.append(f"{name}={100.0 * n / codes.size:.0f}%")
                return " ".join(parts) if parts else "-"

            print(
                f"palm_u L/R (m2*1e6)  : "
                f"p50={_pct(lp, 50):.1f}/{_pct(rp, 50):.1f}  "
                f"p10={_pct(lp, 10):.1f}/{_pct(rp, 10):.1f}  "
                f"(≥10 ≈ real palm; with isActive=0 → false inactive)"
            )
            print(f"orient_src left      : {_orient_hist(lo)}")
            print(f"orient_src right     : {_orient_hist(ro)}")

    # EE quality + stutter localization
    print("-" * 100)
    print(
        f"EE QUALITY  eps_m={eps_m:g}  eps_rad={eps_rad:g}  "
        f"(Hold=continuous same pose; EffUpd=interval between changes)"
    )
    for name in EE_TOPICS:
        r = recs.get(name)
        if r is None or len(r.sig) < 3:
            continue
        t = np.asarray(r.t, dtype=np.float64)
        poses = list(r.sig)
        holds, eff, max_hold = _hold_and_eff(
            t, poses, eps_m=eps_m, eps_rad=eps_rad
        )
        mot = _ee_motion_stats(t, poses)
        tag = "left " if "left" in name else "right"
        print(
            f"{tag} EE  pos_rms={mot['pos_rms_m']*1e3:6.2f} mm  "
            f"ori_rms={mot['ori_rms_rad']*1e3:6.2f} mrad  "
            f"|v|p99={mot['v_p99']:7.3f} m/s  "
            f"|w|p99={mot['w_p99']:7.3f} rad/s  "
            f"|a|p99={mot['a_p99']:7.2f}  |α|p99={mot['alpha_p99']:7.2f}"
        )
        print(
            f"{tag} EE  HoldDuration  p50={_pct(holds, 50):6.2f}  "
            f"p99={_pct(holds, 99):6.2f}  max={max_hold:6.2f} ms   "
            f"EffUpdate  p50={_pct(eff, 50):6.2f}  "
            f"p99={_pct(eff, 99):6.2f}  max="
            f"{(eff.max() if eff.size else float('nan')):6.2f} ms"
        )
        if pico_iv.size and eff.size:
            ratio = _pct(eff, 99) / max(_pct(pico_iv, 99), 1e-9)
            hint = "fa-py?" if ratio > 1.5 else "source-like"
            print(
                f"{tag} EE  stutter_localize: "
                f"eff_p99/pico_p99={ratio:5.2f}  ({hint}; "
                f"align Hold with isActive edges if duty<1)"
            )
        if resample_hz > 0:
            tg, pg = _resample_poses(t, poses, resample_hz)
            mot_r = _ee_motion_stats(tg, pg)
            print(
                f"{tag} EE  resample@{resample_hz:.0f}Hz  "
                f"pos_rms={mot_r['pos_rms_m']*1e3:6.2f} mm  "
                f"ori_rms={mot_r['ori_rms_rad']*1e3:6.2f} mrad  "
                f"|v|p99={mot_r['v_p99']:7.3f}  |w|p99={mot_r['w_p99']:7.3f}"
            )

    print("-" * 100)
    for name in Q_TOPICS:
        r = recs.get(name)
        if r is None or len(r.sig) < 3:
            continue
        arr = np.array([np.asarray(s, dtype=float) for s in r.sig])
        same = np.all(np.isclose(arr[1:], arr[:-1], atol=1e-6), axis=1)
        mdq = np.abs(np.diff(arr, axis=0)).max(axis=1)
        dd = np.abs(np.diff(np.diff(arr, axis=0), axis=0)).max(axis=1)
        print(
            f"{name.split('/')[1]:<22} n={len(arr):5d}  "
            f"frozen={100.0 * same.mean():5.1f}%  "
            f"|dq| p50={_pct(mdq, 50):.4f} p99={_pct(mdq, 99):.4f}  "
            f"|d2q| p99={_pct(dd, 99):.4f} rad"
        )

    for name in JOINT_TOPICS:
        r = recs.get(name)
        if r is None or len(r.sig) < 3:
            continue
        arr = np.array([np.asarray(s, dtype=float) for s in r.sig])
        mdq = np.abs(np.diff(arr, axis=0)).max(axis=1)
        note = ""
        if name == "/joint_states":
            note = "   <- arriv≠unique: driver republish vs joint change"
        print(
            f"{name:<22} n={len(arr):5d}  max|dq| p99={_pct(mdq, 99):.4f}  "
            f"max={mdq.max():.4f} rad{note}"
        )
    print("=" * 100)
    print(
        "Human score (fill after run): jitter=none|mild|strong  "
        "stutter=none|mild|strong  — see EE_TRACKING_QUALITY.md"
    )
    print(
        "Note: long Hold + stable isActive on static ≈ pose within eps (good), "
        "not stutter; trust pos/ori RMS for static jitter."
    )

    if csv_path:
        with open(csv_path, "w", encoding="utf-8") as f:
            f.write("scene,tag,topic,idx,t,dt_ms\n")
            for name, r in recs.items():
                t = np.asarray(r.t)
                for i, v in enumerate(np.diff(t) * 1e3):
                    f.write(
                        f"{scene},{tag or '-'},{name},{i},{t[i + 1]:.6f},{v:.4f}\n"
                    )
        print(f"csv -> {csv_path}")


def write_pose_npz(
    recs: dict[str, Rec],
    path: str,
    *,
    scene: str,
    tag: str,
    settle_sec: float,
    dur: float,
) -> None:
    """Save left/right EE pose timeseries for spectrum / inactive-usability (npz)."""
    out: dict[str, object] = {
        "scene": np.asarray(scene),
        "tag": np.asarray(tag or "-"),
        "settle_sec": np.asarray(float(settle_sec)),
        "dur_s": np.asarray(float(dur)),
    }
    # Diag timeline (isActive / palm) for inactive-usability audits
    d = recs.get(DIAG)
    diag_t = diag_la = diag_ra = diag_lp = diag_rp = None
    if d is not None and d.data and d.t:
        diag_t = np.asarray(d.t, dtype=np.float64)
        diag_la = np.array([x[3] for x in d.data], dtype=np.int64)
        diag_ra = np.array([x[4] for x in d.data], dtype=np.int64)
        if len(d.data[0]) >= 7:
            diag_lp = np.array([x[5] for x in d.data], dtype=np.float64)
            diag_rp = np.array([x[6] for x in d.data], dtype=np.float64)
        out["diag_t"] = diag_t
        out["diag_active_left"] = diag_la
        out["diag_active_right"] = diag_ra
        if diag_lp is not None:
            out["diag_palm_u_left"] = diag_lp
            out["diag_palm_u_right"] = diag_rp

    for side, topic in (
        ("left", "/teleop/left_ee_pose"),
        ("right", "/teleop/right_ee_pose"),
    ):
        r = recs.get(topic)
        if r is None or len(r.sig) < 2:
            continue
        t = np.asarray(r.t, dtype=np.float64)
        poses = np.asarray(r.sig, dtype=np.float64)
        out[f"{side}_t"] = t
        out[f"{side}_xyz"] = poses[:, :3]
        out[f"{side}_quat_xyzw"] = poses[:, 3:7]
        # Nearest-diag isActive / palm on EE timeline
        if diag_t is not None and diag_la is not None:
            idx = np.clip(np.searchsorted(diag_t, t, side="left"), 0, diag_t.size - 1)
            act = diag_la if side == "left" else diag_ra
            out[f"{side}_is_active"] = act[idx].astype(np.int8)
            if diag_lp is not None:
                pu = diag_lp if side == "left" else diag_rp
                out[f"{side}_palm_u"] = pu[idx]
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, **out)
    print(f"pose -> {path}")


def _prompt(msg: str) -> None:
    print(msg, flush=True)


def _settle_then_record(node: Probe, settle_sec: float) -> None:
    """Spin during settle (keep DDS warm) but do not keep samples; then arm recording."""
    settle_sec = max(0.0, float(settle_sec))
    if settle_sec <= 0.0:
        node._recording = True
        _prompt(">>> RECORDING START (settle=0)")
        return

    node._recording = False
    node.clear_recs()
    _prompt(
        f">>> SETTLE {settle_sec:.1f}s — hold still / ignore keyboard motion; "
        "samples discarded"
    )
    t0 = time.monotonic()
    last_left = int(settle_sec) + 1
    while rclpy.ok() and (time.monotonic() - t0) < settle_sec:
        left = int(np.ceil(settle_sec - (time.monotonic() - t0)))
        if left != last_left and left > 0:
            _prompt(f"    settle… {left}s")
            last_left = left
        rclpy.spin_once(node, timeout_sec=0.05)
    node.clear_recs()
    node._recording = True
    _prompt(">>> RECORDING START — keep pose; do not touch keyboard until done")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Teleop EE quality / stutter probe (Quality v1)"
    )
    ap.add_argument("--sec", type=float, default=20.0)
    ap.add_argument(
        "--settle-sec",
        type=float,
        default=1.0,
        help="Discard first N seconds after start (default 1; 0=off). "
        "Avoids keyboard-press / hand-settle transient in static RMS.",
    )
    ap.add_argument("--csv", default="")
    ap.add_argument(
        "--log",
        default="",
        help="Stdout log path (Quality report). Default: same stem as --csv with .txt",
    )
    ap.add_argument(
        "--no-log",
        action="store_true",
        help="Do not write a companion .txt even when --csv is set",
    )
    ap.add_argument(
        "--tag",
        default="",
        help="Label for before/after filter runs, e.g. raw | one_euro_B",
    )
    ap.add_argument("--hand-filter", default="hand")
    ap.add_argument("--no-diag", action="store_true")
    ap.add_argument(
        "--scene",
        default="unspecified",
        choices=SCENE_IDS,
        help="Label for human A/B matrix (EE_TRACKING_QUALITY.md)",
    )
    ap.add_argument(
        "--eps-m",
        type=float,
        default=1e-4,
        help="Pose equality tolerance (meters) for Hold / EffUpdate",
    )
    ap.add_argument(
        "--eps-rad",
        type=float,
        default=1e-3,
        help="Pose equality tolerance (radians) for Hold / EffUpdate",
    )
    ap.add_argument(
        "--resample-hz",
        type=float,
        default=0.0,
        help="If >0, also report motion metrics on a uniform time grid",
    )
    ap.add_argument(
        "--pose-npz",
        default="",
        help="Write left/right EE pose timeseries (.npz). "
        "Default: same stem as --csv with .npz when --csv is set.",
    )
    ap.add_argument(
        "--no-pose-npz",
        action="store_true",
        help="Do not write pose npz even when --csv is set",
    )
    args = ap.parse_args(argv)

    log_path = _resolve_log_path(args.csv, args.log, args.no_log)
    pose_path = args.pose_npz
    if not pose_path and args.csv and not args.no_pose_npz:
        pose_path = str(Path(args.csv).with_suffix(".npz"))
    if args.no_pose_npz:
        pose_path = ""
    tee_cm = _TeeStdout(log_path) if log_path else None

    def _run() -> int:
        rclpy.init(args=[])
        node = Probe(args.hand_filter)
        if args.no_diag:
            node.recs.pop(DIAG, None)

        if log_path:
            _prompt(f">>> LOG {log_path}")
        if pose_path:
            _prompt(f">>> POSE {pose_path}")

        _settle_then_record(node, args.settle_sec)

        t0 = time.monotonic()
        while rclpy.ok() and (time.monotonic() - t0) < args.sec:
            rclpy.spin_once(node, timeout_sec=0.05)
        dur = time.monotonic() - t0
        _prompt(">>> RECORDING END")
        report(
            node.recs,
            dur,
            args.csv,
            eps_m=args.eps_m,
            eps_rad=args.eps_rad,
            scene=args.scene,
            resample_hz=args.resample_hz,
            settle_sec=args.settle_sec,
            tag=args.tag,
        )
        if pose_path:
            write_pose_npz(
                node.recs,
                pose_path,
                scene=args.scene,
                tag=args.tag,
                settle_sec=args.settle_sec,
                dur=dur,
            )
        node.destroy_node()
        rclpy.shutdown()
        return 0

    if tee_cm is not None:
        with tee_cm:
            return _run()
    return _run()


if __name__ == "__main__":
    sys.exit(main())