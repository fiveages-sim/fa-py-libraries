#!/usr/bin/env python3
"""One-shot teleop probe (read-only; never touches xrobotoolkit_sdk).

Measures on the subscriber side with a monotonic clock, per topic:
    arrival Hz / unique Hz / repeat% / dt p50,p95,p99,max / stall count
and the cross-topic numbers that separate "source" from "software":
    pico unique frame rate, left_ee:pico ratio, PC->ROS latency,
    isActive duty & transitions.
For hand commands it also prints |dq| and |d2q| (what the PD actually feels).

Usage
    python3 scripts/teleop_probe.py --sec 20
    python3 scripts/teleop_probe.py --sec 20 --csv /tmp/probe.csv
    python3 scripts/teleop_probe.py --sec 20 --no-diag

Run WITH the teleop stack up, same ROS env (RMW / DOMAIN_ID).
Never start a second XRT client: this only subscribes.
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int64MultiArray

POSE_TOPICS = ("/teleop/head_pose", "/teleop/left_ee_pose", "/teleop/right_ee_pose")
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


def _pct(a: np.ndarray, q: float) -> float:
    return float(np.percentile(a, q)) if a.size else float("nan")


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


class Probe(Node):
    def __init__(self, hand_filter: str) -> None:
        super().__init__("teleop_probe")
        self.recs: dict[str, Rec] = {}
        self.hand_filter = hand_filter
        # BEST_EFFORT: a slow probe must never back-pressure the publisher queue.
        qos = QoSProfile(depth=4000, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        for n in POSE_TOPICS + Q_TOPICS + JOINT_TOPICS + (DIAG,):
            self.recs[n] = Rec()

        for n in POSE_TOPICS:
            self.create_subscription(Pose, n, self._pose_cb(n), qos)
        for n in Q_TOPICS:
            self.create_subscription(Float64MultiArray, n, self._q_cb(n), qos)
        for n in JOINT_TOPICS:
            self.create_subscription(JointState, n, self._joint_cb(n), qos)
        self.create_subscription(Int64MultiArray, DIAG, self._diag_cb, qos)

    def _pose_cb(self, name: str):
        def cb(msg: Pose) -> None:
            p, o = msg.position, msg.orientation
            self.recs[name].add(
                (p.x, p.y, p.z, o.x, o.y, o.z, o.w))
        return cb

    def _q_cb(self, name: str):
        def cb(msg: Float64MultiArray) -> None:
            self.recs[name].add(tuple(msg.data))
        return cb

    def _joint_cb(self, name: str):
        def cb(msg: JointState) -> None:
            idx = [i for i, jn in enumerate(msg.name) if self.hand_filter in jn]
            if not idx:
                return
            self.recs[name].add(tuple(round(float(msg.position[i]), 9) for i in idx))
        return cb

    def _diag_cb(self, msg: Int64MultiArray) -> None:
        d = tuple(int(v) for v in msg.data)
        self.recs[DIAG].add((d[0],), d)


def report(recs: dict[str, Rec], dur: float, csv_path: str) -> None:
    W = 44
    print()
    print("=" * 100)
    print(f"TELEOP PROBE   dur={dur:.2f}s   (subscriber-side, monotonic clock)")
    print("=" * 100)
    print(f"{'topic':<{W}}{'arriv':>8}{'unique':>8}{'rep%':>7}"
          f"{'p50':>8}{'p95':>8}{'p99':>8}{'max':>8}{'stall':>7}")
    print("-" * 100)

    ee_rate = float("nan")
    for name, r in recs.items():
        if not r.t:
            print(f"{name:<{W}}{'-- no data --':>47}")
            continue
        t = np.asarray(r.t)
        dts = np.diff(t) * 1e3
        arr = max(len(t) - 1, 0) / max(dur, 1e-9)
        uniq = sum(1 for i in range(1, len(r.sig)) if r.sig[i] != r.sig[i - 1])
        rep = 100.0 * (1.0 - uniq / max(len(r.sig) - 1, 1))
        med = float(np.median(dts)) if dts.size else float("nan")
        stall = int(np.sum(dts > 3.0 * med)) if dts.size and med > 0 else 0
        print(f"{name:<{W}}{arr:>8.1f}{uniq/max(dur,1e-9):>8.1f}{rep:>6.1f}%"
              f"{med:>8.2f}{_pct(dts,95):>8.2f}{_pct(dts,99):>8.2f}"
              f"{dts.max():>8.2f}{stall:>7}")
        if name == "/teleop/left_ee_pose":
            ee_rate = uniq / max(dur, 1e-9)

    d = recs.get(DIAG)
    if d is not None and d.data:
        pico = np.array([x[0] for x in d.data], dtype=np.int64)
        pc = np.array([x[1] for x in d.data], dtype=np.int64)
        ros = np.array([x[2] for x in d.data], dtype=np.int64)
        la = np.array([x[3] for x in d.data], dtype=np.int64)
        ra = np.array([x[4] for x in d.data], dtype=np.int64)
        uniq_pico = int(np.sum(pico[1:] != pico[:-1])) if pico.size > 1 else 0
        pico_hz = uniq_pico / max(dur, 1e-9)
        lat = (ros - pc) / 1e6
        print("-" * 100)
        print(f"pico unique frames     : {pico_hz:8.1f} Hz   <- 源端新鲜度")
        print(f"PC->ROS latency        : p50 {_pct(lat,50):5.2f} ms  "
              f"p99 {_pct(lat,99):5.2f} ms  max {lat.max():5.2f} ms")
        print(f"left_ee unique / pico  : {ee_rate / max(pico_hz,1e-9):8.3f}   "
              f"(≈1.0 = 1:1 pass-through)")
        for tag, a in (("left", la), ("right", ra)):
            tr = int(np.sum(a[1:] != a[:-1])) if a.size > 1 else 0
            print(f"isActive {tag:<12}: duty={a.mean():.2f}  "
                  f"transitions={tr:3d} / {dur:.0f}s   <- 冻结/恢复次数")
        # Optional tail: palm_u (area_m2*1e6), orient_src codes
        if d.data and len(d.data[0]) >= 9:
            lp = np.array([x[5] for x in d.data], dtype=np.float64)
            rp = np.array([x[6] for x in d.data], dtype=np.float64)
            lo = np.array([x[7] for x in d.data], dtype=np.int64)
            ro = np.array([x[8] for x in d.data], dtype=np.int64)
            _ORIENT = {
                0: "none", 1: "sdk_quat", 2: "xyz_hold_r", 3: "xyz_seed",
                4: "reject", 5: "error", 6: "position_mode",
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
                f"p50={_pct(lp,50):.1f}/{_pct(rp,50):.1f}  "
                f"p10={_pct(lp,10):.1f}/{_pct(rp,10):.1f}  "
                f"(≥10 ≈ real palm; with isActive=0 → false inactive)"
            )
            print(f"orient_src left      : {_orient_hist(lo)}")
            print(f"orient_src right     : {_orient_hist(ro)}")

    print("-" * 100)
    for name in Q_TOPICS:
        r = recs.get(name)
        if r is None or len(r.sig) < 3:
            continue
        arr = np.array([np.asarray(s, dtype=float) for s in r.sig])
        same = np.all(np.isclose(arr[1:], arr[:-1], atol=1e-6), axis=1)
        mdq = np.abs(np.diff(arr, axis=0)).max(axis=1)
        dd = np.abs(np.diff(np.diff(arr, axis=0), axis=0)).max(axis=1)
        print(f"{name.split('/')[1]:<22} n={len(arr):5d}  "
              f"frozen={100.0*same.mean():5.1f}%  "
              f"|dq| p50={_pct(mdq,50):.4f} p99={_pct(mdq,99):.4f}  "
              f"|d2q| p99={_pct(dd,99):.4f} rad")

    for name in JOINT_TOPICS:
        r = recs.get(name)
        if r is None or len(r.sig) < 3:
            continue
        arr = np.array([np.asarray(s, dtype=float) for s in r.sig])
        mdq = np.abs(np.diff(arr, axis=0)).max(axis=1)
        print(f"{name:<22} n={len(arr):5d}  max|dq| p99={_pct(mdq,99):.4f}  "
              f"max={mdq.max():.4f} rad   <- driver 实际执行")
    print("=" * 100)

    if csv_path:
        with open(csv_path, "w", encoding="utf-8") as f:
            f.write("topic,idx,t,dt_ms\n")
            for name, r in recs.items():
                t = np.asarray(r.t)
                for i, v in enumerate(np.diff(t) * 1e3):
                    f.write(f"{name},{i},{t[i+1]:.6f},{v:.4f}\n")
        print(f"csv -> {csv_path}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sec", type=float, default=20.0)
    ap.add_argument("--csv", default="")
    ap.add_argument("--hand-filter", default="hand")
    ap.add_argument("--no-diag", action="store_true")
    args = ap.parse_args(argv)

    rclpy.init(args=[])
    node = Probe(args.hand_filter)
    if args.no_diag:
        node.recs.pop(DIAG, None)
    t0 = time.monotonic()
    while rclpy.ok() and (time.monotonic() - t0) < args.sec:
        rclpy.spin_once(node, timeout_sec=0.05)
    dur = time.monotonic() - t0
    report(node.recs, dur, args.csv)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
