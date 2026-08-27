#!/usr/bin/env python3
"""ROS2 node: XRoboToolkit hand → /hand_joint_controller/target_joint_position.

Phase-1: hybrid remap (tip thumb/index + bone chain mid/ring/pinky) + safety.
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np


def _parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--config",
        default=None,
        help="Hand YAML (default: package configs/xhand1.yaml)",
    )
    p.add_argument("--side", choices=("left", "right"), default="right")
    p.add_argument("--rate", type=float, default=None, help="Override publish rate Hz")
    p.add_argument(
        "--controller",
        default=None,
        help="Override controller name (default from config)",
    )
    p.add_argument(
        "--fsm-topic",
        default=None,
        help="Override FSM topic (default from config)",
    )
    p.add_argument("--no-fsm", action="store_true", help="Do not auto-enter MOVEJ")
    p.add_argument(
        "--flexion-scale",
        type=float,
        default=None,
        help="Override flexion gain",
    )
    p.add_argument(
        "--alpha",
        type=float,
        default=None,
        help="Override EMA alpha in (0,1]; 1.0 = off",
    )
    p.add_argument(
        "--max-delta",
        type=float,
        default=None,
        help="Optional per-frame |Δq| cap (rad); 0 disables (default)",
    )
    p.add_argument(
        "--no-soft-collision",
        action="store_true",
        help="Disable URDF-informed soft self-collision projection",
    )
    p.add_argument(
        "--print-every",
        type=float,
        default=1.0,
        help="Seconds between status prints; 0 disables",
    )
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="No ROS; print remapped q only (still needs xrobotoolkit_sdk)",
    )
    return p.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv)

    try:
        import xrobotoolkit_sdk as xrt
    except ImportError as exc:
        print(
            "xrobotoolkit_sdk not found. Install via:\n"
            "  ./init.sh install-xrobotoolkit\n"
            "  or: cd vr_pose_publisher && bash setup_xrobotoolkit.sh",
            file=sys.stderr,
        )
        raise SystemExit(1) from exc

    from xr_xhand1_retarget.config import load_hand_config
    from xr_xhand1_retarget.remap import openxr26_to_xhand1_q
    from xr_xhand1_retarget.safety import CommandSafety, pose_is_zero

    cfg = load_hand_config(args.config)
    if args.controller:
        cfg.controller = args.controller
    if args.fsm_topic:
        cfg.fsm_topic = args.fsm_topic
    if args.flexion_scale is not None:
        cfg.flexion_scale = args.flexion_scale
    if args.alpha is not None:
        cfg.alpha = args.alpha
    if args.max_delta is not None:
        cfg.max_delta_rad = args.max_delta
    if args.rate is not None:
        cfg.rate_hz = args.rate
    if args.no_fsm:
        cfg.auto_movej = False
    if args.no_soft_collision:
        cfg.soft_collision = False

    side = args.side
    get_active = (
        xrt.get_left_hand_is_active if side == "left" else xrt.get_right_hand_is_active
    )
    get_hand = (
        xrt.get_left_hand_tracking_state
        if side == "left"
        else xrt.get_right_hand_tracking_state
    )

    safety = CommandSafety(
        cfg.limits,
        alpha=cfg.alpha,
        soft_collision=cfg.soft_collision,
        max_delta_rad=cfg.max_delta_rad,
        jump_reject_rad=cfg.jump_reject_rad,
    )

    if args.dry_run:
        print(
            "WARNING: --dry-run does NOT publish to ROS; RViz / hand controller will not move.",
            flush=True,
        )

    print(
        f"config={cfg.hand_id} side={side} topic={cfg.command_topic} "
        f"rate={cfg.rate_hz}Hz alpha={cfg.alpha} soft_collision={cfg.soft_collision}",
        flush=True,
    )
    print("init xrobotoolkit_sdk...", flush=True)
    xrt.init()
    # PC Service streams hand landmarks shortly after connect; active=0 is normal here.
    time.sleep(0.8)
    print("sdk ok", flush=True)

    node = None
    pub = None
    if not args.dry_run:
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import Float64MultiArray, Int32
        except ImportError as exc:
            xrt.close()
            print("rclpy not available; use --dry-run or source ROS2", file=sys.stderr)
            raise SystemExit(1) from exc

        rclpy.init()
        node = Node("xr_xhand1_retarget")
        pub = node.create_publisher(Float64MultiArray, cfg.command_topic, 10)
        node.get_logger().info(
            f"Publishing {cfg.command_topic} ({cfg.dof} joints: {cfg.joint_names})"
        )
        if cfg.auto_movej and cfg.fsm_topic:
            fsm_pub = node.create_publisher(Int32, cfg.fsm_topic, 10)
            time.sleep(0.3)
            msg = Int32()
            msg.data = 3  # MOVEJ
            fsm_pub.publish(msg)
            node.get_logger().info(f"Sent MOVEJ (3) on {cfg.fsm_topic}")

    period = 1.0 / max(cfg.rate_hz, 1.0)
    last_print = 0.0
    n = 0
    held = 0

    try:
        while True:
            t0 = time.time()
            active = int(get_active())
            raw = np.asarray(get_hand(), dtype=np.float64)
            if raw.ndim != 2 or raw.shape[0] != 26:
                raw = np.asarray(raw, dtype=np.float64).reshape(26, -1)

            invalid = (cfg.hold_on_inactive and active == 0) or (
                cfg.hold_on_zero_pose and pose_is_zero(raw)
            )
            if invalid:
                q_cmd = safety.hold()
                held += 1
            else:
                q_raw = openxr26_to_xhand1_q(
                    raw,
                    side=side,
                    flexion_scale=cfg.flexion_scale,
                    limits=cfg.limits,
                    finger_chain=cfg.finger_chain,
                    thumb_tip=cfg.thumb_tip,
                    index_abduction=cfg.index_abduction,
                )
                q_cmd = safety.step(q_raw)

            xyz_abs = float(np.abs(raw[:, :3]).max()) if raw.size else 0.0
            if args.dry_run:
                if args.print_every > 0 and (t0 - last_print) >= args.print_every:
                    last_print = t0
                    reason = "ok"
                    if invalid:
                        if active == 0 and cfg.hold_on_inactive:
                            reason = "inactive"
                        elif cfg.hold_on_zero_pose and pose_is_zero(raw):
                            reason = "zero_pose"
                        else:
                            reason = "hold"
                    print(
                        f"active={active} xyz_absmax={xyz_abs:.3f} reason={reason} "
                        f"held={held} thumb={np.round(q_cmd[:3], 3).tolist()} q="
                        + np.array2string(q_cmd, precision=3, suppress_small=True),
                        flush=True,
                    )
            else:
                msg = Float64MultiArray()
                msg.data = [float(x) for x in q_cmd.tolist()]
                pub.publish(msg)
                if args.print_every > 0 and (t0 - last_print) >= args.print_every:
                    last_print = t0
                    node.get_logger().info(
                        f"active={active} xyz_absmax={xyz_abs:.3f} held={held} "
                        f"thumb={np.round(q_cmd[:3], 3).tolist()} "
                        f"q[3:6]={np.round(q_cmd[3:6], 3).tolist()}"
                    )
                rclpy.spin_once(node, timeout_sec=0.0)

            n += 1
            dt = time.time() - t0
            time.sleep(max(0.0, period - dt))
    except KeyboardInterrupt:
        print(f"\nstopped after {n} frames (held={held})", flush=True)
    finally:
        try:
            xrt.close()
        except Exception:
            pass
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
            try:
                import rclpy

                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
