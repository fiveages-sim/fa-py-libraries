#!/usr/bin/env python3
"""ROS2 node: XRoboToolkit hand → configured dexterous-hand joints.

Switch hands by pointing --config / FA_HAND_CONFIG at a different YAML
(backend: xhand1 | wuji | o6 | l6 | o7). Do not run two SDK clients against PC Service.
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
        help="Hand YAML (default: FA_HAND_CONFIG or configs/wuji_hand2.yaml)",
    )
    p.add_argument("--side", choices=("left", "right", "full"), default="left")
    p.add_argument(
        "--retargeting",
        default=None,
        help="Override yaml type: XHand1 curl|thumb_ik|vector|dexpilot; Wuji curl|official; O6/L6 curl|nest",
    )
    p.add_argument("--rate", type=float, default=None, help="Override publish rate Hz")
    p.add_argument(
        "--controller",
        default=None,
        help=(
            "Override controller for --side left|right. Default: match a live "
            "fa_w2 topic (left_hand_controller / hand_joint_controller / "
            "hand_controller), else yaml ros.controller"
        ),
    )
    p.add_argument("--left-controller", default="left_hand_controller")
    p.add_argument("--right-controller", default="right_hand_controller")
    p.add_argument("--fsm-topic", default=None)
    p.add_argument("--no-fsm", action="store_true")
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
    p.add_argument(
        "--fingers",
        default="all",
        help="Isolate mapping: all | comma list thumb,index,middle,ring,pinky",
    )
    p.add_argument(
        "--no-motion-safety",
        action="store_true",
        help="Disable URDF position / velocity / acceleration filters",
    )
    p.add_argument(
        "--no-attract",
        action="store_true",
        help="Disable pinch attract so curl interpolation is visible",
    )
    return p.parse_args(argv)


_FINGER_SLICES_20 = {
    "thumb": slice(0, 4),
    "index": slice(4, 8),
    "middle": slice(8, 12),
    "ring": slice(12, 16),
    "pinky": slice(16, 20),
}
_FINGER_SLICES_12 = {
    "thumb": slice(0, 3),
    "index": slice(3, 6),
    "middle": slice(6, 8),
    "ring": slice(8, 10),
    "pinky": slice(10, 12),
}
_FINGER_SLICES_6 = {
    "thumb": slice(0, 2),
    "index": slice(2, 3),
    "middle": slice(3, 4),
    "ring": slice(4, 5),
    "pinky": slice(5, 6),
}


_FINGER_SLICES_7 = {
    "thumb": slice(0, 3),
    "index": slice(3, 4),
    "middle": slice(4, 5),
    "ring": slice(5, 6),
    "pinky": slice(6, 7),
}


def _finger_slices(cfg) -> dict[str, slice]:
    if cfg.backend == "o7" or int(cfg.dof) == 7:
        return _FINGER_SLICES_7
    if cfg.backend in ("o6", "l6") or int(cfg.dof) == 6:
        return _FINGER_SLICES_6
    if cfg.backend == "xhand1" or int(cfg.dof) == 12:
        return _FINGER_SLICES_12
    return _FINGER_SLICES_20


def _parse_fingers(spec: str, slices: dict[str, slice]) -> tuple[str, ...]:
    raw = (spec or "all").strip().lower()
    if raw in ("", "all", "*"):
        return tuple(slices)
    names = []
    for part in raw.split(","):
        name = part.strip().replace("index_finger", "index").replace("little", "pinky")
        if name not in slices:
            raise SystemExit(
                f"--fingers unknown {part!r}; use all or thumb,index,middle,ring,pinky"
            )
        names.append(name)
    return tuple(names)


def _mask_q(q: np.ndarray, fingers: tuple[str, ...], slices: dict[str, slice]) -> np.ndarray:
    if len(fingers) == len(slices):
        return q
    out = np.zeros_like(q)
    for name in fingers:
        sl = slices[name]
        n = min(sl.stop, out.shape[0])
        out[sl.start : n] = q[sl.start : n]
    return out


def _fmt_q(q: np.ndarray, slices: dict[str, slice]) -> str:
    parts = []
    for name, sl in slices.items():
        chunk = np.round(q[sl.start : min(sl.stop, q.shape[0])], 3).tolist()
        parts.append(f"{name[0]}={chunk}")
    return " ".join(parts)


def _controller_candidates(side: str) -> tuple[str, ...]:
    """fa_w2 names first: dual hands2, then standalone hand2, then generic."""
    if side == "left":
        return (
            "left_hand_controller",
            "hand_joint_controller",
            "hand_controller",
        )
    return (
        "right_hand_controller",
        "hand_joint_controller",
        "hand_controller",
    )


def _discover_hand_controller(node, side: str, suffix: str, fallback: str) -> str:
    """Pick the controller that already has target_joint_position on the graph."""
    import rclpy

    suffix = suffix.strip().strip("/")
    deadline = time.time() + 2.5
    topic_names: list[str] = []
    while time.time() < deadline:
        topic_names = [name for name, _tys in node.get_topic_names_and_types()]
        if any("hand" in name and name.endswith(f"/{suffix}") for name in topic_names):
            break
        rclpy.spin_once(node, timeout_sec=0.15)

    for name in _controller_candidates(side):
        topic = f"/{name}/{suffix}"
        if topic in topic_names:
            node.get_logger().info(f"Matched live controller topic {topic}")
            return name

    seen = sorted(
        n for n in topic_names if "hand" in n and suffix in n
    ) or sorted(n for n in topic_names if "controller" in n)
    node.get_logger().warn(
        f"No live hand controller for side={side}; using /{fallback}/{suffix}. "
        f"seen={seen[:12]}"
    )
    return fallback


def _try_stream_movej(node, controller: str) -> None:
    """Session-only: BJC MOVEJ linear interpolates every new target.

    Official teleop sends q every frame. Do not edit fa_w2 yaml; set params
    on the running controller (updateParam reads them on each target).
    """
    name = controller.strip().strip("/")
    try:
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters
        import rclpy
    except ImportError:
        return
    client = node.create_client(SetParameters, f"/{name}/set_parameters")
    if not client.wait_for_service(timeout_sec=1.5):
        node.get_logger().warn(
            f"[{name}] 遥操需要 MOVEJ 直通（官方每帧发 q）。请另开终端:\n"
            f"  ros2 param set /{name} movej_interpolation_type none\n"
            f"  ros2 param set /{name} movej_duration 0.02"
        )
        return
    req = SetParameters.Request()
    p_type = Parameter()
    p_type.name = "movej_interpolation_type"
    p_type.value = ParameterValue(
        type=ParameterType.PARAMETER_STRING, string_value="none"
    )
    p_dur = Parameter()
    p_dur.name = "movej_duration"
    p_dur.value = ParameterValue(
        type=ParameterType.PARAMETER_DOUBLE, double_value=0.02
    )
    req.parameters = [p_type, p_dur]
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
    if fut.done() and fut.result() is not None:
        node.get_logger().info(
            f"[{name}] session MOVEJ stream: interpolation=none (fa_w2 yaml unchanged)"
        )
    else:
        node.get_logger().warn(
            f"[{name}] set_parameters timed out; run ros2 param set … interpolation_type none"
        )


def main(argv=None):
    args = _parse_args(argv)

    try:
        import xrobotoolkit_sdk as xrt
    except ImportError as exc:
        print(
            "xrobotoolkit_sdk not found. Install via:\n"
            "  ./init.sh install-xrobotoolkit",
            file=sys.stderr,
        )
        raise SystemExit(1) from exc

    from xr_hand_retarget.config import load_runtime_config
    from xr_hand_retarget.pipeline import SidePipeline, command_topic, wire_j2_lock_sub

    cfg = load_runtime_config(args.config)
    if args.rate is not None:
        cfg.rate_hz = args.rate
    if args.controller:
        cfg.controller = args.controller
    if args.fsm_topic:
        cfg.fsm_topic = args.fsm_topic
    if args.no_fsm:
        cfg.auto_movej = False
    if getattr(args, "no_motion_safety", False) and cfg.motion is not None:
        cfg.motion.enabled = False
    if getattr(args, "no_attract", False) and cfg.o6_curl is not None:
        cfg.o6_curl.attract_enabled = False

    retargeting_type = args.retargeting
    sides = ("left", "right") if args.side == "full" else (args.side,)
    auto_movej = bool(cfg.auto_movej) and args.side != "full" and not args.no_fsm
    slices = _finger_slices(cfg)
    fingers = _parse_fingers(args.fingers, slices)

    if args.dry_run:
        print(
            "WARNING: --dry-run does NOT publish to ROS.",
            flush=True,
        )

    if cfg.backend == "wuji":
        rtype_guess = (retargeting_type or cfg.retargeting_type or "curl").lower()
        if rtype_guess not in ("curl", "curl_calib", "calib"):
            from xr_hand_retarget.backends.wuji import preflight_wuji

            for side in sides:
                preflight_wuji(cfg, side)

    print("init xrobotoolkit_sdk...", flush=True)
    xrt.init()
    time.sleep(0.8)
    print("sdk ok", flush=True)

    pipelines = {
        side: SidePipeline(side, cfg, retargeting_type, xrt=xrt) for side in sides
    }
    rtype = next(iter(pipelines.values())).retargeting_type

    if args.side == "full":
        topics = {
            "left": command_topic(args.left_controller, cfg.topic_suffix),
            "right": command_topic(args.right_controller, cfg.topic_suffix),
        }
    else:
        topics = {args.side: cfg.command_topic}

    node = None
    pubs = {}
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
        node = Node("xr_hand_retarget")
        if args.side != "full" and not args.controller:
            cfg.controller = _discover_hand_controller(
                node, args.side, cfg.topic_suffix, cfg.controller
            )
            topics = {args.side: cfg.command_topic}
        for side, topic in topics.items():
            pubs[side] = node.create_publisher(Float64MultiArray, topic, 10)
            node.get_logger().info(
                f"Publishing {topic} ({cfg.dof} joints: {cfg.joint_names})"
            )
        if auto_movej and cfg.fsm_topic:
            fsm_pub = node.create_publisher(Int32, cfg.fsm_topic, 10)
            time.sleep(0.3)
            msg = Int32()
            msg.data = int(cfg.fsm_command)
            fsm_pub.publish(msg)
            node.get_logger().info(
                f"Sent FSM {cfg.fsm_command} on {cfg.fsm_topic}; waiting for MOVEJ"
            )
            time.sleep(0.5)
        if cfg.stream_movej:
            for topic in topics.values():
                ctrl = topic.strip("/").split("/")[0]
                _try_stream_movej(node, ctrl)
        wire_j2_lock_sub(node, pipelines)

    print(
        f"config={cfg.path} backend={cfg.backend} hand_id={cfg.hand_id} "
        f"side={args.side} type={rtype} dof={cfg.dof} topics={topics} "
        f"rate={cfg.rate_hz}Hz fingers={','.join(fingers)}",
        flush=True,
    )

    period = 1.0 / max(cfg.rate_hz, 1.0)
    last_print = 0.0
    n = 0
    held = {side: 0 for side in sides}
    last_sent = {side: None for side in sides}
    last_force = {side: 0.0 for side in sides}

    try:
        while True:
            t0 = time.time()
            from xr_hand_retarget.sources.xrt import read_hand_frame

            results = {
                side: pipelines[side].step(read_hand_frame(xrt, side)) for side in sides
            }
            for side, step in results.items():
                step.q = _mask_q(step.q, fingers, slices)
                if step.held:
                    held[side] += 1

            if args.dry_run:
                if args.print_every > 0 and (t0 - last_print) >= args.print_every:
                    last_print = t0
                    parts = []
                    for side, step in results.items():
                        reason = getattr(step, "reason", None) or (
                            "hold" if step.held else "ok"
                        )
                        parts.append(
                            f"{side[0].upper()} active={step.active} xyz={step.xyz_abs:.3f} "
                            f"reason={reason} held={held[side]} sat={getattr(step, 'sat', -1)} {_fmt_q(step.q, slices)}"
                        )
                    print(" | ".join(parts), flush=True)
            else:
                from std_msgs.msg import Float64MultiArray

                for side, step in results.items():
                    prev = last_sent[side]
                    skip = (
                        step.held
                        and prev is not None
                        and np.allclose(step.q, prev, atol=1e-6, rtol=0.0)
                    )
                    if skip and (t0 - last_force[side]) >= 1.0:
                        skip = False
                    if skip:
                        continue
                    msg = Float64MultiArray()
                    msg.data = [float(x) for x in step.q.tolist()]
                    pubs[side].publish(msg)
                    last_sent[side] = step.q.copy()
                    last_force[side] = t0
                if args.print_every > 0 and (t0 - last_print) >= args.print_every:
                    last_print = t0
                    parts = []
                    for side, step in results.items():
                        reason = getattr(step, "reason", None) or (
                            "hold" if step.held else "ok"
                        )
                        parts.append(
                            f"{side[0].upper()} active={step.active} xyz={step.xyz_abs:.3f} "
                            f"{reason} held={held[side]} sat={getattr(step, 'sat', -1)} {_fmt_q(step.q, slices)}"
                        )
                    node.get_logger().info(" | ".join(parts))
                import rclpy as _rclpy

                _rclpy.spin_once(node, timeout_sec=0.0)

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
