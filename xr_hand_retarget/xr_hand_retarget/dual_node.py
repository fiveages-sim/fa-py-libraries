"""ROS2 node: one SDK client → left + right hand joint commands."""

from __future__ import annotations

import time

import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32

from xr_hand_retarget.config import HandRuntimeConfig
from xr_hand_retarget.local_profile import HandPlan, SideSpec, format_resolved, print_plan
from xr_hand_retarget.pipeline import SidePipeline, command_topic, wire_j2_lock_sub


class DualHandRetargetNode(Node):
    """Publish both hands. Does not call xrt.init()/close()."""

    def __init__(
        self,
        xrt,
        cfg: HandRuntimeConfig,
        *,
        left_controller: str,
        right_controller: str,
        retargeting_type: str | None = None,
        auto_movej: bool = False,
        fsm_topic: str | None = None,
        print_every: float = 1.0,
        node_name: str = "xr_hand_retarget",
        right_cfg: HandRuntimeConfig | None = None,
        right_retargeting: str | None = None,
        plan: HandPlan | None = None,
    ):
        super().__init__(node_name)
        right_cfg = right_cfg or cfg
        right_retargeting = (
            right_retargeting if right_retargeting is not None else retargeting_type
        )
        self._cfg = cfg
        self._right_cfg = right_cfg
        self._print_every = float(print_every)
        self._last_print = 0.0
        self._held = {"left": 0, "right": 0}
        self._last_sent = {"left": None, "right": None}

        self._left = SidePipeline("left", cfg, retargeting_type, xrt=xrt)
        self._right = SidePipeline("right", right_cfg, right_retargeting, xrt=xrt)
        self._xrt = xrt
        suffix = cfg.topic_suffix
        self._left_topic = command_topic(left_controller, suffix)
        self._right_topic = command_topic(right_controller, suffix)
        self._pub_left = self.create_publisher(Float64MultiArray, self._left_topic, 10)
        self._pub_right = self.create_publisher(Float64MultiArray, self._right_topic, 10)
        wire_j2_lock_sub(self, {"left": self._left, "right": self._right})

        if auto_movej and (fsm_topic or cfg.fsm_topic):
            topic = fsm_topic or cfg.fsm_topic
            fsm_pub = self.create_publisher(Int32, topic, 10)
            time.sleep(0.3)
            msg = Int32()
            msg.data = int(cfg.fsm_command)
            fsm_pub.publish(msg)
            self.get_logger().info(f"Sent FSM {cfg.fsm_command} on {topic}")

        rate = max(float(cfg.rate_hz), float(right_cfg.rate_hz))
        period = 1.0 / max(rate, 1.0)
        self.create_timer(period, self._on_timer)
        self._rtype = self._left.retargeting_type
        if plan is not None:
            print_plan(plan)
            dump = plan.dump_yaml().rstrip()
        else:
            dump = format_resolved(
                SideSpec(cfg.backend, self._left.retargeting_type),
                SideSpec(right_cfg.backend, self._right.retargeting_type),
            ).rstrip()
            print(f"[local.yaml] source={cfg.path}", flush=True)
            print(dump, flush=True)
        self.get_logger().info(
            f"dual-hand\n{dump}\n"
            f"rate={rate}Hz left={self._left_topic} ({cfg.dof}) "
            f"right={self._right_topic} ({right_cfg.dof})"
        )

    def _maybe_publish(self, side: str, pub, step) -> None:
        prev = self._last_sent[side]
        skip = (
            step.held
            and prev is not None
            and np.allclose(step.q, prev, atol=1e-6, rtol=0.0)
        )
        if skip:
            return
        msg = Float64MultiArray()
        msg.data = [float(x) for x in step.q.tolist()]
        pub.publish(msg)
        self._last_sent[side] = step.q.copy()

    def _on_timer(self):
        from xr_hand_retarget.sources.xrt import read_hand_frame

        left_f = read_hand_frame(self._xrt, "left")
        right_f = read_hand_frame(self._xrt, "right")
        left = self._left.step(left_f)
        right = self._right.step(right_f)
        if left.held:
            self._held["left"] += 1
        if right.held:
            self._held["right"] += 1

        self._maybe_publish("left", self._pub_left, left)
        self._maybe_publish("right", self._pub_right, right)

        now = time.time()
        if self._print_every > 0 and (now - self._last_print) >= self._print_every:
            self._last_print = now
            n = min(3, len(left.q), len(right.q))
            lr = getattr(left, "reason", None) or ("hold" if left.held else "ok")
            rr = getattr(right, "reason", None) or ("hold" if right.held else "ok")
            left_tag = f"{self._cfg.backend}/{self._left.retargeting_type}"
            right_tag = f"{self._right_cfg.backend}/{self._right.retargeting_type}"
            tag = left_tag if left_tag == right_tag else f"L={left_tag} R={right_tag}"
            self.get_logger().info(
                f"[{tag}] "
                f"L active={left.active} {lr} held={self._held['left']} "
                f"xyz={left.xyz_abs:.3f} q0={np.round(left.q[:n], 3).tolist()} | "
                f"R active={right.active} {rr} held={self._held['right']} "
                f"xyz={right.xyz_abs:.3f} q0={np.round(right.q[:n], 3).tolist()}"
            )
