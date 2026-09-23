"""ROS2 node: one SDK client → left + right hand joint commands."""

from __future__ import annotations

import time

import numpy as np
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, Int32

from xr_hand_retarget.config import HandRuntimeConfig
from xr_hand_retarget.local_profile import HandPlan, SideSpec, format_resolved, print_plan
from xr_hand_retarget.pipeline import (
    SidePipeline,
    StreamMoveJSession,
    command_topic,
    wire_j2_lock_sub,
)

_FSM_HOME = 1
_FSM_HOLD = 2
_FSM_OCS2 = 3
_CONTROLLER_STATE_TOPIC = "/teleop/controller_state"
_EVENT_MIRROR = 7


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
        frame_source=None,
        use_timer: bool = True,
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
        self._frame_source = frame_source
        self._fsm = 0
        self._last_heartbeat = 0.0
        self._hold_heartbeat_dt = 0.2
        # Match VRInputHandler case 7: left optical → right hand controller and vice versa.
        self._mirror = False

        self._left = SidePipeline("left", cfg, retargeting_type, xrt=xrt)
        self._right = SidePipeline("right", right_cfg, right_retargeting, xrt=xrt)
        self._xrt = xrt
        suffix = cfg.topic_suffix
        self._left_topic = command_topic(left_controller, suffix)
        self._right_topic = command_topic(right_controller, suffix)
        self._pub_left = self.create_publisher(Float64MultiArray, self._left_topic, 10)
        self._pub_right = self.create_publisher(Float64MultiArray, self._right_topic, 10)
        wire_j2_lock_sub(self, {"left": self._left, "right": self._right})
        self._stream_movej = None
        if cfg.stream_movej or right_cfg.stream_movej:
            # Async: do not spin_until_future_complete in __init__ (no executor yet).
            self._stream_movej = StreamMoveJSession(
                self, [left_controller, right_controller]
            )

        fsm_qos = QoSProfile(depth=1)
        fsm_qos.reliability = ReliabilityPolicy.RELIABLE
        fsm_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Int32, "/fsm_state", self._on_fsm, fsm_qos)
        self.create_subscription(
            Int32, _CONTROLLER_STATE_TOPIC, self._on_controller_state, 10
        )

        if auto_movej and (fsm_topic or cfg.fsm_topic):
            topic = fsm_topic or cfg.fsm_topic
            fsm_pub = self.create_publisher(Int32, topic, 10)
            time.sleep(0.3)
            msg = Int32()
            msg.data = int(cfg.fsm_command)
            fsm_pub.publish(msg)
            self.get_logger().info(f"Sent FSM {cfg.fsm_command} on {topic}")

        rate = max(float(cfg.rate_hz), float(right_cfg.rate_hz))
        self._rate_hz = max(rate, 1.0)
        self._min_step_dt = 1.0 / self._rate_hz
        self._last_step = 0.0
        self._use_timer = use_timer
        self._dbg_n = 0
        if use_timer:
            self.create_timer(self._min_step_dt, self._on_timer)
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
            f"rate={self._rate_hz}Hz left={self._left_topic} ({cfg.dof}) "
            f"right={self._right_topic} ({right_cfg.dof})"
            f"{'' if self._use_timer else ' drive=ee-frame'}"
        )

    def _on_fsm(self, msg: Int32) -> None:
        prev = self._fsm
        self._fsm = int(msg.data)
        if prev == self._fsm:
            return
        # #region agent log
        try:
            import json as _json

            with open("/home/fiveages/fa-py-libraries/.cursor/debug-fc4a89.log", "a") as _f:
                _f.write(
                    _json.dumps(
                        {
                            "sessionId": "fc4a89",
                            "hypothesisId": "R3,R4,R5",
                            "location": "dual_node.py:_on_fsm",
                            "message": "fsm transition",
                            "data": {"prev": prev, "fsm": self._fsm},
                            "timestamp": int(time.time() * 1000),
                        }
                    )
                    + "\n"
                )
        except Exception:
            pass
        # #endregion
        if prev == _FSM_OCS2 and self._fsm != _FSM_OCS2:
            self._last_sent = {"left": None, "right": None}
        if self._fsm == _FSM_OCS2 and self._stream_movej is not None:
            self._stream_movej.kick(force=True)

    def _on_controller_state(self, msg: Int32) -> None:
        """Toggle hand L/R publish swap with VRInputHandler mirror (event 7)."""
        if int(msg.data) != _EVENT_MIRROR:
            return
        self._mirror = not self._mirror
        self.get_logger().warn(
            f"Hand mirror={'ON (L optical→R ctrl, R→L)' if self._mirror else 'OFF'}"
        )

    def _pub_for_optical_side(self, side: str):
        """Optical left/right → controller topic (swapped when mirror)."""
        if self._mirror:
            return self._pub_right if side == "left" else self._pub_left
        return self._pub_left if side == "left" else self._pub_right

    def _live(self) -> bool:
        return self._fsm == _FSM_OCS2

    def _stream_ready(self) -> bool:
        return self._stream_movej is None or self._stream_movej.ready

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

    def _republish_last(self, now: float) -> None:
        """HOLD only. HOME interpolates to the nest; do not yank q back."""
        if self._fsm != _FSM_HOLD:
            return
        if now - self._last_heartbeat < self._hold_heartbeat_dt:
            return
        self._last_heartbeat = now
        for side in ("left", "right"):
            prev = self._last_sent[side]
            if prev is None:
                continue
            msg = Float64MultiArray()
            msg.data = [float(x) for x in prev.tolist()]
            self._pub_for_optical_side(side).publish(msg)

    def _skip_teleop(self, now: float, src: str) -> None:
        self._dbg_n += 1
        # #region agent log
        if self._dbg_n <= 4 or self._dbg_n % 50 == 0:
            try:
                import json as _json

                with open("/home/fiveages/fa-py-libraries/.cursor/debug-fc4a89.log", "a") as _f:
                    _f.write(
                        _json.dumps(
                            {
                                "sessionId": "fc4a89",
                                "hypothesisId": "R2,R3",
                                "location": f"dual_node.py:{src}",
                                "message": "skip teleop",
                                "data": {
                                    "n": self._dbg_n,
                                    "fsm": self._fsm,
                                    "stream_ready": self._stream_ready(),
                                    "src": src,
                                },
                                "timestamp": int(time.time() * 1000),
                            }
                        )
                        + "\n"
                    )
            except Exception:
                pass
        # #endregion
        self._republish_last(now)

    def step_once(self) -> None:
        """Retarget from the latest cached OpenXR 26 (same tick as wrist EE)."""
        now = time.monotonic()
        if not self._live() or not self._stream_ready():
            self._skip_teleop(now, "step_once")
            return
        if now - self._last_step < self._min_step_dt:
            return
        self._last_step = now
        try:
            self._on_timer()
        except Exception as exc:
            self.get_logger().error(f"hand step failed: {exc}")

    def _on_timer(self):
        if not self._live() or not self._stream_ready():
            self._skip_teleop(time.monotonic(), "_on_timer")
            return
        self._dbg_n += 1
        left_f = self._read_frame("left")
        right_f = self._read_frame("right")
        left = self._left.step(left_f) if left_f is not None else None
        right = self._right.step(right_f) if right_f is not None else None
        if left is None and right is None:
            return
        if left is not None:
            if left.held:
                self._held["left"] += 1
            self._maybe_publish("left", self._pub_for_optical_side("left"), left)
        if right is not None:
            if right.held:
                self._held["right"] += 1
            self._maybe_publish("right", self._pub_for_optical_side("right"), right)

        now = time.time()
        if self._print_every > 0 and (now - self._last_print) >= self._print_every:
            self._last_print = now
            src = "cache" if self._frame_source is not None else "sdk"
            left_tag = f"{self._cfg.backend}/{self._left.retargeting_type}"
            right_tag = f"{self._right_cfg.backend}/{self._right.retargeting_type}"
            tag = left_tag if left_tag == right_tag else f"L={left_tag} R={right_tag}"
            ltxt = _fmt_side("L", left, self._held["left"]) if left is not None else "L none"
            rtxt = _fmt_side("R", right, self._held["right"]) if right is not None else "R none"
            self.get_logger().info(f"[{tag} {src}] {ltxt} | {rtxt}")

    def _read_frame(self, side: str):
        from xr_hand_retarget.sources.frame import HandFrame
        from xr_hand_retarget.sources.xrt import read_hand_frame

        if self._frame_source is not None:
            sample = self._frame_source(side)
            if sample is None:
                return None
            active, joints = sample
            # #region agent log
            if self._dbg_n <= 8 or self._dbg_n % 20 == 0:
                try:
                    import json as _json
                    j = np.asarray(joints)
                    with open("/home/fiveages/fa-py-libraries/.cursor/debug-fc4a89.log", "a") as _f:
                        _f.write(_json.dumps({"sessionId":"fc4a89","hypothesisId":"H1,H5","location":"dual_node.py:_read_frame","message":"cached joints","data":{"side":side,"n":self._dbg_n,"active":int(active),"jshape":list(j.shape),"xyz_abs":float(np.abs(j[:,:3]).max()) if j.size else 0.0},"timestamp":int(time.time()*1000)})+"\n")
                except Exception:
                    pass
            # #endregion
            return HandFrame(side=side, active=int(active), joints26=joints)
        return read_hand_frame(self._xrt, side)


def _fmt_side(prefix: str, step, held_n: int) -> str:
    reason = getattr(step, "reason", None) or ("hold" if step.held else "ok")
    return (
        f"{prefix} active={step.active} {reason} held={held_n} "
        f"xyz={step.xyz_abs:.3f} q={_fmt_q(step.q)}"
    )


def _fmt_q(q: np.ndarray) -> str:
    q = np.asarray(q, dtype=np.float64).reshape(-1)
    return str(np.round(q, 3).tolist())
