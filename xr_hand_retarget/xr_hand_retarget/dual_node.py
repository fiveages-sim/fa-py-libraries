"""ROS2 node: one SDK client → left + right hand joint commands."""

from __future__ import annotations

import time

import numpy as np
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, Int32

from xr_hand_retarget.config import HandRuntimeConfig
from xr_hand_retarget.control.command_safety import (
    CommandSafetyConfig,
    SideCommandSafety,
    make_fsm_gate,
)
from xr_hand_retarget.local_profile import HandPlan, SideSpec, format_resolved, print_plan
from xr_hand_retarget.mirror_latch import load_mirror, save_mirror
from xr_hand_retarget.pipeline import (
    SidePipeline,
    StreamMoveJSession,
    command_topic,
    wire_j2_lock_sub,
)

_CONTROLLER_STATE_TOPIC = "/teleop/controller_state"
_EVENT_MIRROR = 7


def _side_limits(pipe: SidePipeline, cfg: HandRuntimeConfig) -> np.ndarray:
    if cfg.xhand1 is not None:
        return np.asarray(cfg.xhand1.limits, dtype=float)
    backend = getattr(pipe, "_backend", None)
    for attr in ("limits", "_limits"):
        lim = getattr(backend, attr, None)
        if lim is not None:
            return np.asarray(lim, dtype=float)
    inner = getattr(backend, "_inner", None)
    if inner is not None and hasattr(inner, "limits"):
        return np.asarray(inner.limits, dtype=float)
    n = int(cfg.dof)
    return np.stack([-np.pi * np.ones(n), np.pi * np.ones(n)], axis=1)


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
        self._cmd_cfg: CommandSafetyConfig = cfg.command or CommandSafetyConfig()
        self._gate = make_fsm_gate(self._cmd_cfg)
        self._last_heartbeat = 0.0
        self._hold_heartbeat_dt = 0.2
        self._mirror = load_mirror()

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
            self._stream_movej = StreamMoveJSession(
                self, [left_controller, right_controller]
            )

        self._cmd = {
            "left": SideCommandSafety.create(
                limits=_side_limits(self._left, cfg),
                cfg=self._cmd_cfg,
                backend=cfg.backend,
            ),
            "right": SideCommandSafety.create(
                limits=_side_limits(self._right, right_cfg),
                cfg=self._cmd_cfg,
                backend=right_cfg.backend,
            ),
        }
        self._last_timer_time: float | None = None

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
        vmax_l = self._cmd["left"].filter.max_velocity
        vmax_r = self._cmd["right"].filter.max_velocity
        self.get_logger().info(
            f"dual-hand\n{dump}\n"
            f"rate={self._rate_hz}Hz left={self._left_topic} ({cfg.dof}) "
            f"right={self._right_topic} ({right_cfg.dof})"
            f"{'' if self._use_timer else ' drive=ee-frame'}\n"
            f"command_safety enabled={self._cmd_cfg.enabled} "
            f"publish={self._cmd_cfg.publish_command} "
            f"fsm={list(self._cmd_cfg.allowed_fsm_states)} "
            f"vmax=L{vmax_l}/R{vmax_r} "
            f"lpf={self._cmd_cfg.filter_cutoff_hz}Hz "
            f"timeout={self._cmd_cfg.target_timeout}s"
        )
        if self._mirror:
            self.get_logger().warn(
                "Hand mirror restored ON (same controller still in mirror mode)"
            )

    def _on_fsm(self, msg: Int32) -> None:
        was_open, is_open = self._gate.update(int(msg.data))
        if was_open == is_open and not self._gate.needs_resync:
            return
        if is_open and not was_open:
            for side, pipe in (("left", self._left), ("right", self._right)):
                seed = self._last_sent[side]
                if seed is None:
                    lim = _side_limits(pipe, self._cfg if side == "left" else self._right_cfg)
                    seed = 0.5 * (lim[:, 0] + lim[:, 1])
                self._cmd[side].reset(seed)
            if self._cmd_cfg.require_resync:
                self._gate.mark_resynced()
            self._last_timer_time = time.monotonic()
            self.get_logger().info(
                f"FSM {msg.data}: command gate opened; filter resynced"
            )
            if self._stream_movej is not None:
                self._stream_movej.kick(force=True)
        elif was_open and not is_open:
            self._last_sent = {"left": None, "right": None}
            self.get_logger().info(
                f"FSM {msg.data}: command gate closed; controller commands paused"
            )

    def _on_controller_state(self, msg: Int32) -> None:
        """Toggle hand L/R publish swap with VRInputHandler mirror (event 7)."""
        if int(msg.data) != _EVENT_MIRROR:
            return
        self._mirror = not self._mirror
        save_mirror(self._mirror)
        self.get_logger().warn(
            f"Hand mirror={'ON (L optical→R ctrl, R→L)' if self._mirror else 'OFF'}"
        )

    def _pub_for_optical_side(self, side: str):
        if self._mirror:
            return self._pub_right if side == "left" else self._pub_left
        return self._pub_left if side == "left" else self._pub_right

    def _live(self) -> bool:
        if not self._cmd_cfg.enabled:
            # Legacy: only OCS2 (3).
            return self._gate.state == 3
        return self._gate.can_publish

    def _stream_ready(self) -> bool:
        return self._stream_movej is None or self._stream_movej.ready

    def _maybe_publish(self, side: str, pub, q: np.ndarray) -> None:
        if self._cmd_cfg.enabled and not self._cmd_cfg.publish_command:
            return
        prev = self._last_sent[side]
        if prev is not None and np.allclose(q, prev, atol=1e-6, rtol=0.0):
            return
        msg = Float64MultiArray()
        msg.data = [float(x) for x in np.asarray(q, dtype=float).tolist()]
        pub.publish(msg)
        self._last_sent[side] = np.asarray(q, dtype=float).copy()

    def _republish_last(self, now: float) -> None:
        """HOLD heartbeat: keep last filtered command while gate closed."""
        if self._gate.is_open:
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

    def _skip_teleop(self, now: float) -> None:
        self._republish_last(now)

    def step_once(self) -> None:
        """Retarget from the latest cached OpenXR 26 (same tick as wrist EE)."""
        now = time.monotonic()
        if not self._live() or not self._stream_ready():
            self._skip_teleop(now)
            return
        if now - self._last_step < self._min_step_dt:
            return
        self._last_step = now
        try:
            self._on_timer()
        except Exception as exc:
            self.get_logger().error(f"hand step failed: {exc}")

    def _on_timer(self):
        now = time.monotonic()
        if not self._live() or not self._stream_ready():
            self._skip_teleop(now)
            return
        if self._last_timer_time is None:
            self._last_timer_time = now
        dt = float(np.clip(now - self._last_timer_time, 1.0e-4, 0.1))
        self._last_timer_time = now

        left_f = self._read_frame("left")
        right_f = self._read_frame("right")
        left = self._left.step(left_f) if left_f is not None else None
        right = self._right.step(right_f) if right_f is not None else None
        if left is None and right is None:
            return

        for side, step in (("left", left), ("right", right)):
            if step is None:
                continue
            if step.held:
                self._held[side] += 1
            if self._cmd_cfg.enabled:
                q = self._cmd[side].process(
                    step.q,
                    now=now,
                    dt=dt,
                    target_fresh=not step.held,
                )
                if self._cmd[side].stale and step.held:
                    # Keep publishing held filtered q (stale path).
                    pass
            else:
                q = step.q
            self._maybe_publish(side, self._pub_for_optical_side(side), q)

        wall = time.time()
        if self._print_every > 0 and (wall - self._last_print) >= self._print_every:
            self._last_print = wall
            src = "cache" if self._frame_source is not None else "sdk"
            left_tag = f"{self._cfg.backend}/{self._left.retargeting_type}"
            right_tag = f"{self._right_cfg.backend}/{self._right.retargeting_type}"
            tag = left_tag if left_tag == right_tag else f"L={left_tag} R={right_tag}"
            ltxt = _fmt_side("L", left, self._held["left"]) if left is not None else "L none"
            rtxt = _fmt_side("R", right, self._held["right"]) if right is not None else "R none"
            gate = "open" if self._gate.can_publish else "closed"
            self.get_logger().info(f"[{tag} {src} gate={gate}] {ltxt} | {rtxt}")

    def _read_frame(self, side: str):
        from xr_hand_retarget.sources.frame import HandFrame
        from xr_hand_retarget.sources.xrt import read_hand_frame

        if self._frame_source is not None:
            sample = self._frame_source(side)
            if sample is None:
                return None
            active, joints = sample
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
