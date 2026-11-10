"""ROS2 node: one SDK client → left + right hand joint commands.

Drive modes (async only)
------------------------
* ``async`` + **process** (default teleop): EE writes hand26 shm; spawned
  retarget process solves; Thread C publishes @ publish_hz.
* ``async`` + **thread** (``FA_HAND_PROCESS=0``): in-process retarget worker.

Publish tick is three-state: ``fresh`` / ``held`` (1 Hz heartbeat) / ``lost``
(no publish). See ``docs/dev/ASYNC_TELEOP.md``.
"""

from __future__ import annotations

import csv
import multiprocessing as mp
import os
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64MultiArray, Int32

from xr_hand_retarget.config import HandRuntimeConfig
from xr_hand_retarget.control.command_safety import (
    CommandSafetyConfig,
    SideCommandSafety,
    make_fsm_gate,
)
from xr_hand_retarget.local_profile import HandPlan, SideSpec, format_resolved, print_plan
from xr_hand_retarget.mirror_latch import (
    optical_to_publish_side,
    save_mirror,
)
from xr_hand_retarget.pipeline import (
    SidePipeline,
    StreamMoveJSession,
    command_topic,
    wire_j2_lock_sub,
)

_MIRROR_MODE_TOPIC = "/teleop/mirror_mode"
# Same candidates as vr_pose_publisher.r0_session.FSM_STATE_TOPICS: controller
# publishes a *relative* "fsm_state", so under a robot namespace the absolute
# name never appears and the command gate would stay closed forever.
_FSM_STATE_TOPICS = (
    "/ocs2_wbc_controller/fsm_state",
    "/ocs2_arm_controller/fsm_state",
    "/fsm_state",
)
_DEFAULT_PUBLISH_HZ = 120.0
# Consecutive shm/worker seqs with q=None before treating that side as held.
_MISSING_Q_HELD_SEQS = 5
_CMD_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def _side_limits(pipe: SidePipeline | None, cfg: HandRuntimeConfig) -> np.ndarray:
    if pipe is not None:
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
    if cfg.xhand1 is not None:
        return np.asarray(cfg.xhand1.limits, dtype=float)
    n = int(cfg.dof)
    return np.stack([-np.pi * np.ones(n), np.pi * np.ones(n)], axis=1)


def _publish_rate_hz(local_default: float | None = None) -> float:
    """Priority: FA_HAND_PUBLISH_HZ env > local.yaml teleop.publish_hz > 120."""
    raw = os.environ.get("FA_HAND_PUBLISH_HZ", "").strip()
    if raw:
        return float(np.clip(float(raw), 80.0, 300.0))
    if local_default is not None:
        return float(np.clip(float(local_default), 80.0, 300.0))
    return _DEFAULT_PUBLISH_HZ


_PROCESS_ON = frozenset({"1", "true", "yes", "on", "process", ""})
_PROCESS_OFF = frozenset({"0", "false", "no", "off", "thread"})


def _want_process_retarget() -> bool:
    """True → spawn Process B; False → in-process thread. Invalid env → process + WARN."""
    raw = os.environ.get("FA_HAND_PROCESS", "1").strip().lower()
    if raw in _PROCESS_OFF:
        return False
    if raw in _PROCESS_ON:
        return True
    print(
        f"[WARN] FA_HAND_PROCESS={raw!r} invalid "
        f"(expected 0/1/true/false/thread/process); using process",
        flush=True,
    )
    return True


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
        print_every: float = 0.0,
        node_name: str = "xr_hand_retarget",
        right_cfg: HandRuntimeConfig | None = None,
        right_retargeting: str | None = None,
        plan: HandPlan | None = None,
        frame_source=None,
        async_pipeline: bool = True,
        process_retarget: bool | None = None,
        plan_kwargs: dict[str, Any] | None = None,
        mirror_host=None,
    ):
        super().__init__(node_name)
        right_cfg = right_cfg or cfg
        right_retargeting = (
            right_retargeting if right_retargeting is not None else retargeting_type
        )
        self._cfg = cfg
        self._right_cfg = right_cfg
        self._left_rtype = retargeting_type
        self._right_rtype = right_retargeting
        self._print_every = float(print_every)
        self._last_print = 0.0
        self._held = {"left": 0, "right": 0}
        self._side_held = {"left": False, "right": False}
        self._missing_q_streak = {"left": 0, "right": 0}
        self._last_sent = {"left": None, "right": None}
        self._last_heartbeat_mono = {"left": None, "right": None}
        self._pub_state_side = {"left": "idle", "right": "idle"}
        self._frame_source = frame_source
        self._mirror_host = mirror_host
        self._cmd_cfg: CommandSafetyConfig = cfg.command or CommandSafetyConfig()
        self._gate = make_fsm_gate(self._cmd_cfg)
        self._mirror = False
        self._plan_kwargs = dict(plan_kwargs or {})
        self._hand_plan = plan

        # async_pipeline kept for call-site compat; sync/timer paths are gone.
        _ = async_pipeline
        self._async = True
        self._process_retarget = bool(
            process_retarget if process_retarget is not None else _want_process_retarget()
        )
        # Parent keeps SidePipeline only when retarget runs in-process.
        self._left: SidePipeline | None = None
        self._right: SidePipeline | None = None
        if not self._process_retarget:
            self._left = SidePipeline("left", cfg, retargeting_type, xrt=xrt)
            self._right = SidePipeline("right", right_cfg, right_retargeting, xrt=xrt)
            from xr_hand_retarget.ipc.worker_budget import (
                apply_vector_max_iterations,
                max_iterations_from_env,
            )

            max_iter = max_iterations_from_env(
                getattr(plan, "max_iterations", None) if plan is not None else None
            )
            if max_iter is not None:
                apply_vector_max_iterations(self._left, max_iter)
                apply_vector_max_iterations(self._right, max_iter)
        self._xrt = xrt
        suffix = cfg.topic_suffix
        self._left_topic = command_topic(left_controller, suffix)
        self._right_topic = command_topic(right_controller, suffix)
        self._pub_left = self.create_publisher(
            Float64MultiArray, self._left_topic, _CMD_QOS
        )
        self._pub_right = self.create_publisher(
            Float64MultiArray, self._right_topic, _CMD_QOS
        )
        if self._left is not None and self._right is not None:
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
        fsm_qos = QoSProfile(depth=1)
        fsm_qos.reliability = ReliabilityPolicy.RELIABLE
        fsm_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        # Bind to the first fsm_state topic that publishes; ignore the rest so
        # two controllers cannot disagree on the gate.
        self._fsm_bound_topic: str | None = None
        for topic in _FSM_STATE_TOPICS:
            self.create_subscription(
                Int32,
                topic,
                lambda msg, t=topic: self._on_fsm_bound(t, msg),
                fsm_qos,
            )
        mirror_qos = QoSProfile(depth=1)
        mirror_qos.reliability = ReliabilityPolicy.RELIABLE
        mirror_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool, _MIRROR_MODE_TOPIC, self._on_mirror_mode, mirror_qos
        )

        if auto_movej and (fsm_topic or cfg.fsm_topic):
            topic = fsm_topic or cfg.fsm_topic
            fsm_pub = self.create_publisher(Int32, topic, 10)
            time.sleep(0.3)
            msg = Int32()
            msg.data = int(cfg.fsm_command)
            fsm_pub.publish(msg)
            self.get_logger().info(f"Sent FSM {cfg.fsm_command} on {topic}")

        self._async_lock = threading.Lock()
        self._sample_event = threading.Event()
        self._target_updated = threading.Event()
        self._async_stop = threading.Event()
        self._worker_thread: threading.Thread | None = None
        self._publish_thread: threading.Thread | None = None
        self._frame_seq = 0
        self._q_target: dict[str, np.ndarray | None] = {"left": None, "right": None}
        self._q_target_seq: dict[str, int] = {"left": 0, "right": 0}
        self._q_target_fresh: dict[str, bool] = {"left": False, "right": False}
        self._q_des: dict[str, np.ndarray | None] = {"left": None, "right": None}
        local_pub = getattr(plan, "publish_hz", None) if plan is not None else None
        self._publish_hz = _publish_rate_hz(local_pub)
        from xr_hand_retarget.ipc.worker_budget import retarget_hz_from_env

        self._retarget_hz = retarget_hz_from_env(
            getattr(plan, "retarget_hz", None) if plan is not None else None
        )
        # Stay above 1s at normal rates; scale out if retarget_hz is very low.
        self._ingest_lost_s = max(1.0, 4.0 / max(float(self._retarget_hz), 1e-6))
        self._align_log_path = os.environ.get("FA_HAND_ALIGN_LOG", "").strip()
        self._align_fp = None
        self._align_writer = None
        self._stats_lock = threading.Lock()
        self._target_intervals: list[float] = []
        self._last_target_mono: float | None = None
        self._publish_intervals: list[float] = []
        self._last_publish_mono: float | None = None
        self._target_count = 0
        self._publish_count = 0

        # Process isolation state
        self._mp_ctx = mp.get_context("spawn")
        self._shm = None
        self._worker_proc: mp.Process | None = None
        self._mp_sample_event = None
        self._mp_stop_event = None
        self._last_out_seq = -1
        self._last_ingest_mono: float | None = None
        self._lost_logged = False

        left_tag = (
            self._left.retargeting_type
            if self._left is not None
            else (retargeting_type or cfg.backend)
        )
        right_tag = (
            self._right.retargeting_type
            if self._right is not None
            else (right_retargeting or right_cfg.backend)
        )
        if plan is not None:
            print_plan(plan)
            dump = plan.dump_yaml().rstrip()
            left_tag = plan.left.retargeting_type
            right_tag = plan.right.retargeting_type
        else:
            dump = format_resolved(
                SideSpec(cfg.backend, str(left_tag)),
                SideSpec(right_cfg.backend, str(right_tag)),
            ).rstrip()
            print(f"[local.yaml] source={cfg.path}", flush=True)
            print(dump, flush=True)
        self._left_tag = f"{cfg.backend}/{left_tag}"
        self._right_tag = f"{right_cfg.backend}/{right_tag}"
        vmax_l = self._cmd["left"].filter.max_velocity
        vmax_r = self._cmd["right"].filter.max_velocity
        if self._process_retarget:
            drive = "async(process-retarget,C=publish@%.0fHz)" % self._publish_hz
        else:
            drive = "async(thread-retarget,C=publish@%.0fHz)" % self._publish_hz
        self.get_logger().info(
            f"dual-hand\n{dump}\n"
            f"left={self._left_topic} ({cfg.dof}) "
            f"right={self._right_topic} ({right_cfg.dof}) drive={drive}\n"
            f"command_safety enabled={self._cmd_cfg.enabled} "
            f"publish={self._cmd_cfg.publish_command} "
            f"fsm={list(self._cmd_cfg.allowed_fsm_states)} "
            f"vmax=L{vmax_l}/R{vmax_r} "
            f"lpf={self._cmd_cfg.filter_cutoff_hz}Hz "
            f"timeout={self._cmd_cfg.target_timeout}s "
            f"cmd_qos=depth1+RELIABLE"
        )
        self.get_logger().info(
            f"Hand mirror follows {_MIRROR_MODE_TOPIC} "
            "(ATM EE/trigger mirror_mode_; no local event-7 toggle)"
        )

    # ------------------------------------------------------------------ FSM / mirror

    def _on_fsm_bound(self, topic: str, msg: Int32) -> None:
        """Accept only the first fsm_state publisher; drop the rest."""
        if self._fsm_bound_topic is None:
            self._fsm_bound_topic = str(topic)
            self.get_logger().info(f"fsm_state bound to {self._fsm_bound_topic}")
        elif str(topic) != self._fsm_bound_topic:
            return
        self._on_fsm(msg)

    def _on_fsm(self, msg: Int32) -> None:
        was_open, is_open = self._gate.update(int(msg.data))
        if was_open == is_open and not self._gate.needs_resync:
            return
        if is_open and not was_open:
            for side in ("left", "right"):
                seed = self._last_sent[side]
                if seed is None:
                    cfg = self._cfg if side == "left" else self._right_cfg
                    pipe = self._left if side == "left" else self._right
                    lim = _side_limits(pipe, cfg)
                    seed = 0.5 * (lim[:, 0] + lim[:, 1])
                self._cmd[side].reset(seed)
                with self._async_lock:
                    self._q_des[side] = np.asarray(seed, dtype=float).copy()
            if self._cmd_cfg.require_resync:
                self._gate.mark_resynced()
            # Gate-closed worker skips solves → ingest clock freezes; clear so
            # reopen does not false-trigger LOST on the first publish tick.
            self._last_ingest_mono = None
            self._lost_logged = False
            self._pub_state_side = {"left": "idle", "right": "idle"}
            self.get_logger().info(
                f"FSM {msg.data}: command gate opened; filter resynced "
                f"(ingest_lost_s={self._ingest_lost_s:.2f})"
            )
            if self._stream_movej is not None:
                self._stream_movej.kick(force=True)
        elif was_open and not is_open:
            # Keep _last_sent: BasicJointController holds last pose; reseeding
            # from mid-range on reopen causes a visible finger sweep.
            with self._async_lock:
                self._q_target = {"left": None, "right": None}
                self._q_des = {"left": None, "right": None}
            self._pub_state_side = {"left": "idle", "right": "idle"}
            self.get_logger().info(
                f"FSM {msg.data}: command gate closed; controller commands paused"
            )

    def _on_mirror_mode(self, msg: Bool) -> None:
        self._apply_mirror_level(bool(msg.data), source=_MIRROR_MODE_TOPIC)

    def _apply_mirror_level(self, want: bool, *, source: str) -> None:
        want = bool(want)
        if want == self._mirror:
            return
        with self._async_lock:
            prev_target = {
                "left": None
                if self._q_target["left"] is None
                else self._q_target["left"].copy(),
                "right": None
                if self._q_target["right"] is None
                else self._q_target["right"].copy(),
            }
            prev_des = {
                "left": None
                if self._q_des["left"] is None
                else self._q_des["left"].copy(),
                "right": None
                if self._q_des["right"] is None
                else self._q_des["right"].copy(),
            }
        self._mirror = want
        save_mirror(self._mirror)
        host = self._mirror_host
        if host is not None and hasattr(host, "set_hand_mirror"):
            try:
                host.set_hand_mirror(self._mirror)
            except Exception:
                pass
        if self._mirror:
            self.get_logger().warn(
                f"status: hand mirror ON (optical L→R ctrl, R→L) [{source}]"
            )
        else:
            self.get_logger().warn(
                f"status: hand mirror OFF (optical L→L / R→R) [{source}]"
            )
        with self._async_lock:
            for side in ("left", "right"):
                if prev_target[side] is not None:
                    self._q_target[side] = prev_target[side]
                if prev_des[side] is not None:
                    self._q_des[side] = prev_des[side]

    def _pub_for_optical_side(self, side: str):
        key = optical_to_publish_side(side, self._mirror)
        return self._pub_right if key == "right" else self._pub_left

    def _live(self) -> bool:
        if not self._cmd_cfg.enabled:
            return self._gate.state == 3
        return self._gate.can_publish

    def _stream_ready(self) -> bool:
        return self._stream_movej is None or self._stream_movej.ready

    def _publish_q(self, optical_side: str, q: np.ndarray) -> None:
        if self._cmd_cfg.enabled and not self._cmd_cfg.publish_command:
            return
        pub = self._pub_for_optical_side(optical_side)
        msg = Float64MultiArray()
        msg.data = [float(x) for x in np.asarray(q, dtype=float).tolist()]
        pub.publish(msg)
        self._last_sent[optical_side] = np.asarray(q, dtype=float).copy()

    # ------------------------------------------------------------------ async API

    def notify_frame(self) -> None:
        """EE thread: copy hand26 into shm / signal worker (no retarget here)."""
        if not self._async:
            return
        self._frame_seq += 1
        if self._process_retarget and self._shm is not None:
            left = right = None
            if self._frame_source is not None:
                left = self._frame_source("left")
                right = self._frame_source("right")
            try:
                self._shm.write_input(
                    frame_seq=self._frame_seq,
                    t_mono=time.monotonic(),
                    live=bool(self._live() and self._stream_ready()),
                    left=left,
                    right=right,
                )
            except Exception as exc:
                self.get_logger().error(f"hand shm write failed: {exc}")
                return
            if self._mp_sample_event is not None:
                self._mp_sample_event.set()
            return
        self._sample_event.set()

    def start_async(self) -> None:
        if not self._async:
            return
        if self._publish_thread is not None:
            return
        self._open_align_log()
        self._async_stop.clear()
        if self._process_retarget:
            self._start_process_worker()
        else:
            self._worker_thread = threading.Thread(
                target=self._worker_loop,
                name="hand-retarget-worker",
                daemon=True,
            )
            self._worker_thread.start()
        self._publish_thread = threading.Thread(
            target=self._publish_loop,
            name="hand-publish-250hz",
            daemon=True,
        )
        self._publish_thread.start()
        mode = "process" if self._process_retarget else "thread"
        self.get_logger().info(
            f"async hand pipeline started mode={mode} "
            f"publish={self._publish_hz:.0f}Hz align_log="
            f"{self._align_log_path or 'off'}"
        )

    def _start_process_worker(self) -> None:
        from xr_hand_retarget.ipc.hand_shm import HandShmBundle
        from xr_hand_retarget.ipc.retarget_worker import worker_main

        self._shm = HandShmBundle.create_pair()
        self._mp_sample_event = self._mp_ctx.Event()
        self._mp_stop_event = self._mp_ctx.Event()
        self._worker_proc = self._mp_ctx.Process(
            target=worker_main,
            kwargs={
                "in_shm_name": self._shm.in_name,
                "out_shm_name": self._shm.out_name,
                "sample_event": self._mp_sample_event,
                "stop_event": self._mp_stop_event,
                "plan_kwargs": self._plan_kwargs,
                "left_retargeting": self._left_rtype,
                "right_retargeting": self._right_rtype,
            },
            name="hand-retarget-proc",
            daemon=True,
        )
        self._worker_proc.start()
        from xr_hand_retarget.ipc.worker_budget import retarget_hz_from_env

        rhz = retarget_hz_from_env(
            getattr(self._hand_plan, "retarget_hz", None)
            if self._hand_plan is not None
            else None
        )
        self.get_logger().info(
            f"hand retarget process pid={self._worker_proc.pid} "
            f"retarget_hz={rhz:g} shm_in={self._shm.in_name} shm_out={self._shm.out_name}"
        )

    def stop_async(self) -> None:
        if not self._async:
            return
        try:
            self.get_logger().info("async hand: stopping publish + retarget worker")
        except Exception:
            pass
        self._async_stop.set()
        self._sample_event.set()
        self._target_updated.set()
        if self._mp_stop_event is not None:
            self._mp_stop_event.set()
        if self._mp_sample_event is not None:
            self._mp_sample_event.set()
        if self._worker_proc is not None:
            self._worker_proc.join(timeout=2.0)
            if self._worker_proc.is_alive():
                self._worker_proc.terminate()
                self._worker_proc.join(timeout=1.0)
            self._worker_proc = None
        for th in (self._worker_thread, self._publish_thread):
            if th is not None and th.is_alive():
                th.join(timeout=1.5)
        self._worker_thread = None
        self._publish_thread = None
        if self._shm is not None:
            try:
                self._shm.close()
                self._shm.unlink()
            except Exception:
                pass
            self._shm = None
        self._close_align_log()
        try:
            self._log_async_stats()
        except Exception:
            pass
        try:
            self.get_logger().info("async hand: stopped")
        except Exception:
            pass

    def destroy_node(self) -> bool:
        try:
            self.stop_async()
        except Exception:
            pass
        return super().destroy_node()

    def _open_align_log(self) -> None:
        if not self._align_log_path:
            return
        path = Path(self._align_log_path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        self._align_fp = path.open("w", newline="")
        self._align_writer = csv.writer(self._align_fp)
        self._align_writer.writerow(
            [
                "frame_seq",
                "side",
                "t_recv",
                "t_compute",
                "t_publish",
                "q0",
                "q1",
                "q2",
            ]
        )

    def _close_align_log(self) -> None:
        if self._align_fp is not None:
            try:
                self._align_fp.close()
            except Exception:
                pass
        self._align_fp = None
        self._align_writer = None

    def _log_async_stats(self) -> None:
        with self._stats_lock:
            t_iv = list(self._target_intervals)
            p_iv = list(self._publish_intervals)
            tc, pc = self._target_count, self._publish_count

        def _summ(xs: list[float]) -> str:
            if not xs:
                return "n=0"
            a = np.asarray(xs, dtype=float)
            return (
                f"n={a.size} med={1e3 * float(np.median(a)):.2f}ms "
                f"p99={1e3 * float(np.percentile(a, 99)):.2f}ms "
                f"max={1e3 * float(np.max(a)):.2f}ms "
                f"hz~={1.0 / max(float(np.median(a)), 1e-9):.1f}"
            )

        self.get_logger().info(
            f"async stats targets={tc} ({_summ(t_iv)}) "
            f"publishes={pc} ({_summ(p_iv)})"
        )

    def _ingest_worker_output(self) -> bool:
        """Pull newest q_target from shm (process mode). Returns True if updated."""
        if self._shm is None:
            return False
        try:
            seq = int(self._shm.peek_out_seq())
        except Exception as exc:
            self.get_logger().error(f"hand shm peek failed: {exc}")
            return False
        if seq == self._last_out_seq:
            return False
        try:
            out = self._shm.read_output()
        except Exception as exc:
            self.get_logger().error(f"hand shm read failed: {exc}")
            return False
        if out is None:
            return False
        seq = int(out["frame_seq"])
        if seq == self._last_out_seq:
            return False
        self._last_out_seq = seq
        updated = False
        with self._async_lock:
            for side, q_key, held_key in (
                ("left", "left_q", "left_held"),
                ("right", "right_q", "right_held"),
            ):
                q = out[q_key]
                if q is None:
                    # Side missing VALID/q: do not refresh target; after N seqs
                    # treat as held (1 Hz) so we do not forever replay stale q.
                    self._missing_q_streak[side] += 1
                    if self._missing_q_streak[side] >= _MISSING_Q_HELD_SEQS:
                        self._side_held[side] = True
                    continue
                self._missing_q_streak[side] = 0
                held = bool(out[held_key])
                self._side_held[side] = held
                if held:
                    self._held[side] += 1
                    if self._q_target[side] is None:
                        self._q_target[side] = np.asarray(q, dtype=float).copy()
                        self._q_target_seq[side] = seq
                        self._q_target_fresh[side] = True
                        updated = True
                    continue
                self._q_target[side] = np.asarray(q, dtype=float).copy()
                self._q_target_seq[side] = seq
                self._q_target_fresh[side] = True
                updated = True
        # Any new worker seq (incl. all-held) proves the worker is alive.
        now = time.monotonic()
        self._last_ingest_mono = now
        if updated:
            with self._stats_lock:
                if self._last_target_mono is not None:
                    self._target_intervals.append(now - self._last_target_mono)
                    if len(self._target_intervals) > 2000:
                        self._target_intervals = self._target_intervals[-1000:]
                self._last_target_mono = now
                self._target_count += 1
            self._maybe_print_status_async()
        else:
            self._maybe_print_status_async()
        return True

    # ----- in-process worker (FA_HAND_PROCESS=0) -----

    def _worker_loop(self) -> None:
        from xr_hand_retarget.ipc.worker_budget import retarget_hz_from_env

        hz = retarget_hz_from_env(
            getattr(self._hand_plan, "retarget_hz", None)
            if self._hand_plan is not None
            else None
        )
        min_period = 1.0 / hz
        next_solve_t = 0.0
        self.get_logger().info(f"hand thread-retarget rate cap={hz:g}Hz")
        while not self._async_stop.is_set():
            self._sample_event.wait(timeout=0.05)
            if self._async_stop.is_set():
                break
            self._sample_event.clear()
            while self._sample_event.is_set():
                self._sample_event.clear()
            now = time.monotonic()
            if now < next_solve_t:
                continue
            try:
                self._worker_once()
            except Exception as exc:
                self.get_logger().error(f"hand worker failed: {exc}")
            next_solve_t = now + min_period

    def _worker_once(self) -> None:
        if self._left is None or self._right is None:
            return
        if not self._live() or not self._stream_ready():
            return
        frame_seq = int(self._frame_seq)
        left_f = self._read_frame("left")
        right_f = self._read_frame("right")
        left = self._left.step(left_f) if left_f is not None else None
        right = self._right.step(right_f) if right_f is not None else None
        if left is None and right is None:
            return
        updated = False
        with self._async_lock:
            for side, step in (("left", left), ("right", right)):
                if step is None:
                    self._missing_q_streak[side] += 1
                    if self._missing_q_streak[side] >= _MISSING_Q_HELD_SEQS:
                        self._side_held[side] = True
                    continue
                self._missing_q_streak[side] = 0
                self._side_held[side] = bool(step.held)
                if step.held:
                    self._held[side] += 1
                    if self._q_target[side] is None:
                        self._q_target[side] = np.asarray(step.q, dtype=float).copy()
                        self._q_target_seq[side] = frame_seq
                        self._q_target_fresh[side] = True
                        updated = True
                    continue
                self._q_target[side] = np.asarray(step.q, dtype=float).copy()
                self._q_target_seq[side] = frame_seq
                self._q_target_fresh[side] = True
                updated = True
        now = time.monotonic()
        self._last_ingest_mono = now
        self._target_updated.set()
        if updated:
            with self._stats_lock:
                if self._last_target_mono is not None:
                    self._target_intervals.append(now - self._last_target_mono)
                    if len(self._target_intervals) > 2000:
                        self._target_intervals = self._target_intervals[-1000:]
                self._last_target_mono = now
                self._target_count += 1
            self._maybe_print_status(left, right)
        else:
            self._maybe_print_status_async()

    def _publish_loop(self) -> None:
        period = 1.0 / max(self._publish_hz, 1.0)
        next_t = time.monotonic()
        last_filter_t = next_t
        while not self._async_stop.is_set():
            now = time.monotonic()
            timeout = max(0.0, next_t - now)
            # Process mode: poll shm; thread mode: wait on event.
            if self._process_retarget:
                fired = self._ingest_worker_output()
                if not fired and timeout > 0.0:
                    time.sleep(min(timeout, 0.002))
                    fired = self._ingest_worker_output()
            else:
                fired = self._target_updated.wait(timeout=timeout)
                if fired:
                    self._target_updated.clear()
            if self._async_stop.is_set():
                break
            now = time.monotonic()
            if not fired and now < next_t:
                continue
            dt = float(np.clip(now - last_filter_t, 1.0e-4, 0.1))
            last_filter_t = now
            try:
                did_pub = self._publish_tick(now=now, dt=dt)
            except Exception as exc:
                if self._async_stop.is_set():
                    break
                msg = str(exc)
                # Shutdown race: rclpy context already torn down — exit quietly.
                if "context is invalid" in msg or "rcl_shutdown" in msg:
                    break
                self.get_logger().error(f"hand publish failed: {exc}")
                did_pub = False
            if did_pub:
                with self._stats_lock:
                    if self._last_publish_mono is not None:
                        self._publish_intervals.append(now - self._last_publish_mono)
                        if len(self._publish_intervals) > 4000:
                            self._publish_intervals = self._publish_intervals[-2000:]
                    self._last_publish_mono = now
                    self._publish_count += 1
            if fired and now < next_t:
                continue
            next_t += period
            if now > next_t + period:
                next_t = now + period

    def _worker_lost(self) -> bool:
        """Process B dead with nonzero exit → lost. Thread mode has no proc."""
        if not self._process_retarget or self._worker_proc is None:
            return False
        code = self._worker_proc.exitcode
        return code is not None and code != 0

    def _ingest_lost(self, now: float) -> bool:
        if self._last_ingest_mono is None:
            return False
        return (now - self._last_ingest_mono) > self._ingest_lost_s

    def _publish_state_str(self) -> str:
        return (
            f"L:{self._pub_state_side['left']}/R:{self._pub_state_side['right']}"
        )

    def _publish_tick(self, *, now: float, dt: float) -> bool:
        """Per-side publish: fresh / held (1 Hz heartbeat) / lost (silent).

        ``fresh``: filter + publish @ publish_hz.
        ``held`` (inactive hold / missing q): skip except 1s heartbeat.
        ``lost`` (ingest_age > ingest_lost_s or worker exitcode ∉ {None,0}):
        no publish, ERROR once.
        """
        if not self._live() or not self._stream_ready():
            return False
        if self._cmd_cfg.enabled and not self._cmd_cfg.publish_command:
            return False

        if self._worker_lost() or self._ingest_lost(now):
            self._pub_state_side = {"left": "lost", "right": "lost"}
            if not self._lost_logged:
                self._lost_logged = True
                age = (
                    float("inf")
                    if self._last_ingest_mono is None
                    else now - self._last_ingest_mono
                )
                code = (
                    None
                    if self._worker_proc is None
                    else self._worker_proc.exitcode
                )
                self.get_logger().error(
                    f"hand publish LOST: ingest_age={age:.2f}s "
                    f"limit={self._ingest_lost_s:.2f}s "
                    f"worker_exitcode={code} (no command publish until recover)"
                )
            self._maybe_print_status_async()
            return False
        self._lost_logged = False

        published = False
        for side in ("left", "right"):
            with self._async_lock:
                target = self._q_target[side]
                seq = self._q_target_seq[side]
                was_new = self._q_target_fresh[side]
                held = bool(self._side_held[side])
                if target is None:
                    self._pub_state_side[side] = "idle"
                    continue
                target = target.copy()
                self._q_target_fresh[side] = False

            if held:
                self._pub_state_side[side] = "held"
                last_hb = self._last_heartbeat_mono[side]
                if last_hb is not None and (now - last_hb) < 1.0:
                    continue
                # 1 Hz heartbeat: hold filter, do not chase a stale target.
                target_fresh = False
            else:
                self._pub_state_side[side] = "fresh"
                target_fresh = True

            if self._cmd_cfg.enabled:
                q = self._cmd[side].process(
                    target,
                    now=now,
                    dt=dt,
                    target_fresh=target_fresh,
                )
            else:
                q = target
            with self._async_lock:
                self._q_des[side] = np.asarray(q, dtype=float).copy()
            self._publish_q(side, q)
            self._last_heartbeat_mono[side] = now
            published = True
            if self._align_writer is not None and was_new:
                self._align_writer.writerow(
                    [
                        seq,
                        f"{side}_pub",
                        "",
                        "",
                        f"{now:.6f}",
                        f"{float(q[0]):.6f}" if len(q) > 0 else "",
                        f"{float(q[1]):.6f}" if len(q) > 1 else "",
                        f"{float(q[2]):.6f}" if len(q) > 2 else "",
                    ]
                )
        return published

    def _maybe_print_status_async(self) -> None:
        wall = time.time()
        if self._print_every <= 0 or (wall - self._last_print) < self._print_every:
            return
        self._last_print = wall
        tag = (
            self._left_tag
            if self._left_tag == self._right_tag
            else f"L={self._left_tag} R={self._right_tag}"
        )
        gate = "open" if self._gate.can_publish else "closed"
        with self._async_lock:
            lq = self._q_target["left"]
            rq = self._q_target["right"]
        with self._stats_lock:
            thz = (
                1.0 / max(float(np.median(self._target_intervals)), 1e-9)
                if self._target_intervals
                else 0.0
            )
            phz = (
                1.0 / max(float(np.median(self._publish_intervals)), 1e-9)
                if self._publish_intervals
                else 0.0
            )
        mode = "proc" if self._process_retarget else "thread"
        state = self._publish_state_str()
        self.get_logger().info(
            f"[{tag} {mode} gate={gate} state={state} "
            f"target~{thz:.1f}Hz pub~{phz:.1f}Hz] "
            f"L held={self._held['left']} q={_fmt_q(lq) if lq is not None else 'none'} | "
            f"R held={self._held['right']} q={_fmt_q(rq) if rq is not None else 'none'}"
        )

    def _maybe_print_status(self, left, right) -> None:
        wall = time.time()
        if self._print_every <= 0 or (wall - self._last_print) < self._print_every:
            return
        self._last_print = wall
        src = "cache" if self._frame_source is not None else "sdk"
        tag = (
            self._left_tag
            if self._left_tag == self._right_tag
            else f"L={self._left_tag} R={self._right_tag}"
        )
        ltxt = _fmt_side("L", left, self._held["left"]) if left is not None else "L none"
        rtxt = (
            _fmt_side("R", right, self._held["right"]) if right is not None else "R none"
        )
        gate = "open" if self._gate.can_publish else "closed"
        with self._stats_lock:
            thz = (
                1.0 / max(float(np.median(self._target_intervals)), 1e-9)
                if self._target_intervals
                else 0.0
            )
            phz = (
                1.0 / max(float(np.median(self._publish_intervals)), 1e-9)
                if self._publish_intervals
                else 0.0
            )
        self.get_logger().info(
            f"[{tag} {src} gate={gate} target~{thz:.1f}Hz pub~{phz:.1f}Hz] "
            f"{ltxt} | {rtxt}"
        )

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
