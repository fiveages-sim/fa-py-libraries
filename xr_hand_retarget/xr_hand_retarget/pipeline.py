"""Per-side hand retarget pipeline (no SDK init, no ROS)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from xr_hand_retarget.backends import make_backend
from xr_hand_retarget.config import HandRuntimeConfig
from xr_hand_retarget.sources.frame import HandFrame


@dataclass
class SideStep:
    q: np.ndarray
    active: int
    held: bool
    xyz_abs: float
    reason: str = "ok"
    sat: int = -1


class SidePipeline:
    """HandFrame → robot q for one side. Caller owns source (xrt.init()/close())."""

    def __init__(
        self,
        side: str,
        cfg: HandRuntimeConfig,
        retargeting_type: str | None = None,
        *,
        xrt=None,
    ):
        # ``xrt`` kept only for optional Linker j2-lock button polling.
        self._backend = make_backend(side, cfg, retargeting_type)
        self.side = side
        self.cfg = cfg
        self._xrt = xrt
        self.retargeting_type = self._backend.retargeting_type
        self.vector_profile = getattr(self._backend, "vector_profile", "") or ""

    def step(self, frame: HandFrame | None = None) -> SideStep:
        if frame is None:
            if self._xrt is None:
                raise RuntimeError("SidePipeline.step requires a HandFrame or bound xrt")
            from xr_hand_retarget.sources.xrt import read_hand_frame

            frame = read_hand_frame(self._xrt, self.side)
        # Linker may poll controller buttons via optional xrt kwarg.
        try:
            out = self._backend.step(frame, xrt=self._xrt)
        except TypeError:
            out = self._backend.step(frame)
        return out

    def set_j2_lock(self, on: bool) -> None:
        fn = getattr(self._backend, "set_j2_lock", None)
        if callable(fn):
            fn(bool(on))


def command_topic(controller: str, suffix: str = "target_joint_position") -> str:
    name = controller.strip().strip("/")
    return f"/{name}/{suffix}"


def _stream_movej_request():
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters

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
    return req


def _stream_movej_result_flags(result) -> list[tuple[bool, str]]:
    if result is None:
        return []
    return [(bool(r.successful), str(r.reason or "")) for r in result.results]


def _stream_movej_ok(result) -> bool:
    flags = _stream_movej_result_flags(result)
    return bool(flags) and all(ok for ok, _ in flags)


def _log_stream_movej(controller: str, flags, *, ok: bool, how: str) -> None:
    # #region agent log
    try:
        import json as _json
        import time as _time

        with open("/home/fiveages/fa-py-libraries/.cursor/debug-fc4a89.log", "a") as _f:
            _f.write(
                _json.dumps(
                    {
                        "sessionId": "fc4a89",
                        "hypothesisId": "R1,R2",
                        "location": "pipeline.py:stream_movej",
                        "message": "MOVEJ none result",
                        "data": {
                            "controller": controller,
                            "ok": ok,
                            "how": how,
                            "flags": flags,
                        },
                        "timestamp": int(_time.time() * 1000),
                    }
                )
                + "\n"
            )
    except Exception:
        pass
    # #endregion


def try_stream_movej(node, controller: str) -> None:
    """Blocking set of MOVEJ none. Standalone node.py before its loop.

    Dual-hand vr-xrt must use StreamMoveJSession: spinning here during
    DualHandRetargetNode.__init__ runs before the shared executor exists.
    """
    name = controller.strip().strip("/")
    try:
        from rcl_interfaces.srv import SetParameters
        import rclpy
    except ImportError:
        return
    client = node.create_client(SetParameters, f"/{name}/set_parameters")
    req = _stream_movej_request()
    for _ in range(3):
        if not client.wait_for_service(timeout_sec=1.5):
            node.get_logger().warn(
                f"[{name}] 遥操需要 MOVEJ 直通（官方每帧发 q）。请另开终端:\n"
                f"  ros2 param set /{name} movej_interpolation_type none\n"
                f"  ros2 param set /{name} movej_duration 0.02"
            )
            return
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
        result = fut.result() if fut.done() else None
        flags = _stream_movej_result_flags(result)
        ok = _stream_movej_ok(result)
        _log_stream_movej(name, flags, ok=ok, how="blocking")
        if ok:
            node.get_logger().info(
                f"[{name}] session MOVEJ stream: interpolation=none (fa_w2 yaml unchanged)"
            )
            return
    node.get_logger().warn(
        f"[{name}] set_parameters failed; run ros2 param set … interpolation_type none"
    )


class StreamMoveJSession:
    """Async: keep BJC MOVEJ interpolation=none for this process.

    Do not spin the node. StateMoveJ::updateParam re-reads these on every
    target and on MOVEJ enter, so re-kick after HOME→OCS2.
    """

    def __init__(self, node, controllers):
        from rcl_interfaces.srv import SetParameters

        names = []
        seen = set()
        for raw in controllers:
            name = str(raw).strip().strip("/")
            if not name or name in seen:
                continue
            seen.add(name)
            names.append(name)
        self._node = node
        self._names = names
        self._req = _stream_movej_request()
        self._clients = {
            name: node.create_client(SetParameters, f"/{name}/set_parameters")
            for name in names
        }
        self._ok = {name: False for name in names}
        self._pending = {}
        self._timer = node.create_timer(0.25, self._tick)

    @property
    def ready(self) -> bool:
        return bool(self._names) and all(self._ok.values())

    def kick(self, *, force: bool = False) -> None:
        if force:
            for name in self._names:
                self._ok[name] = False
            self._pending.clear()
        self._tick()

    def _tick(self) -> None:
        for name in self._names:
            if self._ok.get(name):
                continue
            fut = self._pending.get(name)
            if fut is not None:
                if not fut.done():
                    continue
                self._pending.pop(name, None)
                try:
                    result = fut.result()
                except Exception:
                    result = None
                flags = _stream_movej_result_flags(result)
                ok = _stream_movej_ok(result)
                _log_stream_movej(name, flags, ok=ok, how="async")
                if ok:
                    self._ok[name] = True
                    self._node.get_logger().info(
                        f"[{name}] session MOVEJ stream: interpolation=none "
                        "(fa_w2 yaml unchanged)"
                    )
                continue
            client = self._clients[name]
            if not client.service_is_ready():
                continue
            self._pending[name] = client.call_async(self._req)


def wire_j2_lock_sub(node, pipelines) -> None:
    """``/xr_hand_retarget/j2_lock`` Bool sets pad-band lock on every pipeline."""
    from std_msgs.msg import Bool

    def _on(msg):
        on = bool(msg.data)
        items = pipelines.values() if hasattr(pipelines, "values") else pipelines
        for pipe in items:
            fn = getattr(pipe, "set_j2_lock", None)
            if callable(fn):
                fn(on)

    node.create_subscription(Bool, "/xr_hand_retarget/j2_lock", _on, 10)
