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
