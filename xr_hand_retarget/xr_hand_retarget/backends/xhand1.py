"""XHand1 backend — existing curl / thumb_ik / vector / dexpilot pipeline."""

from __future__ import annotations

from xr_hand_retarget.backends.xhand1_pipeline import SidePipeline, SideStep


class XHand1Backend:
    """Wraps ``xhand1_pipeline.SidePipeline``; no Wuji solver involved."""

    def __init__(self, side: str, cfg, retargeting_type: str | None = None):
        if cfg.xhand1 is None:
            raise ValueError("xhand1 backend requires an XHand1 yaml (retargeting: …)")
        self._inner = SidePipeline(side, cfg.xhand1, retargeting_type)
        self.side = side
        self.dof = int(cfg.dof)
        self.retargeting_type = self._inner.retargeting_type
        self.vector_profile = getattr(self._inner, "vector_profile", "") or ""

    def step(self, frame):
        return self._inner.step(frame)
