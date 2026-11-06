"""Backend protocol: HandFrame → robot q."""

from __future__ import annotations

from typing import Protocol

import numpy as np

from xr_hand_retarget.algorithms import method_for_backend
from xr_hand_retarget.sources.frame import HandFrame


class HandBackend(Protocol):
    """One side of one hand model. Caller owns source (e.g. xrt.init()/close())."""

    side: str
    dof: int
    retargeting_type: str

    def step(self, frame: HandFrame) -> "SideStepLike":
        ...


class SideStepLike(Protocol):
    q: np.ndarray
    active: int
    held: bool
    xyz_abs: float


def make_backend(side: str, cfg, retargeting_type: str | None = None):
    requested = retargeting_type or getattr(cfg, "retargeting_type", None)
    if cfg.backend == "xhand1":
        from .xhand1 import XHand1Backend

        method = method_for_backend("xhand1", requested)
        return XHand1Backend(side, cfg, method)
    if cfg.backend == "wuji":
        from .wuji import WujiBackend

        method_for_backend("wuji", requested)  # warn + lock
        return WujiBackend(side, cfg)
    if cfg.backend in ("o6", "l6", "o7", "linker"):
        from .linker import LinkerBackend

        method = method_for_backend(cfg.backend, requested)
        return LinkerBackend(side, cfg, method)
    raise ValueError(
        f"unknown backend {cfg.backend!r}; expected xhand1|wuji|o6|l6|o7|linker"
    )
