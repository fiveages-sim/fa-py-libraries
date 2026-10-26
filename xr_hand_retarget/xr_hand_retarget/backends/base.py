"""Backend protocol: HandFrame → robot q."""

from __future__ import annotations

from typing import Protocol

import numpy as np

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
    if cfg.backend == "xhand1":
        from .xhand1 import XHand1Backend

        return XHand1Backend(side, cfg, retargeting_type)
    if cfg.backend == "wuji":
        rtype = (retargeting_type or cfg.retargeting_type or "curl").lower()
        if rtype in ("curl", "curl_calib", "calib"):
            from xr_hand_retarget.algorithms.curl_hand2 import WujiCurlBackend

            return WujiCurlBackend(side, cfg)
        from .wuji import WujiBackend

        if retargeting_type and rtype not in ("official", "wuji", "analytical"):
            print(
                f"[retarget] backend=wuji unknown type {retargeting_type!r}; "
                "using official solver",
                flush=True,
            )
        return WujiBackend(side, cfg)
    if cfg.backend in ("o6", "l6", "o7", "linker"):
        from .linker import LinkerBackend

        return LinkerBackend(side, cfg, retargeting_type)
    raise ValueError(
        f"unknown backend {cfg.backend!r}; expected xhand1|wuji|o6|l6|o7|linker"
    )
