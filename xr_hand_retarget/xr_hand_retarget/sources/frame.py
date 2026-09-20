"""HandFrame: source → backend contract (no ROS, no vendor SDK)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class HandFrame:
    """One hand sample.

    XRT sources fill ``joints26`` (OpenXR) and usually ``xyz`` (MediaPipe 21).
    Solvers that still need OpenXR use ``joints26``; Wuji official uses ``xyz``.
    """

    side: str
    active: int
    joints26: np.ndarray | None = None  # (26, >=3)
    xyz: np.ndarray | None = None  # (21, 3) MediaPipe order
    timestamp: float | None = None

    def __post_init__(self) -> None:
        if self.side not in ("left", "right"):
            raise ValueError(f"side must be left|right, got {self.side!r}")
        self.active = int(self.active)
        if self.joints26 is not None:
            self.joints26 = np.asarray(self.joints26, dtype=np.float64)
        if self.xyz is not None:
            self.xyz = np.asarray(self.xyz, dtype=np.float64)

    @property
    def xyz_abs(self) -> float:
        src = self.joints26 if self.joints26 is not None else self.xyz
        if src is None or src.size == 0:
            return 0.0
        return float(np.abs(src[:, :3]).max())

    def ensure_joints26(self) -> np.ndarray:
        if self.joints26 is None:
            raise ValueError(f"HandFrame[{self.side}] missing joints26")
        j = self.joints26
        if j.ndim != 2 or j.shape[0] != 26:
            j = j.reshape(26, -1)
            self.joints26 = j
        return j

    def ensure_xyz21(self) -> np.ndarray:
        if self.xyz is not None:
            xyz = np.asarray(self.xyz, dtype=np.float64)
            if xyz.shape != (21, 3):
                raise ValueError(f"HandFrame.xyz expected (21, 3), got {xyz.shape}")
            return xyz
        from xr_hand_retarget.sources.landmarks import openxr26_to_mediapipe21

        xyz = openxr26_to_mediapipe21(self.ensure_joints26())
        self.xyz = xyz
        return xyz
