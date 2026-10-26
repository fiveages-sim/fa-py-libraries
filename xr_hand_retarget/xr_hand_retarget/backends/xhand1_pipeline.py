"""Per-side XHand1 retarget pipeline (vector only; no SDK init, no ROS).

Used by the standalone CLI and by vr_pose_publisher's XRT node so a single
xrobotoolkit_sdk client can drive both hands.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from xr_hand_retarget.algorithms import method_for_backend
from xr_hand_retarget.algorithms.safety_xhand1 import CommandSafety, pose_is_zero
from xr_hand_retarget.algorithms.vector_retarget import (
    VectorRetargeter,
    default_xhand1_vector_config,
)
from xr_hand_retarget.backends.xhand1_config import HandConfig


@dataclass
class SideStep:
    q: np.ndarray
    active: int
    held: bool
    xyz_abs: float


class SidePipeline:
    """OpenXR 26 / HandFrame → XHand1 q[12] for one side (vector / KeyVector)."""

    def __init__(
        self,
        side: str,
        cfg: HandConfig,
        retargeting_type: str | None = None,
        *,
        xrt=None,
    ):
        if side not in ("left", "right"):
            raise ValueError(f"side must be left|right, got {side!r}")
        self.side = side
        self.cfg = cfg
        self._xrt = xrt
        self.retargeting_type = method_for_backend(
            "xhand1", retargeting_type or cfg.retargeting_type
        )

        vector_cfg = cfg.vector
        # teleop_ws AnyDex peer: SomeHand / key-vector profile (not DexPilot).
        if str(getattr(vector_cfg, "profile", "") or "").lower() == "dexpilot":
            vector_cfg = default_xhand1_vector_config()
            print(
                f"[retarget] side={side} forcing vector profile=somehand "
                "(dexpilot retired for XHand1)",
                flush=True,
            )
        self.vector_profile = str(getattr(vector_cfg, "profile", "somehand") or "somehand")
        self.vector = VectorRetargeter(
            side=side,
            limits=cfg.limits,
            config=vector_cfg,
        )
        self.safety = CommandSafety(
            cfg.limits,
            side=side,
            alpha=cfg.alpha,
            soft_collision=cfg.soft_collision,
            collision_gains=cfg.collision,
            max_delta_rad=cfg.max_delta_rad,
            motion=cfg.motion,
            dt_default=1.0 / max(float(cfg.rate_hz), 1.0),
        )
        self._dt = 1.0 / max(float(cfg.rate_hz), 1.0)
        if cfg.motion.enabled:
            lim0 = self.safety.effective_limits[0]
            print(
                f"[{side}] motion_safety URDF: "
                f"limits_from_urdf={self.safety.urdf_limits_applied} "
                f"thumb_j1=[{lim0[0]:.3f},{lim0[1]:.3f}] "
                f"vel0={self.safety._vel_limits[0]:.2f}rad/s "
                f"pairs={len(cfg.motion.clearance_pairs)} "
                f"thumb_palm>={cfg.motion.thumb_palm_clearance_m}m "
                f"thumb_priority={cfg.motion.thumb_priority}",
                flush=True,
            )
        override = "" if retargeting_type is None else f" override={retargeting_type!r}"
        print(
            f"[retarget] side={side} type=vector profile={self.vector_profile} "
            f"engine=VectorRetargeter yaml_type={cfg.retargeting_type}{override}",
            flush=True,
        )

    def step(self, frame=None) -> SideStep:
        from xr_hand_retarget.sources.frame import HandFrame
        from xr_hand_retarget.sources.xrt import read_hand_frame

        if frame is None:
            if self._xrt is None:
                raise RuntimeError("SidePipeline.step requires a HandFrame or bound xrt")
            frame = read_hand_frame(self._xrt, self.side)
        if not isinstance(frame, HandFrame):
            raise TypeError(f"expected HandFrame, got {type(frame)!r}")
        active = int(frame.active)
        raw = frame.ensure_joints26()
        invalid = (self.cfg.hold_on_inactive and active == 0) or (
            self.cfg.hold_on_zero_pose and pose_is_zero(raw)
        )
        if invalid:
            q_cmd = self.safety.hold()
            held = True
        else:
            q_raw = self.vector.retarget(
                raw,
                flexion_scale=self.cfg.flexion_scale,
                finger_chain=self.cfg.finger_chain,
                thumb_tip=self.cfg.thumb_tip,
                index_abduction=self.cfg.index_abduction,
            )
            q_cmd = self.safety.step(q_raw, dt=self._dt)
            held = False
        return SideStep(q=q_cmd, active=active, held=held, xyz_abs=float(frame.xyz_abs))


def command_topic(controller: str, suffix: str = "target_joint_position") -> str:
    name = controller.strip().strip("/")
    return f"/{name}/{suffix}"
