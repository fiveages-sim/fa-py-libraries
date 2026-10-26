"""Per-side hand retarget pipeline (no SDK init, no ROS).

Used by the standalone CLI and by vr_pose_publisher's XRT node so a single
xrobotoolkit_sdk client can drive both hands.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from xr_hand_retarget.backends.xhand1_config import HandConfig
from xr_hand_retarget.algorithms.curl_xhand1 import CurlCalibRetargeter
from xr_hand_retarget.algorithms.safety_xhand1 import CommandSafety, pose_is_zero
from xr_hand_retarget.algorithms.thumb_ik import ThumbIkRetargeter
from xr_hand_retarget.algorithms.vector_retarget import (
    VectorRetargeter,
    default_xhand1_dexpilot_config,
)


@dataclass
class SideStep:
    q: np.ndarray
    active: int
    held: bool
    xyz_abs: float


def _normalize_hand26(raw) -> np.ndarray:
    arr = np.asarray(raw, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[0] != 26:
        arr = arr.reshape(26, -1)
    return arr


def _as_active(flag) -> int:
    try:
        v = np.asarray(flag).reshape(-1)
        if v.size == 0:
            return 0
        return 1 if float(v[0]) > 0.5 else 0
    except Exception:
        return 0


class SidePipeline:
    """OpenXR 26 / HandFrame → XHand1 q[12] for one side."""

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
        rtype = (retargeting_type or cfg.retargeting_type).lower()
        vector_cfg = cfg.vector

        if rtype in ("l2",):
            rtype = "dexpilot"
        elif rtype in ("somehand",):
            rtype = "vector"
        elif rtype in ("ik", "thumb"):
            rtype = "thumb_ik"
        elif rtype in ("curl_calib", "calib", "lerp"):
            rtype = "curl"
        elif rtype == "hybrid":
            raise ValueError(
                "retargeting 'hybrid' removed; use curl|thumb_ik|vector|dexpilot"
            )

        if rtype == "dexpilot":
            if vector_cfg.profile != "dexpilot":
                vector_cfg = default_xhand1_dexpilot_config(
                    gains=cfg.vector.dexpilot,
                    temporal_filter_alpha=cfg.vector.temporal_filter_alpha,
                    max_iterations=max(cfg.vector.max_iterations, 12),
                    norm_delta=(
                        0.004
                        if cfg.vector.norm_delta < 0.002
                        else cfg.vector.norm_delta
                    ),
                    output_alpha=(
                        0.5 if cfg.vector.output_alpha > 0.8 else cfg.vector.output_alpha
                    ),
                    warm_start_remap=getattr(
                        cfg.vector, "warm_start_remap", True
                    ),
                    urdf_path=cfg.vector.urdf_path,
                )
        elif rtype not in ("vector", "thumb_ik", "curl"):
            raise ValueError(
                f"unknown retargeting.type {rtype!r}; "
                "expected curl|thumb_ik|vector|dexpilot"
            )

        self.retargeting_type = rtype
        self.vector_profile = (
            str(vector_cfg.profile)
            if rtype in ("vector", "dexpilot")
            else ""
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
        self.vector: VectorRetargeter | None = None
        self.thumb_ik: ThumbIkRetargeter | None = None
        self.curl: CurlCalibRetargeter | None = None
        if rtype in ("vector", "dexpilot"):
            self.vector = VectorRetargeter(
                side=side,
                limits=cfg.limits,
                config=vector_cfg,
            )
        elif rtype == "thumb_ik":
            self.thumb_ik = ThumbIkRetargeter(
                side=side,
                limits=cfg.limits,
                gains=cfg.thumb_ik,
            )
        elif rtype == "curl":
            self.curl = CurlCalibRetargeter(
                side=side,
                limits=cfg.limits,
                gains=cfg.curl,
            )

        engine = (
            "CurlCalibRetargeter"
            if self.curl is not None
            else "ThumbIkRetargeter"
            if self.thumb_ik is not None
            else "VectorRetargeter"
        )
        override = "" if retargeting_type is None else f" override={retargeting_type!r}"
        profile = f" profile={self.vector_profile}" if self.vector_profile else ""
        print(
            f"[retarget] side={side} type={rtype}{profile} "
            f"engine={engine} yaml_type={cfg.retargeting_type}{override}",
            flush=True,
        )
        if self.curl is not None:
            print(
                f"[retarget] curl calib={cfg.curl.calib_path} "
                f"pinch_target={cfg.curl.pinch_target_link}",
                flush=True,
            )
        if self.vector is not None and self.vector_profile == "dexpilot":
            mix = cfg.vector.dexpilot.mix
            print(
                f"[retarget] dexpilot mix "
                f"w_dex={mix.w_dexpilot} w_bone={mix.w_bone} w_fist={mix.w_fist}",
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
            if self.curl is not None:
                q_raw = self.curl.retarget(raw)
            elif self.thumb_ik is not None:
                q_raw = self.thumb_ik.retarget(
                    raw,
                    flexion_scale=self.cfg.flexion_scale,
                    finger_chain=self.cfg.finger_chain,
                    thumb_tip=self.cfg.thumb_tip,
                    index_abduction=self.cfg.index_abduction,
                )
            elif self.vector is not None:
                q_raw = self.vector.retarget(
                    raw,
                    flexion_scale=self.cfg.flexion_scale,
                    finger_chain=self.cfg.finger_chain,
                    thumb_tip=self.cfg.thumb_tip,
                    index_abduction=self.cfg.index_abduction,
                )
            else:
                raise RuntimeError(
                    f"no retargeter for type={self.retargeting_type!r}"
                )
            q_cmd = self.safety.step(q_raw, dt=self._dt)
            held = False
        xyz_abs = float(frame.xyz_abs)
        return SideStep(q=q_cmd, active=active, held=held, xyz_abs=xyz_abs)


def command_topic(controller: str, suffix: str = "target_joint_position") -> str:
    name = controller.strip().strip("/")
    return f"/{name}/{suffix}"
