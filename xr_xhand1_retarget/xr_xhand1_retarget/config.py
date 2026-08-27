"""Load hand YAML configs (phase-1: xhand1)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .remap import FingerChainGains, IndexAbductionGains, ThumbTipGains

_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
_DEFAULT_CONFIG = _PACKAGE_ROOT / "configs" / "xhand1.yaml"


@dataclass
class HandConfig:
    hand_id: str
    joint_names: list[str]
    limits: np.ndarray  # (N, 2)
    flexion_scale: float
    finger_chain: FingerChainGains
    thumb_tip: ThumbTipGains
    index_abduction: IndexAbductionGains
    controller: str
    topic_suffix: str
    fsm_topic: str
    auto_movej: bool
    alpha: float
    max_delta_rad: float
    jump_reject_rad: float
    hold_on_inactive: bool
    hold_on_zero_pose: bool
    soft_collision: bool
    rate_hz: float

    @property
    def dof(self) -> int:
        return len(self.joint_names)

    @property
    def command_topic(self) -> str:
        return f"/{self.controller}/{self.topic_suffix}"

    # Back-compat
    @property
    def index_tip(self) -> IndexAbductionGains:
        return self.index_abduction


def default_config_path() -> Path:
    return _DEFAULT_CONFIG


def load_hand_config(path: str | Path | None = None) -> HandConfig:
    try:
        import yaml
    except ImportError as exc:
        raise ImportError(
            "PyYAML is required to load configs. Install with: pip install pyyaml"
        ) from exc

    cfg_path = Path(path) if path else _DEFAULT_CONFIG
    if not cfg_path.is_file():
        raise FileNotFoundError(f"hand config not found: {cfg_path}")

    with cfg_path.open("r", encoding="utf-8") as f:
        raw: dict[str, Any] = yaml.safe_load(f) or {}

    hand = raw.get("hand") or {}
    retargeting = raw.get("retargeting") or {}
    ros = raw.get("ros") or {}
    safety = raw.get("safety") or {}
    publish = raw.get("publish") or {}
    chain_raw = retargeting.get("finger_chain") or {}
    thumb_raw = retargeting.get("thumb_tip") or retargeting.get("thumb") or {}
    index_raw = (
        retargeting.get("index_abduction")
        or retargeting.get("index_tip")
        or retargeting.get("index")
        or {}
    )

    names = list(hand.get("joint_names") or [])
    limits = np.asarray(hand.get("limits"), dtype=np.float64)
    if limits.ndim != 2 or limits.shape[1] != 2 or limits.shape[0] != len(names):
        raise ValueError(
            f"hand.limits must be (N,2) matching joint_names; got {limits.shape} vs {len(names)}"
        )

    finger_chain = FingerChainGains(
        mcp_fist_ref_rad=float(chain_raw.get("mcp_fist_ref_rad", 1.15)),
        pip_fist_ref_rad=float(chain_raw.get("pip_fist_ref_rad", 1.05)),
        scale=float(chain_raw.get("scale", 1.0)),
    )
    thumb_tip = ThumbTipGains(
        rest_phi_rad=float(thumb_raw.get("rest_phi_rad", 0.28)),
        opposition_scale=float(thumb_raw.get("opposition_scale", 1.30)),
        opposition_offset=float(thumb_raw.get("opposition_offset", 0.02)),
        invert_cmc=bool(thumb_raw.get("invert_cmc", True)),
        cmc_span=float(thumb_raw.get("cmc_span", 1.832)),
        mcp_fist_ref_rad=float(thumb_raw.get("mcp_fist_ref_rad", 1.10)),
        ip_fist_ref_rad=float(thumb_raw.get("ip_fist_ref_rad", 1.05)),
        mcp_bias=float(thumb_raw.get("mcp_bias", -0.08)),
        flex_scale=float(thumb_raw.get("flex_scale", 1.0)),
    )
    index_abduction = IndexAbductionGains(
        abduction_sign=float(index_raw.get("abduction_sign", -1.0)),
        lateral_span_m=float(index_raw.get("lateral_span_m", 0.028)),
        abduction_curl_fade=float(index_raw.get("abduction_curl_fade", 0.40)),
    )

    return HandConfig(
        hand_id=str(hand.get("id") or "xhand1"),
        joint_names=names,
        limits=limits,
        flexion_scale=float(retargeting.get("flexion_scale", 1.0)),
        finger_chain=finger_chain,
        thumb_tip=thumb_tip,
        index_abduction=index_abduction,
        controller=str(ros.get("controller") or "hand_joint_controller"),
        topic_suffix=str(ros.get("topic_suffix") or "target_joint_position"),
        fsm_topic=str(ros.get("fsm_topic") or "/fsm_command"),
        auto_movej=bool(ros.get("auto_movej", True)),
        alpha=float(safety.get("alpha", 1.0)),
        max_delta_rad=float(safety.get("max_delta_rad", 0.0)),
        jump_reject_rad=float(safety.get("jump_reject_rad", 0.0)),
        hold_on_inactive=bool(safety.get("hold_on_inactive", True)),
        hold_on_zero_pose=bool(safety.get("hold_on_zero_pose", True)),
        soft_collision=bool(safety.get("soft_collision", True)),
        rate_hz=float(publish.get("rate_hz", 50.0)),
    )
