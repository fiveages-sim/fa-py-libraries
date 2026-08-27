"""XRoboToolkit hand tracking → XHand1 12-DOF remapping (phase-1)."""

from .collision import soft_collision_project
from .config import HandConfig, default_config_path, load_hand_config
from .remap import (
    XHAND1_JOINT_NAMES,
    XHAND1_LIMITS,
    FingerChainGains,
    IndexAbductionGains,
    IndexTipGains,
    ThumbTipGains,
    openxr26_to_xhand1_q,
)
from .safety import CommandSafety

__all__ = [
    "CommandSafety",
    "FingerChainGains",
    "HandConfig",
    "IndexAbductionGains",
    "IndexTipGains",
    "ThumbTipGains",
    "XHAND1_JOINT_NAMES",
    "XHAND1_LIMITS",
    "default_config_path",
    "load_hand_config",
    "openxr26_to_xhand1_q",
    "soft_collision_project",
]
