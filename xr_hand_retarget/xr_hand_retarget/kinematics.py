"""Wuji Hand 2 URDF FK (kinematics-only assets; reuses XHandFK)."""

from __future__ import annotations

from pathlib import Path

from xr_hand_retarget.algorithms.kinematics_xhand1 import XHandFK, load_urdf_joint_limits_velocity

_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
_ASSETS = _PACKAGE_ROOT / "assets"

HAND2_JOINT_NAMES = [
    "thumb_cmc_flex",
    "thumb_cmc_abd",
    "thumb_mcp",
    "thumb_ip",
    "index_finger_mcp_flex",
    "index_finger_mcp_abd",
    "index_finger_pip",
    "index_finger_dip",
    "middle_finger_mcp_flex",
    "middle_finger_mcp_abd",
    "middle_finger_pip",
    "middle_finger_dip",
    "ring_mcp_flex",
    "ring_mcp_abd",
    "ring_pip",
    "ring_dip",
    "pinky_mcp_flex",
    "pinky_mcp_abd",
    "pinky_pip",
    "pinky_dip",
]

# Short aliases → links in the kinematics URDF (no side prefix).
HAND2_LINK_ALIASES = {
    "hand_base": "hand_base",
    "wrist": "wrist",
    "thumb_cmc": "thumb_proximal",
    "thumb_mcp": "thumb_proximal_abd",
    "thumb_ip": "thumb_middle",
    "thumb_tip": "thumb_tip",
    "index_mcp": "index_finger_proximal",
    "index_pip": "index_finger_middle",
    "index_dip": "index_finger_distal",
    "index_tip": "index_finger_tip",
    "middle_mcp": "middle_finger_proximal",
    "middle_pip": "middle_finger_middle",
    "middle_dip": "middle_finger_distal",
    "middle_tip": "middle_finger_tip",
    "ring_mcp": "ring_proximal",
    "ring_pip": "ring_middle",
    "ring_dip": "ring_distal",
    "ring_tip": "ring_tip",
    "pinky_mcp": "pinky_proximal",
    "pinky_pip": "pinky_middle",
    "pinky_dip": "pinky_distal",
    "pinky_tip": "pinky_tip",
}


def default_hand2_urdf(side: str) -> Path:
    name = "left" if side.lower().startswith("l") else "right"
    return _ASSETS / f"wuji_hand2_{name}.urdf"


def make_hand2_fk(side: str, *, urdf_path: str | Path | None = None) -> XHandFK:
    path = Path(urdf_path) if urdf_path else default_hand2_urdf(side)
    if not path.is_file():
        raise FileNotFoundError(f"Hand 2 URDF not found: {path}")
    return XHandFK(
        path,
        base_link="hand_base",
        urdf_joint_names=list(HAND2_JOINT_NAMES),
        aliases=dict(HAND2_LINK_ALIASES),
    )


def load_hand2_limits(
    side: str, *, urdf_path: str | Path | None = None
) -> tuple:
    path = Path(urdf_path) if urdf_path else default_hand2_urdf(side)
    return load_urdf_joint_limits_velocity(
        side, urdf_path=path, joint_names=list(HAND2_JOINT_NAMES)
    )


# LinkerHand family — fa_w2 command order (DIP mimic omitted).
O6_JOINT_NAMES = [
    "thumb_joint1",
    "thumb_joint2",
    "index_joint",
    "middle_joint",
    "ring_joint",
    "pinky_joint",
]
L6_JOINT_NAMES = list(O6_JOINT_NAMES)
O7_JOINT_NAMES = [
    "thumb_joint1",
    "thumb_joint2",
    "thumb_joint3",
    "index_joint",
    "middle_joint",
    "ring_joint",
    "pinky_joint",
]

O6_LINK_ALIASES = {
    "hand_base": "hand_base",
    "thumb_mcp": "thumb_metacarpals",
    "thumb_ip": "thumb_distal",
    "thumb_tip": "thumb_tip",
    "thumb_pad": "thumb_pad",
    "index_mcp": "index_proximal",
    "index_pip": "index_distal",
    "index_tip": "index_tip",
    "index_pad": "index_pad",
    "middle_mcp": "middle_proximal",
    "middle_pip": "middle_distal",
    "middle_tip": "middle_tip",
    "middle_pad": "middle_pad",
    "ring_mcp": "ring_proximal",
    "ring_pip": "ring_distal",
    "ring_tip": "ring_tip",
    "ring_pad": "ring_pad",
    "pinky_mcp": "pinky_proximal",
    "pinky_pip": "pinky_distal",
    "pinky_tip": "pinky_tip",
    "pinky_pad": "pinky_pad",
}
O7_LINK_ALIASES = {
    **O6_LINK_ALIASES,
    "thumb_ip": "thumb_proximal",
    "index_pip": "index_middle",
    "middle_pip": "middle_middle",
    "ring_pip": "ring_middle",
    "pinky_pip": "pinky_middle",
}

_LINKER_JOINTS = {
    "o6": O6_JOINT_NAMES,
    "l6": L6_JOINT_NAMES,
    "o7": O7_JOINT_NAMES,
}
_LINKER_ALIASES = {
    "o6": O6_LINK_ALIASES,
    "l6": O6_LINK_ALIASES,
    "o7": O7_LINK_ALIASES,
}


def _linker_model(model: str) -> str:
    key = str(model or "o6").strip().lower().replace("-", "_")
    aliases = {
        "o6": "o6",
        "linkerhand_o6": "o6",
        "l6": "l6",
        "linkerhand_l6": "l6",
        "o7": "o7",
        "linkerhand_o7": "o7",
    }
    if key not in aliases:
        raise ValueError(f"unknown LinkerHand model {model!r}; expected o6|l6|o7")
    return aliases[key]


def default_linker_urdf(model: str, side: str) -> Path:
    name = "left" if side.lower().startswith("l") else "right"
    return _ASSETS / f"{_linker_model(model)}_{name}.urdf"


def default_o6_urdf(side: str) -> Path:
    return default_linker_urdf("o6", side)


def make_linker_fk(
    side: str, *, model: str = "o6", urdf_path: str | Path | None = None
) -> XHandFK:
    key = _linker_model(model)
    path = Path(urdf_path) if urdf_path else default_linker_urdf(key, side)
    if not path.is_file():
        raise FileNotFoundError(f"LinkerHand {key} URDF not found: {path}")
    return XHandFK(
        path,
        base_link="hand_base",
        urdf_joint_names=list(_LINKER_JOINTS[key]),
        aliases=dict(_LINKER_ALIASES[key]),
    )


def make_o6_fk(side: str, *, urdf_path: str | Path | None = None) -> XHandFK:
    return make_linker_fk(side, model="o6", urdf_path=urdf_path)


def load_linker_limits(
    side: str, *, model: str = "o6", urdf_path: str | Path | None = None
) -> tuple:
    key = _linker_model(model)
    path = Path(urdf_path) if urdf_path else default_linker_urdf(key, side)
    return load_urdf_joint_limits_velocity(
        side, urdf_path=path, joint_names=list(_LINKER_JOINTS[key])
    )


def load_o6_limits(
    side: str, *, urdf_path: str | Path | None = None
) -> tuple:
    return load_linker_limits(side, model="o6", urdf_path=urdf_path)
