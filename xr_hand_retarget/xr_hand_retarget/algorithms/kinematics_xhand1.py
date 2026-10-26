"""Minimal URDF forward kinematics for XHand1 named links / tips."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

_PACKAGE_ROOT = Path(__file__).resolve().parents[2]
_DEFAULT_RIGHT_URDF = _PACKAGE_ROOT / "assets" / "xhand_right.urdf"
_DEFAULT_LEFT_URDF = _PACKAGE_ROOT / "assets" / "xhand_left.urdf"

DEFAULT_URDF_JOINT_NAMES_RIGHT = [
    "right_hand_thumb_bend_joint",
    "right_hand_thumb_rota_joint1",
    "right_hand_thumb_rota_joint2",
    "right_hand_index_bend_joint",
    "right_hand_index_joint1",
    "right_hand_index_joint2",
    "right_hand_mid_joint1",
    "right_hand_mid_joint2",
    "right_hand_ring_joint1",
    "right_hand_ring_joint2",
    "right_hand_pinky_joint1",
    "right_hand_pinky_joint2",
]

DEFAULT_URDF_JOINT_NAMES_LEFT = [
    "left_hand_thumb_bend_joint",
    "left_hand_thumb_rota_joint1",
    "left_hand_thumb_rota_joint2",
    "left_hand_index_bend_joint",
    "left_hand_index_joint1",
    "left_hand_index_joint2",
    "left_hand_mid_joint1",
    "left_hand_mid_joint2",
    "left_hand_ring_joint1",
    "left_hand_ring_joint2",
    "left_hand_pinky_joint1",
    "left_hand_pinky_joint2",
]

# Short aliases used in retargeting YAML (resolved per side).
LINK_ALIASES_RIGHT = {
    "hand_base": "right_hand_link",
    "thumb_cmc": "right_hand_thumb_bend_link",
    "thumb_mcp": "right_hand_thumb_rota_link1",
    "thumb_ip": "right_hand_thumb_rota_link2",
    "thumb_tip": "right_hand_thumb_rota_tip",
    "index_abd": "right_hand_index_bend_link",
    "index_mcp": "right_hand_index_rota_link1",
    "index_pip": "right_hand_index_rota_link2",
    "index_tip": "right_hand_index_rota_tip",
    "middle_mcp": "right_hand_mid_link1",
    "middle_pip": "right_hand_mid_link2",
    "middle_tip": "right_hand_mid_tip",
    "ring_mcp": "right_hand_ring_link1",
    "ring_pip": "right_hand_ring_link2",
    "ring_tip": "right_hand_ring_tip",
    "pinky_mcp": "right_hand_pinky_link1",
    "pinky_pip": "right_hand_pinky_link2",
    "pinky_tip": "right_hand_pinky_tip",
}

LINK_ALIASES_LEFT = {
    "hand_base": "left_hand_link",
    "thumb_cmc": "left_hand_thumb_bend_link",
    "thumb_mcp": "left_hand_thumb_rota_link1",
    "thumb_ip": "left_hand_thumb_rota_link2",
    "thumb_tip": "left_hand_thumb_rota_tip",
    "index_abd": "left_hand_index_bend_link",
    "index_mcp": "left_hand_index_rota_link1",
    "index_pip": "left_hand_index_rota_link2",
    "index_tip": "left_hand_index_rota_tip",
    "middle_mcp": "left_hand_mid_link1",
    "middle_pip": "left_hand_mid_link2",
    "middle_tip": "left_hand_mid_tip",
    "ring_mcp": "left_hand_ring_link1",
    "ring_pip": "left_hand_ring_link2",
    "ring_tip": "left_hand_ring_tip",
    "pinky_mcp": "left_hand_pinky_link1",
    "pinky_pip": "left_hand_pinky_link2",
    "pinky_tip": "left_hand_pinky_tip",
}


def _parse_xyz(text: str) -> np.ndarray:
    return np.asarray([float(x) for x in text.split()][:3], dtype=np.float64)


def _rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def _axis_angle_matrix(axis: np.ndarray, theta: float) -> np.ndarray:
    axis = axis / max(float(np.linalg.norm(axis)), 1e-12)
    x, y, z = axis
    c, s = np.cos(theta), np.sin(theta)
    t = 1.0 - c
    return np.array(
        [
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
        ],
        dtype=np.float64,
    )


def _origin_matrix(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    t = np.eye(4, dtype=np.float64)
    t[:3, :3] = _rpy_matrix(rpy)
    t[:3, 3] = xyz
    return t


@dataclass
class _JointRec:
    name: str
    parent: str
    child: str
    joint_type: str
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray
    mimic: tuple[str, float, float] | None = None


class XHandFK:
    """q[12] → named link positions in hand_base frame."""

    def __init__(
        self,
        urdf_path: str | Path,
        *,
        base_link: str,
        urdf_joint_names: list[str],
        aliases: dict[str, str] | None = None,
    ):
        self.urdf_path = Path(urdf_path)
        self.base_link = base_link
        self.urdf_joint_names = list(urdf_joint_names)
        self.aliases = dict(aliases or {})
        self._joint_by_child: dict[str, _JointRec] = {}
        self._joint_by_name: dict[str, _JointRec] = {}
        self._parse_urdf()
        self._all_links = set()
        for rec in self._joint_by_child.values():
            self._all_links.add(rec.parent)
            self._all_links.add(rec.child)
            if rec.name:
                self._joint_by_name[rec.name] = rec
        self._all_links.add(self.base_link)

    def _parse_urdf(self) -> None:
        root = ET.parse(self.urdf_path).getroot()
        for joint in root.findall("joint"):
            origin = joint.find("origin")
            xyz = (
                _parse_xyz(origin.get("xyz", "0 0 0"))
                if origin is not None
                else np.zeros(3)
            )
            rpy = (
                _parse_xyz(origin.get("rpy", "0 0 0"))
                if origin is not None
                else np.zeros(3)
            )
            axis_el = joint.find("axis")
            axis = (
                _parse_xyz(axis_el.get("xyz", "1 0 0"))
                if axis_el is not None
                else np.array([1.0, 0.0, 0.0])
            )
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            mimic_el = joint.find("mimic")
            mimic = None
            if mimic_el is not None and (mimic_el.get("joint") or "").strip():
                mimic = (
                    str(mimic_el.get("joint")),
                    float(mimic_el.get("multiplier", "1") or 1.0),
                    float(mimic_el.get("offset", "0") or 0.0),
                )
            rec = _JointRec(
                name=joint.get("name", ""),
                parent=parent,
                child=child,
                joint_type=joint.get("type", "fixed"),
                xyz=xyz,
                rpy=rpy,
                axis=axis,
                mimic=mimic,
            )
            self._joint_by_child[child] = rec

    def resolve_link(self, name: str) -> str:
        if name in self.aliases:
            return self.aliases[name]
        if name in self._all_links:
            return name
        matches = [ln for ln in self._all_links if ln.endswith(name)]
        if len(matches) == 1:
            return matches[0]
        raise ValueError(f"link not found in URDF: {name}")

    def _joint_angle(self, joint_name: str, q: np.ndarray) -> float:
        if joint_name in self.urdf_joint_names:
            return float(q[self.urdf_joint_names.index(joint_name)])
        rec = self._joint_by_name.get(joint_name)
        if rec is not None and rec.mimic is not None:
            src, mul, off = rec.mimic
            return float(mul) * self._joint_angle(src, q) + float(off)
        return 0.0

    def _link_transform(
        self, link: str, q: np.ndarray, cache: dict[str, np.ndarray]
    ) -> np.ndarray:
        if link in cache:
            return cache[link]
        if link == self.base_link:
            cache[link] = np.eye(4, dtype=np.float64)
            return cache[link]
        rec = self._joint_by_child.get(link)
        if rec is None:
            raise ValueError(f"no joint leads to link {link}")
        t_parent = self._link_transform(rec.parent, q, cache)
        t_joint = _origin_matrix(rec.xyz, rec.rpy)
        if rec.joint_type == "revolute":
            theta = self._joint_angle(rec.name, q)
            r = np.eye(4, dtype=np.float64)
            r[:3, :3] = _axis_angle_matrix(rec.axis, theta)
            t_joint = t_joint @ r
        cache[link] = t_parent @ t_joint
        return cache[link]

    def link_positions(self, q: np.ndarray, link_names: list[str]) -> np.ndarray:
        q = np.asarray(q, dtype=np.float64).reshape(-1)
        cache: dict[str, np.ndarray] = {}
        resolved = [self.resolve_link(n) for n in link_names]
        return np.asarray(
            [self._link_transform(ln, q, cache)[:3, 3].copy() for ln in resolved],
            dtype=np.float64,
        )

    def link_position(self, q: np.ndarray, link_name: str) -> np.ndarray:
        return self.link_positions(q, [link_name])[0]

    def link_positions_jacobians(
        self, q: np.ndarray, link_names: list[str]
    ) -> tuple[np.ndarray, np.ndarray]:
        """Positions (L,3) and geometric Jacobians (L,3,nq) in hand_base frame.

        Matches DexPilot/dex-retargeting style: one FK pass + analytic ∂p/∂q
        for revolute joints (ω × (p − o)), avoiding finite-difference FK.
        """
        q = np.asarray(q, dtype=np.float64).reshape(-1)
        nq = len(self.urdf_joint_names)
        name_to_qi = {n: i for i, n in enumerate(self.urdf_joint_names)}
        cache: dict[str, np.ndarray] = {}
        # Allow already-resolved URDF names (hot path skips alias lookup).
        resolved = [
            n if n in self._all_links else self.resolve_link(n) for n in link_names
        ]
        for ln in resolved:
            self._link_transform(ln, q, cache)

        positions = np.zeros((len(resolved), 3), dtype=np.float64)
        jacs = np.zeros((len(resolved), 3, nq), dtype=np.float64)
        for li, ln in enumerate(resolved):
            p = cache[ln][:3, 3].copy()
            positions[li] = p
            cur = ln
            while cur != self.base_link:
                rec = self._joint_by_child.get(cur)
                if rec is None:
                    break
                if rec.joint_type == "revolute":
                    qi = None
                    scale = 1.0
                    if rec.name in name_to_qi:
                        qi = name_to_qi[rec.name]
                    elif rec.mimic is not None:
                        src, mul, _off = rec.mimic
                        if src in name_to_qi:
                            qi = name_to_qi[src]
                            scale = float(mul)
                    if qi is not None:
                        t_parent = cache[rec.parent]
                        t_joint0 = t_parent @ _origin_matrix(rec.xyz, rec.rpy)
                        omega = t_joint0[:3, :3] @ rec.axis
                        wn = float(np.linalg.norm(omega))
                        if wn > 1e-12:
                            omega = omega / wn
                            o = t_joint0[:3, 3]
                            jacs[li, :, qi] = jacs[li, :, qi] + scale * np.cross(
                                omega, p - o
                            )
                cur = rec.parent
        return positions, jacs


def default_urdf_path(side: str) -> Path:
    return _DEFAULT_LEFT_URDF if side.lower().startswith("l") else _DEFAULT_RIGHT_URDF


def make_hand_fk(side: str, *, urdf_path: str | Path | None = None) -> XHandFK:
    left = side.lower().startswith("l")
    path = Path(urdf_path) if urdf_path else default_urdf_path(side)
    if not path.is_file():
        raise FileNotFoundError(f"URDF not found: {path}")
    return XHandFK(
        path,
        base_link="left_hand_link" if left else "right_hand_link",
        urdf_joint_names=(
            list(DEFAULT_URDF_JOINT_NAMES_LEFT)
            if left
            else list(DEFAULT_URDF_JOINT_NAMES_RIGHT)
        ),
        aliases=LINK_ALIASES_LEFT if left else LINK_ALIASES_RIGHT,
    )


def load_urdf_joint_limits_velocity(
    side: str,
    *,
    urdf_path: str | Path | None = None,
    joint_names: list[str] | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Return (limits (N,2), velocity (N,)) for actuated URDF joints in command order."""
    left = side.lower().startswith("l")
    path = Path(urdf_path) if urdf_path else default_urdf_path(side)
    names = list(
        joint_names
        or (
            DEFAULT_URDF_JOINT_NAMES_LEFT
            if left
            else DEFAULT_URDF_JOINT_NAMES_RIGHT
        )
    )
    root = ET.parse(path).getroot()
    by_name: dict[str, tuple[float, float, float]] = {}
    for joint in root.findall("joint"):
        lim = joint.find("limit")
        if lim is None:
            continue
        name = joint.get("name") or ""
        lo = float(lim.get("lower", "0"))
        hi = float(lim.get("upper", "0"))
        vel = float(lim.get("velocity", "0") or 0.0)
        by_name[name] = (lo, hi, vel)
    limits = np.zeros((len(names), 2), dtype=np.float64)
    velocity = np.zeros(len(names), dtype=np.float64)
    for i, name in enumerate(names):
        if name not in by_name:
            raise KeyError(f"joint {name!r} not found in URDF {path}")
        lo, hi, vel = by_name[name]
        limits[i, 0] = lo
        limits[i, 1] = hi
        velocity[i] = vel
    return limits, velocity
