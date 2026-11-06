"""Switchable command limits: URDF box, velocity, acceleration, FK clearance.

Cartesian layer: link-origin spheres (URDF ``<mimic>`` expanded). Each pair has

  d_i(q) = ||p_a(q) - p_b(q)||
  h_i    = d_i - d_min,i

with per-kind ``d_min`` (rake / pack / pinch). Soft push starts when
``0 <= h_i < activation`` (OCS2-style); hard project when ``h_i < 0``.
Pad-level FCL is not used; pack/pinch prefer ``*_pad`` link origins (fallback
``*_tip``). Optional ``sphere_radius_m`` turns clearance into sphere–sphere:
``h = d - 2r - clearance``. Swap link names later without changing pair policy.

Optional ``fk.cspace``: precomputed joint-space safe grid; projects unsafe ``q``
to weighted nearest safe cell (replaces fist_inside gate + axis search).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

from xr_hand_retarget.cspace import (
    DEFAULT_BINS,
    DEFAULT_TABLE_TEMPLATE,
    DEFAULT_WEIGHTS,
    CspaceTable,
    resolve_cspace_path,
)

# Sweep: thumb bone vs PIP / unreachable fingertips.
_DEFAULT_RAKE_PAIRS: tuple[tuple[str, str], ...] = (
    ("thumb_tip", "ring_tip"),
    ("thumb_tip", "pinky_tip"),
    ("thumb_tip", "index_pip"),
    ("thumb_tip", "middle_pip"),
    ("thumb_tip", "ring_pip"),
    ("thumb_tip", "pinky_pip"),
)

# Pad–pad when URDF has *_pad; tip–tip remains a fallback alias.
_DEFAULT_PACK_PAIRS: tuple[tuple[str, str, str], ...] = (
    ("thumb_pad", "index_pad", "index"),
    ("thumb_pad", "middle_pad", "middle"),
)

_FINGER_JOINT = {
    "index": "index_joint",
    "middle": "middle_joint",
    "ring": "ring_joint",
    "pinky": "pinky_joint",
}

_SOFT_STEP_RAD = 0.15


@dataclass
class MotionSafetyGains:
    """Per-layer gates. ``enabled=False`` skips pos/vel/acc entirely."""

    enabled: bool = True
    position_enabled: bool = True
    position_margin_rad: float = 0.0
    velocity_enabled: bool = False
    use_urdf_velocity: bool = True
    velocity_scale: float = 1.0
    velocity_limit_rad_s: float = 0.0
    velocity_limit_per_joint: tuple[float, ...] | None = None
    accel_enabled: bool = False
    accel_limit_rad_s2: float = 0.0
    mesh_enabled: bool = False
    fk_enabled: bool = False
    fk_min_clearance_m: float = 0.018
    fk_pack_clearance_m: float = 0.040
    fk_pack_j2_min: float = 0.35
    fk_pack_finger_min: float = 0.55
    fk_pinch_clearance_m: float = 0.0
    fk_activation_m: float = 0.010
    fk_thumb_palm_m: float = 0.005
    fk_sphere_radius_m: float = 0.0
    fk_scale_iters: int = 12
    fk_pairs: tuple[tuple[str, str], ...] | None = None
    fk_pinch_pairs: tuple[tuple[str, str], ...] | None = None
    fk_pack_pairs: tuple[tuple[str, str], ...] | None = None
    fk_project: str = "thumb_j2"
    fk_pinch_j2_index: float = 1.0
    fk_pinch_j2_middle: float = 1.3
    fk_pinch_j2_halfwidth: float = 0.25
    # Thumb-outside fist / mid-state policy (robot q, not optics).
    fk_along_j2: float = 0.35
    fk_fist_finger_min: float = 0.90
    fk_hold_fingers_until_thumb_out: bool = True
    # Precomputed joint-space safe set (O6).
    fk_cspace_enabled: bool = False
    fk_cspace_path: str = DEFAULT_TABLE_TEMPLATE
    fk_cspace_weights: tuple[float, ...] = DEFAULT_WEIGHTS
    fk_cspace_bins: tuple[int, ...] = DEFAULT_BINS


def _parse_fk_pairs(raw) -> tuple[tuple[str, str], ...] | None:
    if raw is None:
        return None
    out: list[tuple[str, str]] = []
    for item in raw:
        if isinstance(item, (list, tuple)) and len(item) == 2:
            out.append((str(item[0]), str(item[1])))
    return tuple(out)


def _parse_fk_project(raw) -> str:
    key = str(raw or "thumb_j2").strip().lower().replace("-", "_")
    aliases = {
        "thumb_j2": "thumb_j2",
        "thumb_joint2": "thumb_j2",
        "j2": "thumb_j2",
        "thumb": "thumb_j2",
        "all": "all",
        "scale": "all",
        "q": "all",
    }
    return aliases.get(key, "thumb_j2")


def _pair_key(a: str, b: str) -> tuple[str, str]:
    return (a, b) if a <= b else (b, a)


def _joint_index(
    names: list[str] | None, want: str, n: int, default: int | None
) -> int | None:
    key = want.lower()
    for i, name in enumerate(names or []):
        if str(name).lower() == key:
            return i
    if default is not None and 0 <= default < n:
        return default
    return None


def _finger_of_tip(link: str) -> str | None:
    for name in _FINGER_JOINT:
        if link in (f"{name}_tip", f"{name}_pip", f"{name}_pad"):
            return name
    if link in ("thumb_tip", "thumb_pad"):
        return "thumb"
    return None


def urdf_has_collision_mesh(urdf_path: str | Path | None) -> bool:
    if not urdf_path:
        return False
    path = Path(urdf_path)
    if not path.is_file():
        return False
    root = ET.parse(path).getroot()
    for col in root.iter("collision"):
        mesh = col.find("geometry/mesh")
        if mesh is not None and (mesh.get("filename") or "").strip():
            return True
    return False


def parse_motion_safety(safety: dict | None) -> MotionSafetyGains:
    raw = safety or {}
    motion = raw.get("motion") or {}
    pos = motion.get("position") or raw.get("position") or {}
    vel = motion.get("velocity") or raw.get("velocity") or {}
    acc = motion.get("acceleration") or raw.get("acceleration") or {}
    mesh = motion.get("mesh") or raw.get("mesh") or {}
    fk_block = pos.get("fk") or raw.get("fk") or {}
    pinch_j2 = fk_block.get("pinch_j2") or {}

    if "enabled" in raw:
        master = bool(raw["enabled"])
    elif "enabled" in motion:
        master = bool(motion["enabled"])
    else:
        master = True

    clamp_legacy = bool(raw.get("clamp_urdf", True))
    vel_legacy = float(raw.get("max_joint_vel_rad_s", 0.0) or 0.0)

    per = vel.get("per_joint") or vel.get("limit_per_joint")
    per_tuple: tuple[float, ...] | None = None
    if per is not None:
        per_tuple = tuple(float(x) for x in per)

    limit_rad = float(vel.get("limit_rad_s", motion.get("velocity_limit_rad_s", 0.0)) or 0.0)
    if limit_rad <= 0.0 and vel_legacy > 0.0 and "velocity" not in raw and "velocity" not in motion:
        limit_rad = vel_legacy

    accel_limit = float(acc.get("limit_rad_s2", motion.get("accel_limit_rad_s2", 0.0)) or 0.0)
    accel_on = bool(acc.get("enabled", accel_limit > 0.0))

    vel_on = bool(
        vel.get(
            "enabled",
            per_tuple is not None or limit_rad > 0.0 or vel_legacy > 0.0,
        )
    )

    if "pinch_pairs" in fk_block and fk_block.get("pinch_pairs") is not None:
        pinch_pairs = _parse_fk_pairs(fk_block.get("pinch_pairs")) or ()
    else:
        pinch_pairs = None
    if "pack_pairs" in fk_block and fk_block.get("pack_pairs") is not None:
        pack_pairs = _parse_fk_pairs(fk_block.get("pack_pairs")) or ()
    else:
        pack_pairs = None

    rake = fk_block.get("rake_clearance_m", fk_block.get("min_clearance_m", 0.018))

    cspace = fk_block.get("cspace") or {}
    cspace_weights = cspace.get("weights")
    if cspace_weights is not None:
        w_tuple = tuple(float(x) for x in cspace_weights)
    else:
        w_tuple = DEFAULT_WEIGHTS
    cspace_bins = cspace.get("bins")
    if cspace_bins is not None:
        b_tuple = tuple(int(x) for x in cspace_bins)
    else:
        b_tuple = DEFAULT_BINS

    return MotionSafetyGains(
        enabled=master,
        position_enabled=bool(pos.get("enabled", clamp_legacy)),
        position_margin_rad=float(pos.get("margin_rad", motion.get("position_margin_rad", 0.0))),
        velocity_enabled=vel_on,
        use_urdf_velocity=bool(vel.get("use_urdf", motion.get("use_urdf_velocity", True))),
        velocity_scale=float(vel.get("scale", motion.get("velocity_scale", 1.0))),
        velocity_limit_rad_s=limit_rad,
        velocity_limit_per_joint=per_tuple,
        accel_enabled=accel_on,
        accel_limit_rad_s2=accel_limit,
        mesh_enabled=bool(mesh.get("enabled", False)),
        fk_enabled=bool(fk_block.get("enabled", False)),
        fk_min_clearance_m=float(rake),
        fk_pack_clearance_m=float(fk_block.get("pack_clearance_m", 0.040)),
        fk_pack_j2_min=float(fk_block.get("pack_j2_min", 0.35)),
        fk_pack_finger_min=float(fk_block.get("pack_finger_min", 0.55)),
        fk_pinch_clearance_m=float(fk_block.get("pinch_clearance_m", 0.0)),
        fk_activation_m=float(fk_block.get("activation_m", 0.010)),
        fk_thumb_palm_m=float(fk_block.get("thumb_palm_m", 0.005)),
        fk_sphere_radius_m=float(fk_block.get("sphere_radius_m", 0.0)),
        fk_scale_iters=int(fk_block.get("scale_iters", 12)),
        fk_pairs=_parse_fk_pairs(fk_block.get("pairs")),
        fk_pinch_pairs=pinch_pairs,
        fk_pack_pairs=pack_pairs,
        fk_project=_parse_fk_project(fk_block.get("project", "thumb_j2")),
        fk_pinch_j2_index=float(pinch_j2.get("index", 1.0)),
        fk_pinch_j2_middle=float(pinch_j2.get("middle", 1.3)),
        fk_pinch_j2_halfwidth=float(pinch_j2.get("halfwidth", 0.25)),
        fk_along_j2=float(fk_block.get("along_j2", 0.35)),
        fk_fist_finger_min=float(fk_block.get("fist_finger_min", 0.90)),
        fk_hold_fingers_until_thumb_out=bool(
            fk_block.get("hold_fingers_until_thumb_out", True)
        ),
        fk_cspace_enabled=bool(cspace.get("enabled", False)),
        fk_cspace_path=str(cspace.get("table", DEFAULT_TABLE_TEMPLATE)),
        fk_cspace_weights=w_tuple,
        fk_cspace_bins=b_tuple,
    )


class JointMotionFilter:
    """Clamp / rate-limit commanded q. Disable any layer independently."""

    def __init__(
        self,
        limits: np.ndarray,
        urdf_velocity: np.ndarray | None,
        gains: MotionSafetyGains,
        dt: float,
        *,
        urdf_path: str | Path | None = None,
        fk=None,
        joint_names: list[str] | None = None,
        side: str | None = None,
        load_cspace: bool = True,
    ):
        self.gains = gains
        self._dt = float(max(dt, 1e-4))
        self._side = str(side or "").strip().lower() or None
        base = np.asarray(limits, dtype=np.float64)
        if base.ndim != 2 or base.shape[1] != 2:
            raise ValueError(f"limits must be (N,2), got {base.shape}")
        n = int(base.shape[0])
        margin = max(float(gains.position_margin_rad), 0.0)
        lo = base[:, 0] + margin
        hi = base[:, 1] - margin
        bad = hi < lo
        lo = np.where(bad, base[:, 0], lo)
        hi = np.where(bad, base[:, 1], hi)
        self._lo = lo
        self._hi = hi
        self._vmax = self._resolve_vmax(n, urdf_velocity)
        self._q: np.ndarray | None = None
        self._v: np.ndarray | None = None
        self.velocity_clips = 0
        self.fk_blocks = 0
        self.cspace_hits = 0
        self.mesh_active = False
        self.mode = "along"
        self._cspace: CspaceTable | None = None
        self._fk = fk if (gains.fk_enabled and fk is not None) else None
        self._fk_links: list[str] = []
        self._rake_pairs: list[tuple[int, int]] = []
        self._pack_pairs: list[tuple[int, int, str]] = []
        self._palm_idx: tuple[int, int, int] | None = None
        self._thumb_tip_idx: int | None = None
        self._palm_n_ref: np.ndarray | None = None
        names = list(joint_names) if joint_names else None
        self._joint_names = names
        self._j2_idx = _joint_index(names, "thumb_joint2", n, 1 if n >= 2 else None)
        self._j1_idx = _joint_index(names, "thumb_joint1", n, 0 if n >= 1 else None)
        self._finger_q_idx: dict[str, int] = {}
        for finger, jname in _FINGER_JOINT.items():
            idx = _joint_index(names, jname, n, None)
            if idx is not None:
                self._finger_q_idx[finger] = idx
        if self._fk is not None:
            self._init_fk()
        if load_cspace and gains.fk_cspace_enabled:
            self._load_cspace()
        if self._cspace is not None:
            print(
                f"[safety] cspace on table={self._cspace.path} "
                f"safe_frac={self._cspace.safe_fraction:.3f} "
                f"weights={list(self._cspace.weights)}",
                flush=True,
            )
        else:
            print(
                f"[safety] thumb_out along_j2={gains.fk_along_j2:.2f} "
                f"fist_finger_min={gains.fk_fist_finger_min:.2f} "
                f"hold_fingers={int(gains.fk_hold_fingers_until_thumb_out)}",
                flush=True,
            )
        if gains.mesh_enabled:
            print(
                "[safety] mesh.enabled: no FCL; FK spheres apply if fk.enabled "
                f"(urdf={urdf_path})",
                flush=True,
            )

    def _load_cspace(self) -> None:
        g = self.gains
        side = self._side or "right"
        path = resolve_cspace_path(g.fk_cspace_path, side)
        if not path.is_file():
            print(
                f"[safety] cspace.enabled but table missing: {path}; "
                "falling back to legacy FK project",
                flush=True,
            )
            return
        try:
            table = CspaceTable.load(path)
        except Exception as exc:  # noqa: BLE001
            print(f"[safety] cspace load failed ({path}): {exc}; legacy FK", flush=True)
            return
        n = int(self._lo.shape[0])
        if table.dof != n:
            print(
                f"[safety] cspace dof {table.dof} != filter dof {n}; legacy FK",
                flush=True,
            )
            return
        # Optional runtime weight override from yaml.
        if g.fk_cspace_weights and len(g.fk_cspace_weights) == n:
            table.weights = np.asarray(g.fk_cspace_weights, dtype=np.float64)
        self._cspace = table

    @property
    def cspace_active(self) -> bool:
        return self._cspace is not None

    def _fk_has_link(self, name: str) -> bool:
        if self._fk is None:
            return False
        try:
            self._fk.resolve_link(name)
            return True
        except ValueError:
            return False

    def _pad_or_tip(self, name: str) -> str:
        """Prefer ``*_pad`` when present in URDF; else tip / pip aliases."""
        if name.endswith("_pad"):
            if self._fk_has_link(name):
                return name
            tip = name[: -len("_pad")] + "_tip"
            if self._fk_has_link(tip):
                return tip
        return name

    def _init_fk(self) -> None:
        g = self.gains
        n = int(self._lo.shape[0])
        rake = g.fk_pairs if g.fk_pairs is not None else _DEFAULT_RAKE_PAIRS
        if g.fk_pack_pairs is not None:
            pack_src = [
                (a, b, _finger_of_tip(b) or _finger_of_tip(a) or "")
                for a, b in g.fk_pack_pairs
            ]
        elif g.fk_pinch_pairs is not None:
            pack_src = [
                (a, b, _finger_of_tip(b) or _finger_of_tip(a) or "")
                for a, b in g.fk_pinch_pairs
            ]
        else:
            pack_src = list(_DEFAULT_PACK_PAIRS)

        links: list[str] = []
        seen: set[str] = set()

        def add(name: str) -> int:
            name = self._pad_or_tip(name)
            if name not in seen:
                seen.add(name)
                links.append(name)
            return links.index(name)

        pinch_keys = set()
        if g.fk_pinch_pairs:
            pinch_keys = {_pair_key(a, b) for a, b in g.fk_pinch_pairs}
        else:
            pinch_keys = {
                _pair_key(self._pad_or_tip(a), self._pad_or_tip(b))
                for a, b, _f in _DEFAULT_PACK_PAIRS
            }

        rake_idx: list[tuple[int, int]] = []
        for a, b in rake:
            if _pair_key(a, b) in pinch_keys:
                continue
            rake_idx.append((add(a), add(b)))

        pack_idx: list[tuple[int, int, str]] = []
        for a, b, finger in pack_src:
            if not finger:
                continue
            pack_idx.append((add(a), add(b), finger))

        self._thumb_tip_idx = add("thumb_tip")
        if float(g.fk_thumb_palm_m) > 0.0:
            self._palm_idx = (add("index_mcp"), add("middle_mcp"), add("pinky_mcp"))
        self._fk_links = links
        self._rake_pairs = rake_idx
        self._pack_pairs = pack_idx
        q0 = np.clip(np.zeros(n), self._lo, self._hi)
        pos0 = self._fk.link_positions(q0, self._fk_links)
        if self._palm_idx is not None and self._thumb_tip_idx is not None:
            i0, i1, i2 = self._palm_idx
            nrm = np.cross(pos0[i1] - pos0[i0], pos0[i2] - pos0[i0])
            nn = float(np.linalg.norm(nrm))
            if nn > 1e-9:
                nrm = nrm / nn
                h0 = float(np.dot(pos0[self._thumb_tip_idx] - pos0[i0], nrm))
                if h0 < 0.0:
                    nrm = -nrm
                self._palm_n_ref = nrm
        pack_names = [self._fk_links[ia] for ia, _ib, _f in pack_idx[:1]]
        if pack_names:
            print(
                f"[safety] fk pack links={self._fk_links[pack_idx[0][0]]}/"
                f"{self._fk_links[pack_idx[0][1]]} "
                f"sphere_r={g.fk_sphere_radius_m:.3f}m "
                f"pack_j2_min={g.fk_pack_j2_min:.2f} "
                f"pack_finger_min={g.fk_pack_finger_min:.2f}",
                flush=True,
            )

    def _pinch_fingers(self, q: np.ndarray) -> set[str]:
        if self._j2_idx is None:
            return set()
        g = self.gains
        j2 = float(q[self._j2_idx])
        hw = float(g.fk_pinch_j2_halfwidth)
        out: set[str] = set()
        if abs(j2 - float(g.fk_pinch_j2_index)) <= hw:
            out.add("index")
        if abs(j2 - float(g.fk_pinch_j2_middle)) <= hw:
            out.add("middle")
        return out

    def _pack_thr(self, finger: str, pinch: set[str]) -> float:
        if finger in pinch:
            return float(self.gains.fk_pinch_clearance_m)
        return float(self.gains.fk_pack_clearance_m)

    def _pair_clearance(self, thr: float) -> float:
        """Center–center min distance: clearance + optional sphere radii."""
        r = float(self.gains.fk_sphere_radius_m)
        if r > 0.0:
            return float(thr) + 2.0 * r
        return float(thr)

    def _finger_curled(self, q: np.ndarray, finger: str) -> bool:
        idx = self._finger_q_idx.get(finger)
        if idx is None:
            return False
        return float(q[idx]) >= float(self.gains.fk_pack_finger_min)

    def _fist_fingers_curled(self, q: np.ndarray) -> bool:
        """Index+middle at/above fist_finger_min (robot q, not optics)."""
        thr = float(self.gains.fk_fist_finger_min)
        for name in ("index", "middle"):
            idx = self._finger_q_idx.get(name)
            if idx is None:
                return False
            if float(q[idx]) < thr:
                return False
        return True

    def _j2_along(self, q: np.ndarray) -> bool:
        if self._j2_idx is None:
            return True
        return float(q[self._j2_idx]) <= float(self.gains.fk_along_j2)

    def _classify_mode(self, q: np.ndarray) -> str:
        """along | pinch | fist_inside | fist_outside from commanded robot q."""
        fist = self._fist_fingers_curled(q)
        along = self._j2_along(q)
        if fist:
            return "fist_outside" if along else "fist_inside"
        if self._pinch_fingers(q) and not along:
            return "pinch"
        if along:
            return "along"
        return "pinch" if self._pinch_fingers(q) else "along"

    def _pair_rows(
        self, q: np.ndarray, pos: np.ndarray, *, mode: str | None = None
    ) -> list[tuple[float, str, int | None, int, int]]:
        """``(h, kind, joint_idx, ia, ib)`` with ``h = d - d_min``."""
        g = self.gains
        mode = mode or self.mode
        cspace_label = mode == "cspace"
        pinch = self._pinch_fingers(q)
        rows: list[tuple[float, str, int | None, int, int]] = []
        rake_thr = float(g.fk_min_clearance_m)
        if rake_thr > 0.0:
            d_min = self._pair_clearance(rake_thr)
            for ia, ib in self._rake_pairs:
                d = float(np.linalg.norm(pos[ia] - pos[ib]))
                rows.append((d - d_min, "rake", self._j2_idx, ia, ib))

        # Mid / pinch: skip hard pack-pad spheres so tracking matches no-safety.
        # Fist-outside: keep pad pack. Fist-inside: handled by thumb-out gate first.
        # cspace labeling: always evaluate pack (pinch thr inside corridor).
        allow_pack = cspace_label or mode == "fist_outside"
        if allow_pack:
            for ia, ib, finger in self._pack_pairs:
                if not cspace_label:
                    if self._j2_idx is not None and float(q[self._j2_idx]) < float(
                        g.fk_pack_j2_min
                    ):
                        continue
                    in_pinch = finger in pinch
                    if not in_pinch and not self._finger_curled(q, finger):
                        continue
                else:
                    in_pinch = finger in pinch
                thr = self._pack_thr(finger, pinch)
                if thr <= 0.0 and not cspace_label:
                    continue
                if cspace_label and thr <= 0.0:
                    thr = float(g.fk_pack_clearance_m)
                d = float(np.linalg.norm(pos[ia] - pos[ib]))
                d_min = self._pair_clearance(thr)
                kind = "pinch" if in_pinch else "pack"
                rows.append((d - d_min, kind, self._j1_idx, ia, ib))

        # Palm: only when fist or j2 already off along (not open+along).
        # cspace: always on.
        palm_on = cspace_label or mode in ("fist_inside", "fist_outside") or (
            self._j2_idx is not None
            and float(q[self._j2_idx]) > float(g.fk_along_j2)
        )
        palm_thr = float(g.fk_thumb_palm_m)
        if (
            palm_on
            and palm_thr > 0.0
            and self._palm_idx is not None
            and self._thumb_tip_idx is not None
            and self._palm_n_ref is not None
        ):
            i0, i1, i2 = self._palm_idx
            nrm = np.cross(pos[i1] - pos[i0], pos[i2] - pos[i0])
            nn = float(np.linalg.norm(nrm))
            if nn > 1e-9:
                nrm = nrm / nn
                if float(np.dot(nrm, self._palm_n_ref)) < 0.0:
                    nrm = -nrm
                height = float(np.dot(pos[self._thumb_tip_idx] - pos[i0], nrm))
                rows.append((height - palm_thr, "palm", self._j1_idx, self._thumb_tip_idx, i0))
        return rows

    def _hold_fingers_at_prev(self, q: np.ndarray, q_prev: np.ndarray) -> np.ndarray:
        """Four-finger curl must not deepen until thumb is along (thumb-out fist)."""
        if not self.gains.fk_hold_fingers_until_thumb_out:
            return q
        out = q.copy()
        for idx in self._finger_q_idx.values():
            out[idx] = min(float(out[idx]), float(q_prev[idx]))
        return out

    def _force_thumb_along(self, q: np.ndarray, q_prev: np.ndarray) -> np.ndarray:
        """Pull j2 down to along_j2 (then ease j1). Never writes finger joints.

        Does not use ``_search_axis`` for the along target: in fist_inside mode
        pack pairs are off, so the start pose is often already ``_fk_ok`` and
        search would leave j2 unchanged.
        """
        if self._j2_idx is None:
            return q
        along = float(self.gains.fk_along_j2)
        j2 = self._j2_idx
        j1 = self._j1_idx
        work = q.copy()
        if float(work[j2]) > along:
            work[j2] = along
            work[j2] = float(np.clip(work[j2], self._lo[j2], self._hi[j2]))
        if j1 is not None and self._fk is not None and not self._fk_ok(work):
            # Palm / rake may still need a softer j1 after parking j2.
            work = self._retract_axis(work, q_prev, j1)
        return work

    def _apply_thumb_out_gate(
        self, q: np.ndarray, q_prev: np.ndarray, mode: str
    ) -> np.ndarray:
        if mode != "fist_inside":
            return q
        work = self._force_thumb_along(q, q_prev)
        work = self._hold_fingers_at_prev(work, q_prev)
        return work

    def _fk_ok(
        self, q: np.ndarray, *, pack_focus: int | None = None, mode: str | None = None
    ) -> bool:
        if self._fk is None or not self._fk_links:
            return True
        pos = self._fk.link_positions(q, self._fk_links)
        for h, kind, jidx, _a, _b in self._pair_rows(q, pos, mode=mode):
            if h >= 0.0:
                continue
            if (
                kind in ("pack", "pinch")
                and pack_focus is not None
                and jidx != pack_focus
            ):
                continue
            return False
        return True

    def _soft_push(self, q: np.ndarray, *, mode: str | None = None) -> np.ndarray:
        act = float(self.gains.fk_activation_m)
        if act <= 0.0 or self._fk is None or not hasattr(self._fk, "link_positions_jacobians"):
            return q
        q = q.copy()
        pos, jacs = self._fk.link_positions_jacobians(q, self._fk_links)
        nq = int(jacs.shape[2])
        rows = [
            r
            for r in self._pair_rows(q, pos, mode=mode)
            if r[0] < act and r[1] != "palm" and r[2] in (self._j1_idx, self._j2_idx)
        ]
        rows.sort(key=lambda r: r[0])
        for h, _kind, jidx, ia, ib in rows:
            if jidx is None or jidx >= nq:
                continue
            dvec = pos[ia] - pos[ib]
            dist = float(np.linalg.norm(dvec))
            if dist < 1e-9:
                continue
            nrm = dvec / dist
            grad = float(nrm @ (jacs[ia, :, jidx] - jacs[ib, :, jidx]))
            if abs(grad) < 1e-8:
                continue
            dq = (act - h) / grad
            q[jidx] = float(
                np.clip(
                    q[jidx] + float(np.clip(dq, -_SOFT_STEP_RAD, _SOFT_STEP_RAD)),
                    self._lo[jidx],
                    self._hi[jidx],
                )
            )
        return q

    def _with_axis(self, q: np.ndarray, idx: int, val: float) -> np.ndarray:
        out = q.copy()
        out[idx] = float(np.clip(val, self._lo[idx], self._hi[idx]))
        return out

    def _search_axis(
        self,
        q: np.ndarray,
        idx: int,
        start: float,
        goal: float,
        *,
        pack_focus: int | None = None,
    ) -> np.ndarray | None:
        """Move one joint from ``start`` toward ``goal``. Closest feasible to start."""
        start = float(np.clip(start, self._lo[idx], self._hi[idx]))
        goal = float(np.clip(goal, self._lo[idx], self._hi[idx]))
        q_start = self._with_axis(q, idx, start)
        if self._fk_ok(q_start, pack_focus=pack_focus):
            return q_start
        if abs(goal - start) < 1e-12:
            return None
        q_goal = self._with_axis(q, idx, goal)
        if not self._fk_ok(q_goal, pack_focus=pack_focus):
            return None
        best = q_goal
        lo_t, hi_t = 0.0, 1.0
        iters = int(max(self.gains.fk_scale_iters, 1))
        for _ in range(iters):
            mid = 0.5 * (lo_t + hi_t)
            q_try = self._with_axis(q, idx, start + mid * (goal - start))
            if self._fk_ok(q_try, pack_focus=pack_focus):
                best = q_try
                hi_t = mid
            else:
                lo_t = mid
        return best

    def _retract_axis(
        self,
        q: np.ndarray,
        q_prev: np.ndarray,
        idx: int,
        *,
        pack_focus: int | None = None,
    ) -> np.ndarray:
        for goal in (float(q_prev[idx]), float(self._lo[idx]), float(self._hi[idx])):
            found = self._search_axis(
                q, idx, float(q[idx]), goal, pack_focus=pack_focus
            )
            if found is not None:
                return found
        return q

    def _project_thumb(self, q: np.ndarray, q_prev: np.ndarray) -> np.ndarray:
        """j2 toward along/prev/lo first, then j1. Never writes four fingers."""
        j2 = self._j2_idx
        j1 = self._j1_idx
        if j2 is None:
            return self._project_all(q, q_prev)
        along = float(self.gains.fk_along_j2)
        work = q.copy()
        # Fist-inside priority: drive j2 to along before generic retract.
        if float(work[j2]) > along and self.mode == "fist_inside":
            found = self._search_axis(work, j2, float(work[j2]), along)
            if found is not None:
                work = found
            work[j2] = min(float(work[j2]), along)
            work[j2] = float(np.clip(work[j2], self._lo[j2], self._hi[j2]))
            if self._fk_ok(work):
                return work
        work = self._retract_axis(work, q_prev, j2)
        if self._fk_ok(work):
            return work
        if j1 is not None:
            work = self._retract_axis(work, q_prev, j1)
            if self._fk_ok(work):
                return work
            rec = work.copy()
            rec[j2] = float(np.clip(min(float(rec[j2]), along), self._lo[j2], self._hi[j2]))
            rec[j1] = self._lo[j1]
            return np.clip(rec, self._lo, self._hi)
        rec = work.copy()
        rec[j2] = float(np.clip(min(float(rec[j2]), along), self._lo[j2], self._hi[j2]))
        return np.clip(rec, self._lo, self._hi)

    def _project_pack_then_rake(self, q: np.ndarray, q_prev: np.ndarray) -> np.ndarray:
        """Fingers stay on the map. Pack/rake/palm all retract thumb j2 then j1."""
        if self.gains.fk_project == "all":
            return self._project_all(q, q_prev)
        return self._project_thumb(q, q_prev)

    def _project_all(self, q: np.ndarray, q_prev: np.ndarray) -> np.ndarray:
        if self._fk_ok(q_prev):
            best = q_prev.copy()
            lo_a, hi_a = 0.0, 1.0
            iters = int(max(self.gains.fk_scale_iters, 1))
            for _ in range(iters):
                mid = 0.5 * (lo_a + hi_a)
                q_try = np.clip(q_prev + mid * (q - q_prev), self._lo, self._hi)
                if self._fk_ok(q_try):
                    best = q_try
                    lo_a = mid
                else:
                    hi_a = mid
            return best
        if self._j2_idx is not None:
            return self._project_thumb(q, q_prev)
        q0 = np.clip(np.zeros_like(q), self._lo, self._hi)
        if self._fk_ok(q0):
            return self._project_all(q, q0) if not np.allclose(q0, q_prev) else q0
        return q_prev.copy()

    def _enforce_fk(self, q: np.ndarray, q_prev: np.ndarray) -> np.ndarray:
        if self._cspace is not None:
            return self._project_cspace(q, q_prev)
        if self._fk is None:
            return q
        mode = self.mode
        q = self._soft_push(q, mode=mode)
        if self._fk_ok(q, mode=mode):
            return q
        self.fk_blocks += 1
        return self._project_pack_then_rake(q, q_prev)

    def _cspace_ok(self, q: np.ndarray) -> bool:
        if self._cspace is None:
            return True
        return self._cspace.ok(q)

    def _project_cspace(self, q: np.ndarray, q_prev: np.ndarray) -> np.ndarray:
        assert self._cspace is not None
        q = np.clip(q, self._lo, self._hi)
        if self._cspace.ok(q):
            return q
        self.fk_blocks += 1
        self.cspace_hits += 1
        return self._cspace.project(q, q_prev)

    def _resolve_vmax(
        self, n: int, urdf_velocity: np.ndarray | None
    ) -> np.ndarray:
        g = self.gains
        if g.velocity_limit_per_joint is not None:
            v = np.asarray(g.velocity_limit_per_joint, dtype=np.float64).reshape(-1)
            if v.shape[0] != n:
                raise ValueError(
                    f"velocity per_joint length {v.shape[0]} != dof {n}"
                )
            return np.maximum(v, 0.0)
        if g.velocity_limit_rad_s > 0.0:
            return np.full(n, max(float(g.velocity_limit_rad_s), 0.0))
        if g.use_urdf_velocity and urdf_velocity is not None:
            src = np.asarray(urdf_velocity, dtype=np.float64).reshape(-1)
            if src.shape[0] != n:
                src = src[:n] if src.shape[0] > n else np.pad(src, (0, n - src.shape[0]))
            return np.maximum(src * float(g.velocity_scale), 0.0)
        return np.zeros(n, dtype=np.float64)

    def summary(self) -> str:
        g = self.gains
        if not g.enabled:
            return "off"
        vmax = float(np.max(self._vmax)) if g.velocity_enabled else 0.0
        acc = float(g.accel_limit_rad_s2) if g.accel_enabled else 0.0
        if self._cspace is not None:
            return (
                f"pos={int(g.position_enabled)} margin={g.position_margin_rad:.3f} "
                f"vel={int(g.velocity_enabled)} vmax={vmax:.2f}rad/s "
                f"acc={int(g.accel_enabled)} {acc:.1f}rad/s2 "
                f"cspace=1 safe_frac={self._cspace.safe_fraction:.3f} "
                f"mode={self.mode}"
            )
        return (
            f"pos={int(g.position_enabled)} margin={g.position_margin_rad:.3f} "
            f"vel={int(g.velocity_enabled)} vmax={vmax:.2f}rad/s "
            f"acc={int(g.accel_enabled)} {acc:.1f}rad/s2 "
            f"fk={int(self._fk is not None)} rake={g.fk_min_clearance_m:.3f}m "
            f"pack={g.fk_pack_clearance_m:.3f}m pinch={g.fk_pinch_clearance_m:.3f}m "
            f"act={g.fk_activation_m:.3f}m project={g.fk_project} "
            f"along_j2={g.fk_along_j2:.2f} fist_min={g.fk_fist_finger_min:.2f} "
            f"mode={self.mode}"
        )

    def reset(self) -> None:
        self._q = None
        self._v = None
        self.mode = "along"

    def sync(self, q: np.ndarray) -> None:
        """Keep last published command as the rate-limit origin (hold)."""
        q = np.asarray(q, dtype=np.float64).reshape(-1)
        self._q = q.copy()
        self._v = np.zeros_like(q)
        self.mode = self._classify_mode(q)

    def step(self, q_raw: np.ndarray) -> np.ndarray:
        q = np.asarray(q_raw, dtype=np.float64).reshape(-1).copy()
        g = self.gains
        if not g.enabled:
            return q
        if g.position_enabled:
            q = np.clip(q, self._lo, self._hi)
        q0 = np.clip(np.zeros_like(q), self._lo, self._hi)
        use_cspace = self._cspace is not None

        if self._q is None:
            if use_cspace:
                self.mode = "cspace"
                q = self._enforce_fk(q, q0)
            else:
                self.mode = self._classify_mode(q)
                q = self._apply_thumb_out_gate(q, q0, self.mode)
                self.mode = self._classify_mode(q)
                q = self._enforce_fk(q, q0)
            if g.position_enabled:
                q = np.clip(q, self._lo, self._hi)
            if use_cspace:
                self.mode = "cspace"
            else:
                self.mode = self._classify_mode(q)
            self._q = q.copy()
            self._v = np.zeros_like(q)
            return q.copy()

        q_prev = self._q
        q_in = q.copy()
        if use_cspace:
            self.mode = "cspace"
            want_fist_inside = False
            q = q_in
        else:
            # Classify from raw curl command so holding fingers cannot clear fist_inside.
            self.mode = self._classify_mode(q_in)
            want_fist_inside = self.mode == "fist_inside"
            q = self._apply_thumb_out_gate(q, q_prev, self.mode)

        dt = self._dt
        q_des = q
        err = q_des - q_prev
        vmax = np.full_like(q, np.inf)
        if g.velocity_enabled and np.any(self._vmax > 0.0):
            vmax = np.maximum(self._vmax, 0.0)
        if g.accel_enabled and g.accel_limit_rad_s2 > 0.0:
            a = float(g.accel_limit_rad_s2)
            v_brake = np.sqrt(np.maximum(2.0 * a * np.abs(err), 0.0))
            vmax = np.minimum(vmax, v_brake)
            vmax = np.minimum(vmax, np.abs(err) / dt)
        if np.any(np.isfinite(vmax)):
            max_step = np.where(np.isfinite(vmax), vmax * dt, np.abs(err))
            clipped = np.clip(err, -max_step, max_step)
            if bool(np.any(np.abs(clipped - err) > 1e-12)):
                self.velocity_clips += 1
            q = q_prev + clipped
        else:
            q = q_des

        if g.accel_enabled and g.accel_limit_rad_s2 > 0.0 and self._v is not None:
            v_des = (q - q_prev) / dt
            max_dv = float(g.accel_limit_rad_s2) * dt
            v_new = self._v + np.clip(v_des - self._v, -max_dv, max_dv)
            if np.any(np.isfinite(vmax)):
                cap = np.where(np.isfinite(vmax), vmax, np.abs(v_new))
                v_new = np.clip(v_new, -cap, cap)
            q = q_prev + v_new * dt
            self._v = v_new
        else:
            self._v = (q - q_prev) / dt

        q = np.clip(q, np.minimum(q_prev, q_des), np.maximum(q_prev, q_des))

        if g.position_enabled:
            q = np.clip(q, self._lo, self._hi)
        if use_cspace:
            self.mode = "cspace"
        elif want_fist_inside:
            # After rate-limit: keep fingers from deepening; do not hard-snap j2
            # again (that would bypass velocity). Intent still drives mode label.
            self.mode = "fist_inside"
            if self.gains.fk_hold_fingers_until_thumb_out:
                q = self._hold_fingers_at_prev(q, q_prev)
            # Never raise j2 while clearing an inside fist (vel already chasing along).
            if self._j2_idx is not None:
                q[self._j2_idx] = min(float(q[self._j2_idx]), float(q_prev[self._j2_idx]))
        else:
            self.mode = self._classify_mode(q)
        q = self._enforce_fk(q, q_prev)
        if g.position_enabled:
            q = np.clip(q, self._lo, self._hi)
        if use_cspace:
            self.mode = "cspace"
        elif want_fist_inside and not self._j2_along(q):
            self.mode = "fist_inside"
        else:
            self.mode = self._classify_mode(q)
        self._v = (q - q_prev) / dt
        self._q = q.copy()
        return q.copy()
