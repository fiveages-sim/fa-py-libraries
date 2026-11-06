"""Planning-time pinch clearance: joint ceiling + FK pad-sphere projection.

Used inside retarget (before EMA) so pinch poses respect RViz ``pinch_q`` and
pad clearance. Unlike ``JointMotionFilter``, pack–pad pairs are always evaluated
in the active pinch slot (safety skips them in pinch mode for tracking).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

_DEFAULT_PINCH_CLEARANCE_M = 0.006
_DEFAULT_SPHERE_RADIUS_M = 0.006

_THUMB_J1 = 0
_THUMB_J2 = 1
_THUMB_J3 = 2

_DEFAULT_PACK_PAIRS: tuple[tuple[str, str, str], ...] = (
    ("thumb_pad", "index_pad", "index"),
    ("thumb_pad", "middle_pad", "middle"),
)


@dataclass
class PinchFkConfig:
    pinch_clearance_m: float = _DEFAULT_PINCH_CLEARANCE_M
    sphere_radius_m: float = _DEFAULT_SPHERE_RADIUS_M
    j2_index: float = 1.2
    j2_middle: float = 1.3
    j2_halfwidth: float = 0.25
    scale_iters: int = 12


def pinch_fk_config_from_workspace(ws) -> PinchFkConfig:
    clearance = getattr(ws, "pinch_fk_clearance_m", None)
    sphere = getattr(ws, "pinch_fk_sphere_radius_m", None)
    return PinchFkConfig(
        pinch_clearance_m=float(
            _DEFAULT_PINCH_CLEARANCE_M if clearance is None else clearance
        ),
        sphere_radius_m=float(
            _DEFAULT_SPHERE_RADIUS_M if sphere is None else sphere
        ),
        j2_index=float(getattr(ws, "j2_index", 1.2)),
        j2_middle=float(getattr(ws, "j2_middle", 1.3)),
        j2_halfwidth=float(getattr(ws, "j2_pinch_hw", 0.25)),
    )


class PinchFkProjector:
    """FK pad-sphere projection for one active pinch slot."""

    def __init__(
        self,
        fk,
        limits: np.ndarray,
        *,
        joint_names: list[str] | None,
        config: PinchFkConfig,
    ):
        self._fk = fk
        self._lo = np.asarray(limits[:, 0], dtype=np.float64)
        self._hi = np.asarray(limits[:, 1], dtype=np.float64)
        self.config = config
        names = [str(n).lower() for n in (joint_names or [])]
        n = int(self._lo.shape[0])

        def jidx(want: str, default: int | None) -> int | None:
            for i, name in enumerate(names):
                if name == want.lower():
                    return i
            if default is not None and 0 <= default < n:
                return default
            return None

        self._j1_idx = jidx("thumb_joint1", _THUMB_J1)
        self._j2_idx = jidx("thumb_joint2", _THUMB_J2)
        self._finger_idx = {
            "index": jidx("index_joint", 3 if n >= 7 else 2),
            "middle": jidx("middle_joint", 4 if n >= 7 else 3),
            "ring": jidx("ring_joint", 5 if n >= 7 else 4),
            "pinky": jidx("pinky_joint", 6 if n >= 7 else 5),
        }
        self._links: list[str] = []
        self._pack_pairs: list[tuple[int, int, str]] = []
        self._init_links()

    def _fk_has_link(self, name: str) -> bool:
        try:
            self._fk.resolve_link(name)
            return True
        except ValueError:
            return False

    def _pad_or_tip(self, name: str) -> str:
        if name.endswith("_pad") and not self._fk_has_link(name):
            tip = name[: -len("_pad")] + "_tip"
            if self._fk_has_link(tip):
                return tip
        return name

    def _init_links(self) -> None:
        links: list[str] = []
        seen: set[str] = set()

        def add(name: str) -> int:
            name = self._pad_or_tip(name)
            if name not in seen:
                seen.add(name)
                links.append(name)
            return links.index(name)

        for a, b, finger in _DEFAULT_PACK_PAIRS:
            self._pack_pairs.append((add(a), add(b), finger))
        self._links = links

    def _d_min(self) -> float:
        r = float(self.config.sphere_radius_m)
        c = float(self.config.pinch_clearance_m)
        return c + 2.0 * r if r > 0.0 else c

    def _worst_h(self, q: np.ndarray, *, slot: str | None) -> float:
        if not self._links or self._fk is None:
            return 0.0
        pos = self._fk.link_positions(q, self._links)
        d_min = self._d_min()
        worst = 0.0
        for ia, ib, finger in self._pack_pairs:
            if slot is not None and finger != slot:
                continue
            d = float(np.linalg.norm(pos[ia] - pos[ib]))
            worst = min(worst, d - d_min)
        return worst

    def fk_ok(self, q: np.ndarray, *, slot: str | None) -> bool:
        return self._worst_h(q, slot=slot) >= -1e-9

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
        slot: str | None,
    ) -> np.ndarray | None:
        start = float(np.clip(start, self._lo[idx], self._hi[idx]))
        goal = float(np.clip(goal, self._lo[idx], self._hi[idx]))
        q_start = self._with_axis(q, idx, start)
        if self.fk_ok(q_start, slot=slot):
            return q_start
        if abs(goal - start) < 1e-12:
            return None
        q_goal = self._with_axis(q, idx, goal)
        if not self.fk_ok(q_goal, slot=slot):
            return None
        best = q_goal
        lo_t, hi_t = 0.0, 1.0
        iters = int(max(self.config.scale_iters, 1))
        for _ in range(iters):
            mid = 0.5 * (lo_t + hi_t)
            q_try = self._with_axis(q, idx, start + mid * (goal - start))
            if self.fk_ok(q_try, slot=slot):
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
        slot: str | None,
    ) -> np.ndarray:
        for goal in (float(q_prev[idx]), float(self._lo[idx])):
            found = self._search_axis(q, idx, float(q[idx]), goal, slot=slot)
            if found is not None:
                return found
        return q

    def project(
        self,
        q: np.ndarray,
        q_prev: np.ndarray | None,
        slot: str | None,
    ) -> np.ndarray:
        """Retract thumb j2/j1 then active finger until pad clearance ok."""
        if self._fk is None or not self._links:
            return q
        work = np.asarray(q, dtype=np.float64).copy()
        prev = (
            np.asarray(q_prev, dtype=np.float64).copy()
            if q_prev is not None
            else work.copy()
        )
        if self.fk_ok(work, slot=slot):
            return work
        if self._j2_idx is not None:
            work = self._retract_axis(work, prev, self._j2_idx, slot=slot)
            if self.fk_ok(work, slot=slot):
                return work
        if self._j1_idx is not None:
            work = self._retract_axis(work, prev, self._j1_idx, slot=slot)
            if self.fk_ok(work, slot=slot):
                return work
        if slot is not None:
            fidx = self._finger_idx.get(str(slot))
            if fidx is not None:
                work = self._retract_axis(work, prev, fidx, slot=slot)
        return work


def build_pinch_q_slots(
    *,
    dof: int,
    finger_idx: dict[str, int],
    names: dict[str, int],
    j2_index: float,
    j2_middle: float,
    j3_index: float,
    j3_middle: float,
    j3_ring: float,
    j3_pinky: float,
    close_j1: float,
    close_finger: dict[str, float],
    raw_blocks: dict[str, Any],
) -> dict[str, dict[str, float]]:
    """Per-slot RViz pinch_q ceilings from workspace pinch blocks."""
    j3_default = {
        "index": j3_index,
        "middle": j3_middle,
        "ring": j3_ring,
        "pinky": j3_pinky,
    }
    j2_default = {
        "index": j2_index,
        "middle": j2_middle,
        "ring": j2_middle,
        "pinky": j2_middle,
    }
    out: dict[str, dict[str, float]] = {}
    for slot, raw in raw_blocks.items():
        if raw is None:
            continue
        overlay: dict[int, float] = {}
        if isinstance(raw, dict):
            for key, val in raw.items():
                if val is None:
                    continue
                k = str(key).strip().lower()
                if k in names:
                    overlay[int(names[k])] = float(val)
                elif k.isdigit():
                    overlay[int(k)] = float(val)
                elif k in ("thumb_joint1", "thumb_j1", "j1"):
                    overlay[0] = float(val)
                elif k in ("thumb_joint3", "thumb_j3", "j3") and dof >= 7:
                    overlay[_THUMB_J3] = float(val)
                elif k.endswith("_joint") or k in finger_idx:
                    fname = k.replace("_joint", "")
                    if fname in finger_idx:
                        overlay[int(finger_idx[fname])] = float(val)
        fidx = finger_idx.get(slot)
        if fidx is None:
            continue
        j1 = float(overlay.get(_THUMB_J1, close_j1))
        j2 = float(overlay.get(_THUMB_J2, j2_default.get(slot, j2_middle)))
        j3 = float(overlay.get(_THUMB_J3, j3_default.get(slot, j3_index)))
        fq = float(overlay.get(fidx, close_finger.get(slot, 0.0)))
        out[slot] = {
            "thumb_joint1": j1,
            "thumb_joint2": j2,
            "thumb_joint3": j3,
            f"{slot}_joint": fq,
        }
    return out


def clamp_pinch_ceiling(
    q: np.ndarray,
    ws,
    *,
    slot: str | None,
    margin: float = 0.0,
    thumb_only: bool = False,
    fingers_only: bool = False,
) -> np.ndarray:
    """Cap joints at RViz ``pinch_q[slot]`` when j2 is in the pinch corridor."""
    from xr_hand_retarget.algorithms.palm_tip import (
        _FINGER_IDX,
        _THUMB_J1,
        _THUMB_J2,
        _THUMB_J3,
        _j2_pinch_slot_w,
    )

    if not bool(getattr(ws, "pinch_ceiling_enabled", False)):
        return q
    if int(getattr(ws, "dof", 6)) < 7:
        return q
    slots = getattr(ws, "pinch_q_slots", None) or {}
    if not slot or slot not in slots:
        return q
    out = np.asarray(q, dtype=np.float64).copy()
    if not fingers_only and _j2_pinch_slot_w(float(out[_THUMB_J2]), ws) <= 1e-9:
        return q
    targets = slots[slot]
    m = float(margin)
    if not fingers_only:
        if "thumb_joint1" in targets:
            out[_THUMB_J1] = min(out[_THUMB_J1], float(targets["thumb_joint1"]) + m)
        if "thumb_joint2" in targets:
            out[_THUMB_J2] = min(out[_THUMB_J2], float(targets["thumb_joint2"]) + m)
        if "thumb_joint3" in targets:
            out[_THUMB_J3] = min(out[_THUMB_J3], float(targets["thumb_joint3"]) + m)
    if not thumb_only:
        fidx = (ws.finger_idx or _FINGER_IDX).get(str(slot))
        finger_key = f"{slot}_joint"
        if fidx is not None and finger_key in targets:
            out[fidx] = min(out[fidx], float(targets[finger_key]) + m)
    return out


def apply_pinch_planning(
    q: np.ndarray,
    ws,
    projector: PinchFkProjector | None,
    *,
    slot: str | None,
    q_prev: np.ndarray | None,
    limits: np.ndarray,
    thumb_only: bool = False,
) -> np.ndarray:
    """Joint ceiling then optional FK pad projection (planning-time anti-penetration)."""
    margin = float(getattr(ws, "pinch_ceiling_margin_rad", 0.0) or 0.0)
    out = clamp_pinch_ceiling(
        q, ws, slot=slot, margin=margin, thumb_only=thumb_only
    )
    if (
        projector is not None
        and bool(getattr(ws, "pinch_fk_project_enabled", False))
        and slot
    ):
        out = projector.project(out, q_prev, slot)
    lo = limits[:, 0]
    hi = limits[:, 1]
    return np.clip(out, lo, hi)


def _self_test_pinch_planning() -> None:
    """Synthetic: ceiling caps j1 at pinch_q; build_pinch_q_slots parses blocks."""
    from xr_hand_retarget.algorithms.palm_tip import (
        O6Workspace,
        _FINGER_IDX_7,
        _JOINT_IDX_7,
        _THUMB_J1,
    )

    ws = O6Workspace(
        dof=7,
        finger_idx=dict(_FINGER_IDX_7),
        j2_index=1.2,
        j2_middle=1.3,
        close_j1=0.3,
        close_finger={"index": 0.58, "middle": 0.61, "ring": 0.6, "pinky": 0.62},
        pinch_ceiling_enabled=True,
        pinch_q_slots={
            "index": {
                "thumb_joint1": 0.3,
                "thumb_joint2": 1.2,
                "thumb_joint3": 0.18,
                "index_joint": 0.58,
            }
        },
        j2_pinch_hw=0.25,
    )
    q = np.array([0.5, 1.2, 0.2, 0.7, 0.1, 0.1, 0.1], dtype=np.float64)
    out = clamp_pinch_ceiling(q, ws, slot="index", margin=0.0)
    assert float(out[_THUMB_J1]) <= 0.3 + 1e-9, out[_THUMB_J1]
    assert float(out[3]) <= 0.58 + 1e-9, out[3]
    slots = build_pinch_q_slots(
        dof=7,
        finger_idx=dict(_FINGER_IDX_7),
        names=dict(_JOINT_IDX_7),
        j2_index=1.2,
        j2_middle=1.3,
        j3_index=0.18,
        j3_middle=0.36,
        j3_ring=0.52,
        j3_pinky=0.8,
        close_j1=0.3,
        close_finger={"index": 0.58, "middle": 0.61, "ring": 0.6, "pinky": 0.62},
        raw_blocks={
            "index": {
                "thumb_joint1": 0.3,
                "thumb_joint3": 0.18,
                "index_joint": 0.58,
            }
        },
    )
    assert abs(float(slots["index"]["thumb_joint2"]) - 1.2) < 1e-9

