"""O6 joint-space safety table: grid mask + weighted nearest-safe projection.

Offline labeling uses pad/tip spheres (same geometry as ``safety.JointMotionFilter``)
with a fixed ``cspace`` policy: rake/pack/palm always on; pinch corridor uses
``pinch_clearance``. Runtime projects unsafe ``q`` to the nearest safe grid cell
under thumb-heavy weights (no fist_inside mode machine).
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

DEFAULT_BINS: tuple[int, ...] = (12, 16, 8, 8, 8, 8)
# Move preference when projecting (higher → more willing to change that joint).
# Internally inverted to path-edge costs so thumb retracts before fingers.
DEFAULT_WEIGHTS: tuple[float, ...] = (4.0, 10.0, 0.2, 0.2, 0.2, 0.2)

_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TABLE_TEMPLATE = "assets/cspace/o6_{side}.npz"


def resolve_cspace_path(template: str | Path | None, side: str) -> Path:
    raw = str(template or DEFAULT_TABLE_TEMPLATE).strip()
    raw = raw.replace("{side}", str(side).strip().lower())
    path = Path(raw)
    if not path.is_absolute():
        path = _PACKAGE_ROOT / path
    return path


def axis_centers(lo: np.ndarray, hi: np.ndarray, bins: np.ndarray) -> list[np.ndarray]:
    out: list[np.ndarray] = []
    for i in range(int(lo.shape[0])):
        n = int(bins[i])
        if n <= 1:
            out.append(np.array([0.5 * (float(lo[i]) + float(hi[i]))], dtype=np.float64))
        else:
            out.append(np.linspace(float(lo[i]), float(hi[i]), n, dtype=np.float64))
    return out


def quantize_q(
    q: np.ndarray, lo: np.ndarray, hi: np.ndarray, bins: np.ndarray
) -> tuple[int, ...]:
    idx: list[int] = []
    for i in range(int(q.shape[0])):
        n = int(bins[i])
        if n <= 1:
            idx.append(0)
            continue
        span = float(hi[i] - lo[i])
        if span <= 1e-12:
            idx.append(0)
            continue
        t = (float(q[i]) - float(lo[i])) / span
        j = int(round(t * (n - 1)))
        idx.append(int(np.clip(j, 0, n - 1)))
    return tuple(idx)


def cell_q(
    idx: tuple[int, ...], centers: list[np.ndarray]
) -> np.ndarray:
    return np.array([centers[i][idx[i]] for i in range(len(idx))], dtype=np.float64)


def move_preference_to_dist_weights(preference: np.ndarray) -> np.ndarray:
    """High move-preference → low distance/path cost on that joint."""
    p = np.asarray(preference, dtype=np.float64).reshape(-1)
    return 1.0 / np.maximum(p, 1e-6)


@dataclass
class CspaceTable:
    lo: np.ndarray
    hi: np.ndarray
    bins: np.ndarray
    safe: np.ndarray
    weights: np.ndarray
    path: Path | None = None

    def __post_init__(self) -> None:
        self.lo = np.asarray(self.lo, dtype=np.float64).reshape(-1)
        self.hi = np.asarray(self.hi, dtype=np.float64).reshape(-1)
        self.bins = np.asarray(self.bins, dtype=np.int32).reshape(-1)
        self.weights = np.asarray(self.weights, dtype=np.float64).reshape(-1)
        self.safe = np.asarray(self.safe, dtype=bool)
        n = int(self.lo.shape[0])
        if self.hi.shape[0] != n or self.bins.shape[0] != n:
            raise ValueError("cspace lo/hi/bins length mismatch")
        if self.weights.shape[0] != n:
            raise ValueError(f"cspace weights length {self.weights.shape[0]} != dof {n}")
        if tuple(self.safe.shape) != tuple(int(b) for b in self.bins):
            raise ValueError(
                f"cspace safe shape {self.safe.shape} != bins {tuple(self.bins)}"
            )
        self._centers = axis_centers(self.lo, self.hi, self.bins)
        self.dof = n

    @classmethod
    def load(cls, path: str | Path) -> CspaceTable:
        p = Path(path)
        data = np.load(p, allow_pickle=False)
        return cls(
            lo=data["lo"],
            hi=data["hi"],
            bins=data["bins"],
            safe=data["safe"],
            weights=data["weights"],
            path=p,
        )

    def save(
        self,
        path: str | Path,
        *,
        meta: dict | None = None,
    ) -> Path:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "lo": self.lo,
            "hi": self.hi,
            "bins": self.bins.astype(np.int32),
            "safe": self.safe.astype(bool),
            "weights": self.weights,
        }
        if meta:
            for k, v in meta.items():
                if isinstance(v, (int, float, np.floating, np.integer)):
                    payload[f"meta_{k}"] = np.asarray(v)
                elif isinstance(v, str):
                    payload[f"meta_{k}"] = np.asarray(v)
                elif isinstance(v, (list, tuple)):
                    payload[f"meta_{k}"] = np.asarray(v, dtype=np.float64)
        np.savez_compressed(p, **payload)
        self.path = p
        return p

    def ok(self, q: np.ndarray) -> bool:
        q = np.asarray(q, dtype=np.float64).reshape(-1)
        if q.shape[0] != self.dof:
            return False
        return bool(self.safe[quantize_q(q, self.lo, self.hi, self.bins)])

    def project(self, q: np.ndarray, q_prev: np.ndarray | None = None) -> np.ndarray:
        """Map unsafe ``q`` to a safe cell via thumb-preferring path Dijkstra.

        ``weights`` are move preferences (higher → cheaper to change). Edge cost
        along axis ``i`` is ``(1/w_i) * Δq_i^2``.
        """
        q = np.asarray(q, dtype=np.float64).reshape(-1)
        if q.shape[0] != self.dof:
            raise ValueError(f"q dof {q.shape[0]} != cspace dof {self.dof}")
        q = np.clip(q, self.lo, self.hi)
        start = quantize_q(q, self.lo, self.hi, self.bins)
        if bool(self.safe[start]):
            return q

        bins = self.bins
        centers = self._centers
        w_dist = move_preference_to_dist_weights(self.weights)
        # One-bin edge costs per axis (constant on uniform grids).
        edge = np.zeros(self.dof, dtype=np.float64)
        for i in range(self.dof):
            n = int(bins[i])
            if n <= 1:
                edge[i] = 0.0
            else:
                dq = float(centers[i][1] - centers[i][0]) if n > 1 else 0.0
                edge[i] = float(w_dist[i]) * (dq * dq)

        heap: list[tuple[float, tuple[int, ...]]] = [(0.0, start)]
        best_cost = {start: 0.0}
        visited: set[tuple[int, ...]] = set()

        while heap:
            cost, cur = heapq.heappop(heap)
            if cur in visited:
                continue
            if cost > best_cost.get(cur, float("inf")) + 1e-15:
                continue
            visited.add(cur)
            if bool(self.safe[cur]):
                return cell_q(cur, centers)
            for axis in range(self.dof):
                for delta in (-1, 1):
                    ni = cur[axis] + delta
                    if ni < 0 or ni >= int(bins[axis]):
                        continue
                    nbr_l = list(cur)
                    nbr_l[axis] = ni
                    nt = tuple(nbr_l)
                    if nt in visited:
                        continue
                    ncost = cost + float(edge[axis])
                    prev = best_cost.get(nt)
                    if prev is not None and ncost >= prev - 1e-15:
                        continue
                    best_cost[nt] = ncost
                    heapq.heappush(heap, (ncost, nt))

        if q_prev is not None:
            return np.clip(
                np.asarray(q_prev, dtype=np.float64).reshape(-1), self.lo, self.hi
            )
        return q

    @property
    def safe_fraction(self) -> float:
        return float(np.mean(self.safe)) if self.safe.size else 0.0


def build_cspace_table(
    filter_obj,
    *,
    bins: tuple[int, ...] | list[int] | None = None,
    weights: tuple[float, ...] | list[float] | None = None,
    progress_every: int = 50000,
) -> CspaceTable:
    """Label every grid cell with ``filter_obj._fk_ok(..., mode='cspace')``."""
    lo = np.asarray(filter_obj._lo, dtype=np.float64).copy()
    hi = np.asarray(filter_obj._hi, dtype=np.float64).copy()
    n = int(lo.shape[0])
    b = np.asarray(bins or DEFAULT_BINS[:n], dtype=np.int32).reshape(-1)
    if b.shape[0] != n:
        raise ValueError(f"bins length {b.shape[0]} != dof {n}")
    w = np.asarray(weights or DEFAULT_WEIGHTS[:n], dtype=np.float64).reshape(-1)
    if w.shape[0] != n:
        raise ValueError(f"weights length {w.shape[0]} != dof {n}")

    centers = axis_centers(lo, hi, b)
    shape = tuple(int(x) for x in b)
    safe = np.zeros(shape, dtype=bool)
    total = int(np.prod(shape))
    done = 0
    # Iterate with np.ndindex for clarity.
    for idx in np.ndindex(*shape):
        q = cell_q(tuple(int(i) for i in idx), centers)
        safe[idx] = bool(filter_obj._fk_ok(q, mode="cspace"))
        done += 1
        if progress_every > 0 and done % progress_every == 0:
            print(
                f"[cspace] labeled {done}/{total} ({100.0 * done / total:.1f}%) "
                f"safe_so_far={float(np.mean(safe.ravel()[:done])):.3f}",
                flush=True,
            )
    table = CspaceTable(lo=lo, hi=hi, bins=b, safe=safe, weights=w)
    print(
        f"[cspace] done cells={total} safe_frac={table.safe_fraction:.3f}",
        flush=True,
    )
    return table


def default_build_meta(gains, *, side: str, bins: np.ndarray) -> dict:
    return {
        "side": side,
        "created_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "rake_clearance_m": float(gains.fk_min_clearance_m),
        "pack_clearance_m": float(gains.fk_pack_clearance_m),
        "pinch_clearance_m": float(gains.fk_pinch_clearance_m),
        "sphere_radius_m": float(gains.fk_sphere_radius_m),
        "thumb_palm_m": float(gains.fk_thumb_palm_m),
        "bins": [int(x) for x in bins],
    }
