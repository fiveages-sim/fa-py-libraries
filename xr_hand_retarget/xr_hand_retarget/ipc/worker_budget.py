"""CPU / rate budget for the hand retarget worker (xhand-first).

Keeps vector math unchanged; only caps how often / how hard we solve so
Process B cannot eat ~10 cores via OpenBLAS while EE needs headroom.
"""

from __future__ import annotations

import os
from typing import Any


_DEFAULT_RETARGET_HZ = 80.0  # xhand driver ~83Hz; stay at or below


def limit_blas_threads(n: int = 1) -> None:
    """Clamp BLAS/OpenMP before numpy/scipy import (spawn child only)."""
    n = max(1, int(n))
    for key in (
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "NUMEXPR_NUM_THREADS",
        "VECLIB_MAXIMUM_THREADS",
    ):
        os.environ.setdefault(key, str(n))


def pin_worker_cpus() -> list[int] | None:
    """Pin this process to FA_HAND_WORKER_CPUS or the last 2 logical CPUs."""
    raw = os.environ.get("FA_HAND_WORKER_CPUS", "").strip()
    total = os.cpu_count() or 4
    if raw:
        try:
            cpus = sorted({int(x.strip()) for x in raw.split(",") if x.strip()})
        except ValueError:
            cpus = []
    else:
        # Default: last 2 cores — leave lower IDs for EE / ros2_control / desktop.
        cpus = list(range(max(0, total - 2), total))
    if not cpus:
        return None
    try:
        os.sched_setaffinity(0, cpus)
    except (AttributeError, OSError, ValueError):
        return None
    return cpus


def retarget_hz_from_env(plan_hz: float | None = None) -> float:
    """FA_HAND_RETARGET_HZ > plan teleop.retarget_hz > 80."""
    raw = os.environ.get("FA_HAND_RETARGET_HZ", "").strip()
    if raw:
        return float(max(20.0, min(float(raw), 200.0)))
    if plan_hz is not None:
        return float(max(20.0, min(float(plan_hz), 200.0)))
    return _DEFAULT_RETARGET_HZ


def max_iterations_from_env(plan_iters: int | None = None) -> int | None:
    """Optional override; None = leave VectorRetargetConfig unchanged."""
    raw = os.environ.get("FA_HAND_MAX_ITER", "").strip()
    if raw:
        return int(max(5, min(int(raw), 80)))
    if plan_iters is not None:
        return int(max(5, min(int(plan_iters), 80)))
    return None


def apply_vector_max_iterations(pipe: Any, max_iter: int) -> bool:
    """Patch SidePipeline / XHand1Backend → VectorRetargeter.config.max_iterations."""
    candidates = [pipe]
    backend = getattr(pipe, "_backend", None)
    if backend is not None:
        candidates.append(backend)
        inner = getattr(backend, "_inner", None)
        if inner is not None:
            candidates.append(inner)
    for obj in candidates:
        vector = getattr(obj, "vector", None)
        cfg = getattr(vector, "config", None) if vector is not None else None
        if cfg is not None and hasattr(cfg, "max_iterations"):
            cfg.max_iterations = int(max_iter)
            return True
    return False
