"""Retarget subprocess: read hand26 shm → SidePipeline → write q_target shm.

No SDK init. No ROS. Spawned by DualHandRetargetNode when process isolation is on.

Budget (xhand-first): solve ≤ FA_HAND_RETARGET_HZ (default 80); BLAS threads=1;
optional CPU pin to last 2 cores so EE / ros2_control keep headroom.
"""

from __future__ import annotations

import os
import sys
import time
import traceback
from typing import Any


def _ensure_import_path() -> None:
    # spawn may not inherit sys.path inserts from the parent launcher.
    here = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    if here not in sys.path:
        sys.path.insert(0, here)


def worker_main(
    *,
    in_shm_name: str,
    out_shm_name: str,
    sample_event,
    stop_event,
    plan_kwargs: dict[str, Any],
    left_retargeting: str | None,
    right_retargeting: str | None,
) -> None:
    # Parent owns Ctrl+C; ignore SIGINT so spawn re-import / solve isn't noisy.
    try:
        import signal

        signal.signal(signal.SIGINT, signal.SIG_IGN)
    except Exception:
        pass

    # Before any numpy/scipy import: clamp OpenBLAS/OpenMP (≈1 core).
    for key in (
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "NUMEXPR_NUM_THREADS",
        "VECLIB_MAXIMUM_THREADS",
    ):
        os.environ.setdefault(key, "1")

    _ensure_import_path()
    try:
        from xr_hand_retarget.ipc.hand_shm import HandShmBundle
        from xr_hand_retarget.ipc.worker_budget import (
            apply_vector_max_iterations,
            max_iterations_from_env,
            pin_worker_cpus,
            retarget_hz_from_env,
        )
        from xr_hand_retarget.local_profile import load_hand_plan
        from xr_hand_retarget.pipeline import SidePipeline
        from xr_hand_retarget.sources.frame import HandFrame
    except Exception:
        traceback.print_exc()
        return

    pinned = pin_worker_cpus()
    shm = HandShmBundle.attach(in_shm_name, out_shm_name)
    try:
        plan = load_hand_plan(**plan_kwargs)
        left_pipe = SidePipeline(
            "left",
            plan.left.cfg,
            left_retargeting or plan.left.retargeting_type,
            xrt=None,
        )
        right_pipe = SidePipeline(
            "right",
            plan.right.cfg,
            right_retargeting or plan.right.retargeting_type,
            xrt=None,
        )
        max_iter = max_iterations_from_env(getattr(plan, "max_iterations", None))
        if max_iter is not None:
            for pipe in (left_pipe, right_pipe):
                apply_vector_max_iterations(pipe, max_iter)

        hz = retarget_hz_from_env(getattr(plan, "retarget_hz", None))
        min_period = 1.0 / hz
        next_solve_t = 0.0
        print(
            f"[hand-worker] pid={os.getpid()} "
            f"L={plan.left.cfg.backend}/{left_pipe.retargeting_type} "
            f"R={plan.right.cfg.backend}/{right_pipe.retargeting_type} "
            f"retarget_hz={hz:g} blas_threads=1 "
            f"cpus={pinned if pinned is not None else 'all'} "
            f"max_iter={max_iter if max_iter is not None else 'yaml'}",
            flush=True,
        )
        last_seq = -1
        while not stop_event.is_set():
            sample_event.wait(timeout=0.05)
            if stop_event.is_set():
                break
            while sample_event.is_set():
                sample_event.clear()
            try:
                snap = shm.read_input()
            except Exception as exc:
                print(f"[hand-worker] read_input failed: {exc}", flush=True)
                continue
            seq = int(snap["frame_seq"])
            if seq == last_seq:
                continue
            if not snap["live"]:
                last_seq = seq
                continue

            now = time.monotonic()
            if now < next_solve_t:
                # Drop-old ack: publish LPF keeps last q_target (soft "interpolation").
                last_seq = seq
                continue

            left_step = right_step = None
            try:
                if snap["left_valid"]:
                    j = snap["left_joints"]
                    cols = 7 if j.shape[1] >= 7 else j.shape[1]
                    frame = HandFrame(
                        side="left",
                        active=int(snap["left_active"]),
                        joints26=j[:, :cols].copy(),
                    )
                    left_step = left_pipe.step(frame)
                if snap["right_valid"]:
                    j = snap["right_joints"]
                    cols = 7 if j.shape[1] >= 7 else j.shape[1]
                    frame = HandFrame(
                        side="right",
                        active=int(snap["right_active"]),
                        joints26=j[:, :cols].copy(),
                    )
                    right_step = right_pipe.step(frame)
            except KeyboardInterrupt:
                break
            if left_step is None and right_step is None:
                last_seq = seq
                continue

            left_q = right_q = None
            left_held = right_held = False
            if left_step is not None:
                left_held = bool(left_step.held)
                left_q = left_step.q
            if right_step is not None:
                right_held = bool(right_step.held)
                right_q = right_step.q

            shm.write_output(
                frame_seq=seq,
                t_mono=time.monotonic(),
                left_q=left_q,
                right_q=right_q,
                left_held=left_held,
                right_held=right_held,
            )
            last_seq = seq
            next_solve_t = now + min_period
    except KeyboardInterrupt:
        pass
    except Exception:
        traceback.print_exc()
    finally:
        try:
            shm.close()
        except Exception:
            pass
        print(f"[hand-worker] exit pid={os.getpid()}", flush=True)
