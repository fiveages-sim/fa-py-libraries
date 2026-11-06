"""Retarget subprocess: read hand26 shm → SidePipeline → write q_target shm.

No SDK init. No ROS. Spawned by DualHandRetargetNode when process isolation is on.
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
    _ensure_import_path()
    try:
        from xr_hand_retarget.ipc.hand_shm import HandShmBundle
        from xr_hand_retarget.local_profile import load_hand_plan
        from xr_hand_retarget.pipeline import SidePipeline
        from xr_hand_retarget.sources.frame import HandFrame
    except Exception:
        traceback.print_exc()
        return

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
        print(
            f"[hand-worker] pid={os.getpid()} "
            f"L={plan.left.cfg.backend}/{left_pipe.retargeting_type} "
            f"R={plan.right.cfg.backend}/{right_pipe.retargeting_type}",
            flush=True,
        )
        last_seq = -1
        while not stop_event.is_set():
            sample_event.wait(timeout=0.05)
            if stop_event.is_set():
                break
            # Drop-old: consume event; always read latest shm snapshot.
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
                # Gate closed: do not burn CPU; parent keeps last q_target.
                last_seq = seq
                continue
            left_step = right_step = None
            try:
                if snap["left_valid"]:
                    # Trim padded cols to actual width used by solvers (pose7 ok).
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
                # Always publish a seed q even when held (parent may have empty target).
                left_q = left_step.q
            if right_step is not None:
                right_held = bool(right_step.held)
                right_q = right_step.q

            # For held: parent decides whether to keep previous target; we still
            # write the held q so first-frame seed works. Parent marks fresh only
            # when not held (see DualHandRetargetNode._ingest_worker_output).
            shm.write_output(
                frame_seq=seq,
                t_mono=time.monotonic(),
                left_q=left_q,
                right_q=right_q,
                left_held=left_held,
                right_held=right_held,
            )
            last_seq = seq
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
