"""Remember event-7 hand mirror across a library restart.

VRInputHandler keeps mirror mode in the still-running controller. A fresh
hand node would otherwise publish optical left to the left hand while the
controller still routes the matching wrist to the other arm.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

_STATE_PATH = Path.home() / ".local" / "state" / "fa-py-libraries" / "vr_mirror.json"
_CONTROLLER_MARK = b"arms_target_manager_node"


def controller_identity() -> tuple[int, int] | None:
    """(pid, starttime) of the live arms_target_manager process, if any."""
    proc = Path("/proc")
    try:
        names = os.listdir(proc)
    except OSError:
        return None
    for name in names:
        if not name.isdigit():
            continue
        pid = int(name)
        try:
            raw = (proc / name / "cmdline").read_bytes()
        except OSError:
            continue
        if _CONTROLLER_MARK not in raw:
            continue
        start = _proc_starttime(pid)
        if start is None:
            continue
        return pid, start
    return None


def _proc_starttime(pid: int) -> int | None:
    try:
        stat = Path(f"/proc/{pid}/stat").read_text()
    except OSError:
        return None
    # comm is field 2 and may contain spaces; fields after the last ')' are stable.
    cut = stat.rfind(")")
    if cut < 0:
        return None
    fields = stat[cut + 2 :].split()
    # starttime is field 22; index 19 in the post-comm list.
    if len(fields) < 20:
        return None
    try:
        return int(fields[19])
    except ValueError:
        return None


def mirror_for_controller(saved: dict | None, identity: tuple[int, int] | None) -> bool:
    """True only when this same controller process was left in mirror mode."""
    if not saved or identity is None:
        return False
    try:
        saved_id = (int(saved["pid"]), int(saved["start"]))
    except (KeyError, TypeError, ValueError):
        return False
    if saved_id != identity:
        return False
    return bool(saved.get("mirror"))


def load_mirror() -> bool:
    identity = controller_identity()
    try:
        saved = json.loads(_STATE_PATH.read_text())
    except (OSError, json.JSONDecodeError, TypeError):
        return False
    if not isinstance(saved, dict):
        return False
    return mirror_for_controller(saved, identity)


def save_mirror(enabled: bool) -> None:
    identity = controller_identity()
    payload = {
        "mirror": bool(enabled),
        "pid": None if identity is None else identity[0],
        "start": None if identity is None else identity[1],
    }
    try:
        _STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
        _STATE_PATH.write_text(json.dumps(payload) + "\n")
    except OSError:
        return
