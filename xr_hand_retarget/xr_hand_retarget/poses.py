"""Human pose handbook on the Wuji-21 standard hand (MediaPipe xyz).

Layouts pick a subset. Extra poses (e.g. index_abd) are ignored by hands
without that joint. Robot 20-DOF q is never the catalog.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
_DEFAULT_HANDBOOK = _PACKAGE_ROOT / "configs" / "poses" / "handbook.yaml"

_FALLBACK_CORE = (
    ("open", "开掌：四指自然张开；拇指垂直"),
    ("together", "合掌：四指并拢，拇指垂直于四指"),
    ("fist", "握拳：四指握紧，大拇指收在掌中（掌侧由 thumb_along 标定）"),
    ("good", "点赞：四指握紧程度与 fist 相同；拇指伸出"),
    ("thumb_along", "四指伸直，拇指伸直并与四指同向、贴掌外侧"),
)
_FALLBACK_PINCH = (
    ("pinch_index", "食指捏合：拇指与食指指腹对上即可；其余指不管"),
    ("pinch_middle", "中指捏合：拇指与中指指腹对上；其余指不管"),
    ("pinch_ring", "无名指捏合：拇指与无名指指腹对上；其余指不管"),
    ("pinch_pinky", "小指捏合：拇指与小指指腹对上；其余指不管"),
)


def load_pose_handbook(path: str | Path | None = None) -> dict[str, Any]:
    import yaml

    p = Path(path) if path else _DEFAULT_HANDBOOK
    if not p.is_file():
        return {}
    raw = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
    return raw if isinstance(raw, dict) else {}


def _pose_hint(handbook: dict[str, Any], pose_id: str) -> str:
    poses = handbook.get("poses") or {}
    item = poses.get(pose_id) or {}
    if isinstance(item, dict) and item.get("hint"):
        return str(item["hint"])
    return pose_id


def poses_for_layout(
    layout: str,
    *,
    group: str | None = None,
    handbook: dict[str, Any] | None = None,
) -> tuple[tuple[str, str], ...]:
    """Return (id, hint) pairs. ``group`` filters core|pinch|extra."""
    hb = handbook if handbook is not None else load_pose_handbook()
    layout = str(layout or "").strip().lower()
    ids = list((hb.get("layouts") or {}).get(layout) or [])
    poses = hb.get("poses") or {}
    if not ids:
        if group == "pinch":
            return _FALLBACK_PINCH
        if group in (None, "core"):
            return _FALLBACK_CORE
        return ()
    out: list[tuple[str, str]] = []
    for pose_id in ids:
        spec = poses.get(pose_id) if isinstance(poses, dict) else None
        g = str((spec or {}).get("group") or "")
        if group == "pinch" and g != "pinch" and not str(pose_id).startswith("pinch_"):
            continue
        if group == "core" and (g == "pinch" or str(pose_id).startswith("pinch_")):
            continue
        if group == "extra" and g != "extra":
            continue
        out.append((str(pose_id), _pose_hint(hb, str(pose_id))))
    return tuple(out)
