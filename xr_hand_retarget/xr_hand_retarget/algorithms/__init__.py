"""Shared retarget algorithms (no ROS, no XRT).

Hand yaml only picks a name. Implementations live here.
"""

from __future__ import annotations

# name → one-line character (also in configs/local.yaml.example)
CATALOG: dict[str, str] = {
    "dexpilot": (
        "开源 dex-retargeting 迁来的 tip 投影。捏合最好，合不了掌，吃 CPU。"
    ),
    "vector": "SomeHand 骨链 L-BFGS。跟手较好，捏合一般。",
    "curl": "开掌/握拳/对掌标定插值。Pico 食指侧角：并拢=0，外开到极限=正极限。",
    "nest": "O6 第二条：RViz 食指/中指捏合点当巢，指尖→巢；掌侧/掌内用拳门拆开。",
    "veccurl": "O7 第三条：tip−nest 三标量插到 j3/j2/j1，不是 IK。日常仍用 curl。",
    "mp_curl": (
        "O6/L6/O7：VR26→MediaPipe21 骨角 curl 标定插值（与 PalmTip curl 并列）。"
    ),
    "thumb_ik": "拇指 TipDir IK，四指几何 remap。",
    "official": "Wuji AdaptiveOptimizerAnalytical（第三方仓）。",
}

# local.yaml method → packaged solver
ALIASES: dict[str, str] = {
    "l2": "dexpilot",
    "somehand": "vector",
    "ik": "thumb_ik",
    "thumb": "thumb_ik",
    "curl_calib": "curl",
    "calib": "curl",
    "lerp": "curl",
    "vcurl": "veccurl",
    "mediapipe": "mp_curl",
    "mp21_curl": "mp_curl",
}


def normalize(name: str | None) -> str:
    raw = (name or "").strip().lower()
    return ALIASES.get(raw, raw)
