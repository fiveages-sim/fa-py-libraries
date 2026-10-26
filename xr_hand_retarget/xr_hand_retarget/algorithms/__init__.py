"""Shared retarget algorithms (no ROS, no XRT).

One method per hand family (aligned with teleop_ws):

| hand   | method   | teleop_ws peer              |
|--------|----------|-----------------------------|
| xhand1 | vector   | AnyDex KeyVectorOptimizer   |
| o6/l6/o7 | curl   | GloveYamlRemap (curl expr)  |
| wuji   | official | WujiHand2SdkRetarget        |
"""

from __future__ import annotations

# Canonical method per backend / hands.type
HAND_METHOD: dict[str, str] = {
    "xhand1": "vector",
    "o6": "curl",
    "l6": "curl",
    "o7": "curl",
    "linker": "curl",
    "wuji": "official",
}

CATALOG: dict[str, str] = {
    "vector": "XHand1：骨链键向量优化（对齐 teleop_ws AnyDex KeyVector）。",
    "curl": "Linker O6/L6/O7：开掌/握拳标定插值（对齐手套 YAML curl remap）。",
    "official": "Wuji Hand2：官方 AdaptiveOptimizerAnalytical / SDK retarget。",
}

# Legacy aliases → canonical (then HAND_METHOD may still override by backend)
ALIASES: dict[str, str] = {
    "l2": "vector",
    "dexpilot": "vector",
    "somehand": "vector",
    "ik": "vector",
    "thumb": "vector",
    "thumb_ik": "vector",
    "curl_calib": "curl",
    "calib": "curl",
    "lerp": "curl",
    "nest": "curl",
    "veccurl": "curl",
    "vcurl": "curl",
    "mp_curl": "curl",
    "mediapipe": "curl",
    "mp21_curl": "curl",
    "wuji": "official",
    "analytical": "official",
}


def normalize(name: str | None) -> str:
    raw = (name or "").strip().lower()
    return ALIASES.get(raw, raw)


def method_for_backend(backend: str, requested: str | None = None) -> str:
    """Resolve the single allowed method for a hand backend."""
    key = (backend or "").strip().lower()
    canonical = HAND_METHOD.get(key)
    if canonical is None:
        raise ValueError(
            f"unknown backend {backend!r}; expected "
            f"{'|'.join(sorted(set(HAND_METHOD) - {'linker'}))}"
        )
    if not requested:
        return canonical
    got = normalize(requested)
    if got != canonical:
        print(
            f"[retarget] backend={key} ignores method={requested!r}; "
            f"using {canonical!r} (one method per hand)",
            flush=True,
        )
    return canonical
