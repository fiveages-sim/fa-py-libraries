"""Machine-local hand profile (configs/local.yaml).

Mirrors fa_w2 ``robot.local.yaml``: symmetric ``type`` / ``method``, or
per-side ``left`` / ``right``. Nested ``hands:`` is optional.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from xr_hand_retarget.config import (
    HandRuntimeConfig,
    _REPO_ROOT,
    load_runtime_config,
    _normalize_backend,
)

_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
_DEFAULT_LOCAL = _REPO_ROOT / "configs" / "local.yaml"
_EXAMPLE_LOCAL = _REPO_ROOT / "configs" / "local.yaml.example"

HAND_YAML: dict[str, Path] = {
    "xhand1": _PACKAGE_ROOT / "configs" / "xhand1.yaml",
    "wuji": _PACKAGE_ROOT / "configs" / "wuji_hand2.yaml",
    "o6": _PACKAGE_ROOT / "configs" / "o6.yaml",
    "l6": _PACKAGE_ROOT / "configs" / "l6.yaml",
    "o7": _PACKAGE_ROOT / "configs" / "o7.yaml",
}

_TYPE_KEYS = ("type", "hand", "id", "backend")
_METHOD_KEYS = ("method", "方法", "retargeting")


@dataclass(frozen=True)
class SideSpec:
    """One side before YAML load. ``method`` None → use that hand yaml's type."""

    type: str
    method: str | None = None

    def yaml_path(self) -> Path:
        path = HAND_YAML.get(self.type)
        if path is not None and path.is_file():
            return path
        packaged = _PACKAGE_ROOT / "configs" / f"{self.type}.yaml"
        if packaged.is_file():
            return packaged.resolve()
        raise FileNotFoundError(
            f"no packaged yaml for hand type {self.type!r} (expected {path or packaged})"
        )


@dataclass
class SidePlan:
    spec: SideSpec
    cfg: HandRuntimeConfig

    @property
    def type(self) -> str:
        return self.spec.type

    @property
    def method(self) -> str:
        return self.spec.method or self.cfg.retargeting_type

    @property
    def retargeting_type(self) -> str | None:
        return self.spec.method


@dataclass
class HandPlan:
    left: SidePlan
    right: SidePlan
    source: Path | None
    input_source: str = "xrt"  # local.yaml hands.source (xrt | replay | …)
    user: str | None = None

    def for_side(self, side: str) -> SidePlan:
        if side == "left":
            return self.left
        if side == "right":
            return self.right
        raise ValueError(f"side must be left|right, got {side!r}")

    def dump_yaml(self) -> str:
        return format_resolved(self.left, self.right)


def default_local_path() -> Path:
    env = os.environ.get("FA_HAND_LOCAL")
    if env:
        return Path(env).expanduser()
    return _DEFAULT_LOCAL


def find_local_path(explicit: str | Path | None = None) -> Path | None:
    if explicit:
        path = Path(explicit).expanduser()
        if path.is_file():
            return path.resolve()
        raise FileNotFoundError(f"hand local profile not found: {path}")
    env = os.environ.get("FA_HAND_LOCAL")
    if env:
        path = Path(env).expanduser()
        if path.is_file():
            return path.resolve()
        raise FileNotFoundError(f"FA_HAND_LOCAL points to missing file: {path}")
    if _DEFAULT_LOCAL.is_file():
        return _DEFAULT_LOCAL.resolve()
    return None


def _strip(value: Any) -> str:
    if value is None:
        return ""
    return str(value).strip()


def _lookup(raw: dict[str, Any], keys: tuple[str, ...]) -> str:
    for key in keys:
        text = _strip(raw.get(key))
        if text:
            return text
    return ""


def _parse_side_value(
    value: Any,
    *,
    default_type: str,
    default_method: str | None,
) -> tuple[str, str | None]:
    if value is None or value == "":
        return default_type, default_method
    if isinstance(value, str):
        return _normalize_backend(value), default_method
    if not isinstance(value, dict):
        raise ValueError(f"left/right must be a hand id or mapping, got {type(value).__name__}")
    side_type = _lookup(value, _TYPE_KEYS)
    side_method = _lookup(value, _METHOD_KEYS) or None
    out_type = _normalize_backend(side_type) if side_type else default_type
    out_method = side_method if side_method else default_method
    return out_type, out_method


def _hands_block(raw: dict[str, Any]) -> dict[str, Any]:
    nested = raw.get("hands")
    if isinstance(nested, dict):
        return nested
    defaults = raw.get("defaults")
    if isinstance(defaults, dict):
        eef = defaults.get("end_effectors")
        if isinstance(eef, dict):
            merged = dict(eef)
            methods = defaults.get("methods") or defaults.get("retargeting") or raw.get("methods")
            if isinstance(methods, dict):
                merged.setdefault("method", methods.get("type") or methods.get("method"))
                if "left" not in merged and methods.get("left"):
                    merged["left"] = {"method": methods.get("left")}
                if "right" not in merged and methods.get("right"):
                    merged["right"] = {"method": methods.get("right")}
            if raw.get("method") or raw.get("方法"):
                merged.setdefault("method", raw.get("method") or raw.get("方法"))
            return merged
    return raw


def expand_local_hands(raw: dict[str, Any] | None) -> tuple[SideSpec, SideSpec]:
    """Expand type/method vs left/right, same rules as robot.local.yaml end_effectors."""
    data = raw if isinstance(raw, dict) else {}
    block = _hands_block(data)
    if not isinstance(block, dict):
        block = {}

    sym_type_raw = _lookup(block, _TYPE_KEYS)
    sym_method = _lookup(block, _METHOD_KEYS) or None
    sym_type = _normalize_backend(sym_type_raw) if sym_type_raw else ""

    left_type, left_method = _parse_side_value(
        block.get("left"), default_type=sym_type, default_method=sym_method
    )
    right_type, right_method = _parse_side_value(
        block.get("right"), default_type=sym_type, default_method=sym_method
    )
    if not left_type:
        left_type = right_type
    if not right_type:
        right_type = left_type
    if not left_type or not right_type:
        raise ValueError(
            "local.yaml needs hands.type, or left/right "
            "(xhand1 | wuji | o6 | l6 | o7). Optional hands.user → users/<name>.yaml. "
            "See configs/local.yaml.example"
        )
    return (
        SideSpec(type=left_type, method=left_method or None),
        SideSpec(type=right_type, method=right_method or None),
    )


def _lookup_user(raw: dict[str, Any]) -> str:
    block = _hands_block(raw)
    if not isinstance(block, dict):
        block = {}
    return _lookup(block, ("user", "calib", "person")) or _lookup(
        raw, ("user", "calib", "person")
    )


def _lookup_source(raw: dict[str, Any]) -> str:
    block = _hands_block(raw)
    if not isinstance(block, dict):
        block = {}
    return (
        _lookup(block, ("source", "input"))
        or _lookup(raw, ("source", "input"))
        or "xrt"
    )


def load_local_file(path: str | Path) -> dict[str, Any]:
    cfg_path = Path(path).expanduser()
    raw = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"{cfg_path} must be a mapping")
    return raw


def format_resolved(left: SidePlan | SideSpec, right: SidePlan | SideSpec) -> str:
    """Compact ``type``/``method`` when both sides match; else per-side blocks."""

    def _pair(side: SidePlan | SideSpec) -> tuple[str, str]:
        if isinstance(side, SidePlan):
            return side.type, side.method
        return side.type, side.method or ""

    lt, lm = _pair(left)
    rt, rm = _pair(right)
    if lt == rt and lm == rm:
        payload: dict[str, Any] = {"type": lt}
        if lm:
            payload["method"] = lm
    else:
        payload = {
            "left": {"type": lt, **({"method": lm} if lm else {})},
            "right": {"type": rt, **({"method": rm} if rm else {})},
        }
    return yaml.safe_dump(payload, sort_keys=False, allow_unicode=True)


def _plan_from_specs(
    left: SideSpec,
    right: SideSpec,
    *,
    source: Path | None,
    user: str | None = None,
    input_source: str = "xrt",
) -> HandPlan:
    def _one(spec: SideSpec) -> SidePlan:
        return SidePlan(
            spec=spec,
            cfg=load_runtime_config(spec.yaml_path(), user=user or None),
        )

    return HandPlan(
        left=_one(left),
        right=_one(right),
        source=source,
        input_source=(input_source or "xrt").strip().lower() or "xrt",
        user=user,
    )


def plan_from_single_yaml(
    path: str | Path | None,
    *,
    retargeting: str | None = None,
) -> HandPlan:
    cfg = load_runtime_config(path)
    method = (retargeting or "").strip() or None
    spec = SideSpec(type=cfg.backend, method=method)
    plan_side = SidePlan(spec=spec, cfg=cfg)
    return HandPlan(left=plan_side, right=plan_side, source=cfg.path)


def load_hand_plan(
    *,
    config: str | Path | None = None,
    retargeting: str | None = None,
    local_path: str | Path | None = None,
    use_env_config: bool = True,
) -> HandPlan:
    """CLI yaml > FA_HAND_CONFIG > configs/local.yaml > packaged default.

    ``retargeting`` overrides method on whichever yaml is chosen.
    """
    if config:
        return plan_from_single_yaml(config, retargeting=retargeting)

    if use_env_config:
        for key in ("FA_HAND_CONFIG", "FA_XHAND1_CONFIG"):
            env = os.environ.get(key)
            if env:
                return plan_from_single_yaml(env, retargeting=retargeting)

    found = find_local_path(local_path)
    if found is not None:
        data = load_local_file(found)
        left, right = expand_local_hands(data)
        if retargeting:
            left = SideSpec(type=left.type, method=retargeting)
            right = SideSpec(type=right.type, method=retargeting)
        return _plan_from_specs(
            left,
            right,
            source=found,
            user=_lookup_user(data) or None,
            input_source=_lookup_source(data),
        )

    return plan_from_single_yaml(None, retargeting=retargeting)


def print_plan(plan: HandPlan, *, stream=None) -> None:
    src = str(plan.source) if plan.source else "(default yaml)"
    text = plan.dump_yaml().rstrip()
    extra = f" input={plan.input_source}"
    if plan.user:
        extra += f" user={plan.user}"
    print(f"[local.yaml] file={src}{extra}", flush=True, file=stream)
    print(text, flush=True, file=stream)


def main(argv: list[str] | None = None) -> int:
    import argparse

    p = argparse.ArgumentParser(description="Print resolved configs/local.yaml")
    p.add_argument("--local", default=None, help="Path to local.yaml")
    p.add_argument("--config", default=None, help="Force a single hand yaml")
    p.add_argument("--retargeting", default=None)
    args = p.parse_args(argv)
    plan = load_hand_plan(
        config=args.config,
        retargeting=args.retargeting,
        local_path=args.local,
        use_env_config=not args.local,
    )
    print_plan(plan)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
