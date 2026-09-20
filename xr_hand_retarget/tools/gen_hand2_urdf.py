#!/usr/bin/env python3
"""Expand WujiHand2 xacro into kinematics-only URDFs (no meshes)."""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

_HERE = Path(__file__).resolve().parent
SRC = Path(
    "/home/fiveages/fa_w2_ws/src/robot-descriptions-common/"
    "dexhands/wuji_description/xacro/hand2.xacro"
)
OUT_DIR = _HERE.parent / "assets"

_EXPR = re.compile(r"\$\{([^}]+)\}")
_XACRO = "{http://www.ros.org/wiki/xacro}"


def _eval_expr(expr: str, ctx: dict) -> str:
    val = eval(expr, {"__builtins__": {}}, ctx)  # noqa: S307
    if isinstance(val, bool):
        return "true" if val else "false"
    if isinstance(val, float):
        return f"{val:.12g}"
    return str(val)


def _expand_text(text: str | None, ctx: dict) -> str | None:
    if text is None:
        return None
    return _EXPR.sub(lambda m: _eval_expr(m.group(1), ctx), text)


def _expand_elem(elem: ET.Element, ctx: dict) -> None:
    for key, val in list(elem.attrib.items()):
        elem.attrib[key] = _expand_text(val, ctx) or ""
    if elem.text:
        elem.text = _expand_text(elem.text, ctx)
    if elem.tail:
        elem.tail = _expand_text(elem.tail, ctx)
    for child in list(elem):
        _expand_elem(child, ctx)


def _truthy(val: str | None) -> bool:
    return str(val or "").strip().lower() in ("1", "true", "yes")


def _strip_xacro_control(parent: ET.Element) -> None:
    for child in list(parent):
        tag = child.tag
        if tag in (f"{_XACRO}if", "xacro:if"):
            keep = _truthy(child.attrib.get("value"))
            idx = list(parent).index(child)
            parent.remove(child)
            if keep:
                for sub in list(child):
                    parent.insert(idx, sub)
                    idx += 1
            _strip_xacro_control(parent)
            return
        if tag in (f"{_XACRO}unless", "xacro:unless"):
            keep = not _truthy(child.attrib.get("value"))
            idx = list(parent).index(child)
            parent.remove(child)
            if keep:
                for sub in list(child):
                    parent.insert(idx, sub)
                    idx += 1
            _strip_xacro_control(parent)
            return
        _strip_xacro_control(child)


def _drop_heavy(root: ET.Element) -> None:
    for elem in list(root.iter()):
        for child in list(elem):
            local = child.tag.split("}")[-1]
            if local in ("visual", "collision", "inertial"):
                elem.remove(child)


def generate(direction: int, out_name: str) -> Path:
    ctx = {
        "name": "",
        "direction": float(direction),
        "use_mount": False,
        "fixed_joints": False,
        "radians": math.radians,
        "prefix": "",
        "joint_type": "revolute",
    }
    text = SRC.read_text(encoding="utf-8")
    # Macro body is the WujiHand2 children; wrap as robot.
    tree = ET.fromstring(text)
    macro = None
    for elem in tree.iter():
        tag = elem.tag.split("}")[-1]
        if tag == "macro" and elem.attrib.get("name") == "WujiHand2":
            macro = elem
            break
    if macro is None:
        raise RuntimeError("WujiHand2 macro not found")
    robot = ET.Element("robot", {"name": f"wuji_hand2_{out_name}"})
    for child in list(macro):
        tag = child.tag.split("}")[-1]
        if tag in ("property",):
            continue
        robot.append(child)
    _expand_elem(robot, ctx)
    _strip_xacro_control(robot)
    _drop_heavy(robot)
    # Drop empty comments noise: keep links/joints only.
    kept: list[ET.Element] = []
    for child in list(robot):
        local = child.tag.split("}")[-1]
        if local in ("link", "joint"):
            kept.append(child)
    robot.clear()
    robot.set("name", f"wuji_hand2_{out_name}")
    for child in kept:
        robot.append(child)
    ET.indent(robot, space="  ")
    out = OUT_DIR / f"wuji_hand2_{out_name}.urdf"
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    xml = '<?xml version="1.0"?>\n<!-- Kinematics-only Hand 2 (no meshes). From wuji_description xacro/hand2.xacro. -->\n'
    xml += ET.tostring(robot, encoding="unicode")
    out.write_text(xml, encoding="utf-8")
    return out


if __name__ == "__main__":
    for side, direction in (("left", 1), ("right", -1)):
        path = generate(direction, side)
        print(f"wrote {path}")
