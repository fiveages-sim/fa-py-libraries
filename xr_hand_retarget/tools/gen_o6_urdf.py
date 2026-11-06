#!/usr/bin/env python3
"""Expand LinkerHand O6/L6/O7 xacro into kinematics-only URDFs (no meshes)."""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_XACRO_ROOT = Path(
    "/home/fiveages/fa_w2_ws/src/robot-descriptions-common/"
    "dexhands/linkerhand_description/xacro"
)
OUT_DIR = _HERE.parent / "assets"

_EXPR = re.compile(r"\$\{([^}]+)\}")
_XACRO = "{http://www.ros.org/wiki/xacro}"

_TIPS = (
    ("thumb_tip", "thumb_distal", "0 0 0.022"),
    ("index_tip", "index_distal", "0 0 0.022"),
    ("middle_tip", "middle_distal", "0 0 0.022"),
    ("ring_tip", "ring_distal", "0 0 0.022"),
    ("pinky_tip", "pinky_distal", "0 0 0.022"),
)

# Pad centers (palm-side of distal). Tune with RViz contact; used by FK spheres.
# +X toward palm for LinkerHand finger frames; Z shorter than tip.
_PADS = (
    ("thumb_pad", "thumb_distal", "0.008 0 0.014"),
    ("index_pad", "index_distal", "0.008 0 0.014"),
    ("middle_pad", "middle_distal", "0.008 0 0.014"),
    ("ring_pad", "ring_distal", "0.008 0 0.014"),
    ("pinky_pad", "pinky_distal", "0.008 0 0.014"),
)

_MODELS = (
    ("o6", "LinkerHandO6", "o6.xacro"),
    ("l6", "LinkerHandL6", "l6.xacro"),
    ("o7", "LinkerHandO7", "o7.xacro"),
)


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


def _add_tip(robot: ET.Element, name: str, parent: str, xyz: str) -> None:
    ET.SubElement(robot, "link", {"name": name})
    joint = ET.SubElement(robot, "joint", {"name": f"{name}_fixed", "type": "fixed"})
    ET.SubElement(joint, "origin", {"xyz": xyz, "rpy": "0 0 0"})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": name})


def generate(model: str, macro_name: str, xacro_name: str, direction: int, out_name: str) -> Path:
    src = _XACRO_ROOT / xacro_name
    ctx = {
        "name": "",
        "direction": float(direction),
        "use_flange": False,
        "fixed_joints": False,
        "radians": math.radians,
        "prefix": "",
        "joint_type": "revolute",
    }
    tree = ET.fromstring(src.read_text(encoding="utf-8"))
    macro = None
    for elem in tree.iter():
        tag = elem.tag.split("}")[-1]
        if tag == "macro" and elem.attrib.get("name") == macro_name:
            macro = elem
            break
    if macro is None:
        raise RuntimeError(f"{macro_name} macro not found in {src}")
    robot_name = f"linkerhand_{model}_{out_name}"
    robot = ET.Element("robot", {"name": robot_name})
    for child in list(macro):
        tag = child.tag.split("}")[-1]
        if tag in ("property",):
            continue
        robot.append(child)
    _expand_elem(robot, ctx)
    _strip_xacro_control(robot)
    _drop_heavy(robot)
    kept: list[ET.Element] = []
    for child in list(robot):
        local = child.tag.split("}")[-1]
        if local in ("link", "joint"):
            kept.append(child)
    robot.clear()
    robot.set("name", robot_name)
    for child in kept:
        robot.append(child)
    for tip, parent, xyz in _TIPS:
        _add_tip(robot, tip, parent, xyz)
    for pad, parent, xyz in _PADS:
        _add_tip(robot, pad, parent, xyz)
    ET.indent(robot, space="  ")
    out = OUT_DIR / f"{model}_{out_name}.urdf"
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    xml = (
        '<?xml version="1.0"?>\n'
        f"<!-- Kinematics-only LinkerHand {model.upper()} (no meshes). "
        f"From linkerhand_description xacro/{xacro_name}. -->\n"
    )
    xml += ET.tostring(robot, encoding="unicode")
    out.write_text(xml, encoding="utf-8")
    return out


if __name__ == "__main__":
    for model, macro, xacro in _MODELS:
        for side, direction in (("left", 1), ("right", -1)):
            path = generate(model, macro, xacro, direction, side)
            print(f"wrote {path}")
