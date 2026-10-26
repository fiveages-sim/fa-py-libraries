"""Load XHand1 hand YAML (vector / KeyVector only)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from xr_hand_retarget.algorithms.collision import CollisionGains
from xr_hand_retarget.algorithms.curl_xhand1 import CurlCalibGains
from xr_hand_retarget.algorithms.remap import FingerChainGains, IndexAbductionGains, ThumbTipGains
from xr_hand_retarget.algorithms.safety_xhand1 import MotionSafetyGains
from xr_hand_retarget.algorithms.vector_retarget import (
    DexPilotGains,
    DistanceConstraint,
    FingerAttractorGains,
    FistGateGains,
    FistPoseGains,
    FrameConstraint,
    MixGains,
    ThumbSegGains,
    VectorConstraint,
    VectorRetargetConfig,
    _default_finger_attractors,
    default_xhand1_vector_config,
)

_PACKAGE_ROOT = Path(__file__).resolve().parents[2]
_DEFAULT_CONFIG = _PACKAGE_ROOT / "configs" / "hands" / "xhand1.yaml"


@dataclass
class HandConfig:
    hand_id: str
    joint_names: list[str]
    limits: np.ndarray  # (N, 2)
    retargeting_type: str
    flexion_scale: float
    finger_chain: FingerChainGains
    thumb_tip: ThumbTipGains
    index_abduction: IndexAbductionGains
    vector: VectorRetargetConfig
    curl: CurlCalibGains
    controller: str
    topic_suffix: str
    fsm_topic: str
    auto_movej: bool
    alpha: float
    max_delta_rad: float
    hold_on_inactive: bool
    hold_on_zero_pose: bool
    soft_collision: bool
    collision: CollisionGains
    motion: MotionSafetyGains
    rate_hz: float
    # T_xhand (device once): limits/pinch from URDF; scales stay out of user files
    use_urdf_limits: bool = True
    urdf_path: str | None = None
    finger_scale: dict[str, float] | None = None
    limits_source: str = "yaml"  # yaml | urdf

    @property
    def dof(self) -> int:
        return len(self.joint_names)

    @property
    def command_topic(self) -> str:
        return f"/{self.controller}/{self.topic_suffix}"

    @property
    def index_tip(self) -> IndexAbductionGains:
        return self.index_abduction


def default_config_path() -> Path:
    return _DEFAULT_CONFIG


def _parse_finger_attractors(raw: dict[str, Any]) -> dict[str, FingerAttractorGains]:
    base = _default_finger_attractors()
    block = raw.get("finger_attractors")
    if not block:
        return base
    out = dict(base)
    for name, item in block.items():
        if not isinstance(item, dict):
            continue
        prev = out.get(name) or FingerAttractorGains()
        lo = item.get("length_open", prev.length_open)
        lf = item.get("length_fist", prev.length_fist)
        out[str(name)] = FingerAttractorGains(
            w=float(item.get("w", prev.w)),
            length_open=None if lo is None else float(lo),
            length_fist=None if lf is None else float(lf),
        )
    return out


def _parse_dexpilot_gains(retargeting: dict[str, Any]) -> DexPilotGains:
    raw = retargeting.get("dexpilot") or {}
    tips = (
        raw.get("finger_tip_links")
        or raw.get("tip_links")
        or retargeting.get("finger_tip_links")
    )
    wrist = (
        raw.get("wrist_link")
        or retargeting.get("wrist_link")
        or "hand_base"
    )
    tip_tuple = (
        tuple(str(x) for x in tips)
        if tips
        else (
            "thumb_tip",
            "index_tip",
            "middle_tip",
            "ring_tip",
            "pinky_tip",
        )
    )
    gate_raw = raw.get("fist_gate") or {}
    seg_raw = raw.get("thumb_seg") or {}
    mix_raw = raw.get("mix") or {}
    fist_raw = raw.get("fist_pose") or {}
    q_ref_raw = fist_raw.get("q_ref")
    q_ref = None if q_ref_raw is None else tuple(float(x) for x in q_ref_raw)
    joints_raw = fist_raw.get("joints")
    joints = (
        tuple(int(x) for x in joints_raw)
        if joints_raw is not None
        else (0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11)
    )
    return DexPilotGains(
        scaling_factor=float(raw.get("scaling_factor", 1.8)),
        project_dist=float(raw.get("project_dist", 0.03)),
        escape_dist=float(raw.get("escape_dist", 0.05)),
        eta1=float(raw.get("eta1", 1e-4)),
        eta2=float(raw.get("eta2", 3e-2)),
        w_proj_s1=float(raw.get("w_proj_s1", 200.0)),
        w_proj_s2=float(raw.get("w_proj_s2", 400.0)),
        w_tip_pair_base=float(raw.get("w_tip_pair_base", 1.0)),
        w_wrist_tip=float(raw.get("w_wrist_tip", 15.0)),
        huber_delta=float(raw.get("huber_delta", 0.03)),
        wrist_link=str(wrist),
        tip_links=tip_tuple,
        mix=MixGains(
            w_dexpilot=float(mix_raw.get("w_dexpilot", 1.0)),
            w_bone=float(mix_raw.get("w_bone", 0.0)),
            w_fist=float(mix_raw.get("w_fist", 0.0)),
        ),
        fist_pose=FistPoseGains(
            enabled=bool(fist_raw.get("enabled", False)),
            q_ref=q_ref,
            limit_frac=float(fist_raw.get("limit_frac", 0.98)),
            joints=joints,
            index_abd_ref=float(fist_raw.get("index_abd_ref", 0.0)),
        ),
        attractors_enabled=bool(raw.get("attractors_enabled", False)),
        track_length_mix=float(raw.get("track_length_mix", 1.0)),
        finger_attractors=_parse_finger_attractors(raw),
        fist_gate=FistGateGains(
            crowd_d1=float(gate_raw.get("crowd_d1", 0.028)),
            crowd_d2=float(gate_raw.get("crowd_d2", 0.055)),
            curl_r1=float(gate_raw.get("curl_r1", 0.45)),
            curl_r2=float(gate_raw.get("curl_r2", 0.85)),
            pinch_suppress=float(gate_raw.get("pinch_suppress", 0.0)),
        ),
        thumb_seg=ThumbSegGains(
            enabled=bool(seg_raw.get("enabled", False)),
            w_dir=float(seg_raw.get("w_dir", 12.0)),
            human_ip=int(seg_raw.get("human_ip", 3)),
            human_tip=int(seg_raw.get("human_tip", 4)),
            robot_ip=str(seg_raw.get("robot_ip", "thumb_ip")),
            robot_tip=str(seg_raw.get("robot_tip", "thumb_tip")),
        ),
    )


def _parse_bone_constraints(
    retargeting: dict[str, Any],
) -> tuple[list[VectorConstraint], list[FrameConstraint]]:
    """SomeHand bone/frame lists used when mix.w_bone > 0."""
    base = default_xhand1_vector_config()
    defaults = retargeting.get("constraint_defaults") or {}
    vdef = defaults.get("vector") or {}
    fdef = defaults.get("frame") or {}

    vectors: list[VectorConstraint] = []
    for item in retargeting.get("vector_constraints") or []:
        human = tuple(int(x) for x in item["human"])
        robot = tuple(str(x) for x in item["robot"])
        weight = float(item.get("weight", vdef.get("weight", 1.0)))
        if item.get("robot_types") and "site" in item.get("robot_types", []):
            weight = float(item.get("weight", vdef.get("terminal_weight", weight)))
        vectors.append(VectorConstraint(human=human, robot=robot, weight=weight))
    if not vectors:
        vectors = list(base.vector_constraints)

    frames: list[FrameConstraint] = []
    for item in retargeting.get("frame_constraints") or []:
        frames.append(
            FrameConstraint(
                name=str(item.get("name", "frame")),
                human_origin=int(item["human_origin"]),
                human_primary=int(item["human_primary"]),
                human_secondary=int(item["human_secondary"]),
                robot_origin=str(item["robot_origin"]),
                robot_primary=str(item["robot_primary"]),
                robot_secondary=str(item["robot_secondary"]),
                primary_weight=float(
                    item.get("primary_weight", fdef.get("primary_weight", 2.0))
                ),
                secondary_weight=float(
                    item.get("secondary_weight", fdef.get("secondary_weight", 1.8))
                ),
            )
        )
    if not frames:
        frames = list(base.frame_constraints)
    return vectors, frames


def _parse_vector_config(retargeting: dict[str, Any]) -> VectorRetargetConfig:
    """XHand1 production path is vector (somehand / KeyVector), not DexPilot."""
    prep = retargeting.get("preprocess") or {}
    solver = retargeting.get("solver") or {}
    # Keep DexPilotGains parse for YAML back-compat; unused by somehand profile.
    dexpilot = _parse_dexpilot_gains(retargeting)
    vectors, frames = _parse_bone_constraints(retargeting)

    defaults = retargeting.get("constraint_defaults") or {}
    ddef = defaults.get("distance") or {}
    base = default_xhand1_vector_config()
    _ = prep  # reserved for future preprocess knobs

    distances: list[DistanceConstraint] = []
    weights_by_human = ddef.get("weights_by_human") or {}
    for item in retargeting.get("distance_constraints") or []:
        human = tuple(int(x) for x in item["human"])
        robot = tuple(str(x) for x in item["robot"])
        key = f"{human[0]},{human[1]}"
        weight = float(
            item.get(
                "weight",
                weights_by_human.get(key, ddef.get("weight", 1000.0)),
            )
        )
        distances.append(
            DistanceConstraint(
                human=human,
                robot=robot,
                weight=weight,
                scale=float(item.get("scale", ddef.get("scale", 1.0))),
                threshold=float(item.get("threshold", ddef.get("threshold", 0.04))),
                activation_type=str(
                    item.get("activation_type", ddef.get("activation_type", "linear"))
                ),
            )
        )
    if not distances:
        distances = base.distance_constraints

    return VectorRetargetConfig(
        profile="somehand",
        temporal_filter_alpha=float(prep.get("temporal_filter_alpha", 0.65)),
        max_iterations=int(solver.get("max_iterations", 40)),
        norm_delta=float(solver.get("norm_delta", 0.001)),
        output_alpha=float(solver.get("output_alpha", 0.92)),
        activation_alpha=float(solver.get("activation_alpha", 0.30)),
        warm_start_remap=bool(
            solver.get(
                "warm_start_remap",
                solver.get("warm_start_hybrid", True),
            )
        ),
        urdf_path=retargeting.get("urdf_path"),
        vector_constraints=vectors,
        distance_constraints=distances,
        frame_constraints=frames,
        dexpilot=dexpilot,
    )


def _resolve_calib_path(raw_path: str | None, cfg_path: Path) -> Path | None:
    if not raw_path:
        default = _PACKAGE_ROOT / "configs" / "calib" / "xrt_default.yaml"
        return default if default.is_file() else None
    p = Path(raw_path)
    candidates = [
        p,
        cfg_path.parent / p,
        _PACKAGE_ROOT / "configs" / p,
        _PACKAGE_ROOT / "configs" / "calib" / p.name,
    ]
    for cand in candidates:
        if cand.is_file():
            return cand.resolve()
    return p.resolve()


def _parse_curl(retargeting: dict[str, Any], cfg_path: Path, *, robot: dict[str, Any] | None = None) -> CurlCalibGains:
    raw = retargeting.get("curl") or retargeting.get("curl_calib") or {}
    robot = robot or {}
    calib_raw = raw.get("calib") or raw.get("calib_path")
    target = str(
        robot.get("pinch_target_link")
        or raw.get("pinch_target_link")
        or raw.get("pinch_target")
        or "pip"
    )
    urdf = robot.get("urdf_path") or raw.get("urdf_path")
    return CurlCalibGains(
        calib_path=_resolve_calib_path(
            None if calib_raw is None else str(calib_raw),
            cfg_path,
        ),
        output_alpha=float(raw.get("output_alpha", 0.85)),
        pinch_target_link=target.lower(),
        urdf_path=None if urdf in (None, "") else str(urdf),
        finger_scale=(
            {str(k): float(v) for k, v in (robot.get("finger_scale") or {}).items()}
            if isinstance(robot.get("finger_scale"), dict) and robot.get("finger_scale")
            else None
        ),
    )


def _parse_collision(raw: dict[str, Any]) -> CollisionGains:
    return CollisionGains(
        thumb_index_guard=bool(raw.get("thumb_index_guard", True)),
        thumb_min_opposition_when_curled=float(
            raw.get("thumb_min_opposition_when_curled", 0.38)
        ),
        thumb_curl_trigger=float(raw.get("thumb_curl_trigger", 0.85)),
        index_curl_trigger=float(raw.get("index_curl_trigger", 1.15)),
        middle_curl_trigger=float(raw.get("middle_curl_trigger", 1.0)),
        ease=float(raw.get("ease", 0.88)),
    )


def _parse_motion(raw: dict[str, Any]) -> MotionSafetyGains:
    pairs_raw = raw.get("clearance_pairs")
    pairs: list[tuple[str, str]] | None = None
    if pairs_raw:
        pairs = []
        for item in pairs_raw:
            if len(item) != 2:
                raise ValueError(f"clearance_pairs entry must be [a,b], got {item!r}")
            pairs.append((str(item[0]), str(item[1])))
    vel_per = raw.get("velocity_limit_per_joint")
    vel_tuple = None if vel_per is None else tuple(float(x) for x in vel_per)
    vel_uniform = raw.get("velocity_limit_rad_s")
    gains = MotionSafetyGains(
        enabled=bool(raw.get("enabled", False)),
        use_urdf_limits=bool(raw.get("use_urdf_limits", True)),
        position_margin_rad=float(raw.get("position_margin_rad", 0.02)),
        use_urdf_velocity=bool(raw.get("use_urdf_velocity", True)),
        velocity_scale=float(raw.get("velocity_scale", 0.5)),
        velocity_limit_rad_s=(
            None if vel_uniform is None else float(vel_uniform)
        ),
        velocity_limit_per_joint=vel_tuple,
        accel_limit_rad_s2=float(raw.get("accel_limit_rad_s2", 40.0)),
        strict_collision=bool(raw.get("strict_collision", True)),
        min_clearance_m=float(raw.get("min_clearance_m", 0.014)),
        thumb_palm_clearance_m=float(raw.get("thumb_palm_clearance_m", 0.008)),
        on_violation=str(raw.get("on_violation", "scale")),
        scale_iters=int(raw.get("scale_iters", 12)),
        thumb_priority=bool(raw.get("thumb_priority", True)),
        urdf_path=raw.get("urdf_path"),
    )
    if pairs is not None:
        gains.clearance_pairs = pairs
    return gains


def load_hand_config(path: str | Path | None = None) -> HandConfig:
    try:
        import yaml
    except ImportError as exc:
        raise ImportError(
            "PyYAML is required to load configs. Install with: pip install pyyaml"
        ) from exc

    cfg_path = Path(path) if path else _DEFAULT_CONFIG
    if not cfg_path.is_file():
        raise FileNotFoundError(f"hand config not found: {cfg_path}")

    with cfg_path.open("r", encoding="utf-8") as f:
        raw: dict[str, Any] = yaml.safe_load(f) or {}

    hand = raw.get("hand") or {}
    retargeting = raw.get("retargeting") or {}
    ros = raw.get("ros") or {}
    safety = raw.get("safety") or {}
    publish = raw.get("publish") or {}
    chain_raw = retargeting.get("finger_chain") or {}
    thumb_raw = retargeting.get("thumb_tip") or retargeting.get("thumb") or {}
    index_raw = (
        retargeting.get("index_abduction")
        or retargeting.get("index_tip")
        or retargeting.get("index")
        or {}
    )

    names = list(hand.get("joint_names") or [])
    limits = np.asarray(hand.get("limits"), dtype=np.float64)
    if limits.ndim != 2 or limits.shape[1] != 2 or limits.shape[0] != len(names):
        raise ValueError(
            f"hand.limits must be (N,2) matching joint_names; got {limits.shape} vs {len(names)}"
        )

    robot = raw.get("robot") or raw.get("t_xhand") or {}
    if not isinstance(robot, dict):
        robot = {}
    use_urdf_limits = bool(
        robot.get("use_urdf_limits", hand.get("use_urdf_limits", True))
    )
    urdf_path = (
        robot.get("urdf_path")
        or retargeting.get("urdf_path")
        or hand.get("urdf_path")
    )
    limits_source = "yaml"
    if use_urdf_limits:
        try:
            from xr_hand_retarget.algorithms.kinematics_xhand1 import (
                load_urdf_joint_limits_velocity,
            )

            urdf_lim, _vel = load_urdf_joint_limits_velocity(
                "right", urdf_path=urdf_path
            )
            if urdf_lim.shape == limits.shape:
                limits = urdf_lim
                limits_source = "urdf"
            else:
                print(
                    f"[t_xhand] URDF limits shape {urdf_lim.shape} != yaml {limits.shape}; "
                    "keeping yaml limits",
                    flush=True,
                )
        except Exception as exc:
            print(
                f"[t_xhand] URDF limits unavailable ({exc}); using yaml hand.limits",
                flush=True,
            )

    finger_scale_raw = robot.get("finger_scale") or {}
    finger_scale: dict[str, float] | None = None
    if isinstance(finger_scale_raw, dict) and finger_scale_raw:
        finger_scale = {str(k): float(v) for k, v in finger_scale_raw.items()}

    finger_chain = FingerChainGains(
        mcp_fist_ref_rad=float(chain_raw.get("mcp_fist_ref_rad", 1.15)),
        pip_fist_ref_rad=float(chain_raw.get("pip_fist_ref_rad", 1.05)),
        scale=float(
            chain_raw.get("scale", robot.get("finger_chain_scale", 1.0))
        ),
    )
    open_x_raw = thumb_raw.get("lateral_open_x_m")
    lateral_open_x_m = None if open_x_raw is None else float(open_x_raw)
    thumb_tip = ThumbTipGains(
        j1_mode=str(thumb_raw.get("j1_mode", "lateral_anchor")),
        lateral_open_margin_m=float(thumb_raw.get("lateral_open_margin_m", 0.022)),
        lateral_y_span_m=float(thumb_raw.get("lateral_y_span_m", 0.038)),
        lateral_open_deadband=float(thumb_raw.get("lateral_open_deadband", 0.10)),
        lateral_open_x_m=lateral_open_x_m,
        rest_phi_rad=float(thumb_raw.get("rest_phi_rad", 0.28)),
        opposition_scale=float(thumb_raw.get("opposition_scale", 1.30)),
        opposition_offset=float(thumb_raw.get("opposition_offset", 0.02)),
        invert_cmc=bool(thumb_raw.get("invert_cmc", True)),
        cmc_span=float(thumb_raw.get("cmc_span", 1.832)),
        mcp_fist_ref_rad=float(thumb_raw.get("mcp_fist_ref_rad", 1.10)),
        ip_fist_ref_rad=float(thumb_raw.get("ip_fist_ref_rad", 1.05)),
        mcp_bias=float(thumb_raw.get("mcp_bias", -0.08)),
        flex_scale=float(thumb_raw.get("flex_scale", 1.0)),
    )
    index_abduction = IndexAbductionGains(
        abduction_sign=float(index_raw.get("abduction_sign", -1.0)),
        lateral_span_m=float(index_raw.get("lateral_span_m", 0.028)),
        abduction_curl_fade=float(index_raw.get("abduction_curl_fade", 0.40)),
        lat_rest_m=float(index_raw.get("lat_rest_m", 0.0)),
        middle_curl_gate_lo=float(index_raw.get("middle_curl_gate_lo", 0.30)),
        middle_curl_gate_hi=float(index_raw.get("middle_curl_gate_hi", 0.65)),
        middle_mcp_fist_ref_rad=float(
            index_raw.get("middle_mcp_fist_ref_rad", 1.15)
        ),
    )

    # One method per hand: XHand1 → vector (AnyDex KeyVector peer).
    rtype = "vector"
    requested = str(retargeting.get("type") or "").lower()
    if requested and requested not in ("vector", "somehand", ""):
        print(
            f"[retarget] xhand1 yaml type={requested!r} ignored; using vector",
            flush=True,
        )

    collision_raw = safety.get("collision") or {}
    motion_raw = safety.get("motion") or {}

    return HandConfig(
        hand_id=str(hand.get("id") or "xhand1"),
        joint_names=names,
        limits=limits,
        retargeting_type=rtype,
        flexion_scale=float(retargeting.get("flexion_scale", 1.0)),
        finger_chain=finger_chain,
        thumb_tip=thumb_tip,
        index_abduction=index_abduction,
        vector=_parse_vector_config(retargeting),
        curl=_parse_curl(retargeting, cfg_path, robot=robot),
        controller=str(ros.get("controller") or "hand_joint_controller"),
        topic_suffix=str(ros.get("topic_suffix") or "target_joint_position"),
        fsm_topic=str(ros.get("fsm_topic") or "/fsm_command"),
        auto_movej=bool(ros.get("auto_movej", True)),
        alpha=float(safety.get("alpha", 1.0)),
        max_delta_rad=float(safety.get("max_delta_rad", 0.0)),
        hold_on_inactive=bool(safety.get("hold_on_inactive", True)),
        hold_on_zero_pose=bool(safety.get("hold_on_zero_pose", True)),
        soft_collision=bool(safety.get("soft_collision", True)),  # default ON
        collision=_parse_collision(collision_raw),
        motion=_parse_motion(motion_raw),
        rate_hz=float(publish.get("rate_hz", 50.0)),
        use_urdf_limits=use_urdf_limits,
        urdf_path=None if urdf_path in (None, "") else str(urdf_path),
        finger_scale=finger_scale,
        limits_source=limits_source,
    )
