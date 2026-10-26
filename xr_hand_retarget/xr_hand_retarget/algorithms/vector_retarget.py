"""SomeHand-style vector retargeting + DexPilot profile for XHand1 (URDF FK + SciPy)."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .kinematics_xhand1 import XHandFK, make_hand_fk
from xr_hand_retarget.sources.landmarks import TemporalFilter, openxr26_to_mediapipe21, preprocess_landmarks
from .remap import openxr26_to_xhand1_q

# MediaPipe tip indices (wrist=0 → tips at 4,8,12,16,20).
_MP_TIPS = (4, 8, 12, 16, 20)
_FINGER_NAMES = ("thumb", "index", "middle", "ring", "pinky")


@dataclass
class FingerAttractorGains:
    """Per-finger wrist→tip attractor with open/fist length limits (metres, robot scale)."""

    w: float = 20.0
    # Two length limits: open (longer) and fist (shorter). null → derive from human×scale.
    length_open: float | None = None
    length_fist: float | None = None


@dataclass
class FistGateGains:
    """Optional fist gate (only used when mix.w_fist>0 or fist_pose.enabled)."""

    crowd_d1: float = 0.028
    crowd_d2: float = 0.055
    curl_r1: float = 0.45
    curl_r2: float = 0.85
    # Classic DexPilot: keep 0 so fist crowd never weakens pinch projection.
    pinch_suppress: float = 0.0


@dataclass
class ThumbSegGains:
    """Thumb distal bone direction: thumb_ip → thumb_tip (drives joint3)."""

    enabled: bool = False
    w_dir: float = 12.0
    human_ip: int = 3
    human_tip: int = 4
    robot_ip: str = "thumb_ip"
    robot_tip: str = "thumb_tip"


@dataclass
class MixGains:
    """Blend tip-DexPilot vs bone-chain tracking vs joint-space fist.

    Classic tip-only DexPilot: w_dexpilot=1, w_bone=0, w_fist=0.
    """

    w_dexpilot: float = 1.0
    w_bone: float = 0.0
    w_fist: float = 0.0


@dataclass
class FistPoseGains:
    """Joint-space fist prior — pulls q toward fist so half≠full and can hit limits."""

    enabled: bool = False
    # If set (len=12), use as fist reference; else lo + limit_frac*(hi-lo) on `joints`.
    q_ref: tuple[float, ...] | None = None
    limit_frac: float = 0.98
    # Skip index abduction (joint 3) by default — keep near 0 when fisting.
    joints: tuple[int, ...] = (0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11)
    # index_joint1 target when fist (rad).
    index_abd_ref: float = 0.0


@dataclass
class DexPilotGains:
    """DexPilot tip projection (+ optional bone/fist mix — off by default = classic)."""

    scaling_factor: float = 1.8
    project_dist: float = 0.03
    escape_dist: float = 0.05
    eta1: float = 1e-4
    eta2: float = 3e-2
    w_proj_s1: float = 200.0
    w_proj_s2: float = 400.0
    w_tip_pair_base: float = 1.0
    w_wrist_tip: float = 15.0
    huber_delta: float = 0.03
    wrist_link: str = "hand_base"
    tip_links: tuple[str, ...] = (
        "thumb_tip",
        "index_tip",
        "middle_tip",
        "ring_tip",
        "pinky_tip",
    )
    mix: MixGains = field(default_factory=MixGains)
    fist_pose: FistPoseGains = field(default_factory=FistPoseGains)
    # Tip-length attractors (auxiliary). Prefer fist_pose for true fist; default off.
    attractors_enabled: bool = False
    track_length_mix: float = 1.0
    finger_attractors: dict[str, FingerAttractorGains] = field(default_factory=dict)
    fist_gate: FistGateGains = field(default_factory=FistGateGains)
    thumb_seg: ThumbSegGains = field(
        default_factory=lambda: ThumbSegGains(enabled=False)
    )


def _default_finger_attractors() -> dict[str, FingerAttractorGains]:
    """Optional tip-length limits (m); used only if attractors_enabled."""
    return {
        "thumb": FingerAttractorGains(w=18.0, length_open=0.11, length_fist=0.050),
        "index": FingerAttractorGains(w=22.0, length_open=0.14, length_fist=0.048),
        "middle": FingerAttractorGains(w=22.0, length_open=0.15, length_fist=0.048),
        "ring": FingerAttractorGains(w=20.0, length_open=0.14, length_fist=0.048),
        "pinky": FingerAttractorGains(w=18.0, length_open=0.12, length_fist=0.045),
    }


def _smoothstep01(x: float, lo: float, hi: float) -> float:
    """1 when x<=lo, 0 when x>=hi (for 'smaller distance → more fist')."""
    if hi <= lo:
        return 1.0 if x <= lo else 0.0
    t = (float(x) - lo) / (hi - lo)
    t = min(max(t, 0.0), 1.0)
    s = t * t * (3.0 - 2.0 * t)
    return 1.0 - s


def build_fist_q_ref(limits: np.ndarray, fist: FistPoseGains) -> np.ndarray:
    """12-DOF fist reference from q_ref or URDF limit fraction."""
    lim = np.asarray(limits, dtype=np.float64)
    n = lim.shape[0]
    if fist.q_ref is not None and len(fist.q_ref) == n:
        return np.asarray(fist.q_ref, dtype=np.float64)
    q = 0.5 * (lim[:, 0] + lim[:, 1])
    frac = float(np.clip(fist.limit_frac, 0.0, 1.0))
    for j in fist.joints:
        if 0 <= int(j) < n:
            lo, hi = float(lim[j, 0]), float(lim[j, 1])
            q[j] = lo + frac * (hi - lo)
    if n > 3:
        q[3] = float(fist.index_abd_ref)
        q[3] = float(np.clip(q[3], lim[3, 0], lim[3, 1]))
    return q


@dataclass
class VectorConstraint:
    human: tuple[int, int]
    robot: tuple[str, str]
    weight: float = 1.0


@dataclass
class DistanceConstraint:
    human: tuple[int, int]
    robot: tuple[str, str]
    weight: float = 1000.0
    scale: float = 1.0
    threshold: float = 0.04
    activation_type: str = "linear"


@dataclass
class FrameConstraint:
    name: str
    human_origin: int
    human_primary: int
    human_secondary: int
    robot_origin: str
    robot_primary: str
    robot_secondary: str
    primary_weight: float = 2.0
    secondary_weight: float = 1.8


@dataclass
class VectorRetargetConfig:
    """SomeHand-like solver knobs + constraints; optional DexPilot profile."""

    profile: str = "somehand"  # somehand | dexpilot
    temporal_filter_alpha: float = 0.65
    max_iterations: int = 40
    norm_delta: float = 0.001
    output_alpha: float = 0.92
    activation_alpha: float = 0.30
    fd_eps: float = 1e-4
    warm_start_remap: bool = True
    urdf_path: str | None = None
    vector_constraints: list[VectorConstraint] = field(default_factory=list)
    distance_constraints: list[DistanceConstraint] = field(default_factory=list)
    frame_constraints: list[FrameConstraint] = field(default_factory=list)
    dexpilot: DexPilotGains = field(default_factory=DexPilotGains)


def default_xhand1_vector_config() -> VectorRetargetConfig:
    """Wuji/SomeHand-style constraints remapped onto XHand1 link aliases."""
    vectors = [
        VectorConstraint((1, 2), ("thumb_cmc", "thumb_mcp"), 1.0),
        VectorConstraint((2, 3), ("thumb_mcp", "thumb_ip"), 1.0),
        VectorConstraint((3, 4), ("thumb_ip", "thumb_tip"), 0.9),
        VectorConstraint((5, 6), ("index_mcp", "index_pip"), 1.0),
        VectorConstraint((6, 7), ("index_pip", "index_tip"), 0.9),
        VectorConstraint((5, 8), ("index_abd", "index_tip"), 0.5),
        VectorConstraint((9, 10), ("middle_mcp", "middle_pip"), 1.0),
        VectorConstraint((10, 12), ("middle_pip", "middle_tip"), 0.9),
        VectorConstraint((13, 14), ("ring_mcp", "ring_pip"), 1.0),
        VectorConstraint((14, 16), ("ring_pip", "ring_tip"), 0.9),
        VectorConstraint((17, 18), ("pinky_mcp", "pinky_pip"), 1.0),
        VectorConstraint((18, 20), ("pinky_pip", "pinky_tip"), 0.9),
    ]
    distances = [
        DistanceConstraint((4, 8), ("thumb_tip", "index_tip"), 2000.0),
        DistanceConstraint((4, 12), ("thumb_tip", "middle_tip"), 1500.0),
        DistanceConstraint((4, 16), ("thumb_tip", "ring_tip"), 1000.0),
        DistanceConstraint((4, 20), ("thumb_tip", "pinky_tip"), 800.0),
    ]
    frames = [
        FrameConstraint(
            name="thumb_cmc_frame",
            human_origin=1,
            human_primary=2,
            human_secondary=5,
            robot_origin="thumb_cmc",
            robot_primary="thumb_mcp",
            robot_secondary="index_mcp",
            primary_weight=2.0,
            secondary_weight=1.8,
        )
    ]
    return VectorRetargetConfig(
        profile="somehand",
        vector_constraints=vectors,
        distance_constraints=distances,
        frame_constraints=frames,
    )


def default_xhand1_dexpilot_config(
    gains: DexPilotGains | None = None,
    *,
    temporal_filter_alpha: float = 0.65,
    max_iterations: int = 50,
    norm_delta: float = 0.004,
    output_alpha: float = 0.5,
    warm_start_remap: bool = True,
    urdf_path: str | None = None,
    vector_constraints: list[VectorConstraint] | None = None,
    distance_constraints: list[DistanceConstraint] | None = None,
    frame_constraints: list[FrameConstraint] | None = None,
) -> VectorRetargetConfig:
    """DexPilot + optional bone constraints (for w_bone mix)."""
    g = gains or DexPilotGains()
    if not g.finger_attractors:
        g.finger_attractors = _default_finger_attractors()
    bone = default_xhand1_vector_config()
    return VectorRetargetConfig(
        profile="dexpilot",
        temporal_filter_alpha=temporal_filter_alpha,
        max_iterations=max_iterations,
        norm_delta=norm_delta,
        output_alpha=output_alpha,
        warm_start_remap=warm_start_remap,
        urdf_path=urdf_path,
        vector_constraints=list(
            vector_constraints
            if vector_constraints is not None
            else bone.vector_constraints
        ),
        distance_constraints=list(
            distance_constraints if distance_constraints is not None else []
        ),
        frame_constraints=list(
            frame_constraints if frame_constraints is not None else bone.frame_constraints
        ),
        dexpilot=g,
    )


def _unit(v: np.ndarray) -> np.ndarray | None:
    n = float(np.linalg.norm(v))
    if n < 1e-8:
        return None
    return v / n


def _orthonormalize(primary: np.ndarray, secondary: np.ndarray):
    p = _unit(primary)
    if p is None:
        return None, None
    s = secondary - float(np.dot(secondary, p)) * p
    s = _unit(s)
    return p, s


def _dist_activation(kind: str, threshold: float, raw_dist: float) -> float:
    if threshold <= 0.0:
        return 1.0
    if kind == "gaussian":
        sigma = threshold / 2.0
        return float(np.exp(-((raw_dist / max(sigma, 1e-9)) ** 2)))
    return float(max(0.0, 1.0 - raw_dist / threshold))


def _smooth_l1_scalar(r: float, beta: float) -> float:
    """Match torch.nn.SmoothL1Loss(beta=…) on a scalar residual."""
    r = abs(float(r))
    if r < beta:
        return 0.5 * r * r / max(beta, 1e-12)
    return r - 0.5 * beta


def _smooth_l1_dvec(v: np.ndarray, beta: float) -> np.ndarray:
    """∂ SmoothL1(‖v‖) / ∂v  (matches torch SmoothL1 on the norm)."""
    r = float(np.linalg.norm(v))
    if r < 1e-12:
        return np.zeros(3, dtype=np.float64)
    if r < beta:
        return v / max(beta, 1e-12)
    return v / r


def _dir_loss_dvec(v: np.ndarray, h: np.ndarray) -> tuple[float, np.ndarray]:
    """L = 1 − û·ĥ and ∂L/∂v for v = robot segment vector."""
    vn = float(np.linalg.norm(v))
    hn = float(np.linalg.norm(h))
    if vn < 1e-8 or hn < 1e-8:
        return 0.0, np.zeros(3, dtype=np.float64)
    u = v / vn
    hu = h / hn
    loss = 1.0 - float(np.dot(u, hu))
    # ∂û/∂v = (I − ûûᵀ)/‖v‖ ; ∂L/∂v = −(I − ûûᵀ) ĥ / ‖v‖
    dvec = -(hu - float(np.dot(u, hu)) * u) / vn
    return loss, dvec


def generate_dexpilot_link_indices(num_fingers: int) -> tuple[list[int], list[int]]:
    """Mirror dex-retargeting DexPilotOptimizer.generate_link_indices."""
    origin: list[int] = []
    task: list[int] = []
    for i in range(1, num_fingers):
        for j in range(i + 1, num_fingers + 1):
            origin.append(j)
            task.append(i)
    for i in range(1, num_fingers + 1):
        origin.append(0)
        task.append(i)
    return origin, task


def _dexpilot_projection_cache(num_fingers: int, eta1: float, eta2: float):
    """Mirror DexPilotOptimizer.set_dexpilot_cache."""
    n_proj = num_fingers * (num_fingers - 1) // 2
    projected = np.zeros(n_proj, dtype=bool)
    s2_origin: list[int] = []
    s2_task: list[int] = []
    for i in range(0, num_fingers - 2):
        for j in range(i + 1, num_fingers - 1):
            s2_origin.append(j)
            s2_task.append(i)
    n_s1 = num_fingers - 1
    n_s2 = n_proj - n_s1
    projected_dist = np.array([eta1] * n_s1 + [eta2] * n_s2, dtype=np.float64)
    return projected, s2_origin, s2_task, projected_dist


class VectorRetargeter:
    """Optimize XHand1 q[12]: SomeHand direction/distance or DexPilot vector+project."""

    def __init__(
        self,
        *,
        side: str,
        limits: np.ndarray,
        config: VectorRetargetConfig | None = None,
        fk: XHandFK | None = None,
    ):
        self.side = "left" if side.lower().startswith("l") else "right"
        self.limits = np.asarray(limits, dtype=np.float64)
        self.config = config or default_xhand1_vector_config()
        self.fk = fk or make_hand_fk(self.side, urdf_path=self.config.urdf_path)
        self.landmark_filter = TemporalFilter(self.config.temporal_filter_alpha)
        self._q_prev: np.ndarray | None = None
        self._act_prev: np.ndarray | None = None

        self._profile = str(self.config.profile or "somehand").lower()
        self._dexpilot = self._profile == "dexpilot"

        self._vec_links: list[tuple[str, str]] = []
        self._vec_w: list[float] = []
        self._vec_human: list[tuple[int, int]] = []
        self._dist_links: list[tuple[str, str]] = []
        self._dist_human: list[tuple[int, int]] = []
        self._dist_w: list[float] = []
        self._dist_scale: list[float] = []
        self._dist_thr: list[float] = []
        self._dist_act: list[str] = []
        self._frame_specs = list(self.config.frame_constraints)

        if self._dexpilot:
            self._init_dexpilot()
        else:
            for c in self.config.vector_constraints:
                self._vec_links.append((c.robot[0], c.robot[1]))
                self._vec_w.append(float(c.weight))
                self._vec_human.append((int(c.human[0]), int(c.human[1])))
            for c in self.config.distance_constraints:
                self._dist_links.append((c.robot[0], c.robot[1]))
                self._dist_human.append((int(c.human[0]), int(c.human[1])))
                self._dist_w.append(float(c.weight))
                self._dist_scale.append(float(c.scale))
                self._dist_thr.append(float(c.threshold))
                self._dist_act.append(str(c.activation_type))
        self._opt_link_names = self._needed_links()
        # Resolve once so each optimizer eval skips alias lookups.
        self._opt_link_resolved = [self.fk.resolve_link(n) for n in self._opt_link_names]
        self._opt_link_alias = {
            a: r for a, r in zip(self._opt_link_names, self._opt_link_resolved)
        }

    def _init_dexpilot(self) -> None:
        g = self.config.dexpilot
        if not g.finger_attractors:
            g.finger_attractors = _default_finger_attractors()
        tips = list(g.tip_links)
        n = len(tips)
        if n < 2 or n > 5:
            raise ValueError(f"DexPilot expects 2..5 tips, got {n}")
        link_names = [g.wrist_link] + tips
        human_of_link = [0] + list(_MP_TIPS[:n])

        origin_i, task_i = generate_dexpilot_link_indices(n)
        self._dp_origin_links = [link_names[i] for i in origin_i]
        self._dp_task_links = [link_names[i] for i in task_i]
        self._dp_human_origin = [human_of_link[i] for i in origin_i]
        self._dp_human_task = [human_of_link[i] for i in task_i]
        self._dp_n_fingers = n
        self._dp_len_proj = n * (n - 1) // 2
        (
            self._dp_projected,
            self._dp_s2_origin,
            self._dp_s2_task,
            self._dp_proj_dist,
        ) = _dexpilot_projection_cache(n, g.eta1, g.eta2)
        self._dp_len_s1 = n - 1
        self._dp_len_s2 = self._dp_len_proj - self._dp_len_s1
        self._dp_tip_human = list(_MP_TIPS[:n])
        self._dp_finger_names = list(_FINGER_NAMES[:n])
        self._fist_q = build_fist_q_ref(self.limits, g.fist_pose)

        for o, t in zip(self._dp_origin_links, self._dp_task_links):
            self._vec_links.append((o, t))
            self._vec_human.append((0, 0))
            self._vec_w.append(1.0)

        # Bone-chain constraints for w_bone mix (SomeHand-style).
        self._bone_links: list[tuple[str, str]] = []
        self._bone_human: list[tuple[int, int]] = []
        self._bone_w: list[float] = []
        for c in self.config.vector_constraints:
            self._bone_links.append((c.robot[0], c.robot[1]))
            self._bone_human.append((int(c.human[0]), int(c.human[1])))
            self._bone_w.append(float(c.weight))
        self._frame_specs = list(self.config.frame_constraints)

        seg = g.thumb_seg
        if seg.enabled:
            self._vec_links.append((seg.robot_ip, seg.robot_tip))

    def reset(self) -> None:
        self._q_prev = None
        self._act_prev = None
        self.landmark_filter.reset()
        if self._dexpilot:
            self._dp_projected[:] = False

    def _needed_links(self) -> list[str]:
        names: list[str] = []
        for o, t in self._vec_links:
            names.extend([o, t])
        for a, b in self._dist_links:
            names.extend([a, b])
        for fr in self._frame_specs:
            names.extend([fr.robot_origin, fr.robot_primary, fr.robot_secondary])
        if self._dexpilot:
            g = self.config.dexpilot
            names.append(g.wrist_link)
            names.extend(g.tip_links)
            if g.thumb_seg.enabled:
                names.extend([g.thumb_seg.robot_ip, g.thumb_seg.robot_tip])
            for o, t in getattr(self, "_bone_links", []):
                names.extend([o, t])
            for fr in self._frame_specs:
                names.extend([fr.robot_origin, fr.robot_primary, fr.robot_secondary])
        seen: set[str] = set()
        out: list[str] = []
        for n in names:
            if n not in seen:
                seen.add(n)
                out.append(n)
        return out

    def _build_targets_somehand(self, landmarks21: np.ndarray):
        lm = preprocess_landmarks(landmarks21, hand_side=self.side)
        lm = self.landmark_filter.filter(lm)

        dirs = []
        for a, b in self._vec_human:
            u = _unit(lm[b] - lm[a])
            dirs.append(np.zeros(3) if u is None else u)
        target_dirs = np.asarray(dirs, dtype=np.float64)

        frame_p, frame_s = [], []
        for fr in self._frame_specs:
            p, s = _orthonormalize(
                lm[fr.human_primary] - lm[fr.human_origin],
                lm[fr.human_secondary] - lm[fr.human_origin],
            )
            frame_p.append(np.zeros(3) if p is None else p)
            frame_s.append(np.zeros(3) if s is None else s)
        target_fp = np.asarray(frame_p, dtype=np.float64) if frame_p else None
        target_fs = np.asarray(frame_s, dtype=np.float64) if frame_s else None

        target_d = []
        acts = []
        for i, (a, b) in enumerate(self._dist_human):
            d = float(np.linalg.norm(lm[a] - lm[b]))
            target_d.append(self._dist_scale[i] * d)
            raw_act = _dist_activation(self._dist_act[i], self._dist_thr[i], d)
            if self._act_prev is not None:
                aa = self.config.activation_alpha
                acts.append(aa * raw_act + (1.0 - aa) * float(self._act_prev[i]))
            else:
                acts.append(raw_act)
        self._act_prev = np.asarray(acts, dtype=np.float64) if acts else None

        return {
            "dirs": target_dirs,
            "fp": target_fp,
            "fs": target_fs,
            "dist": np.asarray(target_d, dtype=np.float64),
            "acts": np.asarray(acts, dtype=np.float64) if acts else None,
        }

    def _build_targets_dexpilot(self, landmarks21: np.ndarray):
        """Human tip vectors + projection + per-finger length attractors + thumb seg."""
        g = self.config.dexpilot
        lm = preprocess_landmarks(landmarks21, hand_side=self.side)
        lm = self.landmark_filter.filter(lm)

        n_vec = len(self._dp_origin_links)
        raw = np.zeros((n_vec, 3), dtype=np.float64)
        for i in range(n_vec):
            a = self._dp_human_origin[i]
            b = self._dp_human_task[i]
            raw[i] = lm[b] - lm[a]

        len_proj = self._dp_len_proj
        len_s1 = self._dp_len_s1
        len_s2 = self._dp_len_s2
        dists = np.linalg.norm(raw[:len_proj], axis=1)

        self._dp_projected[:len_s1][dists[0:len_s1] < g.project_dist] = True
        self._dp_projected[:len_s1][dists[0:len_s1] > g.escape_dist] = False
        if len_s2 > 0:
            self._dp_projected[len_s1:len_proj] = np.logical_and(
                self._dp_projected[:len_s1][self._dp_s2_origin],
                self._dp_projected[:len_s1][self._dp_s2_task],
            )
            self._dp_projected[len_s1:len_proj] = np.logical_and(
                self._dp_projected[len_s1:len_proj],
                dists[len_s1:len_proj] <= 0.03,
            )

        # Fist gate: tip crowd + mean curl (tracking-first; suppresses pinch).
        tip_idx = self._dp_tip_human
        n_f = self._dp_n_fingers
        crowd_sum = 0.0
        crowd_n = 0
        for i in range(n_f):
            for j in range(i + 1, n_f):
                crowd_sum += float(np.linalg.norm(lm[tip_idx[i]] - lm[tip_idx[j]]))
                crowd_n += 1
        crowd = crowd_sum / max(crowd_n, 1)
        wrist_len = np.array(
            [float(np.linalg.norm(lm[t] - lm[0])) for t in tip_idx], dtype=np.float64
        )
        open_ref = np.array(
            [
                float(
                    (g.finger_attractors.get(name) or FingerAttractorGains()).length_open
                    or (0.12 if name != "thumb" else 0.10)
                )
                / max(g.scaling_factor, 1e-6)
                for name in self._dp_finger_names
            ],
            dtype=np.float64,
        )
        curl_ratio = float(np.mean(wrist_len / np.maximum(open_ref, 1e-6)))
        gate = g.fist_gate
        a_crowd = _smoothstep01(crowd, gate.crowd_d1, gate.crowd_d2)
        a_curl = _smoothstep01(curl_ratio, gate.curl_r1, gate.curl_r2)
        alpha_fist = float(min(max(0.5 * (a_crowd + a_curl), 0.0), 1.0))
        # Fist extras only when explicitly enabled (classic tip path ignores).
        use_fist_extras = (
            float(g.mix.w_fist) > 1e-9 or bool(g.fist_pose.enabled)
        )
        if not use_fist_extras:
            alpha_fist = 0.0

        base_w = float(g.w_tip_pair_base)
        normal_w = np.full(len_proj, base_w, dtype=np.float64)
        high_w = np.array(
            [g.w_proj_s1] * len_s1 + [g.w_proj_s2] * len_s2, dtype=np.float64
        )
        if use_fist_extras and float(gate.pinch_suppress) > 0.0:
            high_w = high_w * (1.0 - alpha_fist * float(gate.pinch_suppress))
        w_proj = np.where(self._dp_projected, high_w, normal_w)

        # Per-finger wrist→tip weights + length-limited refs.
        attr_map = g.finger_attractors or _default_finger_attractors()
        w_wrist = np.zeros(n_f, dtype=np.float64)
        wrist_raw = raw[len_proj:]  # tip - wrist for each finger
        wrist_ref = np.zeros_like(wrist_raw)
        for i, name in enumerate(self._dp_finger_names):
            fa = attr_map.get(name) or FingerAttractorGains(w=g.w_wrist_tip)
            w_wrist[i] = float(fa.w if g.attractors_enabled else g.w_wrist_tip)
            if use_fist_extras and name != "thumb":
                w_wrist[i] *= 1.0 + 0.75 * alpha_fist

            hu = wrist_raw[i]
            hu_n = float(np.linalg.norm(hu))
            direction = hu / (hu_n + 1e-6)
            scaled_len = hu_n * g.scaling_factor
            lo = fa.length_fist
            hi = fa.length_open
            if g.attractors_enabled and lo is not None and hi is not None:
                lo_f, hi_f = float(lo), float(hi)
                if hi_f < lo_f:
                    lo_f, hi_f = hi_f, lo_f
                finger_open = hi_f / max(g.scaling_factor, 1e-6)
                r_i = hu_n / max(finger_open, 1e-6)
                a_i = _smoothstep01(r_i, gate.curl_r1, gate.curl_r2)
                a_i = max(a_i, alpha_fist)
                # Tracking-first: follow human×scale; only blend toward length
                # limits when fist/curl gate is on. Hard clamp only at extreme fist.
                limit_len = (1.0 - a_i) * hi_f + a_i * lo_f
                track_mix = float(getattr(g, "track_length_mix", 0.9))
                track_mix = min(max(track_mix, 0.0), 1.0)
                # When open (a_i≈0): almost pure scaled_len. When fist: more limit_len.
                target_len = (track_mix + (1.0 - track_mix) * (1.0 - a_i)) * scaled_len + (
                    (1.0 - track_mix) * a_i
                ) * limit_len
                if a_i > 0.85:
                    target_len = min(max(target_len, lo_f), hi_f)
            else:
                target_len = scaled_len
            wrist_ref[i] = direction * target_len

        weights = np.concatenate([w_proj, w_wrist])

        scaled = raw * g.scaling_factor
        dir_vec = raw[:len_proj] / (dists[:, None] + 1e-6)
        projected_vec = dir_vec * self._dp_proj_dist[:, None]
        ref_proj = np.where(self._dp_projected[:, None], projected_vec, scaled[:len_proj])
        ref = np.concatenate([ref_proj, wrist_ref], axis=0)

        thumb_dir = None
        seg = g.thumb_seg
        if seg.enabled:
            thumb_dir = _unit(lm[seg.human_tip] - lm[seg.human_ip])

        bone_dirs = None
        if float(g.mix.w_bone) > 1e-9 and self._bone_links:
            dirs = []
            for a, b in self._bone_human:
                u = _unit(lm[b] - lm[a])
                dirs.append(np.zeros(3) if u is None else u)
            bone_dirs = np.asarray(dirs, dtype=np.float64)

        frame_p, frame_s = None, None
        if float(g.mix.w_bone) > 1e-9 and self._frame_specs:
            fp, fs = [], []
            for fr in self._frame_specs:
                p, s = _orthonormalize(
                    lm[fr.human_primary] - lm[fr.human_origin],
                    lm[fr.human_secondary] - lm[fr.human_origin],
                )
                fp.append(np.zeros(3) if p is None else p)
                fs.append(np.zeros(3) if s is None else s)
            frame_p = np.asarray(fp, dtype=np.float64)
            frame_s = np.asarray(fs, dtype=np.float64)

        return {
            "ref_vecs": ref,
            "weights": weights,
            "alpha_fist": alpha_fist,
            "thumb_dir": thumb_dir,
            "bone_dirs": bone_dirs,
            "frame_p": frame_p,
            "frame_s": frame_s,
        }


    def _loss_somehand(
        self,
        q: np.ndarray,
        *,
        targets: dict,
        q_prev: np.ndarray,
        link_names: list[str],
    ) -> float:
        loss, _ = self._eval_somehand(q, targets=targets, q_prev=q_prev, link_names=link_names)
        return loss

    def _eval_somehand(
        self,
        q: np.ndarray,
        *,
        targets: dict,
        q_prev: np.ndarray,
        link_names: list[str],
    ) -> tuple[float, np.ndarray]:
        """SomeHand loss + analytic ∂L/∂q (paper-style geometric Jacobian)."""
        pos, jacs = self.fk.link_positions_jacobians(
            q, getattr(self, "_opt_link_resolved", link_names)
        )
        idx = {n: i for i, n in enumerate(link_names)}
        nq = q.shape[0]
        grad = np.zeros(nq, dtype=np.float64)
        loss = 0.0
        target_dirs = targets["dirs"]
        target_fp = targets["fp"]
        target_fs = targets["fs"]
        target_d = targets["dist"]
        acts = targets["acts"]

        for i, (o, t) in enumerate(self._vec_links):
            io, it = idx[o], idx[t]
            v = pos[it] - pos[io]
            hu = target_dirs[i]
            term, dvec = _dir_loss_dvec(v, hu)
            if term == 0.0 and float(np.linalg.norm(dvec)) == 0.0:
                continue
            w = self._vec_w[i]
            loss += w * term
            jv = jacs[it] - jacs[io]
            grad += w * (jv.T @ dvec)

        for i, fr in enumerate(self._frame_specs):
            if target_fp is None:
                break
            io = idx[fr.robot_origin]
            ip = idx[fr.robot_primary]
            isp = idx[fr.robot_secondary]
            vp = pos[ip] - pos[io]
            vs = pos[isp] - pos[io]
            hp = target_fp[i]
            hs = target_fs[i] if target_fs is not None else None
            term_p, d_p = _dir_loss_dvec(vp, hp)
            loss += fr.primary_weight * term_p
            jp = jacs[ip] - jacs[io]
            grad += fr.primary_weight * (jp.T @ d_p)
            if hs is not None:
                term_s, d_s = _dir_loss_dvec(vs, hs)
                loss += fr.secondary_weight * term_s
                js = jacs[isp] - jacs[io]
                grad += fr.secondary_weight * (js.T @ d_s)

        if acts is not None and len(target_d):
            for i, (a, b) in enumerate(self._dist_links):
                ia, ib = idx[a], idx[b]
                dvec = pos[ia] - pos[ib]
                rd = float(np.linalg.norm(dvec))
                excess = max(rd - float(target_d[i]), 0.0)
                w = self._dist_w[i] * float(acts[i])
                loss += w * excess * excess
                if excess > 0.0 and rd > 1e-9:
                    # ∂(excess²)/∂pa = 2 excess · (pa−pb)/rd
                    gpos = (2.0 * excess / rd) * dvec
                    ja = jacs[ia] - jacs[ib]
                    grad += w * (ja.T @ gpos)

        loss += self.config.norm_delta * float(np.sum((q - q_prev) ** 2))
        grad += 2.0 * self.config.norm_delta * (q - q_prev)
        return loss, grad

    def _loss_dexpilot(
        self,
        q: np.ndarray,
        *,
        targets: dict,
        q_prev: np.ndarray,
        link_names: list[str],
    ) -> float:
        loss, _ = self._eval_dexpilot(q, targets=targets, q_prev=q_prev, link_names=link_names)
        return loss

    def _eval_dexpilot(
        self,
        q: np.ndarray,
        *,
        targets: dict,
        q_prev: np.ndarray,
        link_names: list[str],
    ) -> tuple[float, np.ndarray]:
        """DexPilot loss + analytic gradient (dex-retargeting / paper style)."""
        g = self.config.dexpilot
        mix = g.mix
        pos, jacs = self.fk.link_positions_jacobians(
            q, getattr(self, "_opt_link_resolved", link_names)
        )
        idx = {n: i for i, n in enumerate(link_names)}
        nq = q.shape[0]
        grad = np.zeros(nq, dtype=np.float64)
        loss = 0.0

        w_dex = float(mix.w_dexpilot)
        if w_dex > 1e-9:
            ref = targets["ref_vecs"]
            weights = targets["weights"]
            n = len(self._dp_origin_links)
            tip_loss = 0.0
            inv_n = 1.0 / max(n, 1)
            for i in range(n):
                o = self._dp_origin_links[i]
                t = self._dp_task_links[i]
                io, it = idx[o], idx[t]
                rvec = pos[it] - pos[io]
                resid = rvec - ref[i]
                tip_loss += float(weights[i]) * _smooth_l1_scalar(
                    float(np.linalg.norm(resid)), g.huber_delta
                )
                dvec = _smooth_l1_dvec(resid, g.huber_delta)
                jv = jacs[it] - jacs[io]
                grad += (w_dex * float(weights[i]) * inv_n) * (jv.T @ dvec)
            tip_loss *= inv_n
            loss += w_dex * tip_loss

            seg = g.thumb_seg
            hu = targets.get("thumb_dir")
            if seg.enabled and hu is not None and float(np.linalg.norm(hu)) > 1e-8:
                io = idx[seg.robot_ip]
                it = idx[seg.robot_tip]
                v = pos[it] - pos[io]
                term, dvec = _dir_loss_dvec(v, np.asarray(hu, dtype=np.float64))
                loss += w_dex * float(seg.w_dir) * term
                jv = jacs[it] - jacs[io]
                grad += (w_dex * float(seg.w_dir)) * (jv.T @ dvec)

        w_bone = float(mix.w_bone)
        bone_dirs = targets.get("bone_dirs")
        if w_bone > 1e-9 and bone_dirs is not None:
            bone_loss = 0.0
            for i, (o, t) in enumerate(self._bone_links):
                io, it = idx[o], idx[t]
                v = pos[it] - pos[io]
                term, dvec = _dir_loss_dvec(v, bone_dirs[i])
                bone_loss += self._bone_w[i] * term
                jv = jacs[it] - jacs[io]
                grad += w_bone * self._bone_w[i] * (jv.T @ dvec)
            target_fp = targets.get("frame_p")
            target_fs = targets.get("frame_s")
            for i, fr in enumerate(self._frame_specs):
                if target_fp is None:
                    break
                io = idx[fr.robot_origin]
                ip = idx[fr.robot_primary]
                isp = idx[fr.robot_secondary]
                hp = target_fp[i]
                hs = target_fs[i] if target_fs is not None else None
                term_p, d_p = _dir_loss_dvec(pos[ip] - pos[io], hp)
                bone_loss += fr.primary_weight * term_p
                grad += w_bone * fr.primary_weight * ((jacs[ip] - jacs[io]).T @ d_p)
                if hs is not None:
                    term_s, d_s = _dir_loss_dvec(pos[isp] - pos[io], hs)
                    bone_loss += fr.secondary_weight * term_s
                    grad += w_bone * fr.secondary_weight * (
                        (jacs[isp] - jacs[io]).T @ d_s
                    )
            loss += w_bone * bone_loss

        fp = g.fist_pose
        alpha_fist = float(targets.get("alpha_fist", 0.0))
        w_fist = float(mix.w_fist)
        if fp.enabled and w_fist > 1e-9 and alpha_fist > 1e-4:
            qf = self._fist_q
            fist_loss = 0.0
            scale = w_fist * alpha_fist
            for j in fp.joints:
                jj = int(j)
                if 0 <= jj < len(q):
                    dq = float(q[jj]) - float(qf[jj])
                    fist_loss += dq * dq
                    grad[jj] += scale * 2.0 * dq
            if len(q) > 3:
                dq = float(q[3]) - float(qf[3])
                fist_loss += dq * dq
                grad[3] += scale * 2.0 * dq
            loss += scale * fist_loss

        loss += self.config.norm_delta * float(np.sum((q - q_prev) ** 2))
        grad += 2.0 * self.config.norm_delta * (q - q_prev)
        return loss, grad

    def retarget(
        self,
        joints26: np.ndarray,
        *,
        remap_seed: np.ndarray | None = None,
        finger_chain=None,
        thumb_tip=None,
        index_abduction=None,
        flexion_scale: float = 1.0,
    ) -> np.ndarray:
        lo = self.limits[:, 0]
        hi = self.limits[:, 1]
        landmarks21 = openxr26_to_mediapipe21(joints26)
        if self._dexpilot:
            targets = self._build_targets_dexpilot(landmarks21)
        else:
            targets = self._build_targets_somehand(landmarks21)

        if remap_seed is not None:
            q0 = np.asarray(remap_seed, dtype=np.float64)
        elif self.config.warm_start_remap:
            q0 = openxr26_to_xhand1_q(
                joints26,
                side=self.side,
                limits=self.limits,
                flexion_scale=flexion_scale,
                finger_chain=finger_chain,
                thumb_tip=thumb_tip,
                index_abduction=index_abduction,
            )
        else:
            q0 = (
                self._q_prev.copy()
                if self._q_prev is not None
                else np.zeros(self.limits.shape[0], dtype=np.float64)
            )

        if self._q_prev is not None:
            q0 = 0.35 * q0 + 0.65 * self._q_prev
        q0 = np.clip(q0, lo, hi)
        q_prev = self._q_prev if self._q_prev is not None else q0.copy()
        link_names = self._opt_link_names

        q_opt = q0.copy()
        try:
            from scipy.optimize import minimize

            def objective_jac(x: np.ndarray):
                qc = np.clip(x, lo, hi)
                if self._dexpilot:
                    return self._eval_dexpilot(
                        qc, targets=targets, q_prev=q_prev, link_names=link_names
                    )
                return self._eval_somehand(
                    qc, targets=targets, q_prev=q_prev, link_names=link_names
                )

            res = minimize(
                objective_jac,
                q0,
                method="L-BFGS-B",
                jac=True,
                bounds=[(float(lo[i]), float(hi[i])) for i in range(len(q0))],
                options={
                    "maxiter": self.config.max_iterations,
                    "ftol": 1e-5,
                    "gtol": 1e-4,
                    "maxls": 8,
                },
            )
            fun0, _ = objective_jac(q0)
            if res.success or float(res.fun) < fun0:
                q_opt = np.clip(res.x, lo, hi)
        except ImportError:
            step = 0.25
            best, _ = (
                self._eval_dexpilot(q_opt, targets=targets, q_prev=q_prev, link_names=link_names)
                if self._dexpilot
                else self._eval_somehand(
                    q_opt, targets=targets, q_prev=q_prev, link_names=link_names
                )
            )
            for _ in range(self.config.max_iterations):
                if self._dexpilot:
                    loss_b, grad = self._eval_dexpilot(
                        q_opt, targets=targets, q_prev=q_prev, link_names=link_names
                    )
                else:
                    loss_b, grad = self._eval_somehand(
                        q_opt, targets=targets, q_prev=q_prev, link_names=link_names
                    )
                q_try = np.clip(q_opt - step * grad, lo, hi)
                if self._dexpilot:
                    loss_try, _ = self._eval_dexpilot(
                        q_try, targets=targets, q_prev=q_prev, link_names=link_names
                    )
                else:
                    loss_try, _ = self._eval_somehand(
                        q_try, targets=targets, q_prev=q_prev, link_names=link_names
                    )
                if loss_try < best - 1e-9:
                    best = loss_try
                    q_opt = q_try
                    step = min(step * 1.1, 0.4)
                else:
                    step *= 0.5
                    if step < 0.01:
                        break

        alpha = self.config.output_alpha
        if self._q_prev is not None and alpha < 1.0:
            q_out = alpha * q_opt + (1.0 - alpha) * self._q_prev
            q_out = np.clip(q_out, lo, hi)
        else:
            q_out = q_opt

        self._q_prev = q_out.copy()
        return q_out


def openxr26_to_xhand1_q_vector(
    joints26: np.ndarray,
    *,
    retargeter: VectorRetargeter,
    flexion_scale: float = 1.0,
    finger_chain=None,
    thumb_tip=None,
    index_abduction=None,
) -> np.ndarray:
    return retargeter.retarget(
        joints26,
        flexion_scale=flexion_scale,
        finger_chain=finger_chain,
        thumb_tip=thumb_tip,
        index_abduction=index_abduction,
    )
