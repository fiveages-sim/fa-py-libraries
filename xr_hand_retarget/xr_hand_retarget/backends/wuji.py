"""Wuji Hand backend — official ``wuji_retargeting.Retargeter`` only.

OpenXR 26 → MediaPipe 21 → official AdaptiveOptimizerAnalytical → q[20].
Does not reuse XHand1 curl / dexpilot / thumb_ik.
"""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path

import numpy as np
import yaml

from xr_hand_retarget.landmarks import (
    openxr26_to_mediapipe21,
    palm_triangle_area_m2,
    pose_is_zero,
)
from xr_hand_retarget.pipeline import SideStep

# ROS Jazzy pinocchio is compiled against NumPy 1.x. Official wuji-retargeting
# allows numpy>=1.21, so pip/uv may pull 2.x and then `import pinocchio` aborts.
_NUMPY_PIN = "numpy>=1.26,<2"


def _assert_numpy_for_ros_pinocchio() -> None:
    major = int(np.__version__.split(".", 1)[0])
    if major < 2:
        return
    raise RuntimeError(
        "ROS Jazzy Pinocchio 与 NumPy 2.x 二进制不兼容，当前是 "
        f"{np.__version__}。请在已激活的虚拟环境执行:\n"
        f"  uv pip install '{_NUMPY_PIN}'\n"
        f"  # 或: python -m pip install '{_NUMPY_PIN}'\n"
        "若 scipy 随后报错，再装: uv pip install 'scipy>=1.11,<1.15'"
    )


def _as_active(flag) -> int:
    try:
        v = np.asarray(flag).reshape(-1)
        if v.size == 0:
            return 0
        return 1 if float(v[0]) > 0.5 else 0
    except Exception:
        return 0


def _normalize_hand26(raw) -> np.ndarray:
    arr = np.asarray(raw, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[0] != 26:
        arr = arr.reshape(26, -1)
    return arr


def _deep_merge(base: dict, overlay: dict) -> dict:
    out = deepcopy(base)
    for key, val in overlay.items():
        if isinstance(val, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], val)
        else:
            out[key] = deepcopy(val)
    return out


def _resolve_optimizer_asset(spec: str, yaml_dir: Path) -> Path | None:
    """Resolve official urdf/mjcf whether the yaml lives in example/config or configs/official."""
    from xr_hand_retarget.config import find_wuji_retargeting_root

    p = Path(spec)
    candidates: list[Path] = []
    if p.is_absolute():
        candidates.append(p)
    else:
        candidates.append((yaml_dir / p).resolve())
        root = find_wuji_retargeting_root()
        if root is not None:
            candidates.append((root / "example" / "config" / p).resolve())
            posix = spec.replace("\\", "/")
            marker = "wuji-description/"
            if marker in posix:
                tail = posix[posix.index(marker) :]
                candidates.append((root / "wuji_retargeting" / tail).resolve())
                candidates.append((root / tail).resolve())
    for cand in candidates:
        try:
            if cand.is_file():
                return cand
        except OSError:
            continue
    return None


def load_official_config(yaml_path: Path, overlay: dict | None) -> dict:
    yaml_path = yaml_path.resolve()
    with yaml_path.open("r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}
    yaml_dir = yaml_path.parent
    config["__yaml_dir"] = str(yaml_dir)
    opt = config.setdefault("optimizer", {})
    for key in ("urdf_path", "mjcf_path"):
        spec = opt.get(key)
        if not spec:
            continue
        found = _resolve_optimizer_asset(str(spec), yaml_dir)
        if found is not None:
            opt[key] = str(found)
    if overlay:
        retarget = config.setdefault("retarget", {})
        config["retarget"] = _deep_merge(retarget, overlay)
    return config


def _resolved_urdf_path(official: dict) -> Path | None:
    spec = (official.get("optimizer") or {}).get("urdf_path")
    if not spec:
        return None
    path = Path(spec)
    if path.is_file():
        return path
    if path.is_absolute():
        return path
    return (Path(official["__yaml_dir"]) / path).resolve()


def _missing_description_hint(urdf: Path) -> str:
    return (
        f"Hand 2 URDF 不存在: {urdf}\n"
        "官方仓库的子模块 wuji-description 没有检出（主仓有了，模型文件没有）。\n"
        "在仓库根执行:\n"
        "  git -C third_party/wuji-retargeting submodule update --init --recursive\n"
        "若子模块仍是空目录，直接 clone:\n"
        "  git clone https://github.com/wuji-technology/wuji-description.git \\\n"
        "    third_party/wuji-retargeting/wuji_retargeting/wuji-description\n"
        "然后再运行原来的 python -m xr_hand_retarget …"
    )


def require_official_urdf(official: dict) -> Path | None:
    urdf = _resolved_urdf_path(official)
    if urdf is None:
        return None
    if urdf.is_file():
        return urdf
    raise FileNotFoundError(_missing_description_hint(urdf))


def preflight_wuji(cfg, side: str) -> None:
    """Fail before SDK init if official YAML / URDF are missing."""
    if cfg.wuji is None:
        raise ValueError("wuji backend requires a wuji: block in the yaml")
    yaml_path = cfg.wuji.yaml_for(side)
    official = load_official_config(yaml_path, cfg.wuji.retarget_overlay())
    require_official_urdf(official)


def canon_hand2_joint(name: str) -> str:
    """Strip side prefixes so official URDF names match fa_w2 BJC names.

    Official: ``r_ring_finger_mcp_flex``; fa_w2: ``ring_mcp_flex``.
    Do **not** treat the letters ``r_`` in ``ring_*`` as a right-hand prefix.
    """
    n = str(name).strip()
    for prefix in ("left_hand_", "right_hand_"):
        if n.startswith(prefix):
            n = n[len(prefix) :]
            break
    else:
        rest = n[2:] if len(n) > 2 else ""
        rest_l = rest.lower()
        if n[:2] in ("l_", "r_", "L_", "R_") and rest_l.startswith(
            ("thumb", "index", "middle", "ring", "pinky")
        ):
            n = rest
    return n.replace("ring_finger_", "ring_")


def perm_pinocchio_to_command(
    pin_names: list[str],
    command_names: list[str],
    q_index: list[int] | None = None,
) -> np.ndarray:
    """``q_cmd = q_pin[perm]`` so BJC slot i is the official joint of the same name.

    ``q_index[k]`` is Pinocchio ``idx_q`` for ``pin_names[k]``. Do not use the
    names-list position as a qpos index: ``model.names`` order can disagree with
    ``qpos``, and that mismatch slams flex into abd limits (binary jump).
    """
    if q_index is not None and len(q_index) != len(pin_names):
        raise ValueError(
            f"q_index length {len(q_index)} != pin_names {len(pin_names)}"
        )
    idx: dict[str, int] = {}
    for i, n in enumerate(pin_names):
        idx[canon_hand2_joint(n)] = int(q_index[i]) if q_index is not None else i
    missing: list[str] = []
    perm: list[int] = []
    for name in command_names:
        key = canon_hand2_joint(name)
        if key not in idx:
            missing.append(f"{name} (canon={key})")
            continue
        perm.append(idx[key])
    if missing:
        have = [canon_hand2_joint(n) for n in pin_names]
        raise ValueError(
            "official Pinocchio joints do not cover fa_w2 command names; "
            "refusing to publish (would drive the wrong fingers).\n"
            f"  missing: {missing}\n"
            f"  pinocchio: {have}"
        )
    if len(perm) != len(command_names):
        raise ValueError("qpos name remap length mismatch")
    return np.asarray(perm, dtype=np.int64)


def pinocchio_name_qindex(robot) -> tuple[list[str], list[int]]:
    """Actuated joint names with their ``qpos`` indices (``idx_q``), not list order."""
    model = robot.model
    names: list[str] = []
    q_index: list[int] = []
    njoints = int(model.njoints)
    for j in range(njoints):
        nq = int(model.nqs[j])
        if nq != 1:
            continue
        names.append(str(model.names[j]))
        q_index.append(int(model.idx_qs[j]))
    if not names:
        names = list(robot.dof_joint_names)
        q_index = list(range(len(names)))
    return names, q_index


class WujiBackend:
    """Official ``Retargeter.from_config`` per side."""

    def __init__(self, side: str, cfg):
        if side not in ("left", "right"):
            raise ValueError(f"side must be left|right, got {side!r}")
        if cfg.wuji is None:
            raise ValueError("wuji backend requires a wuji: block in the yaml")
        _assert_numpy_for_ros_pinocchio()
        try:
            from wuji_retargeting import Retargeter
        except ImportError as exc:
            raise ImportError(
                "wuji-retargeting is not installed. Run:\n"
                "  ./init.sh install-wuji-retargeting\n"
                "then point FA_HAND_CONFIG at configs/wuji_hand2.yaml"
            ) from exc

        self.side = side
        self.dof = int(cfg.dof)
        self.retargeting_type = "official"
        self.vector_profile = ""
        self._cfg = cfg
        self._wuji = cfg.wuji
        yaml_path = self._wuji.yaml_for(side)
        overlay = self._wuji.retarget_overlay()
        official = load_official_config(yaml_path, overlay)
        require_official_urdf(official)
        self._retargeter = Retargeter.from_config(official, hand_side=side)
        robot = self._retargeter.optimizer.robot
        pin_names, pin_qidx = pinocchio_name_qindex(robot)
        cmd_names = list(cfg.joint_names)
        n_pin = int(self._retargeter.num_joints)
        self.dof = int(cfg.dof)
        if self.dof != len(cmd_names):
            raise ValueError(
                f"hand.dof {self.dof} != len(joint_names) {len(cmd_names)}"
            )
        explicit = self._wuji.qpos_permutation(self.dof)
        remap_mode = str(getattr(self._wuji, "qpos_remap", "name") or "name")
        if explicit is not None:
            if int(explicit.min()) < 0 or int(explicit.max()) >= n_pin:
                raise ValueError(
                    f"wuji.qpos_index indexes official qpos of length {n_pin}, "
                    f"got min={int(explicit.min())} max={int(explicit.max())}"
                )
            self._perm = explicit
            src = "yaml qpos_index"
        elif remap_mode == "off":
            if n_pin != self.dof:
                raise ValueError(
                    f"qpos_remap=off requires pin_nq==dof, got {n_pin} vs {self.dof}"
                )
            self._perm = np.arange(self.dof, dtype=np.int64)
            src = "off"
        else:
            self._perm = perm_pinocchio_to_command(
                pin_names, cmd_names, q_index=pin_qidx
            )
            src = "idx_q"
        identity = np.arange(self.dof, dtype=np.int64)
        same = np.array_equal(self._perm, identity)
        self._q_hold: np.ndarray | None = None
        self._was_held = True
        self._unlock = 0
        self._dbg_n = 0
        pin_limits = np.asarray(robot.joint_limits, dtype=np.float64)
        try:
            self._q_limits = pin_limits[self._perm]
        except Exception:
            self._q_limits = None
        if self._q_limits is not None and self._q_limits.shape != (self.dof, 2):
            self._q_limits = None
        dt = 1.0 / max(float(cfg.rate_hz), 1.0)
        vel = max(float(cfg.max_joint_vel_rad_s), 0.0)
        self._max_delta = vel * dt if vel > 0.0 else 0.0
        qidx_of = {n: q for n, q in zip(pin_names, pin_qidx)}
        print(
            f"[retarget] side={side} type=official engine=Retargeter "
            f"official={yaml_path} profile={self._wuji.profile} "
            f"dof={self.dof} pin_nq={n_pin} qpos_remap={src} "
            f"{'identity' if same else self._perm.tolist()} "
            f"clamp_urdf={cfg.clamp_urdf} max_dq={self._max_delta:.4f}rad/step "
            f"palm_area>={cfg.min_palm_area_m2} unlock={cfg.unlock_frames}",
            flush=True,
        )
        print(
            "[qpos map] i  BJC_name                     pin_name                         qidx   lo     hi",
            flush=True,
        )
        for i, cmd in enumerate(cmd_names):
            pi = int(self._perm[i])
            pin_n = next(
                (n for n, q in qidx_of.items() if q == pi),
                pin_names[pi] if pi < len(pin_names) else f"q[{pi}]",
            )
            lo = hi = float("nan")
            if self._q_limits is not None:
                lo = float(self._q_limits[i, 0])
                hi = float(self._q_limits[i, 1])
            print(
                f"  {i:02d} {cmd:28s} {pin_n:32s} {pi:4d} {lo:7.3f} {hi:7.3f}",
                flush=True,
            )
        # #region agent log
        try:
            import json as _json, time as _time
            with open("/home/fiveages/fa-py-libraries/.cursor/debug-fc4a89.log", "a") as _f:
                _f.write(_json.dumps({"sessionId":"fc4a89","hypothesisId":"H3","location":"wuji.py:init","message":"official qpos map","data":{"side":side,"perm":self._perm.tolist(),"identity":bool(same),"src":src,"n_pin":n_pin,"dof":self.dof,"pin_names":[canon_hand2_joint(n) for n in pin_names],"cmd_names":[canon_hand2_joint(n) for n in cmd_names]},"timestamp":int(_time.time()*1000)})+"\n")
        except Exception:
            pass
        # #endregion

    def _hold(self, active: int, xyz_abs: float, reason: str, *, reset_unlock: bool = True) -> SideStep:
        if not self._was_held:
            try:
                self._retargeter.reset()
            except Exception:
                pass
        if reset_unlock:
            self._unlock = 0
        self._was_held = True
        if self._q_hold is None:
            q = np.zeros(self.dof, dtype=np.float64)
        else:
            q = self._q_hold.copy()
        # #region agent log
        self._dbg_n += 1
        if self._dbg_n <= 6 or self._dbg_n % 40 == 0:
            try:
                import json as _json, time as _time
                with open("/home/fiveages/fa-py-libraries/.cursor/debug-fc4a89.log", "a") as _f:
                    _f.write(_json.dumps({"sessionId":"fc4a89","hypothesisId":"H5","location":"wuji.py:hold","message":"official hold","data":{"side":self.side,"n":self._dbg_n,"active":active,"xyz_abs":xyz_abs,"reason":reason},"timestamp":int(_time.time()*1000)})+"\n")
            except Exception:
                pass
        # #endregion
        return SideStep(q=q, active=active, held=True, xyz_abs=xyz_abs, reason=reason, sat=-1)

    def _shape_q(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=np.float64).reshape(-1)
        if q.shape[0] != self.dof:
            if q.shape[0] > self.dof:
                q = q[: self.dof]
            else:
                pad = np.zeros(self.dof, dtype=np.float64)
                pad[: q.shape[0]] = q
                q = pad
        if self._cfg.clamp_urdf and self._q_limits is not None:
            q = np.clip(q, self._q_limits[:, 0], self._q_limits[:, 1])
        if self._max_delta > 0.0 and self._q_hold is not None:
            q = self._q_hold + np.clip(q - self._q_hold, -self._max_delta, self._max_delta)
        return q

    def step(self, frame) -> SideStep:
        from xr_hand_retarget.sources.frame import HandFrame

        if not isinstance(frame, HandFrame):
            raise TypeError(f"expected HandFrame, got {type(frame)!r}")
        active = int(frame.active)
        raw = frame.ensure_joints26()
        xyz_abs = float(frame.xyz_abs)
        if self._cfg.hold_on_inactive and active == 0:
            return self._hold(active, xyz_abs, "inactive")
        if self._cfg.hold_on_zero_pose and pose_is_zero(raw):
            return self._hold(active, xyz_abs, "zero")

        mp21 = frame.ensure_xyz21()
        area = palm_triangle_area_m2(mp21)
        if area < float(self._cfg.min_palm_area_m2):
            return self._hold(active, xyz_abs, f"palm={area:.2e}")

        need = max(int(self._cfg.unlock_frames), 1)
        if self._was_held:
            self._unlock += 1
            if self._unlock < need:
                return self._hold(
                    active, xyz_abs, f"unlock {self._unlock}/{need}", reset_unlock=False
                )
            self._was_held = False
            self._unlock = need

        q_raw = np.asarray(self._retargeter.retarget(mp21), dtype=np.float64).reshape(-1)
        q = q_raw[self._perm]
        q = self._shape_q(q)
        dq = 0.0 if self._q_hold is None else float(np.max(np.abs(q - self._q_hold)))
        self._q_hold = q.copy()
        sat = -1
        if self._q_limits is not None:
            lo, hi = self._q_limits[:, 0], self._q_limits[:, 1]
            sat = int(np.sum((q - lo < 1e-3) | (hi - q < 1e-3)))
        # #region agent log
        self._dbg_n += 1
        if self._dbg_n <= 8 or self._dbg_n % 20 == 0 or dq > 0.15:
            try:
                import json as _json, time as _time
                wr = mp21 - mp21[0:1]
                span = float(np.linalg.norm(wr, axis=1).max())
                bones = [float(np.linalg.norm(mp21[b] - mp21[a])) for a, b in ((0, 5), (5, 8), (0, 9), (9, 12), (1, 4))]
                with open("/home/fiveages/fa-py-libraries/.cursor/debug-fc4a89.log", "a") as _f:
                    _f.write(_json.dumps({"sessionId":"fc4a89","hypothesisId":"H1,H2,H4","location":"wuji.py:step","message":"official step","data":{"side":self.side,"n":self._dbg_n,"active":active,"xyz_abs":xyz_abs,"jshape":list(raw.shape),"ncols":int(raw.shape[1]) if raw.ndim==2 else -1,"mp21":list(mp21.shape),"span_m":span,"bones_m":bones,"area":area,"q_raw_n":int(q_raw.size),"q_raw0":np.round(q_raw[:8],4).tolist(),"q_cmd0":np.round(q[:8],4).tolist(),"dq":dq,"sat":sat,"perm_id":bool(np.array_equal(self._perm, np.arange(self.dof)))},"timestamp":int(_time.time()*1000)})+"\n")
            except Exception:
                pass
        # #endregion
        return SideStep(q=q, active=active, held=False, xyz_abs=xyz_abs, reason="ok", sat=sat)
