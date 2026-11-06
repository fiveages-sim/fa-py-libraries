# 异步遥操解耦（EE ⊥ hand）

> **Hand 接近 controller = 节拍与连续性追平，带宽与延迟追不平。**  
> 不进程化 / 不向量化 / 不改 solver → hand 信息率上不去 100Hz。  
> 交换：target 29→40–55Hz + 250Hz 匀发 + LPF ≈ **+15–20ms 滞后**，换 `|d2q|`↓。

本文是 P0/P1 边界与验收的权威说明。实现见 `dual_node.py`（async 三方）与 `xr_xrobotoolkit_node.py`。

---

## 1. 目标

| 路径 | 目标 | 说明 |
|------|------|------|
| EE | ~404Hz unique（C1：375–381，rep~6%） | 拆 hand 重活后零代价回升 |
| `q_target` | 现 ~29Hz → P0 后 40–55Hz | **单帧耗时上限**；本版不改 solver |
| `q_des` publish | **250Hz** 匀发 | ≈2.5× driver 100Hz |
| 平滑 | `\|d2q\| p99 ≤ 0.05`；`\|dq\|/样本 ≤ 0.06` | 主靠 vmax 斜坡，不是叠 EMA |
| 代价 | **+15–20ms** 滞后 | 一个 target 周期 + ~8Hz LPF 群延迟 |

**验收必须同时报（R5）**：`q_des` 节拍、`q_target` 刷新率、`|d2q| p99`；另加 `/joint_states` 动时重复率 \<5%。只报 250Hz = 自欺。

---

## 2. 框架（进程隔离 + 快照）

实测：线程版 async 后 EE 从 ~29Hz 回到 250–400，但仍会抖到 250（`--no-hands` 稳 400）→ **同进程 GIL 仍伤 EE**。故默认上进程隔离。

```
Process A — EE + publish（rclpy STE）
  读 SDK → 发 head / L_ee / R_ee
  notify_frame()：hand26 → shm + Event（禁止 retarget）
  Thread C @ 250Hz：读 shm q_target → LPF → 发手命令（唯一 publisher）

Process B — hand retarget（multiprocessing spawn）
  等 Event → 读 shm hand26 → SidePipeline L/R → 写 shm q_target
  不 xrt.init()；无 ROS；drop-old
```

回退线程版（调试）：`FA_HAND_PROCESS=0 ./run.sh vr-xrt wrist`

硬约束：

1. **Publish 必须是 `threading.Thread`**，不能用 rclpy timer。
2. **B 只读 shm**，不二次 SDK。
3. **`q_target` 最新覆盖**：不排队。
4. SDK 单客户端：只在 A 初始化。

GIL：A 内仍 `setswitchinterval(0.0005)`。验收：**开 hand 时 EE 应接近 `--no-hands` 的稳 ~400Hz**。

---

## 3. 限制 / 红线

| 红线 | 约束 |
|------|------|
| 不改映射算法 | 不动 `VectorRetargeter` / `kinematics_xhand1.py` / 算法 yaml 块。可动：外层塑形开关、`_dt`、publish/QoS、线程编排。`max_iterations` 只留 backend 读取口，**值本版不动** |
| 不改坐标轴/变换 | wrist 仅允许之后把 `_clamp_ee_step` 换成跳变探测（P3）；轴序/符号/`openxr_to_robot`/yaw/palm_to_grip **原样** |
| 握手信号不变 | 不动 `isActive` 语义、gate∈{3,4} 的 `_live()`、`hold_on_inactive`。hold = worker 不写新 target，publish 继续按节拍发 |
| 同帧对齐 | 命令仍是 `Float64MultiArray`（无 header）。`frame_seq` 走侧信道 CSV（`FA_HAND_ALIGN_LOG`）；新 target 立即补发；确定性同输入同输出 |

本版明确不做：改 `max_iterations` 值、输入新鲜度闸、修 isActive、开 `motion.*`、左右独立 worker、换 `JointState`、进程化、二阶跟踪器。

---

## 4. 塑形（P2，可与 P1 分步）

线上现有 4 层：

```
temporal_filter_alpha 0.65 → output_alpha 0.92 → safety.alpha 0.90 → outer LPF(8Hz, wall dt)
```

定稿：前三 → **1.0**；保留 `soft_collision` + outer LPF。  
`fc ≈ 0.25–0.3 × target_rate`（现 ~8；P0 后可到 12–15）。阶梯主杀器是 **vmax=6**。

`rate_hz`：废弃作吞吐闸；删 `_min_step_dt`。safety `dt` 改 wall（当前 `max_delta_rad=0` + `motion` 关，几乎无害）。

---

## 5. QoS（R7）

现状：`create_publisher(..., 10)` → reliable depth=10 → 积压=延迟。  
本版：**depth=1 + RELIABLE**（先保证与 `left_hand_controller` 匹配）。  
上 BEST_EFFORT 前必须 `ros2 topic info -v …` 确认订阅端不要求 RELIABLE。

---

## 6. P0（独立脚本，不改算法）

见仓库 `scripts/p0_teleop_cpu.sh`：

1. `cpupower frequency-set -g performance`
2. 提示去掉 `nice -n 19`
3. `taskset` 绑 P 核（打印 `lscpu -e` 参考）
4. 说明：`setswitchinterval` 由 teleop 进程内自动设置

期望单帧 34.6ms → 19–25ms（纯环境）。

---

## 7. 验收表

| 指标 | 现在（参考） | 目标 |
|------|--------------|------|
| EE p99Δt/p50Δt | 2.64（被拖） | **≤2.8**（线程 GIL 附加可接受；\>3.5 才进程） |
| EE median unique Hz | ~29 | **≥380** |
| `q_target` Hz | ~29 | 29（本版）/ 40–55（P0 后）；本版**不**把「提高上限」当验收 |
| `q_des` 间隔 | — | ~4ms；**p99≤2ms，max≤5ms** |
| `\|d2q\| p99` | 0.277 | **≤0.05** |
| joint_states 重复率（动） | C2~71% | **\<5%** |

---

## 8. 分期

| 期 | 内容 | 状态 |
|----|------|------|
| P0 | CPU 环境脚本 | `scripts/p0_teleop_cpu.sh` |
| P1 | 三方解耦 + 删门控/allclose/心跳 + QoS depth=1 | 已做（线程版） |
| P1b | **进程隔离**：retarget → spawn + shm；EE 稳 400 | 已做 |
| P1c | 热路径减税：shm 一次 attach、peek seq、publish 无 allclose、`target_fresh` 不超时冻、controller 跳过 `_rates`、`safe_mat_update` 用 frame_id | 本变更 |
| P2 | 三层 EMA→1.0；fc/vmax 定参 | 后续 |
| P3 | wall dt yaml、max_iter 口子、wrist 跳变探测 | 后续 |

热路径备注：`FRAME_GATE` 默认 800Hz（旧 1500 空转税高）；`FA_FRAME_GATE_HZ` 可调。publish 层若出现 `ingest age>1s` 日志，对照曾见的 age≈2.69s worker 停顿。
