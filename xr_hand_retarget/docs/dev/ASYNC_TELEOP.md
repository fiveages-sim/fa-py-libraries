# 异步遥操解耦（EE ⊥ hand）

> **Hand 接近 controller = 节拍与连续性追平，带宽与延迟追不平。**  
> 不进程化 / 不向量化 / 不改 solver → hand 信息率上不去 100Hz。  
> 交换：target 29→40–55Hz + **120Hz** 匀发 + LPF ≈ **+15–20ms** 滞后，换 `|d2q|`↓。

本文是 P0/P1 边界与验收的权威说明。实现见 `dual_node.py`（async 三方）与 `xr_xrobotoolkit_node.py` / `xr_target_node.py`。

---

## 1. 目标

| 路径 | 目标 | 说明 |
|------|------|------|
| EE（wrist+hand） | arriv ≥**400**；p95 ≤**4.2** ms | 对齐 controller 基线；开 hand 不得拖垮 |
| EE unique | ≥380（C1：375–381） | left 受 `isActive` 影响时看 right / pico |
| `q_target` | ~29Hz → P0 后 40–55Hz | **单帧耗时上限**；本版不改 solver |
| `q_des` publish | **120Hz** 匀发（`FA_HAND_PUBLISH_HZ`） | ≈1.2× PD 100Hz；p50 ≈8.3ms |
| 平滑 | `\|d2q\| p99 ≤ 0.05` | 主靠 vmax 斜坡；0.035@120Hz 与 vmax×dt 冲突，不采 |
| 代价 | **+15–20ms** 滞后 | 一个 target 周期 + ~8Hz LPF 群延迟 |

**验收必须同时报（R5）**：`q_des` 节拍、`q_target` 刷新率、`|d2q| p99`；另加 `/joint_states` 动时重复率 \<5%。只报 publish Hz = 自欺。  
**探针顺序**：controller（回归）→ nohand → hands；失败面以 **hands** 为准。

---

## 2. 框架（进程隔离 + 快照）

```
Process A — EE
  Thread gate @ FA_FRAME_GATE_HZ（默认 3000）：读 timeStampNs → 变了才发 /teleop/*
    （不占 STE；XRTargetNode 无 frame_gate timer）
  STE：仅订阅 / 卫生回调（optical armer 等）
  notify_frame()：hand26 → shm + Event（禁止 retarget）
  Thread C @ 120Hz：读 shm q_target → LPF → 发手命令（唯一 publisher）
  hand 节点：独立 STE + spin 线程（卫生）

Process B — hand retarget（multiprocessing spawn）
  等 Event → 读 shm hand26 → SidePipeline L/R → 写 shm q_target
  解算限频默认 **80Hz**（`teleop.retarget_hz` / `FA_HAND_RETARGET_HZ`）；中间帧 drop-old
  BLAS/OpenMP **1 线程**；默认同绑最后 2 核（`FA_HAND_WORKER_CPUS`）
  不 xrt.init()；无 ROS
```

回退线程版（调试）：`FA_HAND_PROCESS=0 ./run.sh vr-xrt wrist`

硬约束：

1. **Publish 必须是 `threading.Thread`**，不能用 rclpy timer（无 sync/`use_timer`/`legacy-ee-frame`）。
2. **Frame gate 必须是 `threading.Thread`**，不能挂 STE timer。
3. **B 只读 shm**，不二次 SDK。
4. **`q_target` 最新覆盖**：不排队。
5. SDK 单客户端：只在 A 初始化。
6. **Publish 三态（分侧）**：`fresh`／`held`／`lost`；日志 `state=L:held/R:fresh`。xhand1：`active==0` 但掌心面积够仍跟（对齐 o6）；面积过小再累计 `inactive_hold_s` 才 held。
7. **关闸保留 `_last_sent`**；开闸重置 `_last_ingest_mono`。LOST 超时 `max(1.0, 4/retarget_hz)`。
8. **`/teleop/xrt_diag`** 尾部：`palm_u`、`orient_src` — 区分假 inactive vs 真遮挡。

GIL：默认 `setswitchinterval(0.0005)`；A/B 用 `FA_HAND_SWITCH_INTERVAL=0.0002`。  
验收：**开 hand 时 EE arriv/p95 接近 controller**。遮挡时 hand **arriv**→0（或 ~1 Hz）；hold 段 `rep%`≈100% 正常，看 arriv 不看 rep%。`target~0` / `pub~1` 在 held 下是预期。

---

## 3. 限制 / 红线

| 红线 | 约束 |
|------|------|
| 不改映射算法 | 不动约束/运动学公式。可动：解算限频、BLAS 线程、可选 `max_iterations`（`FA_HAND_MAX_ITER` / local） |
| 不改坐标轴/变换 | 轴序/符号/`openxr_to_robot`/yaw/palm_to_grip **原样**。软 clamp/跳变 hold **默认关**（光学跳变正常；开 hold 会冻 EE）。EE 热路径无 scipy（`so3_numpy`） |
| 握手信号不变 | 不动 `isActive` 语义、gate∈{3,4} 的 `_live()`、`hold_on_inactive` |
| 同帧对齐 | 命令仍是 `Float64MultiArray`（无 header）。`frame_seq` 走侧信道 CSV（`FA_HAND_ALIGN_LOG`） |

本版明确不做：C 搬进 B、A 绑核/nice−20、改 `max_iterations`、修 isActive、开 `motion.*`、换 `JointState`。

---

## 4. 塑形（P2 — 已落地，xhand1）

```
temporal_filter_alpha 1.0 → output_alpha 1.0 → safety.alpha 1.0 → outer LPF(8Hz, wall dt)
```

前三 EMA **关闭**；保留 `soft_collision` + outer `OnlineCommandFilter`。  
阶梯主杀器 **vmax=6**（外层，wall dt）。Linker/Wuji 未改。

`publish.rate_hz`：**已删**。async 发布 = `FA_HAND_PUBLISH_HZ` / 默认 **120**。  
内层 `CommandSafety` dt = **wall monotonic**。

---

## 5. QoS（R7）

手命令：**depth=1 + RELIABLE**。  
上 BEST_EFFORT 前必须 `ros2 topic info -v …` 确认订阅端。

---

## 6. P0（独立脚本，不改算法）

见仓库 `scripts/p0_teleop_cpu.sh`：performance governor、去掉 `nice -n 19`、`taskset` 参考。  
`setswitchinterval` / `FA_HAND_SWITCH_INTERVAL` 由 teleop 进程设置。

---

## 7. 验收表

| 指标 | 目标（hands） |
|------|----------------|
| pico / teleop arriv | **≥400** |
| EE p95 Δt | **≤4.2 ms** |
| EE unique（有效侧） | **≥380** |
| `q_des` p50 | **≈8.3 ms**（120Hz） |
| `\|d2q\| p99` | **≤0.05** |
| joint_states 动时重复 | **\<5%** |
| EE 热路径 scipy | **归零**（`so3_numpy`） |

---

## 8. 分期

| 期 | 内容 | 状态 |
|----|------|------|
| P0 | CPU 环境脚本 | `scripts/p0_teleop_cpu.sh` |
| P1 | 三方解耦 + QoS depth=1 | 已做 |
| P1b | 进程隔离 retarget | 已做 |
| P1c | 热路径减税 | 已做 |
| P1d | **门控 Thread@3000**；跳变 hold 默认关；publish **120**；hand 独立 executor | **收口**：EE unique ~250、hand ~100@ pico400；未追平 controller，但间隔均匀、少 pub 停顿。后续转顺畅度，不再抠 EE Hz |
| P2 / P2.1 | EMA→1.0；内层 wall dt；删 `publish.rate_hz` | 已做 |
| P3 | max_iter 口子（若仍需） | 后续 |

发布节拍：`FA_HAND_PUBLISH_HZ` → `teleop.publish_hz` → 120。  
解算节拍：`FA_HAND_RETARGET_HZ` → `teleop.retarget_hz` → **80**（对齐 xhand ~83Hz 驱动）。  
Worker CPU：`OMP/OPENBLAS_NUM_THREADS` 默认 1；`FA_HAND_WORKER_CPUS`（默认末 2 核）。  
可选：`FA_HAND_MAX_ITER` / `teleop.max_iterations`。  
其它 Dev：`FA_FRAME_GATE_HZ`、`FA_HAND_SWITCH_INTERVAL`、`FA_HAND_PROCESS`。  
用户层：[CONFIG.md](CONFIG.md)；顺畅度：[../user/SMOOTHNESS.md](../user/SMOOTHNESS.md)。

探针：`python3 scripts/teleop_probe.py --sec 5 --csv /tmp/test_{controller,nohand,hand}.csv`
