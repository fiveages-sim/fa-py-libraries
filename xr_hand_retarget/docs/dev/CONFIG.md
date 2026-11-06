# xr_hand_retarget 配置分层

三层职责硬切：**User（本机） / Product（手型） / Dev（env）**。

```
configs/local.yaml          User：操作员白名单（唯一本机入口）
索引 yaml + hands/*         Product：手型几何 / 约束 / 库默认 command
FA_* env                    Dev：门控/GIL/进程隔离（不进 local）
vr …/xrt_teleop.yaml        VR 腕/手柄产品配置（preset；不与 local 合并成单文件）
```

## 1. User — `configs/local.yaml`

白名单（其它键请勿当用户旋钮）：

| 键 | 作用 |
|----|------|
| `hands.type` / `left`/`right` | 手型 |
| `hands.source` / `hands.user` | 输入源 / 人标定 id |
| `teleop.publish_hz` | async `q_des` Hz（默认 120；`FA_HAND_PUBLISH_HZ` 优先） |
| `teleop.retarget_hz` | Process B 求解上限（默认 80；`FA_HAND_RETARGET_HZ` 优先） |
| `teleop.max_iterations` | 可选压 vector 迭代（`FA_HAND_MAX_ITER` 优先） |
| `teleop.ee_source` | 文档指针 `wrist\|controller\|hand`（实际仍用 `./run.sh` 参数） |
| `command.*` | `max_velocity` / `filter_cutoff_hz` / …（顺畅度） |
| `safety.soft_collision` | 仅 XHand1 |

模板：[`configs/local.yaml.example`](../../../configs/local.yaml.example)。  
顺畅度 A/B：[`../user/SMOOTHNESS.md`](../user/SMOOTHNESS.md)。

```yaml
hands:
  source: xrt
  type: xhand1
teleop:
  publish_hz: 120
command:
  max_velocity: 6.0
  filter_cutoff_hz: 10.0
```

临时换手：`FA_HAND=o7 ./run.sh vr-xrt wrist`。

## 2. Product — 索引 + `hands/*`

| 层 | 文件 | 角色 |
|----|------|------|
| 索引 | `configs/{xhand1,wuji_hand2,o6,l6,o7}.yaml` | backend + **库默认** `safety.command` |
| T_robot | `hands/*` | 捏合槽、尺度、权重、soft_collision 产品默认 |
| T_person | `users/*` / `calib/*` | 人→标准手 |
| 标准手 | `standard_hand.yaml` | 固定骨长 |

**本机体感不要改 Product 的 `safety.command`** — 用 local `command:` 覆盖。  
`publish.rate_hz` 已废弃（async 不读）；内层 EMA alpha 固定 1.0，平滑只走外层 LPF。

### LinkerHand

| 层 | 文件 |
|----|------|
| 索引 | `configs/{o6,l6,o7}.yaml` |
| T_robot | `hands/{o6,l6,o7}.yaml` |
| T_person | `calib/*` / `users/pico.yaml` |
| 默认 | `linker_defaults.yaml` |

### Wuji

| 层 | 文件 |
|----|------|
| 索引 | `wuji_hand2.yaml` |
| T_robot | `hands/wuji.yaml` |
| Official | `official/adaptive_analytical_*.yaml` |

### XHand1

索引 `xhand1.yaml` → `hands/xhand1.yaml`（`robot:` = T_xhand；vector 约束在 retargeting）。

## 3. Dev — env

| env | 默认 | 说明 |
|-----|------|------|
| `FA_HAND_PUBLISH_HZ` | （无则 local / 120） | 覆盖 publish |
| `FA_HAND_RETARGET_HZ` | （无则 local / 80） | 覆盖解算限频 |
| `FA_HAND_MAX_ITER` | （无则 yaml 40） | 覆盖 L-BFGS 迭代 |
| `FA_HAND_WORKER_CPUS` | 末 2 核 | worker `sched_setaffinity` |
| `FA_FRAME_GATE_HZ` | 3000 | 帧门控线程 |
| `FA_HAND_SWITCH_INTERVAL` | 0.0005 | GIL A/B |
| `FA_HAND_PROCESS` | 1 | 0=线程版 retarget |

见 [ASYNC_TELEOP.md](ASYNC_TELEOP.md)。

## 4. 标准手 / 姿态

`configs/standard_hand.yaml` — `xr_hand_retarget.person.load_standard_hand()`。  
姿态手册：`poses/handbook.yaml`。
