# 安装 · 参数 · 执行

本仓唯一手映射包：`xr_hand_retarget`。

## 1. 安装

```bash
./init.sh install
# Wuji official 另需：./init.sh install-wuji-retargeting
cp configs/local.yaml.example configs/local.yaml
```

## 2. 用户层参数（`configs/local.yaml`）

**本机只改这一处。** 完整白名单见 [../dev/CONFIG.md](../dev/CONFIG.md)。

### 选手型

```yaml
hands:
  source: xrt
  type: xhand1    # xhand1 | wuji | o6 | l6 | o7
```

| `type` | 算法 | teleop_ws |
|--------|------|-----------|
| `xhand1` | `vector` | AnyDex KeyVector |
| `o6` / `l6` / `o7` | `curl` | 手套 curl |
| `wuji` | `official` | Wuji SDK |

临时：`FA_HAND=o6 ./run.sh vr-xrt wrist`。

### 顺畅度 + 发布节拍

```yaml
teleop:
  publish_hz: 120
  retarget_hz: 80          # 解算上限（xhand 驱动~83Hz）；中间帧 drop-old
  # max_iterations: 25     # 可选压 CPU

command:
  max_velocity: 6.0          # xhand1=6 / linker=4 / wuji=1.5
  filter_cutoff_hz: 10.0     # 本机推荐；库默认 8
  command_epsilon: 0.001
  target_timeout: 0.25
  allowed_fsm_states: [3, 4]
```

| teleop_ws launch | `local.yaml` | 说明 |
|------------------|--------------|------|
| `max_velocity` | `command.max_velocity` | 斜坡 |
| `filter_cutoff_hz` | `command.filter_cutoff_hz` | LPF |
| — | `teleop.publish_hz` | async `q_des` Hz |
| — | `teleop.retarget_hz` | remapping 求解 Hz |

标定步骤：[SMOOTHNESS.md](SMOOTHNESS.md)。

### Soft collision（XHand1）

```yaml
safety:
  soft_collision: true
```

### 分层

| 层 | 文件 | 作用 |
|----|------|------|
| **User** | `configs/local.yaml` | 白名单旋钮 |
| Product | `xr_hand_retarget/configs/<hand>.yaml` | 库默认 command |
| Product | `configs/hands/*` | 算法 / 几何 |
| VR | `vr_pose_publisher/configs/xrt_teleop.yaml` | 腕 / 手柄 |
| Dev | `FA_*` env | 门控 / GIL / 进程 |

## 3. 执行

同一 PC Service 只能一个 SDK 客户端。

```bash
./run.sh vr-xrt wrist
python3 scripts/teleop_probe.py --sec 5 --csv /tmp/hand.csv
```

Wuji 节点：`python -m xr_hand_retarget --config xr_hand_retarget/configs/wuji_hand2.yaml --side right`

更细开发命令：[../dev/RUN.md](../dev/RUN.md)；发布安全：[../dev/COMMAND_SAFETY.md](../dev/COMMAND_SAFETY.md)。
