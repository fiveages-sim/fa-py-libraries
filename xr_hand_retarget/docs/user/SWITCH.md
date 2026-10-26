# 安装 · 参数 · 执行

本仓唯一手映射包：`xr_hand_retarget`（旧包 `xr_xhand1_retarget` 已删除）。

## 1. 安装

在仓库根：

```bash
./init.sh install
# 或：uv pip install -e xr_hand_retarget

# 仅 Wuji official 需要额外官方 SDK：
./init.sh install-wuji-retargeting
```

复制本机配置：

```bash
cp configs/local.yaml.example configs/local.yaml
```

## 2. 参数（对齐 teleop_ws）

### 选手型（必改）

`configs/local.yaml`：

```yaml
hands:
  source: xrt
  type: xhand1    # xhand1 | wuji | o6 | l6 | o7
```

| `type` | 算法（自动锁定） | 对应 teleop_ws |
|--------|------------------|----------------|
| `xhand1` | `vector` | AnyDex KeyVector |
| `o6` / `l6` / `o7` | `curl` | 手套 YAML curl remap |
| `wuji` | `official` | Wuji SDK |

`method` 可省略。临时覆盖手型：`FA_HAND=o6 ./run.sh vr-xrt wrist`。

### 发布安全（可选，键名同 teleop_ws launch）

teleop_ws 用 launch 参数；本仓用 YAML。**键名一致**，写在 `configs/local.yaml` 顶层 `command:`（推荐）即可覆盖默认：

| teleop_ws launch | `local.yaml` → `command:` | 默认 |
|------------------|---------------------------|------|
| `publish_command` | `publish_command` | `true` |
| `max_velocity` | `max_velocity` | xhand1=`6` / linker=`4` / wuji=`1.5` |
| `filter_cutoff_hz` | `filter_cutoff_hz` | `8.0` |
| `command_epsilon` | `command_epsilon` | `0.001` |
| `target_timeout` | `target_timeout` | `0.25` |
| `allowed_fsm_states` | `allowed_fsm_states` | `[3, 4]` |

也可写在各手型 yaml 的 `safety.command`（库默认）；`local.yaml` 的 `command:` 优先级更高。

### Soft collision（仅 XHand1，可选，默认开）

```yaml
safety:
  soft_collision: true   # false 关闭
```

库默认 / `hands/xhand1.yaml` 均为 `true`。

### 分层对照

| 层 | 文件 | 作用 |
|----|------|------|
| 本机 | `configs/local.yaml` | `type` + 可选 `command` / `safety.soft_collision` |
| 手型索引 | `xr_hand_retarget/configs/<hand>.yaml` | backend + 默认 `safety.command` |
| 细节 | `configs/hands/*.yaml` | 算法 / soft_collision / motion |
| VR | `vr_pose_publisher/configs/xrt_teleop.yaml` | 手柄 / 光学腕 |

## 3. 执行

光学腕 6DoF 控臂与手型号无关，走 `/teleop/*_ee_pose`。  
**同一 PC Service 只能有一个 SDK 客户端**：标定前停掉 `vr-xrt`。

### 日常遥操（Linker / XHand1）

```bash
./run.sh vr-xrt wrist
# 或：FA_HAND=xhand1 ./run.sh vr-xrt wrist
```

### Wuji

```bash
python -m xr_hand_retarget --config xr_hand_retarget/configs/wuji_hand2.yaml --side right
```

### 标定

```bash
python -m xr_hand_retarget.calibrate --config xr_hand_retarget/configs/xhand1.yaml --side full
python -m xr_hand_retarget.calibrate --config xr_hand_retarget/configs/o6.yaml --side full
python -m xr_hand_retarget.calibrate --config xr_hand_retarget/configs/wuji_hand2.yaml
```

### 单手节点（调试）

先起 fa_w2 / Linker mock，再：

```bash
python -m xr_hand_retarget --side left
```

更细的开发命令见 [../dev/RUN.md](../dev/RUN.md)；发布安全细节见 [../dev/COMMAND_SAFETY.md](../dev/COMMAND_SAFETY.md)。
