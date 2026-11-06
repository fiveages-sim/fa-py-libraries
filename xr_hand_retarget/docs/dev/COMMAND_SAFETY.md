# 发布路径安全（对齐 teleop_ws DexCap）

`DualHandRetargetNode` 在 retarget 之后、发控制器命令之前走与 teleop_ws 相同的一层。

异步 teleop（`async_pipeline`）下：retarget 在 Process B 写 `q_target`，**Thread C @ 120Hz**（`FA_HAND_PUBLISH_HZ`）调 `OnlineCommandFilter` 并发布（禁止 rclpy timer）。见 [ASYNC_TELEOP.md](ASYNC_TELEOP.md)。

## 配置位置

1. **本机覆盖（推荐）**：`configs/local.yaml` 顶层 `command:` — 键名与 `dexcap_hand_retarget.launch.py` 相同  
2. 手型索引：`configs/<hand>.yaml` → `safety.command`  
3. 兼容：`hands.command` / `safety.command`（见 `extract_local_command_block`）

| 机制 | 键 | teleop_ws |
|------|-----|-----------|
| FSM 门控（fail-closed） | `allowed_fsm_states: [3, 4]` | `FsmCommandGate` |
| 开门后滤波重同步 | `require_resync` | `needs_resync` |
| LPF + 死区 + 限速 | `filter_cutoff_hz` / `command_epsilon` / `max_velocity` | `OnlineCommandFilter` |
| 目标超时保持 | `target_timeout` | stale hold |
| 总开关 | `publish_command` / `enabled` | `publish_command` |

默认 `max_velocity`：xhand1=`6.0`，Linker=`4.0`，wuji=`1.5`。

关闭整层：

```yaml
# configs/local.yaml
command:
  enabled: false
```

## Soft collision（XHand1）

与发布滤波独立：在求解后做软自碰投影。默认 **开**（`hands/xhand1.yaml` / 代码默认 `True`）。本机关闭：

```yaml
safety:
  soft_collision: false
```

P2：`safety.alpha` / vector `temporal_filter_alpha` / `output_alpha` 均为 **1.0**（内层不做 EMA）。平滑只走本节 `filter_cutoff_hz` + `max_velocity`（wall dt @ publish）。

内层 `CommandSafety` 的 `dt` 亦为 **wall**（`xhand1_pipeline._wall_dt`）；不再用 `publish.rate_hz`。`motion.*` 默认关时该 dt 几乎只记账；开 URDF 速度后才真正限步。

另有 backend 内 `hold_on_inactive`、`motion.*`（URDF 速度/FK），与上述发布滤波叠加，不互相替代。

activation_alpha（0.30）是约束激活平滑，不是输出塑形，故意保留。