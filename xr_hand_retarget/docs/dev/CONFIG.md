# xr_hand_retarget 配置分层（P1）

```
standard_hand.yaml     固定骨长 / 标准手 21 点（T_standard）
users/*  + calib/*     T_person（人→标准手；curl 端点也是 user 文件）
hands/*                T_robot（标准手→设备：捏合槽、尺度、权重）
索引 yaml              o6 / l6 / o7 / wuji_hand2 / xhand1
local.yaml             source + user + type + method（本机）
```

## 1. 本机 `configs/local.yaml`

```yaml
hands:
  source: xrt       # 输入；缺省 xrt（多源见 PIPELINE P3）
  type: o6          # o6 | l6 | o7 | wuji | xhand1
  method: curl
  # user: pico      # 可选；覆盖该手默认 user/calib
```

临时：`FA_HAND=o7 ./run.sh vr-xrt wrist`。  
Wuji 节点验证：`python -m xr_hand_retarget --config xr_hand_retarget/configs/wuji_hand2.yaml --side right`（见 [RUN.md](RUN.md)）。

## 2. LinkerHand（O6 / L6 / O7）

| 层 | 文件 | 角色 |
|----|------|------|
| 索引 | `configs/{o6,l6,o7}.yaml` | `backend: linker` + 路径 |
| T_robot | `hands/{o6,l6,o7}.yaml` | pinch_q、拳包络、j2/j3 |
| T_person | `calib/o6.yaml` / `calib/l6.yaml` / `users/pico.yaml` | 人手 curl/az/h |
| 默认 | `linker_defaults.yaml` | ros / safety / attract 距离 |

不要混会话（O7 的 `users/pico.yaml` 不要给 O6）。

## 3. Wuji

| 层 | 文件 |
|----|------|
| 索引 | `wuji_hand2.yaml` |
| T_robot | `hands/wuji.yaml`（segment_scaling、权重、pinch_thresholds） |
| Official | `official/adaptive_analytical_*.yaml` |
| curl user | `calib/wuji_hand2.yaml` |
| 可选 T_person | `users/<id>.yaml` → `person.segment_scaling`（覆盖 robot） |

## 4. XHand1

索引 `xhand1.yaml` → `hands/xhand1.yaml`：

| 键 | 角色 |
|----|------|
| `robot:` | **T_xhand**：`use_urdf_limits`、`pinch_target_link`、`finger_scale` |
| `retargeting.curl.calib` | **T_person**：`calib/xrt_default.yaml` |

转储 URDF：`python -m xr_hand_retarget.txhand --side right`（见 [TXHAND.md](TXHAND.md)）。

## 5. 标准手

`configs/standard_hand.yaml` — 加载：`xr_hand_retarget.person.load_standard_hand()`。  
姿态手册：`poses/handbook.yaml`（人在 21 点上的语义姿态）。
