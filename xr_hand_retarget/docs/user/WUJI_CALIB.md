# Wuji Hand 2 标定

默认走 **curl**（与 XHand1 相同的开掌 / 握拳 / 对掌采集），捏合用 URDF 反求 `thumb_cmc_flex`。

官方安装 / ABI / beta1·beta2 → [../algorithms/OFFICIAL.md](../algorithms/OFFICIAL.md)。

## curl（默认）

Pico 空手、PC Service 已连接。**同一 PC Service 只能有一个 SDK 客户端**，采集时不要跑 `vr-xrt`。

```bash
python -m xr_hand_retarget.calibrate \
  --config xr_hand_retarget/configs/wuji_hand2.yaml --side full
```

写入 `configs/calib/wuji_hand2.yaml`。URDF 指槽来自本包 `assets/wuji_hand2_{left,right}.urdf`：

- `hand_base` +Z 沿手指，指排沿 **+X（食指）→ −X（小指）**
- 扫 `thumb_cmc_flex`，让 `thumb_tip.x` 对齐该指 PIP / MCP / tip（`pinch_target_link`）
- 四指与拇指 **外展保持 0**（光学侧摆噪声大）
- DIP 默认 `dip_mode: couple`（0.65×PIP）；改 `calib` 才跟人手 DIP 插值

## 手套（官方默认）

Wuji Studio 里建 profile 并完成手套标定，关掉 Studio（手套只接受一个客户端），然后：

```bash
cd third_party/wuji-retargeting/example
python calibrate_offset.py --hand right \
  --config config/adaptive_analytical_wuji_glove_wuji_hand_2_right.yaml
```

把打印的 `wrist_offset_cm` / `thumb_offset_cm` 写进 `xr_hand_retarget/configs/wuji_hand2.yaml` 的 `wuji.overlay`，并把 `profile` 设为 `glove`。

## Pico 光学手势（本仓库 teleop）

官方附录：自定义输入只要变成 MediaPipe 21 点。本库在 `active=0`、掌三角形过小时 hold；出门按官方 beta1 URDF clip；会话内 `movej_interpolation_type=none`。

`profile: optical` 只关掉手套的 `thumb_skip_pip`，**默认保留** 官方 Hand2 的 `mediapipe_rotation`。

## 映射还不齐：按环节看

```
Pico OpenXR 26  →  MP21  →  MANO + mediapipe_rotation  →  官方 IK  →  BJC/RViz
     [A 识别]         [B 下标]           [C 坐标系]            [D 求解]
```

### A+B 预览（不跑 IK）

先停 `./run.sh vr-xrt` 和 `python -m xr_hand_retarget`。

```bash
python -m xr_hand_retarget.preview --side right \
  --config xr_hand_retarget/configs/wuji_hand2.yaml
```

| 窗口 | 含义 |
|------|------|
| 左栏骨架清晰 | Pico + 26→21 OK → 看 C/D |
| 左栏乱线 | 先别调 IK |
| 左栏正常、右栏整掌拧了 | `mediapipe_rotation`（C） |
| 两栏正常，RViz 仍差 | 去 D：官方 tuning_tool |

### D 官方三色骨架

```bash
python -m xr_hand_retarget.calibrate --config xr_hand_retarget/configs/wuji_hand2.yaml \
  --side right --seconds 20 --dump-pkl /tmp/xrt_mp21.pkl
cd third_party/wuji-retargeting/example
mjpython tuning_tool.py --play /tmp/xrt_mp21.pkl --hand right \
  --config config/adaptive_analytical_wuji_glove_wuji_hand_2_right.yaml
```

橙白对不齐时按官方顺序改 yaml / `wuji.overlay`，不要改 fa_w2。RViz 单指：`--fingers index`。

## 与驱动的约定

`wujihand2_ros2_control` 吃 BJC `Float64MultiArray`，**按下标**。`hand.joint_names` 必须与 `hand2.yaml` 一致。若官方 `qpos` 顺序不同，设 `wuji.qpos_index`。
