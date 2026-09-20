# 曲率标定映射（curl）

XRoboToolkit 光学手 26 关节 xyz → 骨链曲率 / 掌面侧向特征 → 按标定端点插值到 XHand1 12 维。

`configs/local.yaml` 设 `type: xhand1` 且 `method: curl`。包装索引默认是 **dexpilot**；只写 curl 却不覆盖 method 时跑的仍是 dexpilot。

| 文件 | 作用 |
|------|------|
| `xr_hand_retarget/configs/xhand1.yaml` | 索引；`retargeting.type` 可被 local.yaml `method` 覆盖 |
| `xr_hand_retarget/configs/calib/xrt_default.yaml` | 开掌 / 握拳 / 对掌 / 食指侧角 / 可选捏合 |
| `xr_hand_retarget/configs/xrt_curl.yaml` | 精简 curl 入口（可选） |

回退其它算法：`local.yaml` 的 `method: dexpilot|vector|thumb_ik`。

食指侧摆（`index_joint1`）单独两拍，且只在 Pico 追踪好时采：五指并拢→0，食指外开到极限→正极限。未采则保持 0。

## 映射

| 标定位 | 测什么 | 映射到 |
|--------|--------|--------|
| `open` 手掌伸直 | 四指 MCP/PIP、拇指屈伸、拇指掌面横向。拇指伸直并**与掌垂直** | 各关节 **0 位**（不含已标定的 `index_joint1`） |
| `fist` 握拳 | **只测四指** MCP/PIP | 四指屈伸 **URDF 上限** |
| `thumb_to_pinky` 对掌 | 其余指伸直，拇指收到小指侧 | 对掌端（URDF 小指槽的 `thumb_joint1`） |
| `index_together` | 五指并拢（Pico 稳定） | `index_joint1` **0** |
| `index_abd` | 食指向外打开到极限（Pico 追踪好） | `index_joint1` **正极限** |
| `pinch.*`（可选） | 拇指收到该指下方的掌面横向 | 该指在 **URDF** 里的横向槽对应的 `thumb_joint1` |

## 采集

Pico 空手手势、PC Service 已连接。**同一 PC Service 只能有一个 SDK 客户端**，采集时不要跑 `vr-xrt`。

```bash
python -m xr_hand_retarget.calibrate --config xr_hand_retarget/configs/xhand1.yaml --side full
python -m xr_hand_retarget.calibrate --config xr_hand_retarget/configs/xhand1.yaml --side right --preview-only
python -m xr_hand_retarget.calibrate --config xr_hand_retarget/configs/xhand1.yaml --side full --skip-pinch
```

每侧：先 3 个核心姿态，再食指侧角，再问是否做捏合。**Enter 采 1s 中位数**，`s` 跳过当前，`k` 集体跳过一组，`q` 放弃。

1. **伸直**：五指自然张开、不要刻意外展；拇指伸直并与手掌垂直。
2. **握拳**：四指握紧；拇指随意。
3. **对掌**：其余指伸直，拇指收到掌面最靠小指一侧。
4. **食指侧角（可整组跳过；Pico 追踪好时采）**：五指并拢 → 0；食指外开到极限 → 正极限。
5. **捏合（可整组跳过）**：拇指依次收到食 / 中 / 无名 / 小指下方。

然后：`./run.sh vr-xrt wrist`

## 调参（`xrt_curl.yaml` → `retargeting.curl`）

| 键 | 默认 | 作用 |
|----|------|------|
| `calib` | `calib/xrt_default.yaml` | 标定文件 |
| `output_alpha` | `0.85` | 关节 EMA |
| `pinch_target_link` | `pip` | 对齐的手指连杆：`mcp` / `pip` / `tip` |
