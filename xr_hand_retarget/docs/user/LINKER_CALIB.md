# LinkerHand O6 / L6 / O7 标定

停掉 `./run.sh vr-xrt`（PC Service 同时只允许一个客户端）。Pico 光学已跟上后：

```bash
python -m xr_hand_retarget.calibrate --config configs/o6.yaml
# 或 configs/l6.yaml / configs/o7.yaml（同一次会话，写入全部导出文件）
```

（`configs/o6.yaml` 会解析到包内 `xr_hand_retarget/configs/o6.yaml`。）

## 一次会话 → 导出

| 输出 | 用途 |
|------|------|
| `configs/calib/linker_session.yaml` | 主会话（PalmTip takes） |
| `configs/calib/o6.yaml` | PalmTip curl，O6 |
| `configs/calib/l6.yaml` | PalmTip curl，L6 |
| `configs/calib/o7.yaml` | PalmTip curl，O7 |

人手特征与具体 Linker 型号无关；机器人 `pinch_q` / `j1_roof` 仍读各手的 `hands/{o6,l6,o7}.yaml`。

仅重导出（不戴头显）：

```bash
python -m xr_hand_retarget.calibrate --config configs/o6.yaml --export-only
```

写回后重启 `./run.sh vr-xrt wrist`。

### 标定姿态

| 姿态 | 用途 |
|------|------|
| `open` | 开掌 |
| `together` | 合掌 |
| `fist` | 握拳 |
| `good` | 点赞 |
| `thumb_along` | 拇指侧掌 |
| `pinch_×4` | 四指捏合 |

每个姿态可采多组（`a` 再采）。

**结点分离**：j2 优先用 **`nest_lat` / `slot_t`（指尖相对四指 PIP 巢）**，`thumb_az` 仅 fallback / O7 j3。采 `thumb_along` 与 `pinch_index` 时务必拉开：

| 检查 | 建议最小差 |
|------|------------|
| `slot_t`（along vs index pinch） | \|Δ\| ≥ 0.30 |
| `nest_r` | \|Δ\| ≥ 0.12 |
| `nest_lat` | 明显分离 |
| `thumb_az` | 仅作参考 |

### 运行

```bash
FA_HAND=o6 ./run.sh vr-xrt wrist
FA_HAND=l6 ./run.sh vr-xrt wrist
FA_HAND=o7 ./run.sh vr-xrt wrist
```

运行时固定 `curl`（PalmTip）。`mp_curl` / `nest` / `veccurl` 已删除。

**O7 防穿模**：`hands/o7.yaml` → `curl.pinch` 启用后，捏合走廊内关节不超过 RViz `pinch_q`（天花板），并可选 FK 垫球投影。

采集时默认弹出 OpenCV 视窗；`--no-preview-mp` 关闭。配置分层见 [../dev/CONFIG.md](../dev/CONFIG.md)。
