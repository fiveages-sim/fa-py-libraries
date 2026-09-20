# LinkerHand O6 / L6 / O7 标定

停掉 `./run.sh vr-xrt`（PC Service 同时只允许一个客户端）。Pico 光学已跟上后：

```bash
python -m xr_hand_retarget.calibrate --config configs/o6.yaml
# 或 configs/l6.yaml / configs/o7.yaml（同一次会话，写入全部导出文件）
```

（`configs/o6.yaml` 会解析到包内 `xr_hand_retarget/configs/o6.yaml`。）

## 一次会话 → 多路径导出

| 输出 | 用途 |
|------|------|
| `configs/calib/linker_session.yaml` | 主会话（PalmTip + MP21 原始 takes） |
| `configs/calib/o6.yaml` | `method: curl`，O6 |
| `configs/calib/l6.yaml` | `method: curl`，L6 |
| `configs/calib/o7.yaml` | `method: curl`，O7 |
| `configs/calib/mp_curl.yaml` | `method: mp_curl`，O6/L6/O7 共用 |

人手特征与具体 Linker 型号无关；机器人 `pinch_q` / `j1_roof` 仍读各手的 `hands/{o6,l6,o7}.yaml`。

仅重导出（不戴头显）：

```bash
python -m xr_hand_retarget.calibrate --config configs/o6.yaml --export-only
```

写回后重启 `./run.sh vr-xrt wrist`。

### 标定姿态（统一采集）

| 姿态 | PalmTip curl | mp_curl |
|------|--------------|---------|
| `open` | ✓ | ✓ |
| `together`（合掌） | ✓ | — |
| `fist` | ✓ | ✓ |
| `good`（点赞） | ✓ | — |
| `thumb_along` | ✓ | ✓ |
| `pinch_×4` | ✓ | ✓ |

每个姿态可采多组（`a` 再采），会话 yaml 保留各组 `palm` + `mp` + `mp21`。

**结点分离（PalmTip curl 必读）**：j2 优先用 **`nest_lat` / `slot_t`（指尖相对四指 PIP 巢）**，`thumb_az` 仅 fallback / O7 j3。采 `thumb_along` 与 `pinch_index` 时务必拉开：

| 检查 | 建议最小差 |
|------|------------|
| `slot_t`（along vs index pinch） | \|Δ\| ≥ 0.30 |
| `nest_r` | \|Δ\| ≥ 0.12 |
| `nest_lat` | 明显分离（勿与 along 几乎相同） |
| `thumb_az` | 仅作参考；差 0.02 rad 仍可能够用若 slot_t/nest 已分离 |

### 运行

```bash
# PalmTip curl（默认）
FA_HAND=o6 ./run.sh vr-xrt wrist
FA_HAND=l6 ./run.sh vr-xrt wrist
FA_HAND=o7 ./run.sh vr-xrt wrist

# mp_curl（VR26→MP21→curl）
FA_HAND=o6 FA_HAND_RETARGETING=mp_curl ./run.sh vr-xrt wrist
FA_HAND=o7 FA_HAND_RETARGETING=mp_curl ./run.sh vr-xrt wrist
```

**O7 mp_curl 分工**（与 `method: curl` 不同处）：

| 关节 | 原理 | 标定 |
|------|------|------|
| 四指 index/middle/ring/pinky | MP21 curl + lat 门控 + `pinch_q`，与 O6/L6 **完全相同** | `configs/calib/mp_curl.yaml` |
| 拇指 j1/j2/j3 | PalmTip O7 curl 链 + 拇指 j1 捏合吸引 | `configs/calib/o7.yaml`（`linker.user`） |

**O7 防穿模（curl / mp_curl 共用）**：`hands/o7.yaml` → `curl.pinch` 启用后，捏合走廊内关节不超过 RViz `pinch_q`（天花板），并可选 FK 垫球投影（规划层，早于 safety）。`pinch_q` 同时仍是 attract 的接触地板。

无 `calib/o7.yaml` 时 O7 拇指退回 MP21 legacy；四指仍正常。可选 `curl.mp_o7_thumb: legacy` 强制 legacy A/B。

合成自检：`python -m xr_hand_retarget.algorithms.mp_curl_o6`（四指 parity + j2 slot_t 扫过步长 < 0.05）。

采集时默认弹出 **MediaPipe-21** OpenCV 视窗（PalmTip + MP 特征 HUD）。`--no-preview-mp` 关闭。

O6 可选 `method: nest`（不必为 nest 重采）见 [../algorithms/O6_NEST.md](../algorithms/O6_NEST.md)。配置分层见 [../dev/CONFIG.md](../dev/CONFIG.md).
