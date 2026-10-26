# 算法（按手型各一条）

对齐 `teleop_ws`：每种手只保留一条生产路径。`local.yaml` 的 `hands.method` 可省略；若与下表不符会被忽略。

| 手型 | method | 实现 | teleop_ws 对照 |
|------|--------|------|----------------|
| **xhand1** | `vector` | `algorithms/vector_retarget.py`（somehand / 键向量） | DexCap → AnyDex `KeyVectorOptimizer` |
| **o6 / l6 / o7** | `curl` | `algorithms/palm_tip.py`（PalmTip 标定插值） | 手套 → `GloveYamlRemapRetarget` |
| **wuji** | `official` | `backends/wuji.py` + wuji-retargeting | `WujiHand2SdkRetarget` |

运行时目录：`xr_hand_retarget.algorithms.HAND_METHOD` / `method_for_backend()`。

```yaml
# configs/local.yaml
hands:
  type: xhand1   # 或 o6 | l6 | o7 | wuji
  # method 可不写
```

已删除源文件：`nest.py`、`mp_curl_o6.py`、`thumb_ik.py`、旧包 `xr_xhand1_retarget`。别名（`nest`/`mp_curl`/`dexpilot`…）仍映射到上表 method，但不会加载退役实现。

Wuji official 安装见 [OFFICIAL.md](OFFICIAL.md)。Linker 标定见 [../user/LINKER_CALIB.md](../user/LINKER_CALIB.md)。
