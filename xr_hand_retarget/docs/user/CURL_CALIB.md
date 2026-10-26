# XHand1 几何 / 暖启动说明

XHand1 **生产路径是 `vector`**（键向量优化，对齐 teleop_ws AnyDex）。  
下文中的 curl 标定端点仍可用于几何暖启动与调试工具，但 `local.yaml` 写 `method: curl` 会被忽略。

| 文件 | 作用 |
|------|------|
| `xr_hand_retarget/configs/xhand1.yaml` | 索引；固定 `retargeting.type: vector` |
| `xr_hand_retarget/configs/hands/xhand1.yaml` | T_robot / vector 约束 |
| `xr_hand_retarget/configs/calib/xrt_default.yaml` | 可选几何标定（暖启动） |

```bash
FA_HAND=xhand1 ./run.sh vr-xrt wrist
python -m xr_hand_retarget.calibrate --config xr_hand_retarget/configs/xhand1.yaml --side full
```

换手与其它手型见 [SWITCH.md](SWITCH.md)、[../algorithms/METHODS.md](../algorithms/METHODS.md)。
