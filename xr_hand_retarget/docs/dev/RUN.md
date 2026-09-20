# 开发验证命令

## 本机选手（O6 / L6 / O7 / XHand1）

改仓库根 `configs/local.yaml` 的 `hands.type` / `hands.method`，再：

```bash
./run.sh vr-xrt wrist
# 或临时：
FA_HAND=o6 ./run.sh vr-xrt wrist
FA_HAND=l6 ./run.sh vr-xrt wrist
FA_HAND=o7 ./run.sh vr-xrt wrist
FA_HAND=xhand1 ./run.sh vr-xrt wrist
```

## Wuji：独立节点（不经 local 也可）

PC Service 已连；**不要**同时跑 `vr-xrt`（同一 SDK 客户端）。

```bash
python -m xr_hand_retarget --config xr_hand_retarget/configs/wuji_hand2.yaml --side right
python -m xr_hand_retarget --config xr_hand_retarget/configs/wuji_hand2.yaml --side left
python -m xr_hand_retarget --config xr_hand_retarget/configs/wuji_hand2.yaml --side full
```

覆盖算法：`--retargeting curl` / `--retargeting official`。只打印 q：`--dry-run`。

Wuji 节点验证：

```bash
python -m xr_hand_retarget --config xr_hand_retarget/configs/wuji_hand2.yaml --side right
```

XHand1 设备标定（T_xhand，非操作员）：

```bash
python -m xr_hand_retarget.txhand --side right
```

配置分层见 [CONFIG.md](CONFIG.md)。T_xhand 见 [TXHAND.md](TXHAND.md)。
