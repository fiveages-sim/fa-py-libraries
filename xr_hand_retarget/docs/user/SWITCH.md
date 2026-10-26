# 换手与启动

改仓库根 `configs/local.yaml`：

```yaml
hands:
  type: o6      # o6 | l6 | o7 | wuji | xhand1
  method: curl  # 见 docs/algorithms/ — 覆盖包装 yaml 的 retargeting.type
```

临时覆盖：

```bash
FA_HAND=xhand1 ./run.sh vr-xrt wrist
FA_HAND=wuji ./run.sh vr-xrt wrist
FA_HAND=o6 ./run.sh vr-xrt wrist
FA_HAND=l6 ./run.sh vr-xrt wrist
FA_HAND=o7 ./run.sh vr-xrt wrist
```

光学腕 6DoF 控臂与手型号无关，一直走 `/teleop/*_ee_pose`。

**同一 PC Service 只能有一个 SDK 客户端**：标定前停掉 `vr-xrt`。

## 标定入口

```bash
python -m xr_hand_retarget.calibrate --config xr_hand_retarget/configs/xhand1.yaml --side full
python -m xr_hand_retarget.calibrate --config xr_hand_retarget/configs/wuji_hand2.yaml
python -m xr_hand_retarget.calibrate --config xr_hand_retarget/configs/o6.yaml --side full
python -m xr_hand_retarget.calibrate --config xr_hand_retarget/configs/l6.yaml --side full
python -m xr_hand_retarget.calibrate --config xr_hand_retarget/configs/o7.yaml --side full
```

## 单手节点

先起 fa_w2 / Linker mock，再：

```bash
python -m xr_hand_retarget --side left
```

`--side left` 优先 `left_hand_controller`，否则 `hand_joint_controller`。双手用 `--side full`。只打印 q：`--dry-run`。
