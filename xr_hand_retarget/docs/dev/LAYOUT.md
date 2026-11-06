# 代码布局

手映射唯一家在 `xr_hand_retarget`。不新开 `fa-hand`；VR 产品仍在本仓 `vr_pose_publisher`。

```
xr_hand_retarget/
  algorithms/     # vector / palm_tip(curl) / remap / collision / …
  backends/       # xhand1, linker, wuji
  control/        # FSM gate + LPF + vmax（对齐 teleop_ws）
  sources/        # OpenXR26 → MediaPipe 21
  node.py / dual_node.py / calibrate.py
  configs/
vr_pose_publisher/    # 产品：import xr_hand_retarget.dual_node
third_party/wuji-retargeting/   # 官方源，可选
```

安装：`uv pip install -e xr_hand_retarget`（或 `./init.sh install`）。旧包 `xr_xhand1_retarget` 已删除；旧 CLI 名 `xr-xhand1-retarget` / `xr-xhand1-calibrate` 仍指向本包入口。

已落地：**P0** HandFrame；**P1** 配置拆开；**P2** T_xhand（URDF 限位/捏合槽，`txhand` CLI）。

下一里程碑：**P3** 多源 `HandSource`。验证命令见 [RUN.md](RUN.md)。
