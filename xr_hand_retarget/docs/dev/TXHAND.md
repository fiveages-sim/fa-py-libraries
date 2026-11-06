# T_xhand（XHand1 设备一次标定）

设备侧映射（标准手 → XHand1），**不是**操作员文件。操作员 curl 端点在 `calib/xrt_default.yaml`（T_person）。

## 原则

1. **先 T_person，再 T_xhand**：参考员采完 open/fist/对掌（及可选 pinch）后再调设备尺度。
2. **开合限位、捏合槽优先 URDF**：`assets/xhand_{left,right}.urdf`；运行时 `robot.use_urdf_limits: true`（默认）。
3. **小指过长等**：写 `hands/xhand1.yaml` → `robot.finger_scale`，不要写进 calib。
4. **活体探针**（URDF 不够时）：Wuji 手套 ＞ DexCap ＞ Pico；Pico 只验收。
5. XHand 路径：四指曲率 + 拇指笛卡尔/指槽，**不要**单走 q 抄录。

## 命令

```bash
# 打印 URDF 限位 + 各指捏合 thumb_joint1 槽
python -m xr_hand_retarget.txhand --side right
python -m xr_hand_retarget.txhand --side both --json
python -m xr_hand_retarget.txhand --side right \
  --compare-config xr_hand_retarget/configs/hands/xhand1.yaml

# T_person（停 vr-xrt）
python -m xr_hand_retarget.calibrate --config xr_hand_retarget/configs/xhand1.yaml --side full
```

## YAML

```yaml
# configs/hands/xhand1.yaml
robot:
  use_urdf_limits: true
  pinch_target_link: pip
  # finger_scale: { pinky: 0.92 }

retargeting:
  curl:
    calib: calib/xrt_default.yaml   # T_person only
```

配置分层见 [CONFIG.md](CONFIG.md)。用户 curl 姿态见 [../user/CURL_CALIB.md](../user/CURL_CALIB.md)。
