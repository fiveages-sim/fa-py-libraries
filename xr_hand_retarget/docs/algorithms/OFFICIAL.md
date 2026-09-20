# 官方源：wuji-retargeting

`retargeting.type: official` 才需要。curl 默认路径不装。

```bash
./init.sh install-wuji-retargeting
```

会 clone 到 `third_party/wuji-retargeting`（含 `wuji-description` 子模块），editable 安装后再钉死 `numpy<2`（ROS Jazzy Pinocchio 与 NumPy 2.x 不兼容）。需要 `pin` 与 `nlopt`。

子模块空时：

```bash
git -C third_party/wuji-retargeting submodule update --init --recursive
```

镜像装不上 `pin`：

```bash
pip install pin==3.8.0 -i https://pypi.org/simple
```

覆盖路径：`WUJI_RETARGETING_ROOT=/path/to/wuji-retargeting`。

## beta1 与 beta2

| | **beta1** | **beta2** |
|--|--|--|
| 是什么 | 坐标系冻结的第一版 | 同一套关节 + 指尖触觉垫 |
| 官方 IK | yaml 指向这里 | 官方求解器还没用 |
| fa_w2 mock | 不直接用 | 从 beta2 改成 ROS xacro |

**不要改 fa_w2 去迁就求解器。** Hand 2 的 `optimizer.urdf_path` / `link_naming` 以官方 yaml 为准。

调参顺序见 [Parameter Tuning](https://docs.wuji.tech/docs/en/wuji-retargeting/latest/tuning/)。现场光学调试步骤见 [../user/WUJI_CALIB.md](../user/WUJI_CALIB.md)。
