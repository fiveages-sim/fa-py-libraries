# xr_hand_retarget

一份 YAML 切换灵巧手。OpenXR 26 → 后端求解器 → 机器人 `q`。本仓**唯一**手映射包。

## 安装 → 参数 → 跑

```bash
./init.sh install
cp configs/local.yaml.example configs/local.yaml   # 改 hands.type
./run.sh vr-xrt wrist                               # 或 FA_HAND=o6 ./run.sh vr-xrt wrist
```

完整步骤与 teleop_ws 键名对照：[docs/user/SWITCH.md](docs/user/SWITCH.md)。

## 文档

| 受众 | 入口 |
|------|------|
| **用户** | [docs/user/](docs/user/) — 安装 / 换手 / 标定 |
| **开发** | [docs/dev/](docs/dev/) — 配置分层、发布安全、迁移 |
| **算法** | [docs/algorithms/](docs/algorithms/) — 按手型一条 method |

## 后端一览

| `type` | method | 标定 |
|--------|--------|------|
| `xhand1` | `vector` | [user/CURL_CALIB](docs/user/CURL_CALIB.md)（暖启动/几何） |
| `wuji` | `official` | [user/WUJI_CALIB](docs/user/WUJI_CALIB.md) |
| `o6` / `l6` / `o7` | `curl` | [user/LINKER_CALIB](docs/user/LINKER_CALIB.md) |

CLI：`xr-hand-retarget` / `xr-hand-calibrate`（旧名 `xr-xhand1-*` 仍指向本包）。  
Wuji 官方包：`./init.sh install-wuji-retargeting`；NumPy 2.x 钉回见 [docs/algorithms/OFFICIAL.md](docs/algorithms/OFFICIAL.md)。
