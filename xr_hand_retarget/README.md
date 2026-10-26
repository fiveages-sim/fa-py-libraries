# xr_hand_retarget

一份 YAML 切换灵巧手。OpenXR 26 → 后端求解器 → 机器人 `q`。本仓唯一手映射包。

## 换手 / 跑

```bash
# 仓库根 configs/local.yaml → hands.type: o6 | l6 | o7 | wuji | xhand1
FA_HAND=o6 ./run.sh vr-xrt wrist
```

标定与单手节点：[docs/user/SWITCH.md](docs/user/SWITCH.md)。开发验证（含 `python -m … --config …`）：[docs/dev/RUN.md](docs/dev/RUN.md)。

## 文档

| 受众 | 入口 |
|------|------|
| **用户** | [docs/user/](docs/user/) — 换手、标定 |
| **开发** | [docs/dev/](docs/dev/) — 配置分层、布局、迁移 |
| **算法 / 官方** | [docs/algorithms/](docs/algorithms/) — method 表、nest、wuji-retargeting |

## 安装

```bash
uv pip install -e xr_hand_retarget
# 或 ./init.sh install

# 仅 official Wuji 需要：
./init.sh install-wuji-retargeting
```

兼容旧命令：`xr-xhand1-retarget` / `xr-xhand1-calibrate`（等同 `xr-hand-retarget` / `xr-hand-calibrate`）。若本机曾装过旧包，先 `pip uninstall xr_xhand1_retarget`。

官方包可能把 NumPy 升到 2.x；钉回：`uv pip install 'numpy>=1.26,<2'`。说明见 [docs/algorithms/OFFICIAL.md](docs/algorithms/OFFICIAL.md)。

## 后端一览

| `type` | method（常用） | 标定 |
|--------|----------------|------|
| `xhand1` | `dexpilot` / `vector` / `curl` / `thumb_ik` | [user/CURL_CALIB](docs/user/CURL_CALIB.md) |
| `wuji` | `official` / `curl` | [user/WUJI_CALIB](docs/user/WUJI_CALIB.md) |
| `o6` / `l6` / `o7` | `curl` / `nest`（o6/l6）/ `veccurl`（o7）/ **`mp_curl`** | [user/LINKER_CALIB](docs/user/LINKER_CALIB.md) |

完整 method 表：[docs/algorithms/METHODS.md](docs/algorithms/METHODS.md)。配置分层：[docs/dev/CONFIG.md](docs/dev/CONFIG.md)。
