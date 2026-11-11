# fa-py-libraries

聚合并快速启动以下 ROS2 相关 Python 包（均为 git 子模块，由 `./init.sh submodules` 拉取）：

- `ros2_robot_interface` — 机器人接口 / 关节录放
- `ros2-viser` — 可视化
- `vr_pose_publisher` — VR 遥操位姿发布（Vuer / XRoboToolkit）
- `xr_hand_retarget` — 灵巧手映射（XHand1 / Wuji / Linker O6·L6·O7）

## 目录结构

- `init.sh`：初始化（子模块、按 backend 建环境、依赖安装）
- `run.sh`：快速启动（按配置激活环境后启动常用入口）
- `configs/local.yaml`：本机换手与遥操白名单参数（从 `local.yaml.example` 复制；已 gitignore）
- `configs/local_xr.sh`：`run.sh` 读取的本地 XR 路径 / 手型映射
- `scripts/vr-bag.sh`：VR `/teleop/*` ros2 bag 录制 / 回放 / 清理
- `release.sh`：发布打包（更新子模块后生成 zip → `dist/`）
- `.fa-env.toml`：选择 `run.sh` / `install` 使用 **conda** 还是 **uv**
- `scripts/fa-env.sh`：环境配置与激活（供上述脚本共用）

## 环境配置

编辑仓库根目录 `.fa-env.toml`：

```toml
backend = "conda"   # conda | uv

[conda]
name = "fa-ros2"

[uv]
venv = ".venv"

[ros2]
workspace = "~/ros2_ws"   # 配置后 run.sh 激活时会 source
```

| 方式 | 说明 |
|------|------|
| `./init.sh set-backend uv` | 写入 `backend`，之后 `run.sh` 走 uv |
| `.fa-env.local.toml` | 个人覆盖（已 gitignore），优先级高于 `.fa-env.toml` |
| `FA_ENV_BACKEND=uv ./run.sh viser` | 单次临时覆盖 |

`init.sh env` 会按 `backend` 创建环境；切换 backend 后可再次执行，不会主动删除已存在环境。

## 快速开始

### 1) 初始化

```bash
./init.sh all
```

等价于：

1. 初始化子模块并切到各子模块最新 `main`
2. 按 `backend` 创建环境（默认 Python `3.12`）
3. 激活环境后按顺序安装：`ros2_robot_interface` → `ros2-viser` → `vr_pose_publisher` → `xr_hand_retarget`

灵巧手额外步骤（按需）：

```bash
cp configs/local.yaml.example configs/local.yaml   # 改 hands.type
# Wuji official（装完须能 import；见 xr_hand_retarget/docs/algorithms/OFFICIAL.md）：
./init.sh install-wuji-retargeting
python -c 'import wuji_retargeting; print(wuji_retargeting.__file__)'
# XRoboToolkit（Pico App + PC Service）：
./init.sh install-xrobotoolkit-pc-service
./init.sh install-xrobotoolkit
```

### 2) 启动

```bash
./run.sh
```

进入交互菜单；也可直接传子命令（见下）。光学腕 + 灵巧手：

```bash
./run.sh vr-xrt-service
./run.sh vr-xrt wrist          # 交互选手型，或事先设好 local.yaml
# 临时换手 / 仅光学腕（无灵巧手）：
FA_HAND=o6 ./run.sh vr-xrt wrist
FA_HAND=none ./run.sh vr-xrt wrist
# 等价：./run.sh vr-xrt wrist --no-hands
```

## 本地 XR / 换手配置

**本机只改** `configs/local.yaml`（已 gitignore；无则从 `local.yaml.example` 复制）。优先级：`FA_HAND` / `FA_HAND_CONFIG` 环境变量 > `local.yaml`。

```yaml
hands:
  source: xrt
  type: xhand1         # xhand1 | wuji | o6 | l6 | o7

teleop:
  publish_hz: 120
  retarget_hz: 80

command:
  max_velocity: 6.0
  filter_cutoff_hz: 10.0
  allowed_fsm_states: [3, 4]
```

| `type` | 算法 | 说明 |
|--------|------|------|
| `xhand1` | `vector` | AnyDex KeyVector |
| `o6` / `l6` / `o7` | `curl` | Linker 手套 curl |
| `wuji` | `official` | 需先 `./init.sh install-wuji-retargeting`，并确认 `import wuji_retargeting` |
| `none`（仅 env / `--no-hands`） | — | 只发光学腕 EE，不启灵巧手；不写入 `local.yaml` |

完整白名单与顺畅度旋钮见 [xr_hand_retarget/docs/user/SWITCH.md](xr_hand_retarget/docs/user/SWITCH.md)。

## run.sh 命令

不带参数时进入交互菜单；也可直接传入子命令：

| 分类 | 命令 | 说明 |
|------|------|------|
| 可视化 | `viser` | 启动 ros2-viser |
| VR 遥操 | `vr` | vr_pose_publisher（Vuer/WebXR） |
| VR 遥操 | `vr-hand` | 交互选手型后启动手部遥操（光学腕 + 灵巧手） |
| VR 遥操 | `vr-xrt [controller\|wrist]` | XRoboToolkit：`controller`=手柄+夹爪；`wrist`=光学腕+灵巧手（可 `--no-hands`） |
| VR 遥操 | `vr-xrt-service [start\|stop]` | 启动 / 关闭 XRoboToolkit PC Service |
| VR 辅助 | `vr-update` | 临时发右摇杆 UPDATE（`controller_state=4`；须已 OCS2） |
| VR 辅助 | `vr-ocs2` | 临时发 OCS2（`/fsm_command=3`，无脚踏板时用） |
| VR 录放 | `vr-record [--name 名称]` | 录制 `/teleop/*` 到 ros2 bag |
| VR 录放 | `vr-playback [选项]` | 回放 bag（`--file` `--rate` `--count`） |
| VR 录放 | `vr-bag-clean [选项]` | 清理 bag（`--all` `--file`） |
| 关节录放 | `record` | interface 关节快照录制（JSON） |
| 关节录放 | `playback [json]` | interface 关节快照回放 |
| 其他 | `versions` | 查看各子库版本号 |

交互菜单编号：

```
  [可视化]           1) ros2-viser launch
  [VR遥操 XRoboToolkit] 2) 手部遥操 (光学腕 + 灵巧手)  — 再选手型并写入 local.yaml
                     3) 手柄遥操 (controller)
                     4) 启动 XRoboToolkit PC Service
                     5) 关闭 XRoboToolkit PC Service
  [VR遥操 Web]       6) vr pose launch (Vuer/WebXR)
  [VR遥操 通用]      7) VR 遥操录包
                     8) VR 遥操回放
                     9) VR bag 清理
  [机器人关节录放]   10) interface 录制
                     11) interface 回放
  [其他]             12) 查看各库版本号
```

同一 PC Service 只能有一个 SDK 客户端；跑 `vr-xrt` 时不要另开独立的 `xr_hand_retarget` 进程。

### XRoboToolkit 后端（可选）

与默认的 Vuer/WebXR 并列，可用 Pico **XRoboToolkit App + PC Service** 作为输入，发布同一套 `/teleop/*` 话题。官方组件说明见 [XR-Robotics](https://github.com/XR-Robotics)。

```bash
./init.sh install-xrobotoolkit-pc-service
./init.sh install-xrobotoolkit

./run.sh vr-xrt-service
./run.sh vr-xrt controller   # 手柄 6DoF 控臂 + 扳机夹爪
./run.sh vr-xrt wrist        # 光学腕 6DoF 控臂 + 灵巧手（空手手势）

./run.sh vr-xrt-service stop
```

| 对比 | `./run.sh vr` | `./run.sh vr-xrt controller` | `./run.sh vr-xrt wrist` |
|------|---------------|------------------------------|-------------------------|
| 输入 | 头显浏览器 WebXR（Vuer） | XRT 手柄 | XRT 光学腕 + 手势 |
| 头显 App | 浏览器 | XRoboToolkit Unity App | 同左 |
| 末端 | 位姿 + 夹爪（扳机） | 位姿 + 夹爪 | 位姿 + 灵巧手 `q_des` |
| 发布话题 | `/teleop/*` | `/teleop/*`（相同契约） | 同左 + 手命令 |

### VR 遥操录包 / 回放

将 VR 发布的 `/teleop/*` 话题录制为 ros2 bag，之后可离线回放以模拟 VR 输入。底层脚本为 `scripts/vr-bag.sh`，推荐通过 `run.sh` 调用。

**前置条件**

- **录制**：另一终端已运行 `./run.sh vr` 或 `./run.sh vr-xrt …`，且 VR 设备已连接
- **回放**：**不要**同时运行 VR 发布节点（避免 `/teleop/*` 冲突）；控制栈与 `VRInputHandler` 需已运行；回放前建议机器人置于 HOLD

**录制**

```bash
./run.sh vr-record
./run.sh vr-record --name grasp_demo
```

操作：输入会话名（可选）→ Enter 开始 → 遥操 → 再 Enter 停止。bag 默认在 `xr_bags/`（可用 `XR_BAG_DIR` 覆盖）。

录制话题：`/teleop/head_pose`、`/teleop/left_ee_pose`、`/teleop/right_ee_pose`、`/teleop/controller_state`、`/teleop/thumbstick_axes`、`/teleop/trigger_values`

**回放 / 清理**

```bash
./run.sh vr-playback
./run.sh vr-playback --file xr_bags/grasp_demo_20260707_150930 --rate 1.0 --count 3
./scripts/vr-bag.sh playback --no-stub   # 不启虚拟 xr_target_node

./run.sh vr-bag-clean              # 交互选择
./run.sh vr-bag-clean --all        # 删全部（需确认）
```

> 回放仅重放 VR 输入层。手臂是否跟随还取决于 FSM / UPDATE 等控制逻辑，详见 `vr_pose_publisher/README_CN.md`。

## 常用命令

```bash
# 子模块
./init.sh submodules
./init.sh submodules --github
./init.sh submodules --gitea
./init.sh update-submodules-main

# 环境 / 安装
./init.sh env 3.12
./init.sh set-backend uv
./init.sh install
./init.sh install --uv
./init.sh install --conda
./init.sh install-wuji-retargeting
./init.sh install-xrobotoolkit-pc-service
./init.sh install-xrobotoolkit
./init.sh ros2-workspace          # --all 则 conda+uv 都挂钩
./init.sh pypi-mirror
./init.sh uv-mirror

# 启动
./run.sh
./run.sh viser
./run.sh vr
./run.sh vr-xrt-service
./run.sh vr-xrt controller
./run.sh vr-xrt wrist
FA_HAND=o6 ./run.sh vr-xrt wrist
FA_HAND=none ./run.sh vr-xrt wrist
./run.sh vr-xrt wrist --no-hands
./run.sh vr-update
./run.sh vr-ocs2
./run.sh vr-record
./run.sh vr-playback
./run.sh vr-bag-clean
./run.sh record
./run.sh playback /path/to/record.json
./run.sh versions
```

## 发布打包

使用 `release.sh` 将精简源码打成 zip（临时目录 staging，不改动工作区）。默认**不**打入 `dependencies/`、本地环境与录包数据；现场再按需安装。

```bash
./release.sh
./release.sh --package-no-git
./release.sh --package
./release.sh -o /path/to/fa-py-libraries.zip
./release.sh --package-no-git --skip-submodules   # 离线：按当前检出打包
```

- 默认输出：`dist/fa-py-libraries-<时间戳>[_nogit].zip`（`dist/` 已 gitignore）
- **始终排除**：`.venv/`、`venv/`、`dist/`、`**/dependencies/`、`xr_bags/`、`joint_records/`、`*.pem`、`__pycache__/`、`*.egg-info/`、`.fa-env.local.toml`
- **`--package-no-git` 额外排除**：`.git/`
- **包含**：根脚本、`configs/`、`.fa-env.toml`、`README.md`、`.gitmodules`，以及 `ros2_robot_interface` / `ros2-viser` / `vr_pose_publisher` / `xr_hand_retarget`（不含其 `dependencies/`）

现场解压后：

```bash
./init.sh install
cp configs/local.yaml.example configs/local.yaml
# 可选：./init.sh install-xrobotoolkit-pc-service && ./init.sh install-xrobotoolkit
# Wuji：./init.sh install-wuji-retargeting
```

## 说明

- `run.sh` 根据 `.fa-env.toml` 的 `backend` 激活 conda 或 `.venv`；若配置了 `[ros2].workspace` 会 source 对应 `install/setup.bash`。
- **uv 与 ROS2**：`rclpy`、`*-msgs` 不在 PyPI；安装时用 `--no-deps` + 单独装 PyPI 依赖。当 `backend=uv` 时，`./init.sh env` 会优先用系统解释器并加 `--system-site-packages`，与 [uv+ROS2 常见做法](https://www.cnblogs.com/yzcat/p/19960512) 一致。执行 `./init.sh ros2-workspace` 后，手动 `source .venv/bin/activate` 也会自动 source ROS2 工作空间（找不到则回退 `/opt/ros/jazzy`）。
- VR（Vuer）需要 SSL 证书，安装时会自动生成，详见 `vr_pose_publisher` README。

## 文档入口

| 文档 | 内容 |
|------|------|
| [xr_hand_retarget/docs/user/](xr_hand_retarget/docs/user/) | 换手 · 安装 · 标定（用户） |
| [SWITCH.md](xr_hand_retarget/docs/user/SWITCH.md) | 用户层参数与执行 |
| [SMOOTHNESS.md](xr_hand_retarget/docs/user/SMOOTHNESS.md) | 顺畅度 fc / vmax |
| [vr_pose_publisher/README_CN.md](vr_pose_publisher/README_CN.md) | VR 发布协议与控制流程 |
| [xr_hand_retarget/docs/dev/](xr_hand_retarget/docs/dev/) | 配置分层 / 发布安全（开发） |
