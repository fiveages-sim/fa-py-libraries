# fa-py-libraries

用于聚合和快速启动以下 ROS2 相关 Python 子模块：

- `ros2_robot_interface`
- `ros2-viser`
- `vr_pose_publisher`

## 目录结构

- `init.sh`：初始化脚本（子模块、按 backend 创建环境、依赖安装）
- `run.sh`：快速启动脚本（按配置激活环境后启动常用入口）
- `scripts/vr-bag.sh`：VR 遥操 `/xr/*` 话题的 ros2 bag 录制 / 回放 / 清理（由 `run.sh` 调用）
- `release.sh`：发布打包脚本（更新子模块后生成 zip，输出到 `dist/`）
- `.fa-env.toml`：选择 `run.sh` / `install` 使用 **conda** 还是 **uv**
- `scripts/fa-env.sh`：环境配置与激活（供上述脚本共用）

## 环境配置（环境可按 backend 创建）

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

### 1) 初始化（默认按 `.fa-env.toml` 的 backend）

```bash
./init.sh all
```

等价于：

1. 初始化子模块并切到各子模块最新 `main`
2. 按 `backend` 创建环境（默认 Python `3.12`）
3. 激活环境后按顺序安装：
   - `ros2_robot_interface`
   - `ros2-viser`
   - `vr_pose_publisher`

### 2) 启动

```bash
./run.sh
```

进入交互菜单后可选择 viser、VR 遥操、VR 录包/回放、interface 关节录放等（见下文 **run.sh 命令**）。

## run.sh 命令

不带参数时进入交互菜单；也可直接传入子命令：

| 分类 | 命令 | 说明 |
|------|------|------|
| 可视化 | `viser` | 启动 ros2-viser |
| VR 遥操 | `vr` | 启动 vr_pose_publisher（Vuer/WebXR） |
| VR 遥操 | `vr-xrt` | 启动 vr_pose_publisher（XRoboToolkit SDK） |
| VR 录放 | `vr-record [--name 名称]` | 录制 `/xr/*` 到 ros2 bag |
| VR 录放 | `vr-playback [选项]` | 回放 bag（`--file` `--rate` `--count`） |
| VR 录放 | `vr-bag-clean [选项]` | 清理 bag（`--all` `--file`） |
| 关节录放 | `record` | interface 关节快照录制（JSON） |
| 关节录放 | `playback [json]` | interface 关节快照回放 |
| 其他 | `versions` | 查看各子库版本号 |

交互菜单编号：

```
  [可视化]        1) ros2-viser launch
  [VR 遥操]       2) vr pose launch (Vuer/WebXR)
                  3) vr pose launch (XRoboToolkit)
                  4) VR 遥操录包
                  5) VR 遥操回放
                  6) VR bag 清理
  [机器人关节录放] 7) interface 录制
                  8) interface 回放
  [其他]          9) 查看各库版本号
```

### XRoboToolkit 后端（可选）

与默认的 Vuer/WebXR 并列，可用 Pico 官方 **XRoboToolkit App + PC Service** 作为输入，发布同一套 `/xr/*` 话题（无 IK）。

```bash
# 一次性安装 SDK（依赖构建在 vr_pose_publisher/dependencies/）
./init.sh install-xrobotoolkit
# 或在子模块内（需已激活 Python 环境）:
#   cd vr_pose_publisher && bash setup_xrobotoolkit.sh

# 启动（需本机 PC Service 已运行，Pico App 已连接）
./run.sh vr-xrt
```

| 对比 | `./run.sh vr` | `./run.sh vr-xrt` |
|------|---------------|-------------------|
| 输入 | 头显浏览器 WebXR（Vuer） | XRoboToolkit SDK |
| 头显 App | 浏览器 | XRoboToolkit Unity App |
| 发布话题 | `/xr/*` | `/xr/*`（相同契约） |

### VR 遥操录包 / 回放

将 VR 发布的 `/xr/*` 话题（头显/手柄位姿、按键、摇杆、扳机）录制为 ros2 bag，之后可离线回放以模拟 VR 输入（供 `VRInputHandler` 消费）。底层脚本为 `scripts/vr-bag.sh`，推荐通过 `run.sh` 调用。

**前置条件**

- **录制**：另一终端已运行 `./run.sh vr` **或** `./run.sh vr-xrt`，且 VR 设备已连接
- **回放**：**不要**同时运行 `./run.sh vr` / `./run.sh vr-xrt`（避免 `/xr/*` 话题冲突）；确保 `arms_target_manager` / `VRInputHandler` 与机器人控制栈已运行；回放前建议将机器人置于 HOLD，结束后再切回 HOLD

**录制**

```bash
./run.sh vr-record
./run.sh vr-record --name grasp_demo

# 或直接运行底层脚本
./scripts/vr-bag.sh record --name grasp_demo
```

操作流程：输入会话名（可选）→ 按 Enter 开始录制 → 进行 VR 遥操 → 再按 Enter 停止。bag 默认保存到 `xr_bags/`（可用环境变量 `XR_BAG_DIR` 覆盖）。

录制话题：`/xr/head_pose`、`/xr/left_ee_pose`、`/xr/right_ee_pose`、`/xr/controller_state`、`/xr/thumbstick_axes`、`/xr/trigger_values`

**回放**

```bash
# 交互选择 bag，并询问次数 / 速率
./run.sh vr-playback

# 指定 bag、速率与重复次数
./run.sh vr-playback --file xr_bags/grasp_demo_20260707_150930 --rate 1.0 --count 3

# 底层脚本（默认自动启动虚拟 xr_target_node，供 VRInputHandler 检测）
./scripts/vr-bag.sh playback --no-stub   # 若不需要虚拟节点
```

**清理**

```bash
./run.sh vr-bag-clean              # 交互选择要删除的 bag
./run.sh vr-bag-clean --all        # 删除全部（需确认）
```

**典型流程**

```bash
# 终端 1：启动 VR 并遥操（Vuer 或 XRoboToolkit 二选一）
./run.sh vr
# 或
./run.sh vr-xrt

# 终端 2：录包
./run.sh vr-record

# 之后（关闭 VR 节点）：回放
./run.sh vr-playback
```

> **说明**：回放仅重放 VR 输入层。虚拟 `xr_target_node` 只解决「节点存在」检测；手臂是否跟随还取决于 FSM 状态与 UPDATE 模式等控制逻辑，详见 `vr_pose_publisher/README_CN.md` 中的控制流程。

## 常用命令

```bash
# 仅初始化子模块
./init.sh submodules
# 指定源初始化子模块
./init.sh submodules --github
./init.sh submodules --gitea

# 将所有子模块更新到最新 main
./init.sh update-submodules-main

# 按当前 backend 创建环境
./init.sh env 3.12

# 切换 run.sh 使用的 backend
./init.sh set-backend uv

# 安装（按 .fa-env.toml；可临时指定）
./init.sh install
./init.sh install --uv
./init.sh install --conda

# 可选：安装 XRoboToolkit SDK（VR XRT 后端）
./init.sh install-xrobotoolkit

# 配置 ROS2 工作空间（写入 .fa-env.toml + 按 backend 写 activate 挂钩）
./init.sh ros2-workspace
# 若 conda 与 uv 都要挂钩：./init.sh ros2-workspace --all

# 配置 NJU PyPI 镜像（pip.conf）
./init.sh pypi-mirror

# 启动
./run.sh
./run.sh viser
./run.sh vr
./run.sh vr-xrt
./run.sh vr-record
./run.sh vr-playback
./run.sh vr-bag-clean
./run.sh record
./run.sh playback
./run.sh playback /path/to/record.json
./run.sh versions
```

> 说明：交互菜单中的“全部执行”就是顺序执行“初始化子模块 + 创建环境 + 安装”。

## 发布打包

使用 `release.sh` 将仓库内容打成 zip，便于分发或离线部署：

```bash
# 默认：先更新三个子模块到 origin/main 最新提交，再打包
./release.sh

# 指定输出路径
./release.sh -o /path/to/fa-py-libraries.zip

# 不拉取远程，按当前检出直接打包（离线场景）
./release.sh --skip-submodules
```

- 默认输出：`dist/fa-py-libraries-<时间戳>.zip`（`dist/` 已加入 `.gitignore`）
- 打包时会排除：`.idea/`、`.venv/`、`venv/`、`dist/`
- 需要能访问子模块远程时，请勿加 `--skip-submodules`

## 说明

- `run.sh` 根据 `.fa-env.toml` 的 `backend` 激活 conda 或 `.venv`；若配置了 `[ros2].workspace` 会 source 对应 `install/setup.bash`。
- **uv 与 ROS2**：`rclpy`、`*-msgs` 不在 PyPI；安装时用 `--no-deps` + 单独装 PyPI 依赖。当 `backend=uv` 时，`./init.sh env` 会优先用系统解释器（如 `/usr/bin/python3.12`，跳过 conda 路径）并加 `--system-site-packages`，与 [uv+ROS2 常见做法](https://www.cnblogs.com/yzcat/p/19960512) 一致。执行 `./init.sh ros2-workspace` 后，手动 `source .venv/bin/activate` 也会自动 source ROS2 工作空间（找不到则回退 `/opt/ros/jazzy`）。
- VR 遥操需要额外的 SSL 证书，安装时会自动生成，详见子模块 README。
