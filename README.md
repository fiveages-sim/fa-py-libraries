# fa-py-libraries

用于聚合和快速启动以下 ROS2 相关 Python 子模块：

- `ros2_robot_interface`
- `ros2-viser`
- `vr_pose_publisher`

## 目录结构

- `init.sh`：初始化脚本（子模块、按 backend 创建环境、依赖安装）
- `run.sh`：快速启动脚本（按配置激活环境后启动常用入口）
- `scripts/vr-bag.sh`：VR 遥操 `/teleop/*` 话题的 ros2 bag 录制 / 回放 / 清理（由 `run.sh` 调用）
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
| VR 遥操 | `vr-xrt` | 启动 vr_pose_publisher（XRoboToolkit SDK）；启动时会询问是否启用图像视频传输并选择话题 |
| VR 遥操 | `vr-xrt-service [start\|stop]` | 启动 / 关闭 XRoboToolkit PC Service（`runService.sh`） |
| VR 录放 | `vr-record [--name 名称]` | 录制 `/teleop/*` 到 ros2 bag |
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
                  4) 启动 XRoboToolkit PC Service
                  5) 关闭 XRoboToolkit PC Service
                  6) VR 遥操录包
                  7) VR 遥操回放
                  8) VR bag 清理
                  9) ROS 图像接入 VR (Remote Vision)
  [机器人关节录放] 10) interface 录制
                  11) interface 回放
  [其他]          12) 查看各库版本号
```

### XRoboToolkit 后端（可选）

与默认的 Vuer/WebXR 并列，可用 Pico **XRoboToolkit App + PC Service** 作为输入，发布同一套 `/teleop/*` 话题（无 IK）。官方组件说明见 [XR-Robotics](https://github.com/XR-Robotics)。

```bash
# 1) 安装官方 PC Service deb（按 Ubuntu 22.04/24.04 自动选择）
./init.sh install-xrobotoolkit-pc-service

# 2) 安装 Python SDK（会先检测 PC Service；构建产物在 vr_pose_publisher/dependencies/）
./init.sh install-xrobotoolkit

# 3) 启动 PC Service，再启动发布节点
./run.sh vr-xrt-service
./run.sh vr-xrt

# 关闭 PC Service
./run.sh vr-xrt-service stop
```

| 对比 | `./run.sh vr` | `./run.sh vr-xrt` |
|------|---------------|-------------------|
| 输入 | 头显浏览器 WebXR（Vuer） | XRoboToolkit SDK |
| 头显 App | 浏览器 | XRoboToolkit Unity App |
| 发布话题 | `/teleop/*` | `/teleop/*`（相同契约）+ `/teleop/tracker_<SN>_pose` |

#### 体感追踪器（可选，仅 `vr-xrt`）

用 Pico 体感追踪器（Tracker Independent Tracking）时：

1. 在头显内完成追踪器**配对与校准**；
2. 打开 XRoboToolkit App 控制面板，**开启 Motion / Tracker 追踪开关**（未开启时 SDK 数据里不含
   `Motion` 字段，节点不会发布对应话题）；
3. 启动 `./run.sh vr-xrt` 后话题按序列号**动态出现**（最多 3 个）：

```bash
ros2 topic list | grep '/teleop/tracker_'
ros2 topic echo /teleop/tracker_<SN>_pose

# 不需要追踪器时关闭该发布
XRT_TRACKERS=0 ./run.sh vr-xrt
```

位姿与头显/手柄处于同一世界坐标系（`geometry_msgs/Pose`，单位米）；详细契约见
`vr_pose_publisher/docs/PROTOCOL_CN.md` 的「体感追踪器位姿话题」一节。

#### 将 ROS 图像接入头显画面（可选，仅 `vr-xrt`）

把任意 ROS2 图像话题（仿真相机、RealSense 等）实时显示到头显 **XRoboToolkit App →
Remote Vision** 面板。基于官方 `XRoboToolkit-Orin-Video-Sender` 的 H.264 TCP 协议，
单目画面会自动复制成左右两半以复用内置的 ZEDMINI 立体视频源（无需改头显配置）。

```bash
# 安装编码依赖（PyAV，自带 ffmpeg/libx264）
./init.sh install-video

# 启动 vr-xrt，命令行会询问「是否启用图像视频传输」
./run.sh vr-xrt
#   -> 回答 y 后列出检测到的 RGB 图像话题
#      输入编号选中，或直接输入以 / 开头的话题名；留空 = 自动搜索

# 跳过询问（CI、管道、后台任务）：用环境变量直接指定
XR_IMAGE_TOPIC=auto ./run.sh vr-xrt                # 自动搜索 RGB 话题
XR_IMAGE_TOPIC=/head_camera/rgb ./run.sh vr-xrt    # 直接指定话题
XR_IMAGE_TOPIC= ./run.sh vr-xrt                    # 留空 = 不启用
```

图像桥由 `XRTargetNode` 拉起的**独立子进程**承载，图像订阅、H.264 编码、TCP
推流全在子进程内完成：视频链路的崩溃或卡顿不会影响 `/teleop/*` 姿态发布，
反之姿态遥操也不会被编码拖累。详见
[`vr_pose_publisher/docs/ROS_IMAGE_TO_VR_CN.md`](vr_pose_publisher/docs/ROS_IMAGE_TO_VR_CN.md)。

头显端：XRoboToolkit App → **Remote Vision** → 视频源选 **ZEDMINI** → **Listen** →
输入 PC 的 IP → **Confirm**。

无头显时可用模拟客户端验证：

```bash
python vr_pose_publisher/tests/fake_headset_client.py --port 13579 --frames 60 --save /tmp/out.h264
ffplay /tmp/out.h264
```

完整说明（参数、环境变量、自动搜索规则、布局、排障）见
[`vr_pose_publisher/docs/ROS_IMAGE_TO_VR_CN.md`](vr_pose_publisher/docs/ROS_IMAGE_TO_VR_CN.md)。

### VR 遥操录包 / 回放

将 VR 发布的 `/teleop/*` 话题（头显/手柄位姿、按键、摇杆、扳机、体感追踪器）录制为 ros2 bag，之后可离线回放以模拟 VR 输入（供 `VRInputHandler` 消费）。底层脚本为 `scripts/vr-bag.sh`，推荐通过 `run.sh` 调用。

**前置条件**

- **录制**：另一终端已运行 `./run.sh vr` **或** `./run.sh vr-xrt`，且 VR 设备已连接
- **回放**：**不要**同时运行 `./run.sh vr` / `./run.sh vr-xrt`（避免 `/teleop/*` 话题冲突）；确保 `arms_target_manager` / `VRInputHandler` 与机器人控制栈已运行；回放前建议将机器人置于 HOLD，结束后再切回 HOLD

**录制**

```bash
./run.sh vr-record
./run.sh vr-record --name grasp_demo

# 或直接运行底层脚本
./scripts/vr-bag.sh record --name grasp_demo
```

操作流程：输入会话名（可选）→ 按 Enter 开始录制 → 进行 VR 遥操 → 再按 Enter 停止。bag 默认保存到 `xr_bags/`（可用环境变量 `XR_BAG_DIR` 覆盖）。

录制话题：`/teleop/head_pose`、`/teleop/left_ee_pose`、`/teleop/right_ee_pose`、`/teleop/controller_state`、`/teleop/thumbstick_axes`、`/teleop/trigger_values`

体感追踪器话题（`/teleop/tracker_<SN>_pose`）名字带序列号，**录制时会按当前 ROS 图自动追加**；回放「核心范围」时也会自动带上该 bag 里实际录到的追踪器话题。未开启追踪器时不会追加，不影响原有流程。

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
./run.sh vr-xrt-service   # XRoboToolkit：先启 PC Service
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

# 可选：安装 XRoboToolkit PC Service deb + Python SDK（VR XRT 后端）
./init.sh install-xrobotoolkit-pc-service
./init.sh install-xrobotoolkit

# 可选：安装 ROS 图像接入 VR（Remote Vision）的编码依赖 PyAV
./init.sh install-video

# 配置 ROS2 工作空间（写入 .fa-env.toml + 按 backend 写 activate 挂钩）
./init.sh ros2-workspace
# 若 conda 与 uv 都要挂钩：./init.sh ros2-workspace --all

# 配置 NJU PyPI 镜像（pip.conf）
./init.sh pypi-mirror

# 启动
./run.sh
./run.sh viser
./run.sh vr
./run.sh vr-xrt-service
./run.sh vr-xrt-service stop
./run.sh vr-xrt
XR_IMAGE_TOPIC=auto ./run.sh vr-xrt        # 图像视频传输：自动搜索 RGB 话题
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

使用 `release.sh` 将精简源码打成 zip（临时目录 staging，不改动工作区），便于分发或离线部署。默认**不**打入 `dependencies/`、本地环境与录包数据；现场再按需安装。

```bash
# 交互菜单（选 package / package-no-git）
./release.sh

# 推荐：不含 .git，体积更小
./release.sh --package-no-git

# 含 .git（主仓 + 已检出的子模块），便于现场 git pull
./release.sh --package

# 指定输出路径（等价 package-no-git 到该路径）
./release.sh -o /path/to/fa-py-libraries.zip

# 不拉取远程，按当前检出直接打包（离线场景）
./release.sh --package-no-git --skip-submodules
```

- 默认输出：`dist/fa-py-libraries-<时间戳>[_nogit].zip`（`dist/` 已加入 `.gitignore`）
- **始终排除**：`.venv/`、`venv/`、`.idea/`、`.vscode/`、`dist/`、`**/dependencies/`、`xr_bags/`、`joint_records/`、`*.pem`、`__pycache__/`、`*.egg-info/`、`.fa-env.local.toml`
- **`--package-no-git` 额外排除**：`.git/`（含子模块内 `.git`）
- **包含**：根脚本（`init.sh` / `run.sh` / `release.sh` / `scripts/`）、`.fa-env.toml`、`README.md`、`.gitmodules`，以及三子模块源码（不含其 `dependencies/`）
- 需要能访问子模块远程时，请勿加 `--skip-submodules`（默认会先 `./init.sh submodules`）

现场解压后：

```bash
./init.sh install
# 可选（VR XRT）：./init.sh install-xrobotoolkit-pc-service && ./init.sh install-xrobotoolkit
```

## 说明

- `run.sh` 根据 `.fa-env.toml` 的 `backend` 激活 conda 或 `.venv`；若配置了 `[ros2].workspace` 会 source 对应 `install/setup.bash`。
- **uv 与 ROS2**：`rclpy`、`*-msgs` 不在 PyPI；安装时用 `--no-deps` + 单独装 PyPI 依赖。当 `backend=uv` 时，`./init.sh env` 会优先用系统解释器（如 `/usr/bin/python3.12`，跳过 conda 路径）并加 `--system-site-packages`，与 [uv+ROS2 常见做法](https://www.cnblogs.com/yzcat/p/19960512) 一致。执行 `./init.sh ros2-workspace` 后，手动 `source .venv/bin/activate` 也会自动 source ROS2 工作空间（找不到则回退 `/opt/ros/jazzy`）。
- VR 遥操需要额外的 SSL 证书，安装时会自动生成，详见子模块 README。
