# fa-py-libraries

用于聚合和快速启动以下 ROS2 相关 Python 子模块：

- `ros2_robot_interface`
- `ros2-viser`
- `vr_pose_publisher`

## 目录结构

- `init.sh`：初始化脚本（子模块、conda/uv 环境、依赖安装）
- `run.sh`：快速启动脚本（按配置激活环境后启动常用入口）
- `.fa-env.toml`：选择 `run.sh` / `install` 使用 **conda** 还是 **uv**
- `scripts/fa-env.sh`：环境配置与激活（供上述脚本共用）

## 环境配置（conda 与 uv 可并存）

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

conda 环境与 uv 的 `.venv` **互不删除**，可按需分别创建。

## 快速开始

### 1) 初始化（默认按 `.fa-env.toml` 的 backend）

```bash
./init.sh all
```

等价于：

1. 初始化子模块并切到各子模块最新 `main`
2. 按 `backend` 创建 conda 或 uv 环境（默认 Python `3.12`）
3. 激活环境后按顺序安装：
   - `ros2_robot_interface`
   - `ros2-viser`
   - `vr_pose_publisher`

### 2) 启动

```bash
./run.sh
```

进入交互菜单后可选择 viser / vr / record / playback 等。

## 常用命令

```bash
# 仅初始化子模块
./init.sh submodules

# 分别创建环境（可并存）
./init.sh conda 3.12
./init.sh uv 3.12

# 切换 run.sh 使用的 backend
./init.sh set-backend uv

# 安装（按 .fa-env.toml；可临时指定）
./init.sh install
./init.sh install --uv
./init.sh install --conda

# 配置 ROS2 工作空间（写入 .fa-env.toml + 按 backend 写 activate 挂钩）
./init.sh ros2-workspace
# 若 conda 与 uv 都要挂钩：./init.sh ros2-workspace --all

# 配置 NJU PyPI 镜像（pip.conf）
./init.sh pypi-mirror

# 启动
./run.sh viser
./run.sh vr
./run.sh record
./run.sh playback
./run.sh playback /path/to/record.json
```

## 说明

- `run.sh` 根据 `.fa-env.toml` 的 `backend` 激活 conda 或 `.venv`；若配置了 `[ros2].workspace` 会 source 对应 `install/setup.bash`。
- **uv 与 ROS2**：`rclpy`、`*-msgs` 不在 PyPI；安装时用 `--no-deps` + 单独装 PyPI 依赖。`./init.sh uv` 会优先用系统解释器（如 `/usr/bin/python3.12`，跳过 conda 路径）并加 `--system-site-packages`，与 [uv+ROS2 常见做法](https://www.cnblogs.com/yzcat/p/19960512) 一致。执行 `./init.sh ros2-workspace` 后，手动 `source .venv/bin/activate` 也会自动 source ROS2 工作空间（找不到则回退 `/opt/ros/jazzy`）。
- VR 遥操需要额外的 SSL 证书，安装时会自动生成，详见子模块 README。
