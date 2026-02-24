# fa-py-libraries

用于聚合和快速启动以下 ROS2 相关 Python 子模块：

- `ros2_robot_interface`
- `ros2-viser`
- `vr_pose_publisher`

## 目录结构

- `init.sh`：初始化脚本（子模块、conda 环境、依赖安装）
- `run.sh`：快速启动脚本（激活 conda 后启动常用入口）

## 快速开始

### 1) 初始化

```bash
./init.sh all
```

等价于：

1. 初始化子模块并切到各子模块最新 `main`
2. 创建 `fa-ros2` conda 环境（默认 Python `3.12`）
3. 激活环境后按顺序安装：
   - `ros2_robot_interface`
   - `ros2-viser`
   - `vr_pose_publisher`

### 2) 启动

```bash
./run.sh
```

进入交互菜单后可选择：

- `ros2-viser` 的 launch
- `vr_pose_publisher` 的 launch
- `record_playback` 录制模式
- `record_playback` 回放模式

## 常用命令

```bash
# 仅初始化子模块
./init.sh submodules

# 创建 conda 环境（指定 Python 版本）
./init.sh conda 3.11

# 仅安装三个子项目
./init.sh install

# 直接启动 viser
./run.sh viser

# 直接启动 VR pose
./run.sh vr

# 启动录制模式
./run.sh record

# 启动回放模式（可指定文件）
./run.sh playback
./run.sh playback /path/to/record.json
```

## 说明

- 所有启动命令会先检查并激活 conda 环境 `fa-ros2`。
- 若环境不存在，请先执行 `./init.sh conda` 或 `./init.sh all`。
- VR遥操需要额外的ssl证书配置，请参考对应子模块的readme
