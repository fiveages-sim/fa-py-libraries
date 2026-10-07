#!/usr/bin/env bash

set -euo pipefail

# =============================================================================
# 初始化
# =============================================================================

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FA_ENV_BACKEND_OVERRIDE="${FA_ENV_BACKEND:-}"
# shellcheck source=scripts/fa-env.sh
source "${ROOT_DIR}/scripts/fa-env.sh"
FA_ENV_ROOT_DIR="$ROOT_DIR"

# =============================================================================
# 帮助
# =============================================================================

usage() {
  echo "用法: $0 [命令] [参数...]"
  echo
  echo "不带参数时进入交互菜单。"
  echo "Python 环境由 .fa-env.toml 的 backend 决定（conda | uv），可用 ./init.sh set-backend 切换。"
  echo "临时覆盖: FA_ENV_BACKEND=uv ./run.sh viser"
  echo
  echo "可视化:"
  echo "  viser                    启动 ros2-viser 的 launch.py"
  echo
  echo "VR 遥操:"
  echo "  vr                       启动 vr_pose_publisher（Vuer/WebXR）"
  echo "  vr-xrt                   启动 vr_pose_publisher（XRoboToolkit SDK）"
  echo "                           启动时可选启用图像视频传输并指定话题"
  echo "  vr-xrt-service [start]   启动 XRoboToolkit PC Service（runService.sh）"
  echo "  vr-xrt-service stop      关闭 XRoboToolkit PC Service"
  echo "  vr-record [--name 名称]  录制 /teleop/* 话题到 ros2 bag"
  echo "  vr-playback [选项]       回放 bag（可选 --file --rate --count）"
  echo "  vr-bag-clean [选项]      清理已录制的 bag"
  echo
  echo "机器人关节录放:"
  echo "  record                   启动 interface 关节快照录制 (JSON)"
  echo "  playback [json文件路径]  启动 interface 关节快照回放"
  echo
  echo "其他:"
  echo "  versions                 一键查看当前各库版本号"
  echo "  all                      交互选择上述任一启动项"
}

# =============================================================================
# 环境
# =============================================================================

ensure_python_env() {
  set +u
  fa_env_activate "$ROOT_DIR" || exit 1
  set -u
}

# =============================================================================
# 可视化 — ros2-viser
# =============================================================================

run_viser_launch() {
  local script_path="$ROOT_DIR/ros2-viser/launch.py"
  if [[ ! -f "$script_path" ]]; then
    echo "未找到脚本: $script_path"
    exit 1
  fi
  ensure_python_env
  echo ">>> 启动 ros2-viser launch"
  python "$script_path"
}

# =============================================================================
# VR 遥操发布 — vr_pose_publisher
# =============================================================================

run_vr_launch() {
  local script_path="$ROOT_DIR/vr_pose_publisher/launch.py"
  if [[ ! -f "$script_path" ]]; then
    echo "未找到脚本: $script_path"
    exit 1
  fi
  ensure_python_env
  echo ">>> 启动 vr pose launch (Vuer/WebXR)"
  python "$script_path"
}

# -----------------------------------------------------------------------------
# ROS 图像 → 头显 Remote Vision（XRoboToolkit）
# -----------------------------------------------------------------------------
# 图像推流已整合进 vr-xrt（XRTargetNode 拉起的独立视频子进程），
# 不再提供独立入口；启动 vr-xrt 时会询问是否启用并选择话题。

# -----------------------------------------------------------------------------
# 启动 vr-xrt 前的视频传输询问（是否启用 + 图像话题）
# -----------------------------------------------------------------------------

# 列出检测到的 RGB 图像话题（编号清单）。
# 复用节点侧的排序逻辑（xr_video_bridge.image_topics_menu_lines），
# 保证这里显示的编号与节点自动搜索时的优先级一致。
#
# 结果同时缓存在 IMAGE_TOPICS_MENU：每次枚举都要另起一个 rclpy 进程等话题发现，
# 若显示与取编号分别枚举一次，不仅慢，还可能因两次看到的 ROS 图不同而错位。
IMAGE_TOPICS_MENU=""
list_image_topics_menu() {
  local script_path="$ROOT_DIR/vr_pose_publisher/launch_xrobotoolkit.py"
  if ! IMAGE_TOPICS_MENU="$(python "$script_path" --list-image-topics 2>/dev/null)"; then
    IMAGE_TOPICS_MENU=""
    echo ">>> 话题枚举失败（仿真未启动？）"
    return 1
  fi
  if [[ -z "$IMAGE_TOPICS_MENU" ]]; then
    echo ">>> 未发现 sensor_msgs/msg/Image 话题；可稍后重试，或直接输入 / 开头的话题名"
  else
    printf '%s\n' "$IMAGE_TOPICS_MENU"
  fi
}

# 询问是否启用视频传输并选择图像话题，结果 export 到 XR_IMAGE_TOPIC。
#
# 跳过询问的情形（直接沿用环境变量，避免破坏自动化/管道用法）：
#   * XR_IMAGE_TOPIC / VR_IMAGE_TOPIC 已设置（显式意图优先）
#   * stdin 不是终端（CI、后台任务、管道）
prompt_vr_video_topic() {
  # 已由环境变量指定 → 不再打扰用户
  if [[ -n "${XR_IMAGE_TOPIC:-}" || -n "${VR_IMAGE_TOPIC:-}" ]]; then
    echo ">>> 视频传输话题已由环境变量指定: ${XR_IMAGE_TOPIC:-${VR_IMAGE_TOPIC}}"
    return 0
  fi
  if [[ ! -t 0 ]]; then
    echo ">>> 非交互终端，跳过视频传输询问（如需启用请设置 XR_IMAGE_TOPIC=/话题名）"
    return 0
  fi

  local answer=""
  read -r -p ">>> 是否启用图像视频传输（送进头显 Remote Vision）? [y/N]: " answer || return 0
  case "${answer,,}" in
    y|yes) ;;
    *) echo ">>> 已跳过图像视频传输"; return 0 ;;
  esac

  echo
  echo ">>> 正在检测可用图像话题..."
  list_image_topics_menu
  echo

  local choice="" topic=""
  local attempts=0
  while (( attempts < 3 )); do
    # EOF（Ctrl-D / 输入被重定向）视作放弃选择，跳过而非中断整个脚本
    read -r -p ">>> 输入图像话题编号，或直接输入以 / 开头的话题名（留空=自动搜索）: " choice || return 0
    choice="${choice#"${choice%%[![:space:]]*}"}"   # 去首空白
    choice="${choice%"${choice##*[![:space:]]}"}"   # 去尾空白

    if [[ -z "$choice" ]]; then
      topic="auto"
      break
    fi
    # 纯数字 → 取清单里的第 N 项（用缓存清单，不重新枚举）
    if [[ "$choice" =~ ^[0-9]+$ ]]; then
      # 清单格式为 "  [N] /topic/name"
      topic="$(printf '%s\n' "$IMAGE_TOPICS_MENU" \
        | sed -n "s/^[[:space:]]*\[\(${choice}\)\][[:space:]]*\(.*\)$/\2/p" | head -n1)"
      if [[ -n "$topic" ]]; then
        break
      fi
      echo ">>> 无效编号: $choice（请在上面的清单范围内，或直接输入 / 开头的话题名）"
    elif [[ "$choice" == /* ]]; then
      topic="$choice"
      break
    else
      echo ">>> 无效输入: $choice（需为编号，或以 / 开头的话题名）"
    fi
    attempts=$((attempts + 1))
  done

  if [[ -z "${topic:-}" ]]; then
    echo ">>> 多次输入无效，已跳过图像视频传输"
    return 0
  fi

  export XR_IMAGE_TOPIC="$topic"
  if ! python -c "import av" >/dev/null 2>&1; then
    echo ">>> ⚠️  未检测到 PyAV（图像编码依赖），视频传输将无法推流"
    echo ">>>     请先运行: ./init.sh install-video"
  fi
  echo ">>> 已启用图像视频传输: XR_IMAGE_TOPIC=$topic"
  echo ">>> 头显: XRoboToolkit App → Remote Vision → 视频源选 ZEDMINI → Listen → 输入本机 IP"
}

# -----------------------------------------------------------------------------
# XRoboToolkit PC Service
# -----------------------------------------------------------------------------

XRT_PC_SERVICE_SCRIPT="/opt/apps/roboticsservice/runService.sh"
XRT_PC_SERVICE_PROCESS="RoboticsServiceProcess"
# Linux comm 最长 15 字符，RoboticsServiceProcess 会被截成 RoboticsService。
XRT_PC_SERVICE_COMM="RoboticsService"

xrt_pc_service_pids() {
  pgrep -x "$XRT_PC_SERVICE_COMM" 2>/dev/null || true
}

xrt_pc_service_running() {
  pgrep -x "$XRT_PC_SERVICE_COMM" >/dev/null 2>&1
}

run_xrt_pc_service() {
  local pids
  if [[ ! -x "$XRT_PC_SERVICE_SCRIPT" ]]; then
    echo "未找到 XRoboToolkit PC Service: $XRT_PC_SERVICE_SCRIPT"
    echo "请先运行: ./init.sh install-xrobotoolkit-pc-service"
    exit 1
  fi
  pids="$(xrt_pc_service_pids | tr '\n' ' ')"
  if [[ -n "${pids// }" ]]; then
    echo ">>> XRoboToolkit PC Service 已在运行（pid: $pids）"
    echo ">>> 接下来可运行: ./run.sh vr-xrt"
    return 0
  fi
  echo ">>> 启动 XRoboToolkit PC Service"
  echo ">>> $XRT_PC_SERVICE_SCRIPT"
  # 官方脚本会后台拉起 RoboticsServiceProcess 后立即退出；nohup 避免父进程退出时带上 SIGHUP。
  nohup bash "$XRT_PC_SERVICE_SCRIPT" >/dev/null 2>&1 &
  local i
  for i in 1 2 3 4 5 6; do
    pids="$(xrt_pc_service_pids | tr '\n' ' ')"
    if [[ -n "${pids// }" ]]; then
      echo ">>> PC Service 已启动（pid: $pids）"
      echo ">>> 接下来可运行: ./run.sh vr-xrt"
      return 0
    fi
    sleep 0.5
  done
  echo ">>> 未能确认 $XRT_PC_SERVICE_PROCESS 正在运行。"
  echo ">>> 无桌面/SSH 时请确认 DISPLAY；也可直接执行: $XRT_PC_SERVICE_SCRIPT"
  exit 1
}

stop_xrt_pc_service() {
  local pids
  pids="$(xrt_pc_service_pids | tr '\n' ' ')"
  if [[ -z "${pids// }" ]]; then
    echo ">>> XRoboToolkit PC Service 未在运行"
    return 0
  fi
  echo ">>> 关闭 XRoboToolkit PC Service（pid: $pids）"
  # shellcheck disable=SC2086
  kill $pids 2>/dev/null || true
  local i
  for i in 1 2 3 4 5 6 7 8 9 10; do
    if ! xrt_pc_service_running; then
      echo ">>> 已关闭"
      return 0
    fi
    sleep 0.3
  done
  echo ">>> SIGTERM 未退出，发送 SIGKILL"
  pids="$(xrt_pc_service_pids | tr '\n' ' ')"
  if [[ -n "${pids// }" ]]; then
    # shellcheck disable=SC2086
    kill -9 $pids 2>/dev/null || true
  fi
  sleep 0.3
  if xrt_pc_service_running; then
    echo ">>> 未能关闭，请手动检查: pgrep -a $XRT_PC_SERVICE_COMM"
    exit 1
  fi
  echo ">>> 已强制关闭"
}

run_xrt_pc_service_cmd() {
  case "${1:-start}" in
    start) run_xrt_pc_service ;;
    stop) stop_xrt_pc_service ;;
    *)
      echo "未知参数: $1"
      echo "用法: ./run.sh vr-xrt-service [start|stop]"
      exit 1
      ;;
  esac
}

run_vr_xrt_launch() {
  local script_path="$ROOT_DIR/vr_pose_publisher/launch_xrobotoolkit.py"
  if [[ ! -f "$script_path" ]]; then
    echo "未找到脚本: $script_path"
    exit 1
  fi
  ensure_python_env
  if ! python -c "import xrobotoolkit_sdk" >/dev/null 2>&1; then
    echo "未检测到 xrobotoolkit_sdk。"
    echo "请先运行: ./init.sh install-xrobotoolkit"
    echo "  或: cd vr_pose_publisher && bash setup_xrobotoolkit.sh"
    exit 1
  fi
  if ! xrt_pc_service_running; then
    echo ">>> 未检测到 PC Service（$XRT_PC_SERVICE_PROCESS）。"
    echo ">>> 可先运行: ./run.sh vr-xrt-service"
    echo ">>> 或从应用菜单打开 XRoboToolkit-PC-Service"
  fi
  # 询问是否启用图像视频传输（已在环境变量里指定则直接沿用）
  prompt_vr_video_topic
  echo ">>> 启动 vr pose launch (XRoboToolkit)"
  echo ">>> 请确认: PC Service 已运行，Pico App 已连接"
  python "$script_path"
}

# =============================================================================
# VR 遥操录包 — ros2 bag (/teleop/*)
# =============================================================================

run_vr_bag_record() {
  local script_path="$ROOT_DIR/scripts/vr-bag.sh"
  if [[ ! -f "$script_path" ]]; then
    echo "未找到脚本: $script_path"
    exit 1
  fi
  echo ">>> 启动 VR 遥操录包"
  bash "$script_path" record "$@"
}

run_vr_bag_playback() {
  local script_path="$ROOT_DIR/scripts/vr-bag.sh"
  if [[ ! -f "$script_path" ]]; then
    echo "未找到脚本: $script_path"
    exit 1
  fi
  echo ">>> 启动 VR 遥操回放"
  bash "$script_path" playback "$@"
}

run_vr_bag_clean() {
  local script_path="$ROOT_DIR/scripts/vr-bag.sh"
  if [[ ! -f "$script_path" ]]; then
    echo "未找到脚本: $script_path"
    exit 1
  fi
  echo ">>> 清理 VR 遥操 bag"
  bash "$script_path" clean "$@"
}

# =============================================================================
# 机器人关节录放 — ros2_robot_interface
# =============================================================================

run_interface_record() {
  local script_path="$ROOT_DIR/ros2_robot_interface/record/record_playback.py"
  if [[ ! -f "$script_path" ]]; then
    echo "未找到脚本: $script_path"
    exit 1
  fi
  ensure_python_env
  echo ">>> 启动 interface record_playback（录制模式）"
  python "$script_path" record
}

run_interface_playback() {
  local script_path="$ROOT_DIR/ros2_robot_interface/record/record_playback.py"
  local json_file="${1:-}"
  if [[ ! -f "$script_path" ]]; then
    echo "未找到脚本: $script_path"
    exit 1
  fi
  ensure_python_env
  echo ">>> 启动 interface record_playback（回放模式）"
  if [[ -n "$json_file" ]]; then
    python "$script_path" playback --file "$json_file"
  else
    python "$script_path" playback
  fi
}

# =============================================================================
# 工具
# =============================================================================

read_project_name_version() {
  local pyproject_file="$1"
  local project_name=""
  local project_version=""

  project_name="$(
    awk -F'=' '
      /^\[project\]/ { in_project=1; next }
      /^\[/ { in_project=0 }
      in_project && $1 ~ /^[[:space:]]*name[[:space:]]*$/ {
        gsub(/^[[:space:]]+|[[:space:]]+$/, "", $2)
        gsub(/^"|"$/, "", $2)
        print $2
        exit
      }
    ' "$pyproject_file"
  )"

  project_version="$(
    awk -F'=' '
      /^\[project\]/ { in_project=1; next }
      /^\[/ { in_project=0 }
      in_project && $1 ~ /^[[:space:]]*version[[:space:]]*$/ {
        gsub(/^[[:space:]]+|[[:space:]]+$/, "", $2)
        gsub(/^"|"$/, "", $2)
        print $2
        exit
      }
    ' "$pyproject_file"
  )"

  echo "${project_name:-unknown}|${project_version:-unknown}"
}

show_library_versions() {
  local libs=("ros2-viser" "vr_pose_publisher" "ros2_robot_interface")
  local lib_dir pyproject info project_name project_version

  fa_env_load_config "$ROOT_DIR"
  echo ">>> 当前库版本 (backend=$FA_ENV_BACKEND):"
  for lib_dir in "${libs[@]}"; do
    pyproject="$ROOT_DIR/$lib_dir/pyproject.toml"
    if [[ ! -f "$pyproject" ]]; then
      echo "  - $lib_dir: 未找到 pyproject.toml"
      continue
    fi

    info="$(read_project_name_version "$pyproject")"
    project_name="${info%%|*}"
    project_version="${info#*|}"
    echo "  - $lib_dir -> $project_name: $project_version"
  done
}

# =============================================================================
# 交互菜单 & 入口
# =============================================================================

interactive_menu() {
  fa_env_load_config "$ROOT_DIR"
  echo "请选择要启动的功能 (backend=$FA_ENV_BACKEND)"
  echo
  echo "  [可视化]"
  echo "    1) ros2-viser launch"
  echo
  echo "  [VR 遥操]"
  echo "    2) vr pose launch (Vuer/WebXR)"
  echo "    3) vr pose launch (XRoboToolkit)"
  echo "    4) 启动 XRoboToolkit PC Service"
  echo "    5) 关闭 XRoboToolkit PC Service"
  echo "    6) VR 遥操录包"
  echo "    7) VR 遥操回放"
  echo "    8) VR bag 清理"
  echo
  echo "  [机器人关节录放]"
  echo "    9) interface 录制"
  echo "    10) interface 回放"
  echo
  echo "  [其他]"
  echo "    11) 查看各库版本号"
  echo "    q) 退出"
  echo
  read -r -p "输入选项 [1-11/q]: " choice

  case "$choice" in
    1) run_viser_launch ;;
    2) run_vr_launch ;;
    3) run_vr_xrt_launch ;;
    4) run_xrt_pc_service ;;
    5) stop_xrt_pc_service ;;
    6) run_vr_bag_record ;;
    7) run_vr_bag_playback ;;
    8) run_vr_bag_clean ;;
    9) run_interface_record ;;
    10)
      read -r -p "可选：输入回放 json 文件路径（留空则启动后自行选择）: " json_file
      run_interface_playback "${json_file:-}"
      ;;
    11) show_library_versions ;;
    q|Q) echo "已退出。" ;;
    *) echo "无效选项。"; exit 1 ;;
  esac
}

main() {
  case "${1:-}" in
    viser)
      run_viser_launch
      ;;
    vr)
      run_vr_launch
      ;;
    vr-xrt-service|xrt-service)
      run_xrt_pc_service_cmd "${2:-start}"
      ;;
    vr-xrt-service-stop|xrt-service-stop)
      stop_xrt_pc_service
      ;;
    vr-xrt|vr-xrobotoolkit)
      run_vr_xrt_launch
      ;;
    vr-record)
      run_vr_bag_record "${@:2}"
      ;;
    vr-playback)
      run_vr_bag_playback "${@:2}"
      ;;
    vr-bag-clean)
      run_vr_bag_clean "${@:2}"
      ;;
    record)
      run_interface_record
      ;;
    playback)
      run_interface_playback "${2:-}"
      ;;
    versions)
      show_library_versions
      ;;
    all)
      interactive_menu
      ;;
    "")
      interactive_menu
      ;;
    -h|--help|help)
      usage
      ;;
    *)
      echo "未知参数: $1"
      usage
      exit 1
      ;;
  esac
}

main "$@"
