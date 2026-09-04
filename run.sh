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
  echo "  vr-xrt-service [start]   启动 XRoboToolkit PC Service（runService.sh）"
  echo "  vr-xrt-service stop      关闭 XRoboToolkit PC Service"
  echo "  vr-record [--name 名称]  录制 /xr/* 话题到 ros2 bag"
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
  echo ">>> 启动 vr pose launch (XRoboToolkit)"
  echo ">>> 请确认: PC Service 已运行，Pico App 已连接"
  python "$script_path"
}

# =============================================================================
# VR 遥操录包 — ros2 bag (/xr/*)
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
