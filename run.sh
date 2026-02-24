#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_NAME="fa-ros2"

usage() {
  echo "用法: $0 [viser|vr|record|playback [json文件路径]|all]"
  echo
  echo "不带参数时进入交互菜单。"
  echo "说明:"
  echo "  viser      启动 ros2-viser 的 launch.py"
  echo "  vr         启动 vr_pose_publisher 的 launch.py"
  echo "  record     启动 interface 的录制模式"
  echo "  playback   启动 interface 的回放模式（可选传入 json 文件路径）"
  echo "  all        交互选择上述任一启动项"
}

ensure_conda_and_activate() {
  if ! command -v conda >/dev/null 2>&1; then
    echo "未检测到 conda，请先安装并配置 conda。"
    exit 1
  fi

  if ! conda env list | awk '{print $1}' | grep -Fxq "$ENV_NAME"; then
    echo "环境 '$ENV_NAME' 不存在，请先执行 ./init.sh conda 创建环境。"
    exit 1
  fi

  # 在脚本内启用 conda shell hook，才能使用 conda activate
  eval "$(conda shell.bash hook)"
  conda activate "$ENV_NAME"
}

run_viser_launch() {
  local script_path="$ROOT_DIR/ros2-viser/launch.py"
  if [[ ! -f "$script_path" ]]; then
    echo "未找到脚本: $script_path"
    exit 1
  fi
  ensure_conda_and_activate
  echo ">>> 启动 ros2-viser launch"
  python "$script_path"
}

run_vr_launch() {
  local script_path="$ROOT_DIR/vr_pose_publisher/launch.py"
  if [[ ! -f "$script_path" ]]; then
    echo "未找到脚本: $script_path"
    exit 1
  fi
  ensure_conda_and_activate
  echo ">>> 启动 vr pose launch"
  python "$script_path"
}

run_interface_record() {
  local script_path="$ROOT_DIR/ros2_robot_interface/record/record_playback.py"
  if [[ ! -f "$script_path" ]]; then
    echo "未找到脚本: $script_path"
    exit 1
  fi
  ensure_conda_and_activate
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
  ensure_conda_and_activate
  echo ">>> 启动 interface record_playback（回放模式）"
  if [[ -n "$json_file" ]]; then
    python "$script_path" playback --file "$json_file"
  else
    python "$script_path" playback
  fi
}

interactive_menu() {
  echo "请选择要启动的功能:"
  echo "  1) ros2-viser launch"
  echo "  2) vr pose launch"
  echo "  3) interface record_playback 录制模式"
  echo "  4) interface record_playback 回放模式"
  echo "  q) 退出"
  read -r -p "输入选项 [1/2/3/4/q]: " choice

  case "$choice" in
    1) run_viser_launch ;;
    2) run_vr_launch ;;
    3) run_interface_record ;;
    4)
      read -r -p "可选：输入回放 json 文件路径（留空则启动后自行选择）: " json_file
      run_interface_playback "${json_file:-}"
      ;;
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
    record)
      run_interface_record
      ;;
    playback)
      run_interface_playback "${2:-}"
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

main "${1:-}" "${2:-}"
