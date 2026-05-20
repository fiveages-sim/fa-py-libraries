#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FA_ENV_BACKEND_OVERRIDE="${FA_ENV_BACKEND:-}"
# shellcheck source=scripts/fa-env.sh
source "${ROOT_DIR}/scripts/fa-env.sh"
FA_ENV_ROOT_DIR="$ROOT_DIR"

usage() {
  echo "用法: $0 [viser|vr|record|playback [json文件路径]|versions|all]"
  echo
  echo "不带参数时进入交互菜单。"
  echo "Python 环境由 .fa-env.toml 的 backend 决定（conda | uv），可用 ./init.sh set-backend 切换。"
  echo "临时覆盖: FA_ENV_BACKEND=uv ./run.sh viser"
  echo
  echo "说明:"
  echo "  viser      启动 ros2-viser 的 launch.py"
  echo "  vr         启动 vr_pose_publisher 的 launch.py"
  echo "  record     启动 interface 的录制模式"
  echo "  playback   启动 interface 的回放模式（可选传入 json 文件路径）"
  echo "  versions   一键查看当前各库版本号"
  echo "  all        交互选择上述任一启动项"
}

ensure_python_env() {
  set +u
  fa_env_activate "$ROOT_DIR" || exit 1
  set -u
}

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

run_vr_launch() {
  local script_path="$ROOT_DIR/vr_pose_publisher/launch.py"
  if [[ ! -f "$script_path" ]]; then
    echo "未找到脚本: $script_path"
    exit 1
  fi
  ensure_python_env
  echo ">>> 启动 vr pose launch"
  python "$script_path"
}

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

interactive_menu() {
  fa_env_load_config "$ROOT_DIR"
  echo "请选择要启动的功能 (backend=$FA_ENV_BACKEND):"
  echo "  1) ros2-viser launch"
  echo "  2) vr pose launch"
  echo "  3) interface record_playback 录制模式"
  echo "  4) interface record_playback 回放模式"
  echo "  5) 查看各库版本号"
  echo "  q) 退出"
  read -r -p "输入选项 [1/2/3/4/5/q]: " choice

  case "$choice" in
    1) run_viser_launch ;;
    2) run_vr_launch ;;
    3) run_interface_record ;;
    4)
      read -r -p "可选：输入回放 json 文件路径（留空则启动后自行选择）: " json_file
      run_interface_playback "${json_file:-}"
      ;;
    5) show_library_versions ;;
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

main "${1:-}" "${2:-}"
