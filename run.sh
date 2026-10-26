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

# Local XR paths — edit configs/local_xr.sh (env vars already set win).
# shellcheck source=configs/local_xr.sh
if [[ -f "${ROOT_DIR}/configs/local_xr.sh" ]]; then
  source "${ROOT_DIR}/configs/local_xr.sh"
fi
FA_XRT_TELEOP_CONFIG="${FA_XRT_TELEOP_CONFIG:-$ROOT_DIR/vr_pose_publisher/configs/xrt_teleop.yaml}"
FA_HAND_CONFIG="${FA_HAND_CONFIG:-${FA_XHAND1_CONFIG:-$ROOT_DIR/xr_hand_retarget/configs/xhand1.yaml}}"
FA_XHAND1_CONFIG="${FA_HAND_CONFIG}"
FA_HAND_RETARGETING="${FA_HAND_RETARGETING:-${FA_XHAND1_RETARGETING:-}}"
FA_XHAND1_RETARGETING="${FA_HAND_RETARGETING}"

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
  echo "本地 XR 配置: configs/local_xr.sh"
  echo "  configs/local.yaml      → 换手: hands.type = xhand1 | wuji | o6 | l6 | o7"
  echo "  FA_HAND                 → 临时覆盖 local.yaml"
  echo "  FA_XRT_TELEOP_CONFIG    → XRT 预置 (xrt_teleop.yaml)"
  echo
  echo "可视化:"
  echo "  viser                    启动 ros2-viser 的 launch.py"
  echo
  echo "VR 遥操:"
  echo "  vr                       启动 vr_pose_publisher（Vuer/WebXR）"
  echo "  vr-xrt [controller|wrist]  启动 XRoboToolkit"
  echo "                           controller=手柄 6DoF 控臂+夹爪"
  echo "                           wrist=光学腕 6DoF 控臂+灵巧手（交互菜单会再选手型）"
  echo "                           例: ./run.sh vr-xrt wrist"
  echo "                           wrist 运行中本终端: o=事件11(A/OCS2)  h=事件12(X/HOLD)，与脚踏板同一 /teleop/controller_state"
  echo "  vr-xrt-service [start]   启动 XRoboToolkit PC Service（runService.sh）"
  echo "  vr-xrt-service stop      关闭 XRoboToolkit PC Service"
  echo "  vr-update                临时发右摇杆 UPDATE（controller_state=4；非 OCS2 无效）"
  echo "  vr-ocs2                  临时发 OCS2（/fsm_command=3，无脚踏板时用）"
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

  if [[ $# -eq 0 ]]; then
    echo ">>> 启动 vr pose launch (XRoboToolkit)"
    echo ">>> 请确认: PC Service 已运行，Pico App 已连接"
    python "$script_path"
    return
  fi

  local extra=()
  local has_config=0 has_hand_config=0 has_hand_retargeting=0
  local arg
  for arg in "$@"; do
    case "$arg" in
      --config|--config=*) has_config=1 ;;
      --hand-config|--hand-config=*) has_hand_config=1 ;;
      --hand-retargeting|--hand-retargeting=*) has_hand_retargeting=1 ;;
    esac
  done
  if [[ $has_config -eq 0 && -n "${FA_XRT_TELEOP_CONFIG:-}" ]]; then
    if [[ ! -f "$FA_XRT_TELEOP_CONFIG" ]]; then
      echo "未找到 FA_XRT_TELEOP_CONFIG: $FA_XRT_TELEOP_CONFIG"
      echo "请编辑 configs/local_xr.sh"
      exit 1
    fi
    extra+=(--config "$FA_XRT_TELEOP_CONFIG")
  fi
  if [[ $has_hand_config -eq 0 && -n "${FA_HAND_CONFIG:-}" ]]; then
    if [[ ! -f "$FA_HAND_CONFIG" ]]; then
      echo "未找到 FA_HAND_CONFIG: $FA_HAND_CONFIG"
      echo "请编辑 configs/local_xr.sh"
      exit 1
    fi
    extra+=(--hand-config "$FA_HAND_CONFIG")
  fi
  if [[ $has_hand_retargeting -eq 0 && -n "${FA_HAND_RETARGETING:-}" ]]; then
    extra+=(--hand-retargeting "$FA_HAND_RETARGETING")
  fi

  echo ">>> 启动 vr pose launch (XRoboToolkit)"
  echo ">>> 请确认: PC Service 已运行，Pico App 已连接"
  if [[ $# -gt 0 ]]; then
    echo ">>> 同一 PC Service 只能有一个 SDK 客户端；不要再另开 xr_hand_retarget"
    echo ">>> local_xr: teleop=$FA_XRT_TELEOP_CONFIG"
    echo ">>> local_xr: hand=${FA_HAND:-?} $FA_HAND_CONFIG"
    if [[ -n "${FA_HAND_RETARGETING:-}" ]]; then
      echo ">>> local_xr: retargeting override=$FA_HAND_RETARGETING"
    fi
    echo ">>> 参数: $*"
  fi
  python "$script_path" "${extra[@]}" "$@"
}

_ros2_topic_pub_once() {
  local topic="$1"
  local type="$2"
  local payload="$3"
  ensure_python_env
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "未找到 ros2。请先 source ROS2 / 工作空间，然后执行:"
    echo "  ros2 topic pub --once ${topic} ${type} \"${payload}\""
    exit 1
  fi
  echo ">>> ros2 topic pub --once ${topic} ${type} \"${payload}\""
  ros2 topic pub --once "${topic}" "${type}" "${payload}"
}

run_vr_update() {
  echo ">>> 临时 UPDATE：右摇杆 /teleop/controller_state=4（须已 OCS2，否则 VRInputHandler 忽略）"
  _ros2_topic_pub_once /teleop/controller_state std_msgs/msg/Int32 "{data: 4}"
}

run_vr_ocs2() {
  echo ">>> 临时 OCS2（等同脚踏板长按 F15）"
  _ros2_topic_pub_once /fsm_command std_msgs/msg/Int32 "{data: 3}"
}

# Apply FA_HAND → FA_HAND_CONFIG (same mapping as configs/local_xr.sh).
_apply_fa_hand() {
  local hand="${1:?}"
  case "${hand}" in
    xhand1|xhand) hand=xhand1; _FA_HAND_YAML="$ROOT_DIR/xr_hand_retarget/configs/xhand1.yaml" ;;
    wuji|wuji_hand2|hand2) hand=wuji; _FA_HAND_YAML="$ROOT_DIR/xr_hand_retarget/configs/wuji_hand2.yaml" ;;
    o6|linkerhand_o6|linker_o6) hand=o6; _FA_HAND_YAML="$ROOT_DIR/xr_hand_retarget/configs/o6.yaml" ;;
    l6|linkerhand_l6|linker_l6) hand=l6; _FA_HAND_YAML="$ROOT_DIR/xr_hand_retarget/configs/l6.yaml" ;;
    o7|linkerhand_o7|linker_o7) hand=o7; _FA_HAND_YAML="$ROOT_DIR/xr_hand_retarget/configs/o7.yaml" ;;
    *)
      echo "未知手型: ${hand}（应为 xhand1 | wuji | o6 | l6 | o7）"
      return 1
      ;;
  esac
  FA_HAND="$hand"
  FA_HAND_CONFIG="$_FA_HAND_YAML"
  FA_XHAND1_CONFIG="$FA_HAND_CONFIG"
}

# Persist last hand choice to configs/local.yaml (create from example if missing).
_save_local_hand_type() {
  local hand="$1"
  local f="$ROOT_DIR/configs/local.yaml"
  local ex="$ROOT_DIR/configs/local.yaml.example"
  if [[ ! -f "$f" ]]; then
    if [[ -f "$ex" ]]; then
      cp "$ex" "$f"
    else
      printf 'hands:\n  type: %s\n' "$hand" >"$f"
      return 0
    fi
  fi
  python3 - "$f" "$hand" <<'PY'
from pathlib import Path
import re, sys
path, hand = Path(sys.argv[1]), sys.argv[2]
text = path.read_text(encoding="utf-8")
new, n = re.subn(
    r"(?m)^([ \t]*type:\s*)([A-Za-z0-9_+-]+)",
    rf"\g<1>{hand}",
    text,
    count=1,
)
if n == 0:
    if re.search(r"(?m)^hands:\s*$", text):
        new = re.sub(r"(?m)^(hands:\s*)$", rf"\1\n  type: {hand}", text, count=1)
    else:
        new = text.rstrip() + f"\n\nhands:\n  type: {hand}\n"
path.write_text(new, encoding="utf-8")
PY
}

_prompt_hand_type() {
  local current="${FA_HAND:-o6}"
  case "${current}" in
    xhand1|xhand) current=xhand1 ;;
    wuji|wuji_hand2|hand2) current=wuji ;;
    o6|linkerhand_o6|linker_o6) current=o6 ;;
    l6|linkerhand_l6|linker_l6) current=l6 ;;
    o7|linkerhand_o7|linker_o7) current=o7 ;;
    *) current=o6 ;;
  esac
  local default_idx=3
  case "${current}" in
    xhand1) default_idx=1 ;;
    wuji) default_idx=2 ;;
    o6) default_idx=3 ;;
    l6) default_idx=4 ;;
    o7) default_idx=5 ;;
  esac

  echo ""
  echo "请选择灵巧手 (上次: ${current}):"
  echo "  1) xhand1"
  echo "  2) wuji"
  echo "  3) o6"
  echo "  4) l6"
  echo "  5) o7"
  echo ""
  local choice hand
  read -r -p "请输入选项 [1-5] (默认: ${default_idx}=${current}): " choice
  choice="${choice:-$default_idx}"
  case "${choice}" in
    1) hand=xhand1 ;;
    2) hand=wuji ;;
    3) hand=o6 ;;
    4) hand=l6 ;;
    5) hand=o7 ;;
    *) echo "无效选项"; exit 1 ;;
  esac
  _apply_fa_hand "$hand" || exit 1
  _save_local_hand_type "$hand"
  echo ">>> 已选手型: FA_HAND=${FA_HAND} → ${FA_HAND_CONFIG}"
}

_prompt_vr_xrt_preset() {
  echo ""
  echo "请选择 XRT 追踪预置:"
  echo "  1) controller — 手柄 6DoF 控臂 + 扳机夹爪"
  echo "  2) wrist      — 光学腕 6DoF 控臂 + 灵巧手（空手手势，关扳机）"
  echo ""
  local choice
  read -r -p "请输入选项 [1-2] (默认: 1): " choice
  choice="${choice:-1}"
  case "${choice}" in
    1) run_vr_xrt_launch controller ;;
    2)
      _prompt_hand_type
      run_vr_xrt_launch wrist
      ;;
    *) echo "无效选项"; exit 1 ;;
  esac
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
    3) _prompt_vr_xrt_preset ;;
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
      run_vr_xrt_launch "${@:2}"
      ;;
    vr-update)
      run_vr_update
      ;;
    vr-ocs2)
      run_vr_ocs2
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
