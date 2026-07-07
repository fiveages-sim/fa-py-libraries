#!/usr/bin/env bash
# VR 遥操 ROS2 录包/回放：录制并回放 /xr/* 话题（位姿、按键、摇杆、扳机）

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=scripts/fa-env.sh
source "$ROOT_DIR/scripts/fa-env.sh"
FA_ENV_ROOT_DIR="$ROOT_DIR"
fa_env_load_config "$ROOT_DIR"
fa_env_try_source_ros2

XR_BAG_TOPICS=(
  /xr/head_pose
  /xr/left_ee_pose
  /xr/right_ee_pose
  /xr/controller_state
  /xr/thumbstick_axes
  /xr/trigger_values
)
XR_BAG_DIR="${XR_BAG_DIR:-$ROOT_DIR/xr_bags}"
RECORD_NODE_NAME="${RECORD_NODE_NAME:-rosbag2_recorder}"

RECORD_PID=""
RECORD_LOG=""
RECORD_BAG_PATH=""
XR_STUB_PID=""
XR_BAG_PLAY_PID=""
XR_PLAYBACK_ABORT=false

xr_bag_start_xr_stub() {
  local stub_script="$ROOT_DIR/scripts/xr-target-node-stub.py"
  if [[ ! -f "$stub_script" ]]; then
    echo "  ✗ 未找到虚拟节点脚本: $stub_script"
    exit 1
  fi
  if ! python3 -c "import rclpy" 2>/dev/null; then
    echo "  ✗ 未找到 rclpy，请先 source ROS2 工作空间"
    exit 1
  fi
  python3 "$stub_script" </dev/null >/dev/null 2>&1 &
  XR_STUB_PID=$!
  sleep 0.5
  if ! kill -0 "$XR_STUB_PID" 2>/dev/null; then
    echo "  ✗ 虚拟 xr_target_node 启动失败"
    XR_STUB_PID=""
    exit 1
  fi
  echo "  ✓ 已启动虚拟 xr_target_node (pid=$XR_STUB_PID)"
  sleep 0.5
  if ros2 node list 2>/dev/null | grep -qxF '/xr_target_node'; then
    echo "  ✓ 已确认 ROS 图中存在 /xr_target_node"
  else
    echo "  ⚠ 未在 ROS 图中看到 /xr_target_node，VRInputHandler 可能仍被禁用"
  fi
}

xr_bag_stop_xr_stub() {
  if [[ -n "$XR_STUB_PID" ]] && kill -0 "$XR_STUB_PID" 2>/dev/null; then
    kill -TERM "$XR_STUB_PID" 2>/dev/null || true
    wait "$XR_STUB_PID" 2>/dev/null || true
  fi
  XR_STUB_PID=""
}

xr_bag_has_tty() {
  [[ -r /dev/tty ]]
}

xr_bag_info_duration() {
  local bag_path="$1"
  local info
  info="$(ros2 bag info "$bag_path" 2>/dev/null || true)"
  grep -E '^Duration:' <<<"$info" | sed 's/^Duration:[[:space:]]*//' | awk '{print $1}'
}

xr_bag_prompt_playback_options() {
  local count_set="$1"
  local rate_set="$2"
  local -n _count=$3
  local -n _rate=$4
  local reply

  if ! xr_bag_has_tty; then
    return 0
  fi

  if [[ "$count_set" != true ]]; then
    reply="$(xr_bag_read_line "回放次数（默认 ${_count}）: ")"
    if [[ -n "$reply" ]]; then
      _count="$reply"
    fi
  fi
  if [[ "$rate_set" != true ]]; then
    reply="$(xr_bag_read_line "回放速率（默认 ${_rate}）: ")"
    if [[ -n "$reply" ]]; then
      _rate="$reply"
    fi
  fi
}

xr_bag_print_tty() {
  local line
  for line in "$@"; do
    if xr_bag_has_tty; then
      printf '%s\n' "$line" >/dev/tty
    else
      printf '%s\n' "$line"
    fi
  done
}

xr_bag_read_line() {
  local prompt="${1:-}"
  local reply=""
  if xr_bag_has_tty; then
    [[ -n "$prompt" ]] && printf '%s' "$prompt" >/dev/tty
    IFS= read -r reply </dev/tty
  else
    IFS= read -r -p "$prompt" reply
  fi
  printf '%s' "$reply"
}

xr_bag_confirm() {
  local prompt="$1"
  local reply
  reply="$(xr_bag_read_line "$prompt")"
  [[ "$reply" =~ ^[Yy]$ ]]
}

usage() {
  echo "用法: $0 <record|playback|clean> [选项]"
  echo
  echo "子命令:"
  echo "  record    录制 VR 遥操 /xr/* 话题到 ros2 bag"
  echo "  playback  回放 bag，模拟 VR 输入"
  echo "  clean     清理已录制的 bag"
  echo
  echo "record 选项:"
  echo "  --name NAME   会话名称（可选，用于目录命名）"
  echo
  echo "playback 选项:"
  echo "  --file DIR    指定 bag 目录（留空则交互选择）"
  echo "  --rate RATE   回放速率（默认 1.0）"
  echo "  --count N     回放次数（默认 1）"
  echo "  --no-stub     不启动虚拟 xr_target_node（默认会自动启动）"
  echo
  echo "clean 选项:"
  echo "  --all         删除全部 bag（需确认）"
  echo "  --file DIR    删除指定 bag 目录（可多次指定）"
  echo
  echo "环境变量:"
  echo "  XR_BAG_DIR    bag 保存目录（默认: \$ROOT_DIR/xr_bags）"
  echo
  echo "示例:"
  echo "  $0 record --name grasp_demo"
  echo "  $0 playback"
  echo "  $0 playback --file $XR_BAG_DIR/grasp_demo_20260707_150930 --rate 1.0 --count 3"
  echo "  $0 clean"
  echo "  $0 clean --all"
  echo "  $0 clean --file $XR_BAG_DIR/grasp_demo_20260707_150930"
}

cleanup_record() {
  xr_bag_stop_record "${RECORD_BAG_PATH:-}"
}

on_record_interrupt() {
  cleanup_record
  echo
  echo "录制已中断。"
  exit 130
}

xr_bag_stop_playback() {
  local pid="$XR_BAG_PLAY_PID"
  local i

  if [[ -z "$pid" ]] || ! kill -0 "$pid" 2>/dev/null; then
    XR_BAG_PLAY_PID=""
    return 0
  fi

  # 尽快结束发布，避免 rosbag2 优雅退出期间仍向 /xr/* 发消息
  kill -INT "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  for i in $(seq 1 5); do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.1
  done

  if kill -0 "$pid" 2>/dev/null; then
    kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    for i in $(seq 1 5); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
  fi

  if kill -0 "$pid" 2>/dev/null; then
    kill -KILL "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
  XR_BAG_PLAY_PID=""
}

xr_bag_playback_cleanup() {
  xr_bag_stop_playback
  if [[ -n "$XR_STUB_PID" ]] && kill -0 "$XR_STUB_PID" 2>/dev/null; then
    xr_bag_stop_xr_stub
  fi
}

on_playback_interrupt() {
  if $XR_PLAYBACK_ABORT; then
    exit 130
  fi
  XR_PLAYBACK_ABORT=true
  echo
  echo ">>> 正在停止回放..."
  xr_bag_stop_playback
  if [[ -n "$XR_STUB_PID" ]] && kill -0 "$XR_STUB_PID" 2>/dev/null; then
    xr_bag_stop_xr_stub
    echo "  ✓ 已停止虚拟 xr_target_node"
  fi
  echo ">>> 回放已中断"
  trap - EXIT INT TERM
  exit 130
}

xr_bag_start_record() {
  local bag_path="$1"
  local log_file="$2"

  setsid ros2 bag record -o "$bag_path" --topics "${XR_BAG_TOPICS[@]}" \
    --disable-keyboard-controls \
    </dev/null >>"$log_file" 2>&1 &
  RECORD_PID=$!
  RECORD_BAG_PATH="$bag_path"
  sleep 1.0

  if ! kill -0 "$RECORD_PID" 2>/dev/null; then
    echo "  ✗ ros2 bag record 启动失败"
    if [[ -s "$log_file" ]]; then
      echo "  最近日志:"
      tail -n 20 "$log_file" | sed 's/^/    /'
    fi
    RECORD_PID=""
    RECORD_BAG_PATH=""
    exit 1
  fi
}

xr_bag_call_stop_service() {
  ros2 service call "/${RECORD_NODE_NAME}/stop" rosbag2_interfaces/srv/Stop "{}" 2>/dev/null
}

xr_bag_try_reindex() {
  local bag_path="$1"

  if [[ -f "$bag_path/metadata.yaml" ]]; then
    return 0
  fi
  if [[ -z "$(find "$bag_path" -maxdepth 1 -name '*.mcap' -print -quit 2>/dev/null)" ]]; then
    return 1
  fi

  echo "  尝试重建 metadata.yaml (ros2 bag reindex)..."
  if ros2 bag reindex "$bag_path" >/dev/null 2>&1; then
    echo "  ✓ metadata.yaml 已重建"
    return 0
  fi
  return 1
}

xr_bag_stop_record() {
  local bag_path="${1:-}"
  local pid="$RECORD_PID"
  local i

  if [[ -z "$pid" ]]; then
    return 0
  fi

  if kill -0 "$pid" 2>/dev/null; then
    echo "正在停止录制..."
    if xr_bag_call_stop_service >/dev/null; then
      :
    else
      echo "  ⚠ stop 服务不可用，尝试 SIGINT..."
      kill -INT "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
    fi

    for i in $(seq 1 50); do
      if ! kill -0 "$pid" 2>/dev/null; then
        break
      fi
      if [[ -n "$bag_path" && -f "$bag_path/metadata.yaml" ]]; then
        break
      fi
      sleep 0.2
    done

    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
      for i in $(seq 1 15); do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.2
      done
      wait "$pid" 2>/dev/null || true
    else
      wait "$pid" 2>/dev/null || true
    fi
  fi

  if [[ -n "$bag_path" && ! -f "$bag_path/metadata.yaml" ]]; then
    xr_bag_try_reindex "$bag_path" || true
  fi

  RECORD_PID=""
  RECORD_BAG_PATH=""
}

xr_bag_wait_stop_recording() {
  local bag_path="$1"

  echo
  echo "录制中...（ros2 bag 日志: $RECORD_LOG）"
  if xr_bag_has_tty; then
    printf '\n>>> 按 Enter 停止录制（或 Ctrl+C）\n' >/dev/tty
    IFS= read -r _ </dev/tty
  else
    echo "  ⚠ 当前无交互终端，请手动停止: ros2 service call /${RECORD_NODE_NAME}/stop rosbag2_interfaces/srv/Stop {}"
    wait "$RECORD_PID" 2>/dev/null || true
    RECORD_PID=""
    RECORD_BAG_PATH=""
    return
  fi
  xr_bag_stop_record "$bag_path"
}

xr_bag_ensure_dir() {
  mkdir -p "$XR_BAG_DIR"
}

xr_bag_sanitize_name() {
  local name="$1"
  name="${name// /_}"
  name="$(echo "$name" | tr -cd '[:alnum:]_-')"
  echo "$name"
}

xr_bag_check_topics() {
  local topic_list
  local missing=()
  local required=(/xr/left_ee_pose /xr/right_ee_pose)

  if ! command -v ros2 >/dev/null 2>&1; then
    echo "  ✗ 未找到 ros2 命令，请先 source ROS2 环境"
    return 1
  fi

  topic_list="$(ros2 topic list 2>/dev/null || true)"
  if [[ -z "$topic_list" ]]; then
    echo "  ⚠ 无法获取话题列表（ROS2 守护进程可能未运行）"
    return 1
  fi

  for topic in "${required[@]}"; do
    if ! grep -qxF "$topic" <<<"$topic_list"; then
      missing+=("$topic")
    fi
  done

  if ((${#missing[@]} > 0)); then
    echo "  ⚠ 以下关键话题当前不可见:"
    for topic in "${missing[@]}"; do
      echo "      - $topic"
    done
    echo "  提示: 请先在另一终端运行 ./run.sh vr"
    if ! xr_bag_confirm "是否仍继续录制? [y/N]: "; then
      echo "已取消录制。"
      exit 0
    fi
  else
    echo "  ✓ 关键 VR 话题已就绪"
  fi
}

xr_bag_list_dirs() {
  local dir
  if [[ ! -d "$XR_BAG_DIR" ]]; then
    return 0
  fi
  while IFS= read -r dir; do
    if [[ -f "$dir/metadata.yaml" ]]; then
      echo "$dir"
    fi
  done < <(
    find "$XR_BAG_DIR" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' 2>/dev/null \
      | sort -rn \
      | cut -d' ' -f2-
  )
}

xr_bag_info_summary() {
  local bag_path="$1"
  local info
  info="$(ros2 bag info "$bag_path" 2>/dev/null || true)"
  if [[ -z "$info" ]]; then
    echo "（无法读取 bag 信息）"
    return
  fi
  grep -E '^(Duration|Messages|Files):' <<<"$info" | tr '\n' ' ' | sed 's/ $//'
}

xr_bag_is_valid_dir() {
  local bag_path="$1"
  [[ -d "$bag_path" && -f "$bag_path/metadata.yaml" ]]
}

xr_bag_is_cleanable_dir() {
  local bag_path="$1"
  [[ -d "$bag_path" ]] || return 1
  if [[ -f "$bag_path/metadata.yaml" ]]; then
    return 0
  fi
  [[ -n "$(find "$bag_path" -maxdepth 1 -name '*.mcap' -print -quit 2>/dev/null)" ]]
}

xr_bag_list_cleanable_dirs() {
  local dir
  if [[ ! -d "$XR_BAG_DIR" ]]; then
    return 0
  fi
  while IFS= read -r dir; do
    if xr_bag_is_cleanable_dir "$dir"; then
      echo "$dir"
    fi
  done < <(
    find "$XR_BAG_DIR" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' 2>/dev/null \
      | sort -rn \
      | cut -d' ' -f2-
  )
}

xr_bag_resolve_path() {
  local input="$1"
  if [[ "$input" != /* ]]; then
    input="$XR_BAG_DIR/$input"
  fi
  if [[ -d "$input" ]]; then
    (cd "$input" && pwd)
  else
    echo "$input"
  fi
}

xr_bag_is_under_root() {
  local bag_path="$1"
  local bag_root
  bag_root="$(cd "$XR_BAG_DIR" && pwd)"
  [[ "$bag_path" == "$bag_root"/* ]]
}

xr_bag_load_dirs() {
  local -n _out=$1
  mapfile -t _out < <(xr_bag_list_dirs)
}

xr_bag_load_cleanable_dirs() {
  local -n _out=$1
  mapfile -t _out < <(xr_bag_list_cleanable_dirs)
}

xr_bag_dir_status() {
  local bag_path="$1"
  local mcap_count size

  if [[ -f "$bag_path/metadata.yaml" ]]; then
    xr_bag_info_summary "$bag_path"
    return
  fi
  if [[ -n "$(find "$bag_path" -maxdepth 1 -name '*.mcap' -print -quit 2>/dev/null)" ]]; then
    mcap_count="$(find "$bag_path" -maxdepth 1 -name '*.mcap' 2>/dev/null | wc -l)"
    size="$(du -sh "$bag_path" 2>/dev/null | awk '{print $1}')"
    echo "状态: 不完整（录制中断，缺少 metadata.yaml）| mcap: ${mcap_count} | 大小: ${size:-未知}"
    return
  fi
  echo "状态: 无法识别"
}

xr_bag_print_list() {
  local -a bags=("$@")
  local bag count idx status_line session_line

  xr_bag_print_tty "" "找到 ${#bags[@]} 个 bag:" \
    "----------------------------------------------------------------------"
  for idx in "${!bags[@]}"; do
    bag="${bags[$idx]}"
    count=$((idx + 1))
    status_line="$(xr_bag_dir_status "$bag")"
    session_line=""
    if [[ -f "$bag/session_info.txt" ]]; then
      session_line="$(grep -E '^session_name=|^recorded_at=' "$bag/session_info.txt" 2>/dev/null | tr '\n' ' ')"
    fi
    xr_bag_print_tty \
      "  [$count] $(basename "$bag")" \
      "      路径: $bag" \
      "      $status_line"
    if [[ -n "$session_line" ]]; then
      xr_bag_print_tty "      $session_line"
    fi
  done
  xr_bag_print_tty "----------------------------------------------------------------------"
}

xr_bag_delete_dir() {
  local bag_path="$1"
  bag_path="$(xr_bag_resolve_path "$bag_path")"

  if ! xr_bag_is_under_root "$bag_path"; then
    echo "  ✗ 拒绝删除：路径不在 $XR_BAG_DIR 内"
    return 1
  fi
  if ! xr_bag_is_cleanable_dir "$bag_path"; then
    echo "  ✗ 不是可清理的 bag 目录: $bag_path"
    return 1
  fi

  rm -rf "$bag_path"
  echo "  ✓ 已删除: $(basename "$bag_path")"
}

xr_bag_delete_dirs() {
  local -a targets=("$@")
  local bag_path

  for bag_path in "${targets[@]}"; do
    xr_bag_delete_dir "$bag_path"
  done
}

xr_bag_select_dir() {
  local -n _selected=$1
  local -a bags=()
  local choice

  xr_bag_load_dirs bags

  if ((${#bags[@]} == 0)); then
    echo "  ✗ 在 $XR_BAG_DIR 中未找到有效的 bag 目录"
    echo "  请先使用 record 模式录制，或检查 XR_BAG_DIR 环境变量"
    exit 1
  fi

  xr_bag_print_list "${bags[@]}"

  while true; do
    choice="$(xr_bag_read_line "请选择要回放的 bag (1-${#bags[@]})，或输入 q 退出: ")"
    if [[ "$choice" =~ ^[Qq]$ ]]; then
      xr_bag_print_tty "已退出。"
      exit 0
    fi
    if [[ "$choice" =~ ^[0-9]+$ ]] && ((choice >= 1 && choice <= ${#bags[@]})); then
      _selected="${bags[$((choice - 1))]}"
      return 0
    fi
    xr_bag_print_tty "  ⚠ 无效选择，请输入 1-${#bags[@]} 或 q"
  done
}

xr_bag_parse_selection() {
  local selection="$1"
  local -n _bags=$2
  local -n _out=$3
  local -a parts=()
  local part idx n

  if [[ "$selection" =~ ^[Aa][Ll][Ll]$ ]]; then
    _out=("${_bags[@]}")
    return 0
  fi

  IFS=',' read -r -a parts <<<"$selection"
  for part in "${parts[@]}"; do
    part="${part//[[:space:]]/}"
    [[ -z "$part" ]] && continue
    if [[ ! "$part" =~ ^[0-9]+$ ]]; then
      echo "  ⚠ 无效编号: $part"
      return 1
    fi
    n=$((10#$part))
    if ((n < 1 || n > ${#_bags[@]})); then
      echo "  ⚠ 编号超出范围: $part"
      return 1
    fi
  done

  for part in "${parts[@]}"; do
    part="${part//[[:space:]]/}"
    [[ -z "$part" ]] && continue
    idx=$((10#$part - 1))
    _out+=("${_bags[$idx]}")
  done
}

xr_bag_write_session_info() {
  local bag_path="$1"
  local session_name="$2"
  local recorded_at="$3"
  local info_file="$bag_path/session_info.txt"
  local topic

  {
    echo "session_name=$session_name"
    echo "recorded_at=$recorded_at"
    echo "bag_path=$bag_path"
    echo -n "topics="
    local first=1
    for topic in "${XR_BAG_TOPICS[@]}"; do
      if ((first)); then
        echo -n "$topic"
        first=0
      else
        echo -n ",$topic"
      fi
    done
    echo
  } >"$info_file"
}

cmd_record() {
  local session_name=""
  local bag_name bag_path recorded_at

  trap on_record_interrupt INT TERM

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --name)
        session_name="${2:-}"
        shift 2
        ;;
      -h|--help|help)
        usage
        exit 0
        ;;
      *)
        echo "未知参数: $1"
        usage
        exit 1
        ;;
    esac
  done

  echo "======================================================================"
  echo "                    VR 遥操录包模式"
  echo "======================================================================"
  xr_bag_ensure_dir
  echo
  echo "保存目录: $XR_BAG_DIR"
  echo "录制话题: ${XR_BAG_TOPICS[*]}"
  echo
  echo "操作说明:"
  echo "  - 请确保另一终端已运行 ./run.sh vr"
  echo "  - 安全建议：开始录制前将机器人置于 HOLD 状态"
  echo "  - 按 Enter 开始录制"
  echo "  - 录制开始后，按 Enter 停止（需等待「正在停止录制...」）"
  echo "  - 停止录制后建议再次切回 HOLD 状态"
  echo "  - Ctrl+C 也可中断"
  echo

  if [[ -z "$session_name" ]]; then
    session_name="$(xr_bag_read_line "会话名称（可选，直接 Enter 跳过）: ")"
  fi
  session_name="$(xr_bag_sanitize_name "$session_name")"

  xr_bag_read_line "按 Enter 开始录制..." >/dev/null
  echo
  echo "检查 VR 话题..."
  xr_bag_check_topics

  recorded_at="$(date '+%Y-%m-%d %H:%M:%S')"
  if [[ -n "$session_name" ]]; then
    bag_name="${session_name}_$(date +%Y%m%d_%H%M%S)"
  else
    bag_name="$(date +%Y%m%d_%H%M%S)"
  fi
  bag_path="$XR_BAG_DIR/$bag_name"
  RECORD_LOG="$(mktemp -t vr-bag-record.XXXXXX.log)"

  echo
  echo ">>> 开始录制 -> $bag_path"
  xr_bag_start_record "$bag_path" "$RECORD_LOG"
  xr_bag_wait_stop_recording "$bag_path"
  echo
  trap - INT TERM

  if [[ ! -f "$bag_path/metadata.yaml" ]]; then
    echo "  ✗ 录制未正常结束（缺少 metadata.yaml）"
    if xr_bag_is_cleanable_dir "$bag_path"; then
      echo "  ⚠ 已生成不完整 bag，可用 ./run.sh vr-bag-clean 清理后重试"
    fi
    if [[ -s "$RECORD_LOG" ]]; then
      echo "  最近日志:"
      tail -n 20 "$RECORD_LOG" | sed 's/^/    /'
    fi
    exit 1
  fi

  xr_bag_write_session_info "$bag_path" "$session_name" "$recorded_at"
  echo "  ✓ 录制已保存: $bag_path"
  echo
  ros2 bag info "$bag_path"
}

cmd_playback() {
  local bag_path=""
  local rate="1.0"
  local count=1
  local use_stub=true
  local count_set=false
  local rate_set=false
  local i duration

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --file)
        bag_path="${2:-}"
        shift 2
        ;;
      --rate)
        rate="${2:-1.0}"
        rate_set=true
        shift 2
        ;;
      --count)
        count="${2:-1}"
        count_set=true
        shift 2
        ;;
      --no-stub)
        use_stub=false
        shift
        ;;
      -h|--help|help)
        usage
        exit 0
        ;;
      *)
        if [[ -z "$bag_path" && -d "$1" ]]; then
          bag_path="$1"
          shift
        else
          echo "未知参数: $1"
          usage
          exit 1
        fi
        ;;
    esac
  done

  echo "======================================================================"
  echo "                    VR 遥操回放模式"
  echo "======================================================================"
  xr_bag_ensure_dir
  echo
  echo "保存目录: $XR_BAG_DIR"
  echo
  echo "注意:"
  echo "  - 回放时会向 /xr/* 话题发布数据，请勿同时运行真实 VR（./run.sh vr）"
  echo "  - 默认自动启动虚拟 xr_target_node，让 VRInputHandler 保持启用"
  echo "  - 安全建议：回放前将机器人置于 HOLD；回放结束后再切回 HOLD"
  echo "  - 请确保 arms_target_manager 与机器人控制栈已运行"
  echo

  if [[ -z "$bag_path" ]]; then
    xr_bag_select_dir bag_path
  fi

  if [[ ! -d "$bag_path" ]]; then
    echo "  ✗ bag 目录不存在: $bag_path"
    exit 1
  fi
  if [[ ! -f "$bag_path/metadata.yaml" ]]; then
    echo "  ✗ 不是有效的 ros2 bag 目录（缺少 metadata.yaml）: $bag_path"
    exit 1
  fi

  xr_bag_prompt_playback_options "$count_set" "$rate_set" count rate

  if [[ ! "$count" =~ ^[0-9]+$ ]] || ((count < 1)); then
    echo "  ✗ 回放次数须为正整数: $count"
    exit 1
  fi

  duration="$(xr_bag_info_duration "$bag_path")"
  echo
  echo ">>> 回放: $(basename "$bag_path")"
  echo "    路径: $bag_path"
  echo "    速率: $rate | 次数: $count"
  if [[ -n "$duration" ]]; then
    echo "    单次时长: 约 ${duration}"
  fi
  echo
  if $use_stub; then
    echo ">>> 启动虚拟 xr_target_node（供 VRInputHandler 检测）..."
    xr_bag_start_xr_stub
    trap on_playback_interrupt INT TERM
    trap xr_bag_playback_cleanup EXIT
  else
    trap on_playback_interrupt INT TERM
  fi
  for ((i = 1; i <= count; i++)); do
    if $XR_PLAYBACK_ABORT; then
      break
    fi
    if ((count > 1)); then
      echo
      echo ">>> 第 ${i}/${count} 次回放"
    fi
    if [[ -n "$duration" ]]; then
      echo "    正在回放...（约 ${duration}，Ctrl+C 终止全部回放）"
    else
      echo "    正在回放...（Ctrl+C 终止全部回放）"
    fi
    setsid ros2 bag play "$bag_path" --rate "$rate" --disable-keyboard-controls &
    XR_BAG_PLAY_PID=$!
    play_rc=0
    wait "$XR_BAG_PLAY_PID" || play_rc=$?
    XR_BAG_PLAY_PID=""
    if $XR_PLAYBACK_ABORT || ((play_rc == 130 || play_rc == 143)); then
      on_playback_interrupt
    fi
    if ((play_rc != 0)); then
      echo "  ✗ 第 ${i} 次回放失败 (exit=$play_rc)"
      xr_bag_playback_cleanup
      trap - EXIT INT TERM
      exit "$play_rc"
    fi
    echo "  ✓ 第 ${i} 次回放完成"
  done
  if $use_stub; then
    xr_bag_stop_xr_stub
    trap - EXIT INT TERM
    echo "  ✓ 已停止虚拟 xr_target_node"
  else
    trap - INT TERM
  fi
  echo
  echo ">>> 回放全部完成"
}

cmd_clean() {
  local delete_all=false
  local -a explicit_targets=()
  local -a bags=()
  local -a to_delete=()
  local selection

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --all)
        delete_all=true
        shift
        ;;
      --file)
        explicit_targets+=("${2:-}")
        shift 2
        ;;
      -h|--help|help)
        usage
        exit 0
        ;;
      *)
        if [[ -d "$1" ]]; then
          explicit_targets+=("$1")
          shift
        else
          echo "未知参数: $1"
          usage
          exit 1
        fi
        ;;
    esac
  done

  echo "======================================================================"
  echo "                    VR 遥操 bag 清理"
  echo "======================================================================"
  xr_bag_ensure_dir
  echo
  echo "保存目录: $XR_BAG_DIR"
  echo

  if ((${#explicit_targets[@]} > 0)); then
    echo "将删除以下 bag:"
    for bag_path in "${explicit_targets[@]}"; do
      echo "  - $bag_path"
    done
    if ! xr_bag_confirm "确认删除? [y/N]: "; then
      echo "已取消。"
      exit 0
    fi
    to_delete=("${explicit_targets[@]}")
  elif $delete_all; then
    xr_bag_load_cleanable_dirs bags
    if ((${#bags[@]} == 0)); then
      echo "  ✓ 没有可清理的 bag"
      exit 0
    fi
    xr_bag_print_list "${bags[@]}"
    if ! xr_bag_confirm "确认删除全部 ${#bags[@]} 个 bag? [y/N]: "; then
      echo "已取消。"
      exit 0
    fi
    xr_bag_delete_dirs "${bags[@]}"
    echo
    echo "  ✓ 已清理全部 bag"
    exit 0
  else
    xr_bag_load_cleanable_dirs bags
    if ((${#bags[@]} == 0)); then
      echo "  ✓ 没有可清理的 bag"
      exit 0
    fi

    xr_bag_print_list "${bags[@]}"
    echo
    echo "操作说明:"
    echo "  - 输入编号删除单个或多个（如 1 或 1,3）"
    echo "  - 输入 all 删除全部"
    echo "  - 输入 q 退出"
    echo

    while true; do
      selection="$(xr_bag_read_line "请选择要删除的 bag: ")"
      if [[ "$selection" =~ ^[Qq]$ ]]; then
        echo "已退出。"
        exit 0
      fi
      to_delete=()
      if xr_bag_parse_selection "$selection" bags to_delete; then
        ((${#to_delete[@]} > 0)) && break
        echo "  ⚠ 未选择任何 bag"
      fi
    done

    echo
    echo "将删除以下 ${#to_delete[@]} 个 bag:"
    for bag_path in "${to_delete[@]}"; do
      echo "  - $(basename "$bag_path")"
    done
    if ! xr_bag_confirm "确认删除? [y/N]: "; then
      echo "已取消。"
      exit 0
    fi
  fi

  xr_bag_delete_dirs "${to_delete[@]}"
  echo
  echo "  ✓ 清理完成"
}

main() {
  local cmd="${1:-}"
  shift || true

  case "$cmd" in
    record)
      cmd_record "$@"
      ;;
    playback)
      cmd_playback "$@"
      ;;
    clean)
      cmd_clean "$@"
      ;;
    -h|--help|help|"")
      usage
      [[ -z "$cmd" ]] && exit 1
      ;;
    *)
      echo "未知子命令: $cmd"
      usage
      exit 1
      ;;
  esac
}

main "$@"
