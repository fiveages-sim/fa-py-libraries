#!/usr/bin/env bash
# fa-py-libraries 发布打包（借鉴 fa_w2_ws staging 思路）
# 在临时目录 rsync 后 zip，排除 dependencies/.venv/bags 等大目录，不修改工作区。
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIST_DIR="${ROOT_DIR}/dist"
DEFAULT_BASENAME="fa-py-libraries"

usage() {
  cat <<EOF
用法: $0 [选项]

维护者打包发布 zip（临时目录 staging，不修改当前工作区）。

选项:
  --package              打包 zip（含 .git；主仓 + 已检出于模块）
  --package-no-git       打包 zip（不含 .git，体积更小；推荐）
  -o, --output <path>    指定输出 zip 路径（默认 package-no-git 到 dist/）
  --skip-submodules      跳过打包前的 ./init.sh submodules
  -h, --help             显示帮助

无参数时进入交互菜单。

排除（始终）:
  .venv/  venv/  .idea/  .vscode/  dist/
  **/dependencies/   xr_bags/  joint_records/
  cert.pem  key.pem  *.pem
  __pycache__/  *.pyc  *.egg-info/
  .fa-env.local.toml

包含:
  init.sh  run.sh  release.sh  scripts/  .fa-env.toml  README.md  .gitmodules
  ros2_robot_interface/  ros2-viser/  vr_pose_publisher/（不含 dependencies/）

现场解压后:
  ./init.sh install
  # 可选 XRoboToolkit: ./init.sh install-xrobotoolkit-pc-service && ./init.sh install-xrobotoolkit
EOF
}

need_cmd() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "缺少命令: $cmd"
    return 1
  fi
  return 0
}

update_submodules() {
  if ! git -C "$ROOT_DIR" rev-parse --git-dir >/dev/null 2>&1; then
    echo ">>> 当前目录不是 Git 仓库，跳过子模块更新。"
    return 0
  fi
  if [[ ! -x "${ROOT_DIR}/init.sh" ]]; then
    echo "未找到可执行的 init.sh，无法更新子模块。"
    exit 1
  fi
  echo ">>> 更新子模块到 origin/main 最新提交..."
  "$ROOT_DIR/init.sh" submodules
}

resolve_output_path() {
  local custom_output="${1:-}"
  local nogit_tag="${2:-0}"
  local suffix=""

  if [[ -n "$custom_output" ]]; then
    if [[ "$custom_output" != /* ]]; then
      custom_output="${ROOT_DIR}/${custom_output}"
    fi
    if [[ "$custom_output" != *.zip ]]; then
      custom_output="${custom_output}.zip"
    fi
    echo "$custom_output"
    return
  fi

  mkdir -p "$DIST_DIR"
  if [[ "$nogit_tag" == "1" ]]; then
    suffix="_nogit"
  fi
  echo "${DIST_DIR}/${DEFAULT_BASENAME}-$(date +%Y%m%d-%H%M%S)${suffix}.zip"
}

# Build rsync exclude args. $1 = 1 to also exclude .git
build_rsync_excludes() {
  local exclude_git="${1:-0}"
  RSYNC_EXCLUDES=(
    --exclude='.venv/'
    --exclude='venv/'
    --exclude='.idea/'
    --exclude='.vscode/'
    --exclude='dist/'
    --exclude='dependencies/'
    --exclude='xr_bags/'
    --exclude='joint_records/'
    --exclude='.fa-env.local.toml'
    --exclude='cert.pem'
    --exclude='key.pem'
    --exclude='*.pem'
    --exclude='__pycache__/'
    --exclude='*.pyc'
    --exclude='*.egg-info/'
    --exclude='*.zip'
  )
  if [[ "$exclude_git" == "1" ]]; then
    # 目录 .git/ 与子模块 gitlink 文件 .git 都排除
    RSYNC_EXCLUDES+=(--exclude='.git' --exclude='.git/')
  fi
}

prepare_staging() {
  local include_git="${1:-0}"
  local staging exclude_git=1

  need_cmd rsync || return 1

  if [[ "$include_git" == "1" ]]; then
    exclude_git=0
  fi
  build_rsync_excludes "$exclude_git"

  staging="$(mktemp -d "${TMPDIR:-/tmp}/${DEFAULT_BASENAME}_release.XXXXXX")" || return 1
  echo ">>> 复制到临时目录: $staging" >&2
  rsync -a "${RSYNC_EXCLUDES[@]}" "${ROOT_DIR}/" "${staging}/" || {
    rm -rf "$staging"
    return 1
  }
  printf '%s' "$staging"
}

cleanup_staging() {
  local staging="${1:-}"
  if [[ -n "$staging" && -d "$staging" ]]; then
    echo ">>> 清理临时目录: $staging"
    rm -rf "$staging"
  fi
}

create_release_zip() {
  local staging="$1"
  local archive_path="$2"
  local include_git="${3:-0}"
  local -a zip_excludes=(
    "*.zip"
  )

  need_cmd zip || return 1
  mkdir -p "$(dirname "$archive_path")"
  if [[ -f "$archive_path" ]]; then
    echo "输出文件已存在: $archive_path"
    return 1
  fi

  if [[ "$include_git" != "1" ]]; then
    zip_excludes+=(".git" ".git/*" "*/.git" "*/.git/*")
  fi

  echo ">>> 打包目录: $staging"
  echo ">>> 输出文件: $archive_path"
  if [[ "$include_git" == "1" ]]; then
    echo ">>> 模式: 含 .git"
  else
    echo ">>> 模式: 不含 .git"
  fi

  (
    cd "$staging" || exit 1
    zip -r -q "$archive_path" . -x "${zip_excludes[@]}"
  ) || return 1

  echo ">>> 完成: $archive_path ($(du -h "$archive_path" | cut -f1))"
}

do_package() {
  local include_git="${1:-0}"
  local skip_submodules="${2:-0}"
  local output_path="${3:-}"
  local staging=""
  local archive_path

  package_cleanup() {
    cleanup_staging "$staging"
    staging=""
  }

  if [[ "$skip_submodules" -eq 0 ]]; then
    update_submodules
  else
    echo ">>> 已跳过子模块更新。"
  fi

  archive_path="$(resolve_output_path "$output_path" "$([[ "$include_git" == "1" ]] && echo 0 || echo 1)")"
  staging="$(prepare_staging "$include_git")" || return 1
  trap package_cleanup EXIT

  create_release_zip "$staging" "$archive_path" "$include_git" || return 1

  trap - EXIT
  package_cleanup
  return 0
}

interactive_menu() {
  local choice
  echo ""
  echo "fa-py-libraries 发布打包"
  echo "  1) package-no-git（推荐，体积更小）"
  echo "  2) package（含 .git）"
  echo "  0) 取消"
  echo ""
  read -r -p "请输入选项 [0-2]: " choice
  case "$choice" in
    1) do_package 0 0 "" ;;
    2) do_package 1 0 "" ;;
    0|"") echo "已取消。" ;;
    *) echo "无效选项。"; exit 1 ;;
  esac
}

main() {
  local include_git=""
  local skip_submodules=0
  local output_path=""
  local mode=""

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --package)
        mode="package"
        include_git=1
        shift
        ;;
      --package-no-git)
        mode="package-no-git"
        include_git=0
        shift
        ;;
      -o|--output)
        [[ $# -ge 2 ]] || { echo "缺少 -o 参数值"; usage; exit 1; }
        output_path="$2"
        # 兼容旧用法：仅 -o 时按 no-git 打包
        if [[ -z "$mode" ]]; then
          mode="package-no-git"
          include_git=0
        fi
        shift 2
        ;;
      --skip-submodules)
        skip_submodules=1
        shift
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

  case "$mode" in
    package|package-no-git)
      do_package "$include_git" "$skip_submodules" "$output_path"
      ;;
    "")
      interactive_menu
      ;;
  esac
}

main "$@"
