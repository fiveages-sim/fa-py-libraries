#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIST_DIR="${ROOT_DIR}/dist"
DEFAULT_BASENAME="fa-py-libraries"

usage() {
  echo "用法: $0 [-o 输出路径.zip] [--skip-submodules] [-h|--help]"
  echo
  echo "将仓库根目录下除 venv / .idea 外的文件打包为 zip。"
  echo "默认输出: dist/${DEFAULT_BASENAME}-<时间戳>.zip"
  echo
  echo "打包前会执行 ./init.sh submodules，将三个子模块更新到 origin/main 最新提交。"
  echo "  --skip-submodules  跳过子模块更新（离线或沿用当前检出时使用）"
  echo
  echo "排除目录:"
  echo "  .idea/  .venv/  venv/"
}

update_submodules() {
  if ! git -C "$ROOT_DIR" rev-parse --git-dir >/dev/null 2>&1; then
    echo "当前目录不是 Git 仓库，跳过子模块更新。"
    return
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
  echo "${DIST_DIR}/${DEFAULT_BASENAME}-$(date +%Y%m%d-%H%M%S).zip"
}

create_release_archive() {
  local archive_path="$1"

  if ! command -v zip >/dev/null 2>&1; then
    echo "未找到 zip 命令，请先安装: sudo apt install zip"
    exit 1
  fi

  mkdir -p "$(dirname "$archive_path")"
  if [[ -f "$archive_path" ]]; then
    echo "输出文件已存在: $archive_path"
    exit 1
  fi

  echo ">>> 打包目录: $ROOT_DIR"
  echo ">>> 输出文件: $archive_path"
  echo ">>> 排除: .idea/  .venv/  venv/  dist/"

  (
    cd "$ROOT_DIR"
    zip -r -q "$archive_path" . \
      -x ".idea/*" \
      -x "*/.idea/*" \
      -x ".venv/*" \
      -x "*/.venv/*" \
      -x "venv/*" \
      -x "*/venv/*" \
      -x "dist/*"
  )

  echo ">>> 完成: $archive_path ($(du -h "$archive_path" | cut -f1))"
}

main() {
  local output_path=""
  local skip_submodules=0

  while [[ $# -gt 0 ]]; do
    case "$1" in
      -o|--output)
        [[ $# -ge 2 ]] || { echo "缺少 -o 参数值"; usage; exit 1; }
        output_path="$2"
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

  if [[ "$skip_submodules" -eq 0 ]]; then
    update_submodules
  else
    echo ">>> 已跳过子模块更新。"
  fi

  create_release_archive "$(resolve_output_path "$output_path")"
}

main "$@"
