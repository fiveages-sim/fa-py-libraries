#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_NAME="fa-ros2"
DEFAULT_PYTHON_VERSION="3.12"
PYTHON_VERSION="${PYTHON_VERSION:-}"

print_usage() {
  echo "用法: $0 [submodules|conda [python版本]|install|pypi-mirror|all [python版本]]"
  echo
  echo "不带参数时进入交互菜单。"
  echo "未指定版本时默认使用 Python $DEFAULT_PYTHON_VERSION。"
}

init_submodules() {
  local submodule_paths=()
  local path_line

  echo ">>> 初始化子模块..."
  git -C "$ROOT_DIR" submodule update --init --recursive

  while IFS= read -r path_line; do
    submodule_paths+=("$path_line")
  done < <(git -C "$ROOT_DIR" config --file .gitmodules --get-regexp '^submodule\..*\.path$' | awk '{print $2}')

  echo ">>> 切换每个子模块到最新 main 分支..."
  for submodule_path in "${submodule_paths[@]}"; do
    local submodule_dir="$ROOT_DIR/$submodule_path"
    echo ">>> 处理子模块: $submodule_path"

    if ! git -C "$submodule_dir" rev-parse --git-dir >/dev/null 2>&1; then
      echo "    跳过：目录不是有效 Git 仓库"
      continue
    fi

    if ! git -C "$submodule_dir" show-ref --verify --quiet refs/remotes/origin/main; then
      echo "    跳过：未找到 origin/main"
      continue
    fi

    git -C "$submodule_dir" fetch origin main
    if git -C "$submodule_dir" show-ref --verify --quiet refs/heads/main; then
      git -C "$submodule_dir" checkout main
    else
      git -C "$submodule_dir" checkout -b main --track origin/main
    fi
    git -C "$submodule_dir" pull --ff-only origin main
  done

  echo ">>> 子模块初始化并切换 main 完成。"
}

resolve_python_version() {
  local input_version="${1:-}"
  if [[ -n "$input_version" ]]; then
    echo "$input_version"
  elif [[ -n "$PYTHON_VERSION" ]]; then
    echo "$PYTHON_VERSION"
  else
    echo "$DEFAULT_PYTHON_VERSION"
  fi
}

create_conda_env() {
  local selected_python_version
  selected_python_version="$(resolve_python_version "${1:-}")"

  echo ">>> 创建 conda 环境: $ENV_NAME (Python $selected_python_version)"

  if ! command -v conda >/dev/null 2>&1; then
    echo "未检测到 conda，请先安装并配置 conda。"
    exit 1
  fi

  if conda env list | awk '{print $1}' | grep -Fxq "$ENV_NAME"; then
    echo "环境 '$ENV_NAME' 已存在，跳过创建。"
    return 0
  fi

  conda create -n "$ENV_NAME" "python=$selected_python_version" -y
  echo ">>> conda 环境创建完成: $ENV_NAME"
}

install_projects() {
  local interface_dir="$ROOT_DIR/ros2_robot_interface"
  local viser_dir="$ROOT_DIR/ros2-viser"
  local vr_dir="$ROOT_DIR/vr_pose_publisher"

  if ! command -v conda >/dev/null 2>&1; then
    echo "未检测到 conda，请先安装并配置 conda。"
    exit 1
  fi

  if ! conda env list | awk '{print $1}' | grep -Fxq "$ENV_NAME"; then
    echo "环境 '$ENV_NAME' 不存在，请先创建 conda 环境。"
    exit 1
  fi

  for project_dir in "$interface_dir" "$viser_dir" "$vr_dir"; do
    if [[ ! -d "$project_dir" ]]; then
      echo "未找到目录: $project_dir"
      echo "请先执行子模块初始化。"
      exit 1
    fi
  done

  echo ">>> 激活 conda 环境并依次安装 interface -> viser -> vr"
  (
    eval "$(conda shell.bash hook)"
    conda activate "$ENV_NAME"
    python -m pip install -e "$interface_dir"
    python -m pip install -e "$viser_dir"
    python -m pip install -e "$vr_dir"
  )
  echo ">>> 安装完成。"
}

configure_nju_pypi_mirror() {
  local pip_config_dir="$HOME/.config/pip"
  local pip_config_file="$pip_config_dir/pip.conf"

  mkdir -p "$pip_config_dir"

  if [[ -f "$pip_config_file" ]]; then
    cp "$pip_config_file" "$pip_config_file.bak.$(date +%Y%m%d%H%M%S)"
    echo ">>> 已备份现有配置: $pip_config_file.bak.<timestamp>"
  fi

  cat > "$pip_config_file" <<'EOF'
[global]
index-url = https://mirrors.nju.edu.cn/pypi/web/simple
format = columns
EOF

  echo ">>> 已配置 PyPI 镜像为 NJU: https://mirrors.nju.edu.cn/pypi/web/simple"
  echo ">>> 配置文件: $pip_config_file"
}

run_all() {
  local python_version="${1:-}"
  init_submodules
  create_conda_env "$python_version"
  install_projects
}

main() {
  local python_version_arg="${2:-}"
  case "${1:-}" in
    submodules)
      init_submodules
      ;;
    conda)
      create_conda_env "$python_version_arg"
      ;;
    install)
      install_projects
      ;;
    pypi-mirror)
      configure_nju_pypi_mirror
      ;;
    all)
      run_all "$python_version_arg"
      ;;
    "")
      echo "请选择操作:"
      echo "  1) 初始化子模块"
      echo "  2) 创建 fa-ros2 conda 环境"
      echo "  3) 安装 interface / viser / vr"
      echo "  4) 全部执行"
      echo "  5) 配置 NJU PyPI 镜像"
      echo "  q) 退出"
      read -r -p "输入选项 [1/2/3/4/5/q]: " choice
      case "$choice" in
        1) init_submodules ;;
        2)
          read -r -p "输入 Python 版本（默认 $DEFAULT_PYTHON_VERSION）: " input_python_version
          create_conda_env "${input_python_version:-$DEFAULT_PYTHON_VERSION}"
          ;;
        3)
          install_projects
          ;;
        4)
          read -r -p "输入 Python 版本（默认 $DEFAULT_PYTHON_VERSION）: " input_python_version
          run_all "${input_python_version:-$DEFAULT_PYTHON_VERSION}"
          ;;
        5)
          configure_nju_pypi_mirror
          ;;
        q|Q) echo "已退出。" ;;
        *) echo "无效选项。"; exit 1 ;;
      esac
      ;;
    -h|--help|help)
      print_usage
      ;;
    *)
      echo "未知参数: $1"
      print_usage
      exit 1
      ;;
  esac
}

main "${1:-}" "${2:-}"
