#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 在 source 前保留环境变量覆盖（source fa-env.sh 会重置 FA_ENV_BACKEND）
FA_ENV_BACKEND_OVERRIDE="${FA_ENV_BACKEND:-}"
# shellcheck source=scripts/fa-env.sh
source "${ROOT_DIR}/scripts/fa-env.sh"
FA_ENV_ROOT_DIR="$ROOT_DIR"

ENV_NAME="fa-ros2"
DEFAULT_PYTHON_VERSION="3.12"
INSTALL_BACKEND_OVERRIDE=""
PYTHON_VERSION="${PYTHON_VERSION:-}"

print_usage() {
  echo "用法: $0 [submodules|update-submodules-main|env [python版本]|install [--conda|--uv]|"
  echo "      install-xrobotoolkit [--conda|--uv]|install-xrobotoolkit-pc-service|"
  echo "      set-backend conda|uv|install-uv|install-miniconda|"
  echo "      pypi-mirror|uv-mirror|ros2-workspace [--all]|all [python版本]]"
  echo
  echo "环境:"
  echo "  env            按 .fa-env.toml 的 backend 创建环境"
  echo "  install        按 .fa-env.toml 的 backend 安装；可用 --conda / --uv 临时指定"
  echo "  install-xrobotoolkit-pc-service  安装官方 XRoboToolkit-PC-Service deb（按 arch / Ubuntu 版本下载）"
  echo "  install-xrobotoolkit  安装 PC Service（若未装）+ PyPI xrobotoolkit-sdk>=1.1.0"
  echo "  set-backend    修改 .fa-env.toml 中 backend（run.sh 读取）"
  echo "  install-uv     安装 uv 包管理器（https://astral.sh/uv/）"
  echo "  install-miniconda  安装 Miniconda 到 ~/miniconda3 并关闭 base 自动激活"
  echo "  pypi-mirror    配置 pip 使用 NJU PyPI 镜像"
  echo "  uv-mirror      配置 uv 使用清华 PyPI 镜像（backend=uv 时生效）"
  echo "  ros2-workspace 按 backend 写对应 activate 挂钩；--all 则 conda+uv 都写"
  echo "  配置见 .fa-env.toml；个人覆盖: .fa-env.local.toml；临时: FA_ENV_BACKEND=uv"
  echo
  echo "子模块（见 .gitmodules）:"
  echo "  ros2_robot_interface  ros2-viser  vr_pose_publisher  xr_hand_retarget"
  echo
  echo "不带参数时进入交互菜单。"
  echo "未指定版本时默认使用 Python $DEFAULT_PYTHON_VERSION。"
}

init_submodules() {
  local submodule_paths=()
  local path_line

  echo ">>> 初始化子模块..."
  git -C "$ROOT_DIR" submodule sync --recursive
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

submodules_have_content() {
  local submodule_paths=()
  local submodule_path submodule_dir

  while IFS= read -r submodule_path; do
    submodule_paths+=("$submodule_path")
  done < <(git -C "$ROOT_DIR" config --file .gitmodules --get-regexp '^submodule\..*\.path$' | awk '{print $2}')

  if [[ ${#submodule_paths[@]} -eq 0 ]]; then
    return 1
  fi

  for submodule_path in "${submodule_paths[@]}"; do
    submodule_dir="$ROOT_DIR/$submodule_path"
    if [[ ! -d "$submodule_dir" ]]; then
      return 1
    fi
    if ! git -C "$submodule_dir" rev-parse --git-dir >/dev/null 2>&1; then
      return 1
    fi
    if [[ -z "$(ls -A "$submodule_dir" 2>/dev/null)" ]]; then
      return 1
    fi
  done

  return 0
}

update_submodules_to_latest_main() {
  local submodule_paths=()
  local submodule_path

  echo ">>> 更新所有子模块到最新 main..."
  git -C "$ROOT_DIR" submodule update --init --recursive

  while IFS= read -r submodule_path; do
    submodule_paths+=("$submodule_path")
  done < <(git -C "$ROOT_DIR" config --file .gitmodules --get-regexp '^submodule\..*\.path$' | awk '{print $2}')

  for submodule_path in "${submodule_paths[@]}"; do
    local submodule_dir="$ROOT_DIR/$submodule_path"
    echo ">>> 处理子模块: $submodule_path"

    if ! git -C "$submodule_dir" rev-parse --git-dir >/dev/null 2>&1; then
      echo "    跳过：目录不是有效 Git 仓库"
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

  echo ">>> 所有子模块已更新到最新 main。"
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
  local env_name
  selected_python_version="$(resolve_python_version "${1:-}")"
  fa_env_load_config "$ROOT_DIR"
  env_name="$FA_ENV_CONDA_NAME"

  echo ">>> 创建 conda 环境: $env_name (Python $selected_python_version)"

  if ! command -v conda >/dev/null 2>&1; then
    echo "未检测到 conda，请先安装并配置 conda。"
    exit 1
  fi

  if conda env list | awk '{print $1}' | grep -Fxq "$env_name"; then
    echo "环境 '$env_name' 已存在，跳过创建。"
    return 0
  fi

  conda create -n "$env_name" "python=$selected_python_version" -y
  echo ">>> conda 环境创建完成: $env_name"
}

create_uv_env() {
  local selected_python_version
  local venv_path
  selected_python_version="$(resolve_python_version "${1:-}")"
  fa_env_load_config "$ROOT_DIR"
  venv_path="$(fa_env_uv_venv_path)"

  echo ">>> 创建 uv 虚拟环境: $venv_path (Python $selected_python_version)"

  if ! command -v uv >/dev/null 2>&1; then
    echo "未检测到 uv，请先安装: https://docs.astral.sh/uv/"
    exit 1
  fi

  if [[ -f "${venv_path}/bin/activate" ]]; then
    echo "虚拟环境已存在: $venv_path，跳过创建。"
    return 0
  fi

  # 优先系统解释器 + --system-site-packages（与 ROS2 /opt/ros 的 Python 对齐）
  fa_env_create_uv_venv "$selected_python_version" "$venv_path"
  echo ">>> uv 虚拟环境创建完成: $venv_path（--system-site-packages）"
  if [[ -n "${FA_ENV_ROS2_WORKSPACE:-}" ]]; then
    fa_env_write_uv_ros2_hook "$venv_path" "$FA_ENV_ROS2_WORKSPACE"
    echo ">>> 已根据 .fa-env.toml 为 venv 写入 ROS2 activate 挂钩。"
  else
    echo ">>> 提示: 执行 ./init.sh ros2-workspace 后，手动 source .venv/bin/activate 也会自动 source ROS2。"
  fi
}

parse_install_backend_args() {
  INSTALL_BACKEND_OVERRIDE=""
  local arg
  for arg in "$@"; do
    case "$arg" in
      --conda) INSTALL_BACKEND_OVERRIDE="conda" ;;
      --uv) INSTALL_BACKEND_OVERRIDE="uv" ;;
      *)
        echo "未知参数: $arg"
        print_usage
        exit 1
        ;;
    esac
  done
}

install_projects() {
  local interface_dir="$ROOT_DIR/ros2_robot_interface"
  local viser_dir="$ROOT_DIR/ros2-viser"
  local vr_dir="$ROOT_DIR/vr_pose_publisher"
  local hand_dir="$ROOT_DIR/xr_hand_retarget"

  fa_env_load_config "$ROOT_DIR"
  if [[ -n "$INSTALL_BACKEND_OVERRIDE" ]]; then
    FA_ENV_BACKEND="$INSTALL_BACKEND_OVERRIDE"
  fi

  for project_dir in "$interface_dir" "$viser_dir" "$vr_dir" "$hand_dir"; do
    if [[ ! -d "$project_dir" ]]; then
      echo "未找到目录: $project_dir"
      echo "请先执行子模块初始化。"
      exit 1
    fi
  done

  echo ">>> 使用 backend=$FA_ENV_BACKEND，依次安装 interface -> viser -> vr -> hand"
  (
    set +u
    fa_env_activate "$ROOT_DIR"
    fa_env_install_editable_projects "$interface_dir" "$viser_dir" "$vr_dir" "$hand_dir"
  )

  if [[ -f "$vr_dir/cert.pem" && -f "$vr_dir/key.pem" ]]; then
    echo ">>> vr_pose_publisher 证书已存在，跳过生成。"
  else
    echo ">>> 为 vr_pose_publisher 生成 SSL 证书（全部使用回车默认）..."
    (cd "$vr_dir" && printf '\n\n\n\n\n\n\n' | openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout key.pem -out cert.pem)
    echo ">>> 证书已生成: $vr_dir/cert.pem, $vr_dir/key.pem"
  fi
  echo ">>> 安装完成。"
}

# Official PC Service debs from https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases
# See also org overview: https://github.com/XR-Robotics
XRT_PCS_RELEASE_BASE="https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0"

detect_ubuntu_version_id() {
  if command -v lsb_release >/dev/null 2>&1; then
    lsb_release -rs
    return 0
  fi
  if [[ -f /etc/os-release ]]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    echo "${VERSION_ID:-}"
    return 0
  fi
  echo ""
}

xrobotoolkit_pc_service_installed() {
  [[ -x /opt/apps/roboticsservice/runService.sh ]] && return 0
  dpkg -l 2>/dev/null | grep -qiE 'xrobotoolkit.*pc.?service|roboticsservice' && return 0
  return 1
}

install_xrobotoolkit_pc_service() {
  local vr_dir="$ROOT_DIR/vr_pose_publisher"
  local deb_dir="$vr_dir/dependencies/debs"
  local os_ver deb_name deb_url deb_path arch force=0
  local arg

  for arg in "$@"; do
    case "$arg" in
      --force) force=1 ;;
      *)
        echo "未知参数: $arg（支持 --force）"
        exit 1
        ;;
    esac
  done

  if [[ ! -d "$vr_dir" ]]; then
    echo "未找到目录: $vr_dir"
    echo "请先执行子模块初始化。"
    exit 1
  fi

  if [[ "$force" -eq 0 ]] && xrobotoolkit_pc_service_installed; then
  echo ">>> 检测到 XRoboToolkit PC Service 已安装（/opt/apps/roboticsservice 或 dpkg），跳过。"
  echo ">>> 启动（推荐）: 应用菜单打开 XRoboToolkit-PC-Service"
  echo ">>> 命令行备选: /opt/apps/roboticsservice/runService.sh"
  echo ">>> 强制重装: $0 install-xrobotoolkit-pc-service --force"
    return 0
  fi

  arch="$(dpkg --print-architecture)"
  case "$arch" in
    arm64)
      # 官方 arm64 deb 不区分 Ubuntu 版本，v1.0.0 下同时提供桌面版与 headless 版。
      # 默认桌面版；无显示环境的控制器可用 XRT_PCS_HEADLESS=1 切换。
      if [[ "${XRT_PCS_HEADLESS:-0}" == "1" ]]; then
        deb_name="XRoboToolkit-PC-Service-headless_1.0.0.0_arm64.deb"
      else
        deb_name="XRoboToolkit-PC-Service_1.0.0.0_arm64.deb"
      fi
      ;;
    amd64)
      os_ver="$(detect_ubuntu_version_id)"
      case "$os_ver" in
        22.04)
          deb_name="XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb"
          ;;
        24.04)
          deb_name="XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb"
          ;;
        *)
          echo ">>> 当前系统版本: ${os_ver:-unknown}"
          echo ">>> 官方 amd64 deb 仅提供 Ubuntu 22.04 / 24.04（见 XR-Robotics releases）。"
          read -r -p "输入要下载的版本 [22.04/24.04]（默认 24.04）: " os_ver
          os_ver="${os_ver:-24.04}"
          case "$os_ver" in
            22.04) deb_name="XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb" ;;
            24.04) deb_name="XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb" ;;
            *)
              echo "不支持的版本: $os_ver"
              exit 1
              ;;
          esac
          ;;
      esac
      ;;
    *)
      echo "不支持的架构: $arch（官方仅提供 amd64 / arm64）"
      exit 1
      ;;
  esac

  deb_url="${XRT_PCS_RELEASE_BASE}/${deb_name}"
  deb_path="${deb_dir}/${deb_name}"
  mkdir -p "$deb_dir"

  echo ">>> 安装 XRoboToolkit-PC-Service（官方 deb）"
  echo ">>> 参考: https://github.com/XR-Robotics"
  echo ">>> Release: https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/tag/v1.0.0"
  echo ">>> 包: $deb_name"
  echo ">>> 保存到: $deb_path（相对 vr_pose_publisher）"

  if [[ ! -f "$deb_path" ]]; then
    if command -v wget >/dev/null 2>&1; then
      wget -O "$deb_path" "$deb_url"
    elif command -v curl >/dev/null 2>&1; then
      curl -fL -o "$deb_path" "$deb_url"
    else
      echo "需要 wget 或 curl 下载 deb。"
      exit 1
    fi
  else
    echo ">>> 已存在 deb 文件，跳过下载。"
  fi

  # 用 apt-get 装本地 deb，才能解析并安装依赖；dpkg -i 不会拉依赖。
  # 路径必须是绝对路径或 ./ 开头，否则 apt 会当成仓库包名。
  local apt_args=(-y)
  if [[ "$force" -eq 1 ]]; then
    apt_args+=(--reinstall)
  fi
  echo ">>> 执行: sudo apt-get install ${apt_args[*]} $deb_path"
  sudo apt-get install "${apt_args[@]}" "$deb_path"

  echo ">>> PC Service 安装完成。"
  echo ">>> 启动（推荐）: 应用菜单打开 XRoboToolkit-PC-Service"
  echo ">>> 命令行备选: /opt/apps/roboticsservice/runService.sh"
  echo ">>> 头显端需安装 XRoboToolkit APK（adb install -g ...），见 README。"
}

install_xrobotoolkit() {
  local vr_dir="$ROOT_DIR/vr_pose_publisher"
  local setup_script="$vr_dir/setup_xrobotoolkit.sh"

  fa_env_load_config "$ROOT_DIR"
  if [[ -n "$INSTALL_BACKEND_OVERRIDE" ]]; then
    FA_ENV_BACKEND="$INSTALL_BACKEND_OVERRIDE"
  fi

  if [[ ! -d "$vr_dir" ]]; then
    echo "未找到目录: $vr_dir"
    echo "请先执行子模块初始化与 ./init.sh install。"
    exit 1
  fi
  if [[ ! -f "$setup_script" ]]; then
    echo "未找到脚本: $setup_script"
    exit 1
  fi

  # PC Service is required at runtime; install deb first when missing.
  install_xrobotoolkit_pc_service

  echo ">>> 使用 backend=$FA_ENV_BACKEND 安装 XRoboToolkit Python SDK（PyPI xrobotoolkit-sdk）"
  echo ">>> 安装脚本: $setup_script"

  (
    set +u
    fa_env_activate "$ROOT_DIR"
    bash "$setup_script"
  )
}

configure_ros2_workspace_source() {
  local ws_input ws_stored apply_all=0
  local arg
  fa_env_load_config "$ROOT_DIR"

  for arg in "$@"; do
    case "$arg" in
      --all) apply_all=1 ;;
      *)
        echo "未知参数: $arg"
        print_usage
        exit 1
        ;;
    esac
  done

  read -r -p "输入 ROS2 工作空间路径（默认 ~/ros2_ws）: " ws_input
  ws_input="${ws_input:-~/ros2_ws}"
  ws_stored="${ws_input/#$HOME/\~}"

  fa_env_set_ros2_workspace "$ws_stored"
  echo ">>> 已写入 .fa-env.toml [ros2].workspace = $ws_stored"

  if [[ "$apply_all" -eq 1 ]]; then
    echo ">>> 为 conda 与 uv 同时写入 activate 挂钩..."
    fa_env_apply_ros2_hooks "$ws_stored" 1
  else
    echo ">>> 按 backend=$FA_ENV_BACKEND 写入 activate 挂钩..."
    fa_env_apply_ros2_hooks "$ws_stored" 0
  fi

  case "$FA_ENV_BACKEND" in
    uv)
      echo ">>> source .venv/bin/activate 时会自动 source ROS2。"
      ;;
    conda)
      echo ">>> conda activate ${FA_ENV_CONDA_NAME} 时会自动 source ROS2。"
      ;;
  esac
  echo ">>> run.sh 也会读取同一配置。"
}

install_miniconda() {
  local miniconda_dir="${HOME}/miniconda3"
  local installer="${miniconda_dir}/miniconda.sh"
  local conda_bin="${miniconda_dir}/bin/conda"

  if [[ -x "$conda_bin" ]]; then
    echo ">>> Miniconda 已安装: $miniconda_dir ($("$conda_bin" --version))"
  elif command -v conda >/dev/null 2>&1; then
    echo ">>> 已检测到 conda: $(command -v conda)（非 ~/miniconda3），跳过 Miniconda 安装。"
    return 0
  else
    echo ">>> 安装 Miniconda 到 $miniconda_dir ..."

    if ! command -v wget >/dev/null 2>&1; then
      echo "未检测到 wget，请先安装 wget。"
      exit 1
    fi

    local mc_arch
    case "$(uname -m)" in
      x86_64)  mc_arch="x86_64" ;;
      aarch64) mc_arch="aarch64" ;;
      *)
        echo "不支持的架构: $(uname -m)（Miniconda 仅提供 x86_64 / aarch64）"
        exit 1
        ;;
    esac

    mkdir -p "$miniconda_dir"
    wget "https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-${mc_arch}.sh" -O "$installer"
    bash "$installer" -b -u -p "$miniconda_dir"
    rm -f "$installer"
    echo ">>> Miniconda 安装完成: $miniconda_dir"
  fi

  export PATH="${miniconda_dir}/bin:${PATH}"

  echo ">>> 配置 conda：初始化 shell 并关闭 base 自动激活..."
  # shellcheck disable=SC1091
  source "${miniconda_dir}/bin/activate"
  conda init --all
  conda config --set auto_activate_base False
  echo ">>> conda 配置完成（auto_activate_base=False）。"
  echo ">>> 若当前 shell 找不到 conda，请重新打开终端。"
}

install_uv() {
  if command -v uv >/dev/null 2>&1; then
    echo ">>> uv 已安装: $(command -v uv) ($(uv --version))"
    return 0
  fi

  echo ">>> 安装 uv..."
  curl -LsSf https://astral.sh/uv/install.sh | sh

  # 安装脚本默认写入 ~/.local/bin
  if [[ -d "${HOME}/.local/bin" ]]; then
    export PATH="${HOME}/.local/bin:${PATH}"
  fi

  if command -v uv >/dev/null 2>&1; then
    echo ">>> uv 安装完成: $(command -v uv) ($(uv --version))"
  else
    echo ">>> uv 安装脚本已执行，但未在当前 PATH 中找到 uv。"
    echo ">>> 请重新打开终端，或将 ~/.local/bin 加入 PATH。"
    exit 1
  fi
}

configure_uv_mirror() {
  local uv_config_dir="$HOME/.config/uv"
  local uv_config_file="$uv_config_dir/uv.toml"

  mkdir -p "$uv_config_dir"

  if [[ -f "$uv_config_file" ]]; then
    cp "$uv_config_file" "$uv_config_file.bak.$(date +%Y%m%d%H%M%S)"
    echo ">>> 已备份现有配置: $uv_config_file.bak.<timestamp>"
  fi

  cat > "$uv_config_file" <<'EOF'
[[index]]
url = "https://pypi.tuna.tsinghua.edu.cn/simple"
default = true
EOF

  echo ">>> 已配置 uv PyPI 镜像为清华: https://pypi.tuna.tsinghua.edu.cn/simple"
  echo ">>> 配置文件: $uv_config_file"
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

create_env_for_configured_backend() {
  local python_version="${1:-}"
  fa_env_load_config "$ROOT_DIR"
  case "$FA_ENV_BACKEND" in
    uv) create_uv_env "$python_version" ;;
    conda) create_conda_env "$python_version" ;;
    *)
      echo "未知 backend: $FA_ENV_BACKEND"
      exit 1
      ;;
  esac
}

run_all() {
  local python_version="${1:-}"
  if submodules_have_content; then
    echo ">>> 子模块已存在内容，跳过初始化。"
  else
    init_submodules
  fi
  create_env_for_configured_backend "$python_version"
  install_projects
}

interactive_menu() {
  fa_env_load_config "$ROOT_DIR"
  echo "请选择要执行的操作 (backend=$FA_ENV_BACKEND)"
  echo
  echo "  [子模块]"
  echo "    1) 初始化子模块"
  echo "    2) 更新所有子模块到最新 main"
  echo
  echo "  [环境与安装]"
  echo "    3) 按当前 backend 创建环境"
  echo "    4) 安装 interface / viser / vr / hand"
  echo "    5) 安装 XRoboToolkit PC Service（官方 deb）"
  echo "    6) 安装 XRoboToolkit SDK（VR XRT 后端，含 PC Service 检测）"
  echo
  echo "  [一键执行]"
  echo "    7) 全部执行（子模块 + 环境 + 安装）"
  echo
  echo "  [配置]"
  if [[ "$FA_ENV_BACKEND" == "conda" ]]; then
    echo "    8) 安装 Miniconda"
    echo "    9) 配置 NJU PyPI 镜像（pip）"
  else
    echo "    8) 安装 uv"
    echo "    9) 配置 uv PyPI 镜像（清华）"
  fi
  echo "   10) 配置 ROS2 工作空间"
  echo "   11) 切换 backend (conda/uv)"
  echo
  echo "  [其他]"
  echo "    q) 退出"
  echo
  read -r -p "输入选项 [1-11/q]: " choice

  case "$choice" in
    1)
      init_submodules
      ;;
    2)
      update_submodules_to_latest_main
      ;;
    3)
      read -r -p "输入 Python 版本（默认 $DEFAULT_PYTHON_VERSION）: " input_python_version
      create_env_for_configured_backend "${input_python_version:-$DEFAULT_PYTHON_VERSION}"
      ;;
    4)
      install_projects
      ;;
    5)
      install_xrobotoolkit_pc_service
      ;;
    6)
      install_xrobotoolkit
      ;;
    7)
      read -r -p "输入 Python 版本（默认 $DEFAULT_PYTHON_VERSION）: " input_python_version
      run_all "${input_python_version:-$DEFAULT_PYTHON_VERSION}"
      ;;
    8)
      if [[ "$FA_ENV_BACKEND" == "conda" ]]; then
        install_miniconda
      else
        install_uv
      fi
      ;;
    9)
      if [[ "$FA_ENV_BACKEND" == "conda" ]]; then
        configure_nju_pypi_mirror
      else
        configure_uv_mirror
      fi
      ;;
    10)
      configure_ros2_workspace_source
      ;;
    11)
      read -r -p "输入 backend [conda/uv]（当前 $FA_ENV_BACKEND）: " backend_choice
      backend_choice="${backend_choice:-$FA_ENV_BACKEND}"
      fa_env_set_backend "$backend_choice"
      ;;
    q|Q) echo "已退出。" ;;
    *) echo "无效选项。"; exit 1 ;;
  esac
}

main() {
  local cmd="${1:-}"
  shift || true
  case "$cmd" in
    submodules)
      init_submodules
      ;;
    update-submodules-main)
      update_submodules_to_latest_main
      ;;
    env)
      create_env_for_configured_backend "${1:-}"
      ;;
    set-backend)
      if [[ -z "${1:-}" ]]; then
        echo "用法: $0 set-backend conda|uv"
        exit 1
      fi
      fa_env_set_backend "$1"
      ;;
    install)
      parse_install_backend_args "$@"
      install_projects
      ;;
    install-xrobotoolkit)
      parse_install_backend_args "$@"
      install_xrobotoolkit
      ;;
    install-xrobotoolkit-pc-service)
      install_xrobotoolkit_pc_service "$@"
      ;;
    install-uv)
      install_uv
      ;;
    install-miniconda)
      install_miniconda
      ;;
    pypi-mirror)
      configure_nju_pypi_mirror
      ;;
    uv-mirror)
      configure_uv_mirror
      ;;
    ros2-workspace)
      configure_ros2_workspace_source "$@"
      ;;
    all)
      if [[ -n "${2:-}" ]]; then
        echo "未知参数: $2"
        print_usage
        exit 1
      fi
      run_all "${1:-}"
      ;;
    "")
      interactive_menu
      ;;
    -h|--help|help)
      print_usage
      ;;
    *)
      echo "未知参数: ${cmd}"
      print_usage
      exit 1
      ;;
  esac
}

main "$@"
