#!/usr/bin/env bash
# =============================================================================
#  Dual_UR_Demo 初始化脚本（init_repo.sh）
#
#  功能：
#    1. 选择环境（assembly_exp / lab_bench / space_sim）
#    2. 拉取子模块（1. 仅当前环境所需  2. 全部）
#    3. 更新子模块（1. 仅当前环境已拉取  2. 全部已拉取）
#    4. 安装依赖（rosdep 安装当前工作区所有包的依赖 + OCS2 三个 deb 包）
#
#  用法：./init_repo.sh
#  说明：环境选择会保存到 ${HOME}/.config/dual_ur_demo/env，quick_start.sh 共用。
# =============================================================================

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}" || { echo "无法进入脚本所在目录"; exit 1; }

# ============================== 常量配置区 ==============================
# （版本号、仓库、环境映射等集中在此，便于修改）

# ---- 支持的环境（与 src/envs/ 下的目录名一致） ----
SUPPORTED_ENVS=(assembly_exp lab_bench space_sim)
ENV_DEFAULT="lab_bench"

# ---- 环境记忆文件（quick_start.sh 共用，不在仓库内） ----
ENV_PREF_FILE="${HOME}/.config/dual_ur_demo/env"

# ---- ROS 发行版（未 source ROS 环境时按此发行版查找） ----
ROS_DISTRO_DEFAULT="jazzy"

# ---- 环境 → 子模块映射（对应 .gitmodules 中的 path） ----
# lab_bench / assembly_exp：UR 双臂 + Robotiq 2F-85 / DH-AG95 夹爪 + MuJoCo 仿真
# space_sim：UR 双臂（无夹爪）+ MuJoCo 仿真 + 场景/对接仿真插件
submodules_for_env() {
  case "$1" in
    assembly_exp | lab_bench)
      echo "lib/ros2_robotiq_gripper lib/dh_ag95_ros2 lib/serial lib/mujoco_ros2_control lib/mujoco_vendor"
      ;;
    space_sim)
      echo "lib/mujoco_ros2_control lib/mujoco_vendor lib/mujoco_ros2_control_plugins/mujoco_god_plugin lib/mujoco_ros2_control_plugins/mujoco_dock_sim_plugin"
      ;;
    *) echo "" ;;
  esac
}

# ---- OCS2 适配的三个 deb 包（固定版本通道 conf 使用下面 tag） ----
# 安装顺序（勿调整）：ocs2 → robot-descriptions-common → arms
# deb 文件名规则：<包前缀>_<版本号(去掉 v 前缀)>_<架构>.deb
# 版本更新：在 GitHub Releases 页面查看最新 tag 后修改；
#           latest / pre-release 通道运行时通过 GitHub API 自动解析。
#
# 1) OCS2 核心库（ocs2_core / ocs2_mpc / ocs2_ros_interfaces 等）
DEB_OCS2_PREFIX="ros-jazzy-ocs2"
DEB_OCS2_REPO="legubiao/ocs2_ros2"
DEB_OCS2_TAG="v1.4.0"
# 2) 机器人描述公共库（被 arms_ros2_control 依赖）
DEB_COMMON_PREFIX="ros-jazzy-robot-descriptions-common"
DEB_COMMON_REPO="fiveages-sim/robot-descriptions-common"
DEB_COMMON_TAG="v1.6.0"
# 3) 双臂控制栈（ocs2_arm_controller / adaptive_gripper_controller /
#    arms_target_manager / arms_rviz_control_plugin 等）
DEB_ARMS_PREFIX="ros-jazzy-arms-ros2-control"
DEB_ARMS_REPO="fiveages-sim/arms_ros2_control"
DEB_ARMS_TAG="v1.6.0"
# arms deb 变体：standard=双臂控制栈；full=额外含 ht_ros2_control 硬件驱动
DEB_ARMS_VARIANT="standard"

# ---- 额外系统包（无法通过 package.xml + rosdep 表达的依赖） ----
# 以下依赖均已写入工作区各 package.xml，由 rosdep 自动安装，无需在此硬编码：
#   - UR 系列：ur_description / ur_robot_driver（*_description 包）
#              ur_controllers / ur_client_library（launch_controller）
#   - ros2_control / ros2_controllers：controller_manager、joint_state_broadcaster、
#     joint_trajectory_controller、position_controllers、velocity_controllers、
#     effort_controllers、pose_broadcaster、parallel_gripper_controller、
#     force_torque_sensor_broadcaster（launch_controller、*_moveit_config、*_mujoco_description）
#   - MoveIt 系列：moveit_ros_move_group、moveit_kinematics、moveit_servo、
#     moveit_ros_control_interface 等（*_moveit_config、launch_controller）
#   - RealSense：realsense2_description（*_description 包）、
#     realsense2_camera（lab_bench_description，librealsense2 由其依赖自动拉取）
EXTRA_APT_PKGS=(
  # 仅保留无法用 rosdep key 表达的包：
  # librealsense2-utils 提供 rs-enumerate-devices 相机探测工具（无 rosdep 规则）
  "ros-${ROS_DISTRO:-${ROS_DISTRO_DEFAULT}}-librealsense2-utils"
)

# ============================== 公共函数 ==============================

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_info() { echo -e "${GREEN}[INFO]${NC} $*"; }
print_warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
print_error() { echo -e "${RED}[ERROR]${NC} $*"; }
print_hr() { echo -e "${BLUE}========================================${NC}"; }

# 菜单选择：$1=提示 $2=默认序号 $3..=选项；所选序号输出到 stdout
# 传入 --exit 时追加显示 "0) 退出"，且输入 0 有效
menu_select() {
  local show_exit=0
  if [ "${1:-}" = "--exit" ]; then
    show_exit=1
    shift
  fi
  local prompt="$1" default="$2"
  shift 2
  local -a opts=("$@")
  local n="${#opts[@]}" i choice
  echo "" >&2
  for ((i = 0; i < n; i++)); do
    if [ $((i + 1)) -eq "${default}" ]; then
      echo " *$((i + 1))) ${opts[i]}" >&2
    else
      echo "  $((i + 1))) ${opts[i]}" >&2
    fi
  done
  if [ "${show_exit}" -eq 1 ]; then
    echo "  0) 退出" >&2
    read -rp "${prompt} [0-${n}]（回车=默认 ${default}，0=退出）: " choice || choice=""
    if [ "${choice}" = "0" ]; then
      printf '0'
      return
    fi
  else
    read -rp "${prompt} [1-${n}]（回车=默认 ${default}）: " choice || choice=""
  fi
  if ! [[ "${choice}" =~ ^[0-9]+$ ]] || [ "${choice}" -lt 1 ] || [ "${choice}" -gt "${n}" ]; then
    choice="${default}"
  fi
  printf '%s' "${choice}"
}

confirm_yn() {
  local ans
  read -rp "$1 [y/N]: " ans || ans=""
  case "${ans}" in
    y | Y | yes | YES) return 0 ;;
    *) return 1 ;;
  esac
}

# 加载/保存环境选择
load_env_pref() {
  local e=""
  if [ -f "${ENV_PREF_FILE}" ]; then
    e="$(tr -d '[:space:]' < "${ENV_PREF_FILE}" 2>/dev/null || true)"
  fi
  local s
  for s in "${SUPPORTED_ENVS[@]}"; do
    [ "${e}" = "${s}" ] && {
      printf '%s' "${e}"
      return
    }
  done
  printf '%s' "${ENV_DEFAULT}"
}

save_env_pref() {
  mkdir -p "$(dirname "${ENV_PREF_FILE}")"
  printf '%s\n' "$1" > "${ENV_PREF_FILE}"
}

ensure_ros_env() {
  local distro="${ROS_DISTRO:-${ROS_DISTRO_DEFAULT}}"
  if [ ! -f "/opt/ros/${distro}/setup.bash" ]; then
    print_error "未找到 /opt/ros/${distro}/setup.bash，请确认 ROS 已安装"
    return 1
  fi
  # ROS 的 setup.bash 在 set -u 下会引用未定义变量，source 期间临时关闭
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${distro}/setup.bash"
  set -u
  return 0
}

# ============================== 功能 1：选择环境 ==============================

flow_select_env() {
  local cur
  cur="$(load_env_pref)"
  local c
  c="$(menu_select "请选择环境" 1 \
    "assembly_exp  （装配实验）" \
    "lab_bench     （实验台：UR5 + UR5e + 夹爪）" \
    "space_sim     （太空仿真，无夹爪）")"
  local env
  env="${SUPPORTED_ENVS[$((c - 1))]}"
  save_env_pref "${env}"
  if [ "${env}" != "${cur}" ]; then
    print_info "环境已切换: ${cur} → ${env}"
  else
    print_info "环境保持: ${env}"
  fi
}

# ============================== 功能 2：拉取子模块 ==============================

flow_fetch_submodules() {
  local env c rc=0
  env="$(load_env_pref)"

  c="$(menu_select "拉取哪些子模块？" 1 \
    "仅当前环境（${env}）所需的子模块" \
    "全部子模块")"

  local -a paths=()
  if [ "${c}" = "1" ]; then
    read -r -a paths <<< "$(submodules_for_env "${env}")"
    if [ "${#paths[@]}" -eq 0 ]; then
      print_error "未定义环境 ${env} 的子模块映射"
      return 1
    fi
    print_info "拉取环境 ${env} 的子模块:"
    printf '  - %s\n' "${paths[@]}"
    git submodule update --init --recursive -- "${paths[@]}" || rc=$?
  else
    print_info "拉取全部子模块..."
    git submodule update --init --recursive || rc=$?
  fi

  if [ "${rc}" -eq 0 ]; then
    print_info "子模块拉取完成"
  else
    print_error "子模块拉取失败（返回码 ${rc}）"
    print_info "  若因 SSH 访问失败（git@github.com: 形式的子模块），可改用 HTTPS："
    print_info "    git config --global url.https://github.com/.insteadOf git@github.com:"
    print_info "  然后重新运行本脚本"
  fi
}

# ============================== 功能 3：更新子模块 ==============================

flow_update_submodules() {
  local env c
  env="$(load_env_pref)"

  c="$(menu_select "更新哪些子模块？" 1 \
    "仅当前环境（${env}）已拉取的子模块" \
    "全部已拉取的子模块")"

  local -a paths=()
  if [ "${c}" = "1" ]; then
    local p
    for p in $(submodules_for_env "${env}"); do
      if [ -e "${p}/.git" ]; then
        paths+=("${p}")
      fi
    done
    if [ "${#paths[@]}" -eq 0 ]; then
      print_warn "环境 ${env} 的子模块尚未拉取，请先执行「拉取子模块」"
      return 0
    fi
  else
    # 已拉取（git submodule status 首字符不为 '-'）的子模块
    local line st p
    while IFS= read -r line; do
      [ -n "${line}" ] || continue
      st="${line:0:1}"
      p="$(echo "${line}" | awk '{print $2}')"
      if [ "${st}" != "-" ]; then
        paths+=("${p}")
      fi
    done < <(git submodule status --recursive)
    if [ "${#paths[@]}" -eq 0 ]; then
      print_warn "没有已拉取的子模块，请先执行「拉取子模块」"
      return 0
    fi
  fi

  print_info "更新子模块（--remote，更新到远程分支最新提交）:"
  printf '  - %s\n' "${paths[@]}"
  print_warn "注意：子模块更新后主仓库记录的版本会发生变化，使 git status 显示子模块有改动；"
  print_warn "      如需固定版本，请在主仓库 git add <子模块> 并提交。"

  local rc=0
  git submodule update --remote -- "${paths[@]}" || rc=$?
  if [ "${rc}" -eq 0 ]; then
    print_info "子模块更新完成"
  else
    print_error "子模块更新失败（返回码 ${rc}）"
    print_info "  若子模块内有本地修改导致无法更新，请先在子模块内提交/暂存修改"
  fi
}

# ============================== 功能 4：安装依赖 ==============================

# 下载/安装 OCS2 三个 deb
_api_asset_url() {
  # 通过 GitHub API 查询 release 中的 deb asset 下载地址
  # $1=api_url $2=包前缀 $3=架构
  local api="$1" prefix="$2" arch="$3"
  local json
  json="$(curl -fsSL --retry 3 "${api}" 2>/dev/null)" || {
    print_error "GitHub API 查询失败: ${api}"
    print_info "  可能触发 API 限流或网络不可达，稍后重试或改用 conf 通道"
    return 1
  }
  _ASSET_PREFIX="${prefix}" _ASSET_ARCH="${arch}" python3 -c '
import json, os, sys
data = json.load(sys.stdin)
prefix = os.environ["_ASSET_PREFIX"]
arch = os.environ["_ASSET_ARCH"]
for a in data.get("assets", []):
    n = a.get("name", "")
    if n.startswith(prefix + "_") and n.endswith("_" + arch + ".deb"):
        print(a["browser_download_url"])
        break
' <<< "${json}"
}

resolve_deb_url() {
  # 解析 deb 下载地址：$1=repo $2=固定tag $3=通道 $4=包前缀
  local repo="$1" tag="$2" channel="$3" prefix="$4"
  local arch
  arch="$(dpkg --print-architecture)"
  case "${channel}" in
    latest)
      _api_asset_url "https://api.github.com/repos/${repo}/releases/latest" "${prefix}" "${arch}"
      ;;
    pre-release)
      _api_asset_url "https://api.github.com/repos/${repo}/releases/tags/pre-release" "${prefix}" "${arch}"
      ;;
    *)
      # conf：固定 tag 直接拼接下载地址
      local ver="${tag#v}"
      ver="${ver#V}"
      printf '%s' "https://github.com/${repo}/releases/download/${tag}/${prefix}_${ver}_${arch}.deb"
      ;;
  esac
}

install_ocs2_debs() {
  local channel="$1"
  local arms_prefix="${DEB_ARMS_PREFIX}"
  [ "${DEB_ARMS_VARIANT}" = "full" ] && arms_prefix="${DEB_ARMS_PREFIX}-full"

  # 安装顺序（勿调整）：ocs2 → robot-descriptions-common → arms
  local -a specs=(
    "${DEB_OCS2_REPO}|${DEB_OCS2_TAG}|${DEB_OCS2_PREFIX}"
    "${DEB_COMMON_REPO}|${DEB_COMMON_TAG}|${DEB_COMMON_PREFIX}"
    "${DEB_ARMS_REPO}|${DEB_ARMS_TAG}|${arms_prefix}"
  )

  print_info "安装 OCS2 相关 deb 包（通道: ${channel}，arms 变体: ${DEB_ARMS_VARIANT}）"
  local tmpdir
  tmpdir="$(mktemp -d /tmp/dual_ur_debs.XXXXXX)"
  local rc=0 s
  for s in "${specs[@]}"; do
    local repo tag prefix url fn
    IFS='|' read -r repo tag prefix <<< "${s}"

    url="$(resolve_deb_url "${repo}" "${tag}" "${channel}" "${prefix}")" || {
      print_warn "跳过 ${prefix}（URL 解析失败）"
      rc=1
      continue
    }
    if [ -z "${url}" ]; then
      print_error "${prefix}: 未找到匹配的 deb（仓库 ${repo}，通道 ${channel}）"
      rc=1
      continue
    fi

    fn="${url##*/}"
    print_info "下载 ${fn} ..."
    if ! curl -fL --retry 3 --connect-timeout 20 -o "${tmpdir}/${fn}" "${url}"; then
      print_error "下载失败: ${url}"
      rc=1
      continue
    fi

    print_info "安装 ${fn} ..."
    if ! sudo dpkg -i "${tmpdir}/${fn}"; then
      print_warn "dpkg 报告依赖问题，尝试 apt-get install -f 修复后重装..."
      sudo apt-get install -f -y
      sudo dpkg -i "${tmpdir}/${fn}" || {
        print_error "${prefix} 安装失败"
        rc=1
      }
    fi
  done

  rm -rf "${tmpdir}"
  if [ "${rc}" -eq 0 ]; then
    print_info "OCS2 deb 包安装完成（ocs2 / robot-descriptions-common / arms-ros2-control）"
  else
    print_error "部分 deb 包安装失败，请检查上方日志"
    print_info "  常见修复: sudo apt --fix-broken install"
  fi
  return "${rc}"
}

run_rosdep() {
  if ! command -v rosdep > /dev/null 2>&1; then
    print_error "未找到 rosdep 命令"
    print_info "  安装:   sudo apt install python3-rosdep -y"
    print_info "  初始化: sudo rosdep init && rosdep update"
    return 1
  fi

  local distro="${ROS_DISTRO:-${ROS_DISTRO_DEFAULT}}"
  if ! rosdep resolve --rosdistro "${distro}" rclcpp > /dev/null 2>&1; then
    print_warn "rosdep 数据库不可用（未初始化或未更新）"
    print_info "  首次使用请执行: sudo rosdep init && rosdep update"
  fi

  print_info "rosdep install --from-paths src lib --ignore-src -r -y"
  if rosdep install --rosdistro "${distro}" --from-paths src lib --ignore-src -r -y; then
    print_info "rosdep 依赖安装完成"
  else
    print_warn "rosdep 安装未完全成功（部分包可能缺少 rosdep 规则，可忽略个别警告）"
  fi
}

install_extra_apt() {
  local p
  for p in "${EXTRA_APT_PKGS[@]}"; do
    print_info "apt 安装: ${p}"
    if ! sudo apt-get install -y "${p}" > /dev/null 2>&1; then
      print_warn "${p} 安装失败（可能源中不存在），跳过"
    fi
  done
  print_info "额外系统包安装完成"
}

flow_install_deps() {
  local c
  c="$(menu_select "请选择安装内容" 1 \
    "全部（rosdep + 额外系统包 + OCS2 deb 包）" \
    "仅 rosdep 依赖（工作区所有包）" \
    "仅额外系统包（UR / MoveIt / RealSense 等）" \
    "仅 OCS2 deb 包")"

  local channel=""
  if [ "${c}" = "1" ] || [ "${c}" = "4" ]; then
    local ch
    ch="$(menu_select "选择 OCS2 deb 发布通道" 1 \
      "conf        固定版本（ocs2=${DEB_OCS2_TAG} / common=${DEB_COMMON_TAG} / arms=${DEB_ARMS_TAG}）" \
      "latest      GitHub Latest 稳定版（需访问 GitHub API）" \
      "pre-release 各仓库 pre-release 滚动版（需访问 GitHub API）")"
    case "${ch}" in
      2) channel="latest" ;;
      3) channel="pre-release" ;;
      *) channel="conf" ;;
    esac
  fi

  case "${c}" in
    1)
      ensure_ros_env
      run_rosdep
      install_extra_apt
      install_ocs2_debs "${channel}"
      ;;
    2)
      ensure_ros_env
      run_rosdep
      ;;
    3)
      install_extra_apt
      ;;
    4)
      install_ocs2_debs "${channel}"
      ;;
  esac
}

# ============================== 主入口 ==============================

if [ ! -d ".git" ]; then
  print_error "当前目录不是 git 仓库，请在项目根目录运行本脚本"
  exit 1
fi

print_hr
print_info "Dual_UR_Demo 初始化"
print_info "工作空间: ${SCRIPT_DIR}"

# 单次执行：选择并完成一个操作后退出（需要其他操作时重新运行本脚本）
local_env="$(load_env_pref)"
echo ""
print_hr
echo -e "${BLUE}  Dual_UR_Demo 初始化${NC}"
echo -e "${BLUE}  工作空间: ${SCRIPT_DIR}${NC}"
echo -e "${BLUE}  当前环境: ${local_env}${NC}"
print_hr
c="$(menu_select --exit "请选择操作" 1 \
  "选择环境（assembly_exp / lab_bench / space_sim）" \
  "拉取子模块（仅当前环境 / 全部）" \
  "更新子模块（仅当前环境 / 全部已拉取）" \
  "安装依赖（rosdep + OCS2 deb 包）")"

case "${c}" in
  1) flow_select_env ;;
  2) flow_fetch_submodules ;;
  3) flow_update_submodules ;;
  4) flow_install_deps ;;
  0)
    echo "退出"
    exit 0
    ;;
esac

echo ""
print_hr
print_info "完成！"
print_info "  编译与启动请使用: ./quick_start.sh"
print_hr
exit 0
