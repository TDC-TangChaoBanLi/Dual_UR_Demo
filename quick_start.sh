#!/usr/bin/env bash
# =============================================================================
#  Dual_UR_Demo 快速启动脚本（quick_start.sh）
#
#  功能：
#    1. 编译（当前环境相关包）
#    2. 启动（ocs2_arms_control / moveit / joints_controller）
#       运行模式：mock_sim（虚拟硬件）/ mujoco_sim（MuJoCo 仿真）/ real_hardware（真机）
#       - moveit：后台启动 joints_controller，再拉起 MoveIt move_group
#       - real_hardware：先启动 dashboard_client 并调用 README 中的服务
#         启动机械臂（上电 / 释放刹车 / 运行程序），再拉起控制器
#    3. 启动相机（探测 RealSense 相机并由用户选择启动）
#
#  用法：./quick_start.sh [--env <环境>]
#  说明：环境默认读取 ${HOME}/.config/dual_ur_demo/env（由 init_repo.sh 设置）。
# =============================================================================

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}" || { echo "无法进入脚本所在目录"; exit 1; }

# ============================== 常量配置区 ==============================

# ---- 支持的环境（与 src/envs/ 下的目录名一致） ----
SUPPORTED_ENVS=(assembly_exp lab_bench space_sim)
ENV_DEFAULT="lab_bench"

# ---- 环境记忆文件（与 init_repo.sh 共用） ----
ENV_PREF_FILE="${HOME}/.config/dual_ur_demo/env"

# ---- ROS 发行版（未 source ROS 环境时按此发行版查找） ----
ROS_DISTRO_DEFAULT="jazzy"

# ---- 启动通用参数 ----
# RViz 是否启动在「启动」流程中通过 Y/n 询问（默认启动）
MUJOCO_HEADLESS="false"     # MuJoCo 是否隐藏 Simulate 视窗（true=无界面）

# ---- RealSense 相机配置（realsense1_* / realsense2_* / realsense3_*） ----
# REALSENSEn_NAME   : 相机名（camera_name，同时用于日志文件名）
# REALSENSEn_SN     : 相机序列号（rs-enumerate-devices -s 查看）
# REALSENSEn_ENABLE : 是否在菜单中展示（true/false）
# REALSENSEn_ARGS   : 传给 realsense2_camera rs_launch.py 的启动参数
REALSENSE1_NAME="arm_A"
REALSENSE1_SN="401522071845"
REALSENSE1_ENABLE="true"
REALSENSE1_ARGS=(
  "camera_namespace:=camera"
  "enable_depth:=true"
  "enable_color:=true"
  "depth_module.depth_profile:=640x480x30"
  "rgb_camera.color_profile:=640x480x30"
  "align_depth.enable:=true"
  "pointcloud.enable:=true"
  "spatial_filter.enable:=true"
  "temporal_filter.enable:=true"
)

REALSENSE2_NAME="arm_B"
REALSENSE2_SN="335222076295"
REALSENSE2_ENABLE="true"
REALSENSE2_ARGS=(
  "camera_namespace:=camera"
  "enable_depth:=true"
  "enable_color:=true"
  "depth_module.depth_profile:=640x480x30"
  "rgb_camera.color_profile:=640x480x30"
  "align_depth.enable:=true"
  "pointcloud.enable:=true"
  "spatial_filter.enable:=true"
  "temporal_filter.enable:=true"
)

REALSENSE3_NAME="global"
REALSENSE3_SN="115222071006"
REALSENSE3_ENABLE="true"
REALSENSE3_ARGS=(
  "camera_namespace:=camera"
  "enable_depth:=true"
  "enable_color:=true"
  "depth_module.depth_profile:=640x480x30"
  "rgb_camera.color_profile:=640x480x30"
  "align_depth.enable:=true"
  "pointcloud.enable:=false"
  "spatial_filter.enable:=true"
  "temporal_filter.enable:=true"
  "unite_imu_method:=false"
)

# ---- 真机（real_hardware）启动常量 ----
# UR 程序名（已存储在机器人控制柜上）
UR_PROGRAM_NAME="TDC-ROS2.urp"
# 双臂命名空间（与 launch_controller 中 dashboard 客户端一致）
ARM_NAMESPACES=(arm_A arm_B)
# dashboard_client 启动文件
DASHBOARD_LAUNCH_PKG="launch_controller"
DASHBOARD_LAUNCH_FILE="start_dual_ur_dashboard_client.launch.py"
# 等待 dashboard 服务出现的超时（秒）
DASHBOARD_WAIT_TIMEOUT=60
# CAN 接口（DH-AG95 夹爪 socketcan），设为 "" 可跳过
CAN_INTERFACE="can0"
CAN_BITRATE="500000"

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

# source ROS 环境 + 工作区 overlay
ensure_ros_env() {
  local distro="${ROS_DISTRO:-${ROS_DISTRO_DEFAULT}}"
  if [ ! -f "/opt/ros/${distro}/setup.bash" ]; then
    print_error "未找到 /opt/ros/${distro}/setup.bash，请确认 ROS 已安装"
    return 1
  fi
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${distro}/setup.bash"
  if [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "${SCRIPT_DIR}/install/setup.bash"
  fi
  set -u
  return 0
}

# 后台进程清理（脚本退出时）
BG_PIDS=()
cleanup() {
  local pid
  for pid in "${BG_PIDS[@]}"; do
    kill "${pid}" 2> /dev/null
  done
}
trap cleanup EXIT

# ============================== 功能 1：编译 ==============================

flow_build() {
  ensure_ros_env || return 1
  local env
  env="$(load_env_pref)"

  if ! command -v colcon > /dev/null 2>&1; then
    print_error "未找到 colcon，请先安装: sudo apt install python3-colcon-common-extensions -y"
    return 1
  fi

  print_info "收集工作区包列表（环境: ${env}）..."
  local -a all_pkgs=()
  mapfile -t all_pkgs < <(colcon list --names-only 2> /dev/null | sort -u)
  if [ "${#all_pkgs[@]}" -eq 0 ]; then
    print_error "未在工作区中找到任何 ROS 包（src/ 或 lib/ 下无 package.xml）"
    return 1
  fi

  has_pkg() {
    local p
    for p in "${all_pkgs[@]}"; do
      [ "${p}" = "$1" ] && return 0
    done
    return 1
  }

  # 当前环境相关包候选（存在才编译）：
  #   {env}_* 环境描述/仿真/MoveIt 配置 + launch_controller
  #   DH-AG95 / Robotiq 夹爪（lab_bench / assembly_exp 使用）
  #   MuJoCo ros2_control + 插件（仿真使用）
  local -a candidates=(
    "${env}_description"
    "${env}_mujoco_description"
    "${env}_moveit_config"
    "launch_controller"
    "dh_ag95_description"
    "dh_ag95_msgs"
    "dh_ag95_core_driver"
    "dh_ag95_controllers"
    "robotiq_description"
    "robotiq_driver"
    "robotiq_controllers"
    "robotiq_hardware_tests"
    "serial"
    "mujoco_vendor"
    "mujoco_ros2_control"
    "mujoco_ros2_control_msgs"
    "mujoco_ros2_control_plugins"
    "mujoco_god_plugin"
    "mujoco_dock_sim_plugin"
  )

  local -a pkgs=()
  local p
  for p in "${candidates[@]}"; do
    if has_pkg "${p}"; then
      pkgs+=("${p}")
    else
      print_warn "跳过不存在的包: ${p}"
    fi
  done

  if [ "${#pkgs[@]}" -eq 0 ]; then
    print_error "没有可编译的包，请先检查 src/ lib/ 目录或按照 README 拉取子模块"
    return 1
  fi

  print_info "编译包（共 ${#pkgs[@]} 个，含工作区依赖）:"
  printf '  - %s\n' "${pkgs[@]}"

  if colcon build --symlink-install --packages-up-to "${pkgs[@]}"; then
    print_info "编译完成"
    print_info "如需在当前终端使用，请执行: source install/setup.bash"
  else
    print_error "编译失败，请检查上方日志"
    return 1
  fi
}

# ============================== 功能 2：启动 ==============================

# 服务调用并检查返回值（success=True 视为成功）；失败返回 1
call_service_checked() {
  # $1=服务名 $2=服务类型 $3=请求内容
  local out
  out="$(timeout 15 ros2 service call "$1" "$2" "$3" 2>&1)" || {
    print_error "服务调用失败: $1（服务无响应或超时）"
    return 1
  }
  if grep -q "success=True" <<< "${out}"; then
    print_info "服务调用成功: $1"
    return 0
  fi
  print_error "服务返回失败: $1"
  echo "${out}" | grep -E "message=|success=" | head -2 | sed 's/^/    /'
  return 1
}

# 打开 CAN 接口（DH-AG95 夹爪 socketcan）
setup_can_interface() {
  [ -n "${CAN_INTERFACE}" ] || return 0
  print_info "配置 CAN 接口 ${CAN_INTERFACE}（bitrate ${CAN_BITRATE}）"
  sudo ip link set "${CAN_INTERFACE}" down 2> /dev/null || true
  sudo ip link set "${CAN_INTERFACE}" type can bitrate "${CAN_BITRATE}" 2> /dev/null \
    || print_warn "配置 ${CAN_INTERFACE} 失败（可能无此接口或无权限，按需忽略）"
  sudo ip link set "${CAN_INTERFACE}" up 2> /dev/null || true
}

wait_for_dashboard() {
  local timeout="${DASHBOARD_WAIT_TIMEOUT}" i
  print_info "等待 dashboard_client 服务就绪（最长 ${timeout}s）..."
  for ((i = 0; i < timeout; i++)); do
    if ros2 service list 2> /dev/null | grep -q "arm_A/dashboard_client/load_program"; then
      print_info "dashboard_client 服务已就绪"
      return 0
    fi
    sleep 2
  done
  return 1
}

# 启动 dashboard_client（后台）；成功返回 0 并设置全局 DASHBOARD_PID
DASHBOARD_PID=""
start_dashboard_client() {
  print_info "启动 dashboard_client（后台）..."
  ros2 launch "${DASHBOARD_LAUNCH_PKG}" "${DASHBOARD_LAUNCH_FILE}" &
  DASHBOARD_PID=$!
  if ! wait_for_dashboard; then
    print_error "dashboard_client 服务未就绪，请检查机械臂网络连接（IP 配置见 lab_bench_description）"
    return 1
  fi
  return 0
}

# dashboard 初始化服务：load_program → power_on → brake_release
# 任一失败返回 1（失败内容由 call_service_checked 打印）
run_dashboard_init_services() {
  local ns rc=0
  for ns in "${ARM_NAMESPACES[@]}"; do
    call_service_checked "/${ns}/dashboard_client/load_program" \
      "ur_dashboard_msgs/srv/Load" "{filename: '${UR_PROGRAM_NAME}'}" || rc=1
  done
  for ns in "${ARM_NAMESPACES[@]}"; do
    call_service_checked "/${ns}/dashboard_client/power_on" "std_srvs/srv/Trigger" "{}" || rc=1
  done
  for ns in "${ARM_NAMESPACES[@]}"; do
    call_service_checked "/${ns}/dashboard_client/brake_release" "std_srvs/srv/Trigger" "{}" || rc=1
  done
  return "${rc}"
}

# 控制器启动后重新启动 UR 程序（先 stop 再 play）
# play 失败时抛出错误（[ERROR]）并返回 1
restart_ur_program() {
  local ns rc=0
  print_info "重新启动 UR 机械臂程序（stop → play，程序: ${UR_PROGRAM_NAME}）..."

  # 1) 停止程序
  for ns in "${ARM_NAMESPACES[@]}"; do
    call_service_checked "/${ns}/dashboard_client/stop" "std_srvs/srv/Trigger" "{}" \
      || print_error "停止 ${ns} 机械臂程序失败（stop）"
  done

  # 2) 启动程序（play 失败必须抛出错误）
  for ns in "${ARM_NAMESPACES[@]}"; do
    call_service_checked "/${ns}/dashboard_client/play" "std_srvs/srv/Trigger" "{}" \
      || {
        print_error "启动 ${ns} 机械臂程序失败（play），机械臂将不响应控制命令！"
        rc=1
      }
  done

  if [ "${rc}" -eq 0 ]; then
    print_info "UR 程序已重新启动（${UR_PROGRAM_NAME}）"
  else
    print_error "UR 程序重新启动失败"
  fi
  return "${rc}"
}

# 检查 dashboard_client 服务是否已就绪（需先通过主菜单「启动 dashboard_client」选项启动）
check_dashboard_ready() {
  ros2 service list 2> /dev/null | grep -q "arm_A/dashboard_client/play"
}

# 真机准备：CAN 配置 + 检查 dashboard_client 已就绪
# （dashboard_client 由用户预先通过主菜单「启动 dashboard_client」选项启动并完成初始化）
run_real_setup() {
  setup_can_interface || true

  if ! check_dashboard_ready; then
    print_error "dashboard_client 未运行或服务未就绪"
    print_info "  请先运行本脚本主菜单的「启动 dashboard_client」选项，"
    print_info "  完成机械臂初始化（load_program / power_on / brake_release）后，再启动控制器。"
    return 1
  fi
  print_info "dashboard_client 服务已就绪（在后台运行中）"
  return 0
}

wait_for_controller_manager() {
  local timeout=90 i
  print_info "等待 controller_manager 就绪（最长 ${timeout}s）..."
  for ((i = 0; i < timeout; i++)); do
    if ros2 service list 2> /dev/null | grep -q "^/controller_manager/"; then
      print_info "controller_manager 已就绪"
      return 0
    fi
    sleep 2
  done
  print_warn "等待 controller_manager 超时，继续执行"
  return 1
}

# 等待控制器激活（active）
wait_for_controllers_active() {
  local timeout=60 i
  print_info "等待控制器激活（最长 ${timeout}s）..."
  for ((i = 0; i < timeout; i++)); do
    if ros2 control list_controllers 2> /dev/null | grep -qE "[[:space:]]active$"; then
      print_info "控制器已激活"
      return 0
    fi
    sleep 2
  done
  print_warn "等待控制器激活超时，继续执行"
  return 1
}

# 询问是否启动 RViz（默认启动，回车=Y）
ask_rviz_default_yes() {
  local ans
  read -rp "是否启动 RViz？[Y/n]（回车=启动）: " ans || ans=""
  case "${ans}" in
    n | N | no | NO) return 1 ;;
    *) return 0 ;;
  esac
}

# 启动控制器 launch（前台等待）；真机模式下后台启动，
# 等控制器就绪后重新启动 UR 程序（stop → play）再保持等待
run_controller_launch() {
  # $1=launch 文件；其余为启动参数
  local launch_file="$1"
  shift
  local -a largs=("$@")

  if [ "${is_real}" -eq 1 ]; then
    ros2 launch launch_controller "${launch_file}" "${largs[@]}" &
    local lpid=$!
    BG_PIDS+=("${lpid}")
    wait_for_controller_manager
    wait_for_controllers_active
    sleep 2
    restart_ur_program || true # 失败时 restart_ur_program 内部已打印详细错误
    print_info "控制器运行中（Ctrl+C 结束）"
    wait "${lpid}"
  else
    ros2 launch launch_controller "${launch_file}" "${largs[@]}"
  fi
}

flow_launch() {
  local env
  env="$(load_env_pref)"
  local env_desc="${env}_description"

  # 1) 先选择运行模式
  local mode
  mode="$(menu_select "请选择运行模式" 1 \
    "mock_sim      （虚拟硬件 use_fake_hardware:=true）" \
    "mujoco_sim    （MuJoCo 仿真 env:=${env}_mujoco）" \
    "real_hardware （真实机械臂 use_fake_hardware:=false）")"

  # 2) 再选择启动项
  local app
  app="$(menu_select "请选择启动项" 1 \
    "ocs2_arms_control  （OCS2 双臂 MPC 控制器 + Interactive Marker）" \
    "joints_controller  （传统关节控制器，start_joints_controllers）" \
    "moveit             （后台 joints_controller + MoveIt move_group）")"

  # 3) 询问是否启动 RViz（回车=启动）
  local launch_rviz="true"
  ask_rviz_default_yes || launch_rviz="false"

  # 模式 → 启动参数
  local launch_env="${env}" use_fake="true" is_real=0 is_mujoco=0
  case "${mode}" in
    2)
      launch_env="${env}_mujoco"
      use_fake="true"
      is_mujoco=1
      ;;
    3)
      use_fake="false"
      is_real=1
      ;;
  esac

  # 真机配置检查
  if [ "${is_real}" -eq 1 ]; then
    if [ ! -f "${SCRIPT_DIR}/install/${env_desc}/share/${env_desc}/config/ros2_control/hardware_interface_controller_params.yaml" ] \
      && [ ! -f "${SCRIPT_DIR}/src/envs/${env}/${env_desc}/config/ros2_control/hardware_interface_controller_params.yaml" ]; then
      print_warn "环境 ${env} 未找到真机硬件配置文件（hardware_interface_controller_params.yaml）"
      if [ "${env}" = "space_sim" ]; then
        print_warn "space_sim 为太空仿真环境（无夹爪、无真机配置），真机模式可能无法运行"
      fi
      confirm_yn "仍要继续以真机模式启动？" || return 0
    fi
  fi

  ensure_ros_env || return 1

  # launch_controller 是否已编译
  if ! ros2 pkg prefix launch_controller > /dev/null 2>&1; then
    print_error "未找到 launch_controller 包，请先执行「编译」"
    return 1
  fi

  case "${app}" in
    1)
      # ---- OCS2 双臂控制器 ----
      local -a args=(
        "env:=${launch_env}"
        "launch_rviz:=${launch_rviz}"
      )
      if [ "${is_mujoco}" -eq 1 ]; then
        args+=("mujoco_headless:=${MUJOCO_HEADLESS}")
      else
        args+=("use_fake_hardware:=${use_fake}")
      fi

      if [ "${is_real}" -eq 1 ]; then
        run_real_setup || return 1
      fi

      print_info "启动 OCS2 双臂控制器（env=${launch_env}）..."
      run_controller_launch start_ocs2_arms_controller.launch.py "${args[@]}"
      ;;
    2)
      # ---- 传统关节控制器（joints_controller） ----
      local -a args=(
        "env:=${launch_env}"
        "launch_rviz:=${launch_rviz}"
      )
      if [ "${is_mujoco}" -eq 1 ]; then
        args+=("mujoco_headless:=${MUJOCO_HEADLESS}")
      else
        args+=("use_fake_hardware:=${use_fake}")
      fi

      if [ "${is_real}" -eq 1 ]; then
        run_real_setup || return 1
      fi

      print_info "启动 joints_controller（env=${launch_env}）..."
      run_controller_launch start_joints_controllers.launch.py "${args[@]}"
      ;;
    3)
      # ---- MoveIt（后台 joints_controller + 前台 move_group） ----
      if ! ros2 pkg prefix "${env}_moveit_config" > /dev/null 2>&1; then
        print_error "未找到 ${env}_moveit_config 包（该环境可能没有 MoveIt 配置或未编译）"
        return 1
      fi

      local -a joints_args=(
        "env:=${launch_env}"
        "launch_rviz:=false"
      )
      if [ "${is_mujoco}" -eq 1 ]; then
        joints_args+=("mujoco_headless:=${MUJOCO_HEADLESS}")
      else
        joints_args+=("use_fake_hardware:=${use_fake}")
      fi

      if [ "${is_real}" -eq 1 ]; then
        run_real_setup || return 1
      fi

      local joints_log="/tmp/dual_ur_joints_controller.log"
      print_info "后台启动 joints_controller（日志: ${joints_log}）..."
      ros2 launch launch_controller start_joints_controllers.launch.py "${joints_args[@]}" \
        > "${joints_log}" 2>&1 &
      BG_PIDS+=($!)

      wait_for_controller_manager
      wait_for_controllers_active

      # 真机：控制器就绪后重新启动 UR 程序（stop → play）
      if [ "${is_real}" -eq 1 ]; then
        sleep 2
        restart_ur_program || true # 失败时 restart_ur_program 内部已打印详细错误
      fi

      # MoveIt 需要 scaled_joint_trajectory_controller（joints_controller 默认已激活）
      print_info "启动 MoveIt move_group（env=${env}）..."
      ros2 launch launch_controller start_moveit_group.launch.py \
        "env:=${env}" "launch_rviz:=${launch_rviz}"
      ;;
  esac
}

# ============================== 功能 2b：启动 dashboard_client ==============================

# 启动 dashboard_client 并自动执行初始化服务（load / power_on / brake_release）；
# 任一失败则打印失败内容、停止 dashboard_client 并退出
flow_dashboard_client() {
  ensure_ros_env || return 1

  if ! ros2 pkg prefix launch_controller > /dev/null 2>&1; then
    print_error "未找到 launch_controller 包，请先执行「编译」"
    return 1
  fi

  setup_can_interface || true

  if ! start_dashboard_client; then
    exit 1
  fi

  if ! run_dashboard_init_services; then
    print_error "机械臂初始化服务失败，停止 dashboard_client 并退出"
    kill "${DASHBOARD_PID}" 2> /dev/null
    exit 1
  fi

  print_info "机械臂初始化完成（已加载 ${UR_PROGRAM_NAME}、上电、释放刹车）"
  print_info "dashboard_client 已在后台运行（停止: pkill -f dashboard_client）"
  print_info "  下一步：启动控制器（真机模式），控制器就绪后脚本会自动重新启动 UR 程序（stop → play）"
}

# ============================== 功能 3：启动相机 ==============================

# 输出已连接的 RealSense 序列号（每行一个）；无工具时返回 1
detect_realsense_sns() {
  if ! command -v rs-enumerate-devices > /dev/null 2>&1; then
    return 1
  fi
  rs-enumerate-devices -s 2> /dev/null \
    | awk -F':' '/Serial Number/ {gsub(/[[:space:]]/, "", $2); if ($2 != "") print $2}'
}

# 读取 REALSENSEn_<field> 变量：rs_get <n> <field>
rs_get() {
  local n="$1" field="$2"
  local var="REALSENSE${n}_${field}"
  eval "printf '%s' \"\${${var}:-}\""
}

start_realsense() {
  local n="$1"
  local name sn
  name="$(rs_get "${n}" NAME)"
  sn="$(rs_get "${n}" SN)"
  local -a args=()
  eval "args=(\"\${REALSENSE${n}_ARGS[@]}\")"

  local log="/tmp/realsense_${name}.log"
  print_info "启动相机 ${name}（SN: ${sn}，日志: ${log}）"
  ros2 launch realsense2_camera rs_launch.py \
    "serial_no:=_${sn}" "camera_name:=${name}" "${args[@]}" \
    > "${log}" 2>&1 &
  disown
}

flow_cameras() {
  ensure_ros_env || return 1

  if ! ros2 pkg prefix realsense2_camera > /dev/null 2>&1; then
    print_error "未找到 realsense2_camera 包，请先安装: sudo apt install ros-\${ROS_DISTRO}-realsense2-camera -y"
    return 1
  fi

  # 探测已连接的相机
  local -a connected=()
  local have_tool=1
  if command -v rs-enumerate-devices > /dev/null 2>&1; then
    mapfile -t connected < <(detect_realsense_sns)
  else
    have_tool=0
    print_warn "未找到 rs-enumerate-devices（librealsense2-utils），无法探测相机连接状态"
    print_info "  安装: sudo apt install ros-\${ROS_DISTRO}-librealsense2-utils -y"
  fi

  is_connected() {
    local s
    for s in "${connected[@]}"; do
      [ "${s}" = "$1" ] && return 0
    done
    return 1
  }

  # 展示配置的相机
  echo ""
  echo "已配置的 RealSense 相机:"
  local -a idxs=()
  local n name sn state idx_no
  for n in 1 2 3 4; do
    name="$(rs_get "${n}" NAME)"
    [ -n "${name}" ] || continue
    [ "$(rs_get "${n}" ENABLE)" = "true" ] || continue
    sn="$(rs_get "${n}" SN)"
    if [ "${have_tool}" -eq 1 ]; then
      if is_connected "${sn}"; then
        state="${GREEN}[已连接]${NC}"
      else
        state="${YELLOW}[未连接]${NC}"
      fi
    else
      state="[未知]"
    fi
    idxs+=("${n}")
    idx_no="${#idxs[@]}"
    echo -e "  ${idx_no}) ${name}   SN ${sn}  ${state}"
  done

  if [ "${#idxs[@]}" -eq 0 ]; then
    print_warn "没有启用任何相机（请在脚本常量区检查 REALSENSEn_ENABLE）"
    return 0
  fi

  local sel
  echo ""
  read -rp "请输入要启动的相机序号（逗号分隔，如 1,3；all=全部；回车=不启动）: " sel
  if [ -z "${sel}" ]; then
    echo "已跳过相机启动"
    return 0
  fi

  local -a chosen=()
  if [ "${sel,,}" = "all" ]; then
    chosen=("${idxs[@]}")
  else
    local item pos
    local -a items=()
    IFS=',' read -ra items <<< "${sel}"
    for item in "${items[@]}"; do
      item="$(echo "${item}" | tr -d '[:space:]')"
      [[ "${item}" =~ ^[0-9]+$ ]] || continue
      pos=$((item - 1))
      if [ "${pos}" -ge 0 ] && [ "${pos}" -lt "${#idxs[@]}" ]; then
        chosen+=("${idxs[pos]}")
      else
        print_warn "忽略无效序号: ${item}"
      fi
    done
  fi

  if [ "${#chosen[@]}" -eq 0 ]; then
    echo "未选择任何相机"
    return 0
  fi

  for n in "${chosen[@]}"; do
    start_realsense "${n}"
    sleep 1
  done

  print_info "相机已在后台启动（脚本退出后保持运行）"
  print_info "停止相机: pkill -f realsense2_camera"
}

# ============================== 功能 4：切换环境 ==============================

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

# ============================== 主入口 ==============================

# 支持 --env <环境> 直接指定
if [ "${1:-}" = "--env" ] && [ -n "${2:-}" ]; then
  save_env_pref "$2"
fi

print_hr
print_info "Dual_UR_Demo 快速启动"
print_info "工作空间: ${SCRIPT_DIR}"

# 单次执行：完成一个操作后退出（需要其他操作时重新运行本脚本）
cur_env="$(load_env_pref)"
echo ""
print_hr
echo -e "${BLUE}  Dual_UR_Demo 快速启动${NC}"
echo -e "${BLUE}  工作空间: ${SCRIPT_DIR}${NC}"
echo -e "${BLUE}  当前环境: ${cur_env}${NC}"
print_hr
c="$(menu_select --exit "请选择操作" 1 \
  "编译（当前环境相关包）" \
  "启动（ocs2_arms_control / joints_controller / moveit）" \
  "启动 dashboard_client（真实机械臂初始化：启动 + 初始化服务）" \
  "启动相机（RealSense）" \
  "切换环境（assembly_exp / lab_bench / space_sim）")"

case "${c}" in
  1) flow_build ;;
  2) flow_launch ;;
  3) flow_dashboard_client ;;
  4) flow_cameras ;;
  5) flow_select_env ;;
  0)
    echo "退出"
    exit 0
    ;;
esac

echo ""
print_hr
print_info "完成！"
print_hr
exit 0
