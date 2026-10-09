# space_sim 镜片动作接口

本包提供两个初始化/重置 service 和四个可独立调用的 ROS action。C++ 节点只执行请求的动作，
六片装配流程由 `scripts/assembly_flow.py` 按 YAML 编排。取片或转运失败会返回失败，
**不会自动把已取出的镜片送回储存器**。需要人工决定后续动作，或调用重置服务重新初始化。

## 接口

存储接口释放后的下降只使用保持原姿态的笛卡尔直线，目标为世界 −z 方向 2 cm，
以 1 mm 步长计算。若不能完整下降，则执行通过碰撞检查的可达前缀（可能为 0 mm），
报告实时实际下降距离，再由 `ExitStorage` 规划移出。不旋转镜片，也不使用 STOMP 回退。
计算服务、轨迹检查或控制器执行失败仍返回失败；后续移出仍必须通过碰撞检查。

| 接口 | ROS 名称 | 主要输入与效果 |
| --- | --- | --- |
| `std_srvs/Trigger` | `/initialize_planning_environment` | 每次调用均重置 MuJoCo：双臂恢复 SRDF home，六片镜片恢复存储对接；同步 MoveIt 碰撞场景、附着关系和任务记录 |
| `ResetPlanningEnvironment` | `/reset_planning_environment` | `joint_states` 指定部分或全部关节角；`lenses` 指定存储、中心对接或机械臂抓取状态；同步重置 MuJoCo、控制器和 MoveIt |
| `DockStorage` | `/dock_storage_lens` | `arm`、`lens`（0 表示在 `allowed_lenses` 内选择）、`grasp_face` 或 `allowed_faces`、`phase_deg`、`randomize`；规划预抓取、沿轴抓取并磁吸，释放镜片与储存器接口，然后保持原姿态沿世界 −z 尽可能下降 2 cm |
| `ExitStorage` | `/exit_storage` | `arm`、`exit_distance`（0 使用配置值）、`hover_z`（非零时指定镜片 body 原点的世界 z；0 保持 TCP 高度）；保持**镜片镜面竖直**移出储存区，允许绕镜面法向调整姿态 |
| `MoveLens` | `/move_lens` | `arm`、`target`：`world` 坐标系中**镜片 body**的目标位姿 |
| `DockCentre` | `/dock_centre` | `arm`、`centre_face` 或 `allowed_faces`、`return_home`；直接规划到 2 cm 预装配位、沿轴插入、dock、释放后沿夹爪抓取接口 +z 后退 2 cm，恢复夹爪与镜片的碰撞检查，可选用 STOMP 回 home |

同一机械臂同时只接受一个动作目标；A/B 臂分别拥有 MoveIt 规划器和轨迹控制器，
可以并发执行。发送控制器前检查两条在途轨迹的双臂碰撞。自由空间动作优先使用
STOMP；若优化失败，可执行逐点通过双臂碰撞检查的确定性关节直连。
短距离接口轴线动作使用 MoveIt 的笛卡尔插补和碰撞检查，不切换 Pilz 或随机搜索规划器。

`ExitStorage` 的外移、侧向避让和分段外移均只执行 STOMP 优化轨迹，
不回退到笛卡尔插补或确定性关节直连。允许绕镜面法向选择不同的目标姿态；
同一目标依次尝试确定性 IK 支路，每条 STOMP 轨迹都必须满足镜面竖直检查。
使用工作区 `space_sim_stomp_planner` 插件，其镜面约束直接限制
`abs((R_tcp * normal_in_tcp).z()) <= mirror_vertical_tolerance`。
绕镜面法向旋转和法向在水平面内转向均自由。插件修正 MoveIt 2.12.4 的
“容差内的姿态偏差仍被当作无效”问题，并为每个规划请求独立保存机器人状态。
同时平移/旋转的笛卡尔参考只作为 STOMP 优化初值，不直接执行；优化后仍做
真实镜片法向、平滑后碰撞复检。
STOMP 使用随机轨迹扰动，用户已授权使用；不启用 OMPL/RRT 回退。
狭窄通道仍可能无解；所有候选失败时 action 明确返回失败。

## 指定初始状态

`/reset_planning_environment` 使用 `space_sim_task_control/srv/ResetPlanningEnvironment`。
空请求与 `/initialize_planning_environment` 的效果相同。`joint_states.name/position`
按关节名指定角度（rad），未指定的关节采用 SRDF `arm_A_home`/`arm_B_home`；
`velocity/effort` 留空。`lenses` 中未指定的镜片仍在存储区。

每条 `LensInitialState` 的 `lens` 为 1–6，`location` 为：

| location | 状态 | 需要的字段 |
| --- | --- | --- |
| 0 (`STORAGE`) | 存储器对接 | 默认镜片接口 `S0` |
| 1 (`CENTRE`) | 已安装到中心镜片 | `centre_face: S0..S5`；`lens_face` 默认 `S0` |
| 2 (`ARM`) | 已吸附到机械臂末端 | `arm: arm_A/arm_B` 和 `lens_face: S0..S5` |

`phase_deg` 为绕对接口轴的对接相位，默认 0。中心/末端状态的镜片位姿由
相应 site 的局部位姿和指定关节角计算，同时建立真实 MuJoCo weld 与 MoveIt 碰撞对象。
镜片编号、中心接口和持片机械臂不可重复占用；每片镜片只指定一种状态。
请求会校验关节限位及字段；正在执行任一 action 时拒绝重置。
重置后返回实际关节角和补齐后的六片镜片状态；重置或同步失败时不启用后续动作，
应检查返回原因并重新调用重置服务。指定状态需在物理上可实现，不能让持片镜片与环境重叠。

例如，镜片 1 已安装在中心 `S5`，其余镜片仍在存储区：

```bash
ros2 service call /reset_planning_environment space_sim_task_control/srv/ResetPlanningEnvironment \
  '{lenses: [{lens: 1, location: 1, centre_face: S5, lens_face: S0}]}'
ros2 service call /initialize_planning_environment std_srvs/srv/Trigger '{}'
```

两个服务都直接重置仿真状态；正常 action 的失败恢复仍只使用机械臂运动规划。
底层 `/mujoco_ros2_control_node/reset_scene` 会在同一仿真锁内恢复初始模型 weld 绑定，
再应用关节、自由体位姿和新 weld，避免把上次装配的动态 weld 误当作存储对接。

STOMP 优化轨迹但不承诺严格几何直线，也不保证平移与旋转进度同步。镜片移出储存区后直接避障到
中心接口外 2 cm 的预装配位；最终 2 cm 插入仍沿接口轴。
实测仅用 CHOMP 规划抓取段时，
A 臂轨迹偏离接口轴线，B 臂无解，因此该段必须用几何直线插补。
接近和插入段在规划完成后逐点 FK 检查末端
是否位于起终点轴线的 `axis_tolerance_m` 管道内；移出储存区逐点检查镜片 body
法向的世界 z 分量，不用 TCP 的 z 轴代替镜面姿态；绕镜面法向的旋转不改变竖直条件。
检查未通过则拒绝执行。

## 运行日志

日志使用 `[动作开始/阶段/运动开始/规划成功/回退/动作失败/动作完成]` 等标记。
INFO 保留 `arm`、`镜片=len_ring_N`、阶段、目标接口编号、规划方式和成功/失败/回退摘要。
接口坐标及每阶段的规划前、规划轨迹起止点和结束实测 TCP 位姿、关节角均使用 DEBUG。
规划前/结束实测记录完整机器人关节状态（含对侧臂）；规划轨迹端点记录关节名和角度，
关节目标同时记录 FK 计算的目标 TCP 位姿。数据保留 17 位有效数字，
坐标为 `world xyz(m)`，姿态为四元数 `quat(xyzw)`，关节角单位 rad。
规划方式明确区分 STOMP、笛卡尔插值、关节空间直连；失败记录错误码解释、服务或
控制器返回原因及下一步候选/重试方案。`-31` 不直接解释为碰撞；笛卡尔服务未返回
中断原因时明确注明原因未返回。下降不足 2 cm 会报告可执行比例及实时实际下降距离。

默认 `task_log_level:=info`。需要保留复现数据时启动：

```bash
ros2 launch space_sim_task_control task_actions.launch.py task_log_level:=debug
```

DEBUG 文件保存于 ROS 日志目录；仅有状态端点不能完整还原当时的碰撞场景，
精确复现还需要对应场景及规划器配置。
DEBUG 还记录规划原始轨迹、TOTG/Ruckig 处理后的轨迹及重叠碰撞检查触发时的完整关节状态。
失败与回退复现结果见 [2026-10-06 测试报告](docs/failure_audit_20261006.md)。

重置接口的联合仿真回归脚本为 `test/test_reset_environment.py`。
在独立 ROS domain 中启动 MuJoCo、MoveIt 和 `run_flow:=false` 的任务服务后运行该脚本；
它会执行取片/移出、重置到混合状态和六片已安装状态，检查接口对合、附着对象、
默认存储 weld 的恢复，并验证非法请求和运动中重置会被拒绝。

## 启动与 YAML

先启动 MuJoCo 控制器和 `space_sim_moveit_config` 的 `move_group`，再运行：

```bash
colcon build --symlink-install --packages-select mujoco_ros2_control_msgs mujoco_ros2_control mujoco_god_plugin space_sim_stomp_planner space_sim_task_control space_sim_moveit_config
ros2 launch space_sim_task_control task_actions.launch.py
```

`run_flow:=false` 只启动接口服务端。`flow:=/path/to/flow.yaml` 选择自定义流程。
更新上述重置接口后，需要重启 MuJoCo 控制器和任务服务节点。
默认 [assembly_flow.yaml](config/assembly_flow.yaml) 中两臂各有独立任务序列；
两臂并行抓取和装配 S0/S3，先完成者先安装。S0 镜片固定抓取 S2 接口，
避开从储存区直达预装配位时 S1 抓取姿态反复碰撞的路径。
后续任务只在中心面安装顺序确有要求时使用 `after`；默认流程不再让
B 臂等待 S0 完成才开始 S3，也不再让 S4 等待 S5。
另有 [assembly_flow_s1_then_s0.yaml](config/assembly_flow_s1_then_s0.yaml)
和 [assembly_flow_s2_then_s3.yaml](config/assembly_flow_s2_then_s3.yaml)，分别安排
S1→S0 与 S2→S3 为前两片，同时改变储存器取片顺序。配置中的
`expected_prefix` 会检查实际安装顺序；六片完成后还会检查目标面集合。
自带备用配置仅保留前两片顺序所需的跨臂依赖；后续取片和转运独立推进，
共享区冲突由执行器复检和最后插入阶段的邻面互斥处理。
`return_home` action 输入默认关闭，可逐任务开启；当前默认 YAML 只在每臂最后一片装配后开启。
`after` 仅在进入中心共享区前等待指定中心面清场；`before_pick: exit:S1`
或 `before_pick: S5` 用于相邻镜片共享存储区与中心装配区域时的防碰撞通行。
可以用 `lens_candidates`、
`grasp_faces`、`centre_faces` 限定随机集合，再设置 `randomize_pick` 或
`randomize_centre`。中心装配若仅规划失败，`dock_retries`（默认 2）允许以
当前真实关节状态重新求解；对接或释放后失败不会重放动作。若仍失败，Python
流程退出非零状态，保留当前镜片状态供诊断。
独立规划失败不会等待对侧动作完成；只有明确的共享区域碰撞冲突才协调让行。
初始化 service 响应超时会重试一次。

## 通用 STOMP / MPC 执行栈验证

通用实现位于 `lib/multi_arm_motion`。它提供 `/execute_motion` 并发 action，
共享 MPC 循环统一检查两臂运动，控制器指令均附带经过检查的减速停止尾段。
默认 `motion_backend:=direct` 保持原控制器后端。可选择 `motion_backend:=hybrid`
让镜片 action 通过通用执行器运动；此时任务节点不会直接向控制器发轨迹。
不要同时运行其他会向相同控制器发送轨迹的任务节点。

在同一个独立 ROS domain 中启动 MuJoCo 控制器和 MoveIt，然后运行：

```bash
ros2 launch space_sim_moveit_config hybrid_motion.launch.py enable_execution:=true
python3 src/control/task_control/space_sim/test/test_hybrid_motion.py \
  --output /tmp/hybrid_motion_validation.json
```

测试向两臂同时提交不同关节目标，记录实际关节速度重叠、两路控制器相同的
起始时间戳、最终关节误差，并用 MoveIt 复检测得的中间状态。`--offset -0.04`
可反向复跑。停止、恢复及预测缓存重置分别使用 `/stop_motion`、
`/resume_motion` 和 `/reset_motion_predictions`；仿真重置后应从新鲜静止反馈恢复。
更严格的装配精度需配置执行容差，并由镜片 action 根据实时 site 位姿核验对接。

装配验证入口（同一隔离 domain，先启动 MuJoCo 和 MoveIt）：

```bash
ros2 launch space_sim_moveit_config hybrid_motion.launch.py enable_execution:=true \
  joint_tolerance:=0.0002 control_period:=0.2 \
  command_lead:=0.16 optimization_budget:=0.05
ros2 launch space_sim_task_control task_actions.launch.py motion_backend:=hybrid
```

卫星部署的默认参数为控制周期 0.2 s、计算预算 0.05 s、提前量 0.16 s、
关节到位容差 0.0002 rad；通用库的默认参数不随此部署修改。

以上两个命令在不同终端运行。自由空间动作先由任务层生成参考路径，交给动态
STOMP 按其他机械臂未来轨迹再次优化；最终时间网格在优化前确定，执行前复检。
2 cm 直线段由 MoveIt 笛卡尔插补产生，提交为不再优化几何的参考轨迹；
MPC 同时检查 2 mm 轴线管道及姿态约束，不能通过避障偏离接口轴。
移出存储区的镜面竖直约束同时传入动态 STOMP 与 MPC。
`home_planning_time_s` 默认为 30 秒，单独用于装配后回 home 的动态全局优化；
`centre_planning_time_s` 默认为 30 秒，用于到中心预装配位的动态全局优化。
其他自由空间动作使用 `planning_time_s`（默认 12 秒）。优化失败仍明确返回失败，
回 home 的安全中继位也必须重新规划和复检。

混合后端会在选择预抓取 IK 分支时预检后续 2 cm 接近段，使用完整场景和
5 mm 执行间距；只在预检场景中允许预定镜片与本臂对接面的接触。
预检参考不会执行，实际接近仍重新计算并由 MPC 检查。
自由空间动态 STOMP 在执行前失败时尝试其他确定性 IK 终点；
接触直线失败不会改为自由空间轨迹。按组制动候选允许未受阻的机械臂继续推进，
混合轨迹仍须通过双臂碰撞、任务约束和制动尾段检查。
初始化/重置会先停止通用执行器并清除预测，物理环境重置完成后从新鲜反馈恢复。

持片运动每 200 ms 检查一次镜片在 TCP 体坐标系中的相对位姿，位置误差限
3 mm、姿态误差限 0.03 rad。监测读取同一物理时间戳的镜片和 TCP 位姿订阅快照，
不在执行等待中同步调用位姿服务；快照超过 0.5 s 未更新则明确报告数据失效。
超差或数据失效进入错误状态
并请求停止双臂执行，因为附着对象预测已不再可信。该过程不会重置仿真或传送镜片。
中心装配保持原抓取变换，不通过重新计算变换掩盖镜片偏移。此检查是采样监测，
不构成连续持片或物理接触安全证明。

`verify_assembly.py --record` 同时采样通用 MuJoCo `get_contacts` 服务。
运行 `python3 test/summarize_contacts.py <记录目录>` 汇总物体对、最小间距和最大法向力；
同时检查首次接触阶段、邻面关系，以及存储器下降离开和夹爪撤退后的接触结束时间。
接口接触必须结合阶段日志审查，不能仅凭分类名自动判定安全。

执行记录与最终六片对合检查：

```bash
python3 src/control/task_control/space_sim/test/verify_assembly.py --record --until-flow-end --output /tmp/assembly_validation
# 流程结束后停止上面的记录程序，再执行只读检查：
python3 src/control/task_control/space_sim/test/verify_assembly.py --output /tmp/assembly_validation
```

只读检查从实际 MuJoCo site 位姿匹配六个中心口，要求六片各占一个口、位置误差
小于 1 mm、姿态误差小于 0.01 rad、无残留附着对象、六片世界碰撞对象齐全，
并检查最终双臂状态。记录程序保存双臂实际运动重叠、控制器时间戳和采样碰撞检查。
