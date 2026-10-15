# space_sim 环境与镜片装配任务

双臂 UR5e 在轨装配场景：卫星下方储存 6 片镜片，双臂把镜片取出并安装到中心镜片（穹顶）的 6 个侧面上。
本文档描述环境几何、接口约定、已实测的可行性结论，以及 MoveIt 装配任务的设计。

相关包：

| 包 | 内容 |
|---|---|
| `space_sim_description` | 几何/运动学描述（`xacro/space_sim.xacro` 为纯描述入口，不含 ros2_control）、`meshes/dome_assembly.txt` 几何源头 |
| `space_sim_mujoco_description` | MuJoCo 侧：`mjcf/space_sim_mujoco.xml`、`config/space_sim_config.json`（site/weld 生成配置）、`config/mujoco_ros2_control_plugins.yaml`、`config/ros2_control/*` |
| `space_sim_moveit_config` | MoveIt 配置：`arm_A` / `arm_B` / `arm_BOTH` 三个规划组 |
| `space_sim_task_control`（规划中） | 装配任务状态机节点 |

---

## 1. 场景与坐标系

### 1.1 命名链

```
world ──(fixed)── base_link ──(fixed)── satellite            # 卫星本体 0.6³ m, 300 kg
                                        ├── len_shelf ── len_center      # 顶部中心镜片 z=+0.40
                                        ├── solar_panel_A / _B
                                        ├── len_dock_pannel             # 底部接口盘, z=-0.31
                                        │    └── len_dock_1..6 ── len_ring_1..6   # 6 个插槽 + 6 片镜片
                                        └── ur_mount_A / ur_mount_B      # x=±0.3, rpy=0,±90°,0
                                             └── arm_X_base_dock ── arm_X_base_main
                                                  └── arm_X_ur_base_link ── … ── arm_X_ur_tool0
                                                       └── arm_X_realsense_shelf ── arm_X_gripper_base
                                                            └── arm_X_gripper_dock (0.1×0.04×0.02)
                                                                 └── arm_X__tcp        # 磁吸接口, 距 base 0.12 m
```

* 两臂均为 **UR5e**，关节名 `arm_{A,B}_ur_shoulder_pan_joint` … `arm_{A,B}_ur_wrist_3_joint`（12 个可动关节）。
* 末端 `arm_X__tcp`（注意双下划线）是磁吸夹爪的对接参考点，**无质量/碰撞几何**；
  `arm_X_gripper_base` 带磁吸夹爪 mesh（碰撞），`arm_X_gripper_dock` 是与镜片侧面凸台配合的 0.1×0.04×0.02 凸台。
* 本场景 `gripper_type=none`：没有开合夹爪，**抓取=磁吸对接**（由 dock 插件激活 weld 实现）。
* TF 中只有 URDF 内的连杆；**6 片镜片在 MuJoCo 里是浮动刚体（freejoint），不在 TF 里**，
  其位姿只能通过 `mujoco_god_plugin` 查询。

### 1.2 对接口 site 约定（本场景所有接口统一）

* `+z` = 对接面**外法向**（背离所属物体、指向配合件）
* `+x` = 接口定位长边方向（本场景 = 0.1 m 长边）
* 原点 = 对接参考点（插合时对方 site 原点落在此点）

于是两个插合的矩形接口相对姿态恒为**绕 y 轴 180°**：`(w,x,y,z) = (0,0,1,0)`，
也就是 dock 插件的 `dock_target`。若把 site1 系内的 180° 相位（绕自身 z）也算合法，
就是矩形的“两种相位”。

6 个侧面的编号（俯视，绕块轴顺时针，`S0` 起）：

| 中心镜片 S | 方位 | 归属 |
|---|---|---|
| S0 | +x 面 | arm_A 半区（正对臂 A） |
| S1 | (+x, −y) | arm_A 半区（臂 A 左） |
| S2 | (−x, −y) | arm_B 半区（臂 B 右） |
| S3 | −x 面 | arm_B 半区（正对臂 B） |
| S4 | (−x, +y) | arm_B 半区（臂 B 左） |
| S5 | (+x, +y) | arm_A 半区（臂 A 右） |

> 臂的“左/右”按**面朝卫星内侧**定义：arm_A（x=+0.3）右=+y、左=−y；arm_B（x=−0.3）右=−y、左=+y。

### 1.3 储存位与穹顶位

* 6 片镜片沿 **y 轴**排成两列，间距 0.1 m（`len_ring_1/2/3` 在 +y，`4/5/6` 在 −y），
  片厚 0.05 m、体心 z≈−0.61，**S0 面朝上**插在 `len_dock_i_site`（z≈−0.34）上，靠 `dock_weld_i` 焊住。
* 每片镜片的 6 个面法向都在**世界 X-Z 平面**内（厚度方向 = ±y）：
  S0=上、S3=下、S1/S5=斜上(±x)、S2/S4=斜下(±x)。
* 穹顶：`len_ring_i` 的 S0 面与 `len_center_S<k>` 对合，环块位于半径 0.513 m、z≈0.49、俯仰 20°。
  模型里 `center_weld_i` 记录了默认对应关系，但**插件在 dock 时按 site 重写 weld**，
  因此“哪片镜片装到哪个面”在运动学上是自由的（本任务用它做随机化）。

---

## 2. 运动与接口清单

### 2.1 控制器（MuJoCo 环境）

`space_sim_mujoco_description/config/ros2_control/traditional_controllers.yaml`

| 控制器 | 类型 | 说明 |
|---|---|---|
| `joint_state_broadcaster` | JointStateBroadcaster | 全局 |
| `arm_{A,B}_ur_joint_trajectory_controller` | JointTrajectoryController | **MoveIt 执行用**（position 指令） |
| `arm_{A,B}_ur_forward_position/velocity/effort_controller` | 位置/速度/力矩组控制器 | 预留 |

> MuJoCo 环境**没有** `scaled_joint_trajectory_controller`，MoveIt 的
> `moveit_controllers_SimpleControllerManager.yaml` 只能指 `joint_trajectory_controller`。

启动：

```bash
# 仿真 + 控制器
ros2 launch launch_controller start_joints_controllers.launch.py \
    env:=space_sim_mujoco initial_ur_controller:=joint_trajectory_controller
# move_group（默认带 RViz）—— demo.launch.py **只起 move_group**，不再起仿真/控制器
ros2 launch space_sim_moveit_config demo.launch.py
# 或统一入口（env 任意已构建的 {env}_moveit_config，带 _mujoco 后缀自动重定向）
ros2 launch launch_controller start_moveit_group.launch.py env:=space_sim
```

### 2.2 dock 插件（`mujoco_dock_sim_plugin`）

阻塞式服务：`/mujoco_dock_sim_plugin/dock`、`/mujoco_dock_sim_plugin/release`
（同名 action 可选），请求含 `site1`、`site2`、`group`、`timeout`，响应含 `success`、`state`、`relpose`。

| group | 参与的 site | 容差 | 对称性 |
|---|---|---|---|
| `default` | 36 个镜片侧面 + `len_center_S0..S5` + `len_dock_1..6_site` | 0.01 m / 0.15 rad | `none`（矩形接口固定相位） |
| `grasp` | 36 个镜片侧面 + `arm_{A,B}__tcp_site` | 0.01 m / 0.15 rad | `discrete` 绕 z [0°,180°] |

* 认定“对接成功”的条件是相对位姿在容差内**持续** `dock_time`（0.5 s），随后插件把 weld 焊到
  “离当前位姿最近的合法目标位姿”。默认超时 6.5 s。
* 具体调用（本任务用到的三组）：

```text
取出镜片: release(site1=len_ring_i_S0, site2=len_dock_i_site)          # 解除储存焊合
磁吸抓取: dock(site1=arm_X__tcp_site, site2=len_ring_i_S<g>, group=grasp)
穹顶装配: dock(site1=len_ring_i_S0, site2=len_center_S<k>, group=default)
释放末端: release(site1=arm_X__tcp_site, site2=len_ring_i_S<g>)
```

### 2.3 god 插件（`mujoco_god_plugin`）

场景读写：服务 `~/get_object_pose`（`type=body|site|geom`、`name`、可选 `frame_id=某 body`
用于查**局部位姿**）、`~/set_object_pose`、`~/get_joint_states`、`~/actuator_state` 等；
话题 `~/pose/{body,site,geom}/<name>`（30 Hz）。

任务节点用它读取：`body:len_ring_i`、`site:len_ring_i_S<f>`（世界系与镜片体系）、
`site:len_center_S<k>`、`site:arm_X__tcp_site`。

---

## 3. 装配任务定义

### 3.1 状态与目标

* **初始态**：6 片镜片分别焊在 6 个储存插槽上（`dock_weld_1..6` 初始 `active=true`）。
* **目标态**：6 片镜片的 S0 面分别与 `len_center_S0..S5` 对合（焊合、无漂移）。
* 装配动作链（单片）：`release 储存 → 磁吸抓取(grasp) → 转运 → 插入中心面(default) → release 末端`。

### 3.2 顺序硬约束（来自需求）

1. **状态机实现**，两条臂**并行**装配各自一侧。
2. 每臂：先装两个**倾斜**对接口（`arm_A: S1, S5` / `arm_B: S2, S4`），
   最后装**正对机械臂**的那个（`arm_A: S0` / `arm_B: S3`）。
3. 存储在某臂**左侧**的镜片只能装到该臂**左上**对接口，右侧相反。
4. 最后那片（正对面）需要**另一臂辅助更换“机械臂↔待装镜片”的对接口**（重新抓取不同侧面）
   —— **本轮不实现**，状态机只保留 `REGRASP_ASSIST` 钩子。
5. 抓取对象、抓取面、相位、以及臂↔镜片分配尽量**随机**，但必须在实测可行集合内。

### 3.3 实测可行性结论（`tmp/space_sim_assembly_probe.py`）

完整数据见 [`tmp/space_sim_feasibility_report.md`](../../../tmp/space_sim_feasibility_report.md)
（720 个候选 = 2 臂 × 6 镜片 × 5 抓取面 × 2 相位 × 6 中心面）。

**抓取（是否有无碰撞接近位姿）**

| 臂 | +y 列（len1/2/3） | −y 列（len4/5/6） |
|---|---|---|
| arm_A | 只能用 **S5** | 只能用 **S1** |
| arm_B | 只能用 **S1** | 只能用 **S5** |

* **只有 S1/S5 可用**：S2/S4 能解出抓取 IK，但 0.1 m 外的接近位姿必然碰撞（被接口盘/相邻镜片挡住），
  S3 完全不可行。
* **不必只取最外侧**：6 片镜片在各自可用面上都能给出无碰撞接近位姿，中间镜片同样可取。
* 与需求 3.2.3 一致：arm_A 左列（−y）→ S1、右列（+y）→ S5；arm_B 左列（+y）→ S1、右列（−y）→ S5。
* 两个相位（0°/180°）在多数组合下都可行，可作为随机维度。

**放置（插入中心镜片侧面）**

| 臂 | 抓取面 | 可行目标面（本臂半区） |
|---|---|---|
| arm_A | S1（−y 列） | `S1` ✓（另有半区外的 S2） |
| arm_A | S5（+y 列） | `S5` ✓、`S0` ✓（另有半区外的 S4） |
| arm_B | S1（+y 列） | `S4` ✓、`S3` ✓（另有半区外的 S5） |
| arm_B | S5（−y 列） | `S2` ✓、`S3` ✓（另有半区外的 S1） |

**随机化空间**：4 个倾斜对接口各有 3 片镜片 × 1~2 个可用抓取面 × 1~2 个相位可选；
满足“两臂取片不重复”的合法分配约 **144 种**（详见报告第 4 节），
足以让每次运行的抓取顺序与分配都不同。

---

## 4. MoveIt 装配任务设计

### 4.1 规划组

| 组 | 类型 | 用途 |
|---|---|---|
| `arm_A` / `arm_B` | 单臂链（base=`arm_X_ur_base_link`，tip=`arm_X__tcp`） | 单臂流水线规划（另一臂在飞目标位姿作为障碍） |
| `arm_BOTH` | 双臂 12 维 | 两臂**同步**规划，保证同时运动时轨迹自洽无互碰 |

* `move_group` 一个节点同时服务三个组；任务节点为每条臂建独立 node + 独立线程
  （`MoveGroupInterface` 是同步阻塞的，不这样无法并行）。
* `move_group_common.yaml` 中 **`moveit_manage_controllers: false` 必须保留**：
  执行只是向对应 JTC 发 `FollowJointTrajectory`，两条臂的轨迹才能真正并发执行。

### 4.2 目标位姿推导（全部由 site 帧在运行时解出）

```text
抓取:  T_tcp = T(len_ring_i_S<g>) · Q_DOCK · Rz(-phase)          # phase ∈ {0°, 180°}
放置:  T(len_ring_i_S0) = T(len_center_S<k>) · Q_DOCK
       T_lensBody = T(len_ring_i_S0) · inv(T_S0_in_lenBody)
       T_tcp = T_lensBody · inv(T_tcp_lens)
       （T_tcp_lens = inv(T_world_tcp)·T_world_lens，抓取成功瞬间由 god 插件读一次，刚性不变）
接近:  pre-grasp / pre-place = 目标位姿沿自身 -z 平移 approach（默认 0.1 m）
```

* `T_S0_in_lenBody` 由 god 插件以 `frame_id=len_ring_i` 查询 S0 site 得到。
* MoveIt 的 pose goal 用 `frame_id="world"`（模型系，不需要 TF；用 `base_link` 会因缺 TF 报 `-21`）。

### 4.3 状态机（每条臂一个 ArmAgent 实例）

```mermaid
stateDiagram-v2
  [*] --> IDLE
  IDLE --> TAKE_NEXT: 队列非空
  TAKE_NEXT --> APPROACH_PREGRASP: ConflictGuard 通过 / 换队列项 / 升级 dual
  APPROACH_PREGRASP --> DESCEND_GRASP: MoveIt 规划+执行
  DESCEND_GRASP --> DOCK_TCP: 笛卡尔直线插入
  DOCK_TCP --> LIFT: dock(grasp) 成功
  LIFT --> TRANSFER --> APPROACH_PREPLACE
  APPROACH_PREPLACE --> DESCEND_PLACE: MoveIt 规划+执行
  DESCEND_PLACE --> DOCK_CENTER: 笛卡尔直线插入
  DOCK_CENTER --> RELEASE_TCP: dock(default) 成功
  RELEASE_TCP --> RETREAT --> TAKE_NEXT
  TAKE_NEXT --> [*]: 队列空
  APPROACH_PREGRASP --> ERROR: 规划/IK/dock 失败
  ERROR --> TAKE_NEXT: 换面 / 换片 / 换相位 / 换队列项
  IDLE --> REGRASP_ASSIST: 正对面需换面（本轮 stub）
```

* 队列里的备选项来自随机化后的可行集合，**先完成先取下一项**，另一臂不必等待。
* `planning_mode: auto` —— 两臂下一段落在同一共享区域时用 `arm_BOTH` 同步规划，否则单臂规划；
  单臂模式下另用时间栅格采样检查新轨迹与另一臂在飞轨迹是否互碰。

### 4.4 运行

```bash
colcon build --symlink-install --packages-select space_sim_moveit_config space_sim_task_control

# 仿真 + 控制器
ros2 launch launch_controller start_joints_controllers.launch.py \
  env:=space_sim_mujoco initial_ur_controller:=joint_trajectory_controller \
  mujoco_headless:=true launch_rviz:=false
# move_group（demo.launch.py 现在只起它；也可用统一入口 env:=space_sim）
ros2 launch space_sim_moveit_config demo.launch.py launch_rviz:=false
# 可行性探测 / 复核
python3 tmp/space_sim_assembly_probe.py --stage ik
python3 tmp/space_sim_probe_report.py
# 装配任务（dry_run 先看随机方案；规划器默认 chomp，可用 planner:= 换）
ros2 launch space_sim_task_control space_sim_lens_assembly.launch.py dry_run:=true
```

---

## 5. 工具与验证脚本

| 脚本 | 作用 |
|---|---|
| `tmp/space_sim_moveit_smoke.py` | MoveIt 配置冒烟：rest 态有效性、FK/IK、折叠臂必须判碰撞、`arm_BOTH` 12 维规划、单臂规划 |
| `tmp/space_sim_assembly_probe.py` | 可行性探测：枚举候选 → IK（含/不含碰撞检查）→ 可选路径规划，产出能力矩阵 YAML |
| `tmp/space_sim_probe_report.py` | 把探测数据汇总成可行性报告（抓取/放置矩阵、结论、随机化空间） |
| `tmp/gen_space_sim_moveit_srdf.py` | 生成 SRDF 的 `disable_collisions`（见下） |
| `tmp/check_mjcf.py`、`tmp/e2e_test.sh`、`tmp/check_dock_symmetry.py` | 原有 MJCF/dock 语义回归 |

### SRDF 碰撞矩阵的裁剪规则

`tmp/gen_space_sim_moveit_srdf.py` 调用 MoveIt 的无头工具（`collisions_updater`，即 Setup Assistant
所用的同一算法）生成候选，然后**只保留可证明安全**的条目：

* 全部 `reason="Adjacent"`（焊接相邻，网格已被验证不互相穿透）；
* `reason="Never"` 中**两个 link 属于同一刚性连通分量**（仅用 fixed joint 连接 ⇒ 相对位姿恒定，
  “采样从未碰撞”即“永不碰撞”）。

其余（尤其是**跨臂**与**臂↔卫星/镜片**）一律**不**禁用 —— 双臂并行装配必须真检查这些对。
当前 SRDF 共 308 条 disable（43 Adjacent + 265 同刚体），206 对保持激活；
已用“把 arm_A 折叠进卫星必须判为无效”验证（得到 9 个接触点）。

---

## 6. 已知限制与风险

1. **正对面（A=`S0` / B=`S3`）的最后一装**需要另一臂辅助换面（重新抓取镜片的另一个侧面），
   本轮未实现；因此本轮验收范围为 4 片倾斜对接口。
2. MoveIt 的规划场景里被搬运镜片仍固定在储存位（URDF 里镜片是 fixed link）：
   搬运期间需用 ACM 放开该镜片，装配完成后按实测位姿把它加回为碰撞体。
3. 探测只做 IK 判定；IK ✓ 仍需路径规划复核（`--stage plan`）。
4. 接近距离（默认 0.1 m）会影响判定；改动后应重跑探测。
5. 储存位/穹顶几何若重新生成 MJCF，需要重跑探测与 SRDF 生成。
