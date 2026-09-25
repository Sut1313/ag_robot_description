# ag_robot_description

**AG_robot** —— 一台四轮独立悬挂差速底盘 + 立柱/导轨丝杠升降台 + 两条松灵 **AgileX PiPER** 六轴机械臂 + **Velodyne VLP-16** 3D 激光雷达的移动操作机器人，用 URDF/xacro 描述，配套 RViz 显示与 Gazebo Harmonic 仿真。

本包**完全自包含**：24 个网格全部在 `meshes/` 下，不依赖任何其他自研包。

```
底盘 ── 4 轮差速 + 独立悬挂 ── 立柱(3030 型材,T 型槽) ── 丝杠升降台(FBX150 推杆, 行程 400 mm)
                                                        └─ 两条 PiPER 六轴臂
```

---

## 一、功能与内容

### 1. 模型结构

28 个 link / 27 个关节（10 固定 + 4 连续 + 12 旋转 + 1 平移），单一根节点：

```
base_footprint                          地面投影帧, Nav2 用; 无几何、无惯量
└─ base_link                            底盘(车架+顶板+4 套 MD60 独立悬挂+电池+主控) + 固定在底盘上的平置导轨
   ├─ imu_link                          IMU 坐标系, 无几何(IMU 集成在主控上)
   ├─ wheel_1 … wheel_4                 四轮, continuous, 差速驱动
   └─ base_frame_link                   立柱 + 竖直导轨 + FBX150 电动推杆 (固定)
      ├─ velodyne_link                  VLP-16 雷达，暂用圆柱体，固定在立柱顶部正中心
      └─ lift_platform_link             升降台: 卡在立柱两侧竖向 T 型槽里上下滑移 (prismatic, 0–0.40 m)
         ├─ arm1_base_link … arm1_tool0 松灵 PiPER 六轴臂 (revolute ×6 + 法兰 + tool0)
         └─ arm2_base_link … arm2_tool0 松灵 PiPER 六轴臂
```

| 能力 | 说明 |
|---|---|
| 移动 | 四轮差速，`wheel_separation = 0.6038 m`、`wheel_radius = 0.11349 m` |
| 升降 | 单自由度丝杠升降台，`lift_joint` 行程 `0 … 0.40 m`（`lift_travel` 参数可改） |
| 操作 | 两条 6 轴臂，末端 `arm{1,2}_tool0` 法兰帧（外端面 z = 0.0105 m）备用夹爪 |
| 传感器 | VLP-16 3D 雷达：16 线、360°、10 Hz，Gazebo 发布点云并由 RViz 默认显示；相机暂未启用 |

### 2. 仿真与显示

| 项 | 内容 |
|---|---|
| 显示 | `launch/display.launch.py`：robot_state_publisher + joint_state_publisher_gui 滑条 + RViz2 |
| 仿真 | `launch/gazebo.launch.py`：Gazebo **Harmonic**（gz-sim 8）+ `gz_ros2_control` + VLP-16 话题桥接 + RViz |
| 世界 | `worlds/ag_robot.sdf`：斜射太阳、阴影及 3 个雷达演示障碍物（见第七节） |
| 控制器 | 5 个：`joint_state_broadcaster`、`diff_drive_controller`、`lift_controller`、`arm1_controller`、`arm2_controller`（后三个都是 `JointTrajectoryController`） |

---

## 二、目录结构

```
ag_robot_description/
├── urdf/
│   ├── AG_robot.urdf.xacro        整车模型（要改就改这个）
│   ├── piper_arm.xacro            单条 PiPER 六轴臂宏（参数核对过官方 URDF）
│   ├── gazebo.xacro               gz_ros2_control + 轮子摩擦 + VLP-16 GPU lidar
│   └── sensors_disabled.xacro     旧版 2D 雷达/相机定义，当前未 include
├── meshes/
│   ├── *.stl                      车体 8 个（底盘/导轨/立柱/升降台/4 轮，毫米制）
│   │                              + 臂碰撞 8 个（官方网格，米制）
│   │                              + 旧传感器 2 个（car_camera_link / car_laser，当前未引用）
│   ├── *.dae                      臂视觉 8 个（官方网格，米制，不加 scale）
│   └── *.obj + ag_robot.mtl       SolidWorks 导出的**逐部件配色源数据**，当前不参与渲染，且不进版本库（见 .gitignore）
├── config/ros2_control.yaml       差速 + 升降 + 双臂 共 5 个控制器
├── launch/
│   ├── display.launch.py          RViz
│   └── gazebo.launch.py           gz sim + ROS 桥接 + RViz
├── rviz/AG_robot.rviz             RViz 配置（机器人 + TF + /velodyne_points，Fixed Frame = base_link）
├── scripts/                       雷达测量与校验工具（仿真运行时执行，见七.7）
│   ├── lidar_blind_zone.py        抓一帧点云做盲区分类统计
│   ├── lidar_target_check.py      近场靶标校验（配 worlds/lidar_blindzone_targets.sdf）
│   └── lidar_capture_png.py       抓点云出图（俯视 + 3D，需 matplotlib）
├── worlds/
│   ├── ag_robot.sdf               仿真世界（注意：世界名是 ag_robot，不是 empty）
│   └── lidar_blindzone_targets.sdf 近场盲区校验靶板（0.40/0.48/0.52/0.68 m 四块）
├── .gitignore
├── CMakeLists.txt
└── package.xml
```

---

## 三、依赖与安装

### 1. 环境要求

- **ROS 2 Humble**（本包在 Humble 上开发验证）
- **Gazebo Harmonic**（`gz sim` 8.x）。**不要**用 Gazebo Classic：`gazebo.xacro` 走的是 `gz_ros2_control` 那套插件，Classic 里不适用

### 2. apt 依赖

```bash
# ROS 侧
sudo apt install ros-humble-xacro ros-humble-robot-state-publisher \
     ros-humble-joint-state-publisher-gui ros-humble-rviz2 \
     ros-humble-ros2-control ros-humble-ros2-controllers \
     ros-humble-controller-manager ros-humble-diff-drive-controller \
     ros-humble-joint-trajectory-controller ros-humble-joint-state-broadcaster

# Gazebo 侧
sudo apt install gz-harmonic ros-humble-ros-gzharmonic-sim ros-humble-ros-gzharmonic-bridge
```

### 3. `gz_ros2_control` 需要源码编译（重要）

Humble 的 apt 源里**没有** `ros-humble-gz-ros2-control`，所以这一项必须自己编：

```bash
mkdir -p ~/gz_ros2_control_ws/src && cd ~/gz_ros2_control_ws/src
git clone https://github.com/ros-controls/gz_ros2_control -b humble
cd ~/gz_ros2_control_ws && rosdep install -r --from-paths src -i -y --rosdistro humble
colcon build
```

### 4. 编译本包

```bash
mkdir -p ~/ag_robot_ws/src && cd ~/ag_robot_ws/src
# 把 ag_robot_description 放进来（git clone 或直接拷贝）
cd ~/ag_robot_ws && colcon build && source install/setup.bash

# 每次开新终端：ROS 环境 → gz_ros2_control → 本包，顺序不能反
source /opt/ros/humble/setup.bash
source ~/gz_ros2_control_ws/install/setup.bash
source ~/ag_robot_ws/install/setup.bash
```

> **不需要手动设 `GZ_SIM_RESOURCE_PATH`**：`gazebo.launch.py` 会把包的 share 目录自动加进去。少了这一步，`model://ag_robot_description/...` 会解析失败、30 多个网格全部加载不出来（模型只剩空壳）。

---

## 四、运行方法

### 1. RViz（只看模型、手动摆关节）

```bash
ros2 launch ag_robot_description display.launch.py
# 只开 RViz、不要滑条窗口：gui:=false
```

会打开 `joint_state_publisher_gui` 滑条，拖动即可看升降台和两条臂的运动。

> ⚠ **不要和 Gazebo 仿真同时开**：滑条窗口会往 `/joint_states` 发消息，和仿真里的
> `joint_state_broadcaster` 互相打架，TF 会抖。看 RViz 时先关掉仿真。

### 2. Gazebo Harmonic 仿真

```bash
ros2 launch ag_robot_description gazebo.launch.py                # 带界面
ros2 launch ag_robot_description gazebo.launch.py gazebo_gui:=false   # 无界面（服务器端）
ros2 launch ag_robot_description gazebo.launch.py rviz:=false     # 不打开 RViz
```

启动流程：`gz sim` 加载 `worlds/ag_robot.sdf` → `ros_gz_sim create` 在 z=0.003 处生成 `AG_robot` → 桥接 `/clock`、雷达 LaserScan 和 PointCloud2 → 打开 RViz → `robot_state_publisher` → 依次加载 5 个控制器。

### 3. 控制器与话题

| 话题 / 动作 | 类型 | 说明 |
|---|---|---|
| `/diff_drive_controller/cmd_vel_unstamped` | `geometry_msgs/msg/Twist` | 底盘速度指令（**注意是 unstamped 话题、Twist 而不是 TwistStamped**） |
| `/diff_drive_controller/odom` | `nav_msgs/msg/Odometry` | 里程计（同时发布 `odom → base_link` 的 TF） |
| `/lift_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 升降台（关节 `lift_joint`，单位 m） |
| `/arm1_controller/follow_joint_trajectory`、`/arm2_controller/…` | 同上 | 两条臂（各 6 关节，rad） |
| `/joint_states` | `sensor_msgs/msg/JointState` | 17 个可动关节 |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 完整 16 线 3D 点云，RViz 默认显示 |
| `/velodyne_scan` | `sensor_msgs/msg/LaserScan` | 底层扫描调试；消息不含垂直维度，3D 应用请使用 PointCloud2 |

```bash
# 底盘：前进 0.3 m/s（顶一下 Ctrl+C 停）
ros2 topic pub --rate 20 /diff_drive_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

# 原地左转 0.5 rad/s
ros2 topic pub --rate 20 /diff_drive_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# 升降台升到 0.15 m
ros2 action send_goal /lift_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [lift_joint], points: [{positions: [0.15], time_from_start: {sec: 2}}]}}"

# 看控制器状态 / 关节状态 / TF
ros2 control list_controllers
ros2 topic echo --once /joint_states
ros2 run tf2_tools view_frames        # 生成 frames.pdf
```

---

## 五、坐标系与 TF

### 1. TF 树

```
base_footprint
└─ base_link ─┬─ imu_link
              ├─ wheel_1 … wheel_4
              └─ base_frame_link ── lift_platform_link ─┬─ arm1_base_link → arm1_link1…6 → arm1_flange_link → arm1_tool0
                                                        └─ arm2_base_link → arm2_link1…6 → arm2_flange_link → arm2_tool0
```

### 2. IMU

`imu_link` 是**纯坐标系、无几何**（IMU 集成在主控上，没有独立安装件）：

- 在 `base_link` 下：`xyz = (0, 0, −0.0754) m`，`rpy = (0,0,0)`（XY 居中、顶板下表面之下 75.4 mm）
- 换算到 `base_footprint`：`(0, 0, 0.1431) m`
- 发 `sensor_msgs/Imu` 时用它做 frame；装在别处就改 `imu_joint` 的 origin

### 3. ⚠ `base_link` 的轴约定与 ROS 惯例不一致（**未处理，见第八节**）

`base_link` 沿用 SolidWorks 装配坐标（网格就是这套坐标）：

| 轴 | 车体含义 |
|---|---|
| `base_link` ±x | **左右**（四个轮子沿 x 排布，轮距 603.8 mm 就是 x 向间距）|
| `base_link` ±y | **行进方向**（前后轮沿 y 排布，轴距 400 mm）|
| `base_link` +z | 竖直向上 |

立柱与两条臂都在 `base_link` 的 **−y** 一侧。`base_joint` 的 `rpy = (0,0,+π/2)` 只决定整车在世界里的朝向，不改变上面这些内部含义。

因此 `cmd_vel.linear.x` 实际让车沿 `base_link` **+y** 走，而 `diff_drive_controller` 的里程计按 `base_link` **+x** 报，两者方向差 90°。详见第八节第 1 条。

---

## 六、关键尺寸与物理参数

### 尺寸（CAD 实测）

| 量 | 值 |
|---|---|
| 底盘外形 | 547 × 550 × 162 mm |
| 轮径 / 轮宽 | Ø227 / 73 mm（含胎纹 Ø240） |
| 轴距 / 轮距 | 400.0 / 603.8 mm |
| 轮心高度 | 218.5 mm |
| 立柱 | 4 根竖直 3030 型材（2×300 mm + 2×200 mm）|
| 升降驱动 | FBX150 有效行程 700 mm 电动推杆、本体 909 mm；URDF 内行程限 400 mm |
| 两臂间距 | 377.732 mm（沿 x），臂安装面 z = 32 mm |
| 立柱顶 / 推杆顶 | 离地 1.13 m / 1.34 m |

### 质量与惯量（当前 URDF 内的值，整车合计约 **79.14 kg**）

| link | 质量 (kg) | 惯量 origin (m) | ixx / iyy / izz |
|---|---|---|---|
| `base_link` | 19.352 | (0, 0, −0.052246) | 0.539 / 0.534 / 0.970 |
| `wheel_1..4` | 5.471 ×4 | (0, 0.0072, 0) | 0.0266 / 0.0470 / 0.0266 |
| `base_frame_link` | 25.293 | (0, −0.114898, 0.455004) | 1.783 / 2.280 / 0.557 |
| `lift_platform_link` | 3.602 | (0, −0.202447, 0.117) | 0.026 / 0.091 / 0.085 |
| 臂连杆 ×7 + 法兰 | 4.210 / 条 | 官方值 | 官方完整惯量张量 |
| `velodyne_link` | 0.590 | (0, 0, 0) | 0.000547 / 0.000547 / 0.000584 |

> **这些是估算值，不是实测**：车体/轮/立柱/升降台按"网格体积 × 密度"算质量（铝 2.7，FBX150 推杆按钢 7.85），惯量按"包围盒均质长方体"近似；只有机械臂用的是官方 URDF 的完整惯量。要接实际控制器，建议用实测质量替换。**`base_frame_link` 的 25.3 kg 尤其可疑**（推杆按钢算偏重，真实 FBX150 大约 8–15 kg）。

### 机械臂安装点（用户实测值）

```xml
<!-- arm1 -->  <origin xyz=" 0.187732 -0.249047 0.032000" rpy="0 0 -1.5707963"/>
<!-- arm2 -->  <origin xyz="-0.190000 -0.247667 0.032000" rpy="0 0 -1.5707963"/>
```

臂本体参数（关节限位/速度/惯量/网格）与官方仓库 [agilexrobotics/agx_arm_urdf](https://github.com/agilexrobotics/agx_arm_urdf) 的 `piper_description.urdf` 逐项核对一致。

---

## 七、网格与显示说明

### 1. 单位

| 网格 | 单位 | scale |
|---|---|---|
| 车体 `*.stl`（底盘/导轨/立柱/升降台/四轮）| 毫米 | `0.001`（集中在 `mesh_scale` 属性一处）|
| 臂 `*.dae` / `*.stl`（官方网格）| 米 | **不加 scale** |

### 2. 车体视觉用 `.stl` 而不是 `.obj`（**颜色问题**）

SolidWorks 导出的 `.obj` 自带逐部件颜色（`ag_robot.mtl` 里 9 档灰），但在 Gazebo Harmonic 里**给 `.obj` 挂材质无效** —— 加载器会给网格带上一份内嵌材质，网格自带材质优先于 visual 上的材质，结果无论 URDF 里写什么颜色都渲染成默认白。实测对照：同样一份显式材质，挂在纯 `box` 几何上能渲染出洋红，挂在 `.obj` 网格上无效。

所以车体视觉统一改用**无内嵌材质**的 `.stl`（与碰撞网格同一份文件），颜色由每个 `<visual>` 的 `<material>` 决定；`.obj` 与 `.mtl` 作为逐部件配色源数据留在本地、不进版本库。

### 3. 车体网格写入了逐面法线

这 8 个车体 `.stl` 原始文件的法线**全是 0**，渲染器只能自己合成、并按相邻面平均（平滑着色），于是所有面与面的交界被抹圆、薄壁凹槽糊成一团。2026-09-25 已用三角形叉积**逐面写入真实单位法线**（顶点字节一个未动，几何/质量/惯量/碰撞完全不变），棱角恢复清晰。备份见仓库外的 `mesh_backup_zonormals/`。

> 若以后换回 `.obj` 做视觉，必须同时补写顶点法线（这些 `.obj` 里 `vn` 数量为 0），否则又会糊。

### 4. 世界文件的光照是刻意调的

`empty.sdf` 的太阳是 `<pose>0 0 10 0 0 0</pose>`，**垂直向下照**：所有竖直面 N·L≈0、只剩环境光，模型就没有明暗层次。`worlds/ag_robot.sdf` 里太阳斜射 + 环境光由 0.4 降到 0.22 + 一盏反向弱补光 + 地面压暗，结构层次才出来。想更亮就把 `<scene><ambient>` 调回 0.3 左右。

### 5. 现在的配色

| 部件 | 材质 | 观感 |
|---|---|---|
| 底盘 `chassis.stl` | `grey 0.28 0.30 0.34` | 深钢灰 |
| 立柱 `base_frame.stl` | `steel 0.52 0.54 0.57` | 中灰 |
| 升降台 / 导轨 | `alu 0.78 0.79 0.81` | 亮铝 |
| 四个轮子 | `black 0.06 0.06 0.07` | 近黑 |
| 两条臂 | 官方 `.dae` 自带材质 | 深灰 + 蓝（accent） |

> 颜色值写在 `urdf/AG_robot.urdf.xacro` 顶部的 `<material>` 定义里。注意 **sdformat 把 URDF 颜色转 SDF 时会统一 ×1.25**，所以按 URDF 值调色要留这个余量。改完 `colcon build` 重启仿真即可。

### 6. VLP-16 临时模型与扫描参数

雷达临时使用直径 89 mm、高 72 mm 的圆柱体，固定在 `base_frame_link` 顶部正中心。立柱网格实测中心为 `(0, -0.114898)` m、顶面 `z=0.911` m，所以 `velodyne_joint` 的暂定位姿是 `(0, -0.114898, 0.947)` m。正式 CAD 网格完成后，替换 `velodyne_link` 的 visual/collision，再精调这一处 origin 即可。

Gazebo 使用 `gpu_lidar` 模拟当前官方 VLP-16：水平 360°、垂直 40°、16 线、量程 0.5–200 m、距离分辨率 1 cm。官方 ±3 cm 精度近似按 ±3σ 建模，因此 Gaussian 噪声标准差取 1 cm。当前设为 10 Hz、每圈 1800 个水平采样，即约 28.8 万点/秒；改成 20 Hz 时约 57.6 万点/秒，接近官方标称的约 60 万点/秒。世界中额外放置了墙、方箱和圆柱，便于直接观察 3D 扫描轮廓。

### 7. 盲区实测（仿真测定）

工具都在 `scripts/` 下，仿真运行时执行：

| 脚本 | 作用 |
|---|---|
| `lidar_blind_zone.py` | 抓一帧点云，按"地面 / 本体 / 环境 / 无回波"分类，输出逐扫描线统计与盲区锥面 |
| `lidar_target_check.py` | 近场靶标校验，配合 `worlds/lidar_blindzone_targets.sdf` 使用 |
| `lidar_capture_png.py` | 抓点云出图（俯视 + 3D 双面板），需 matplotlib |

用 `lidar_blind_zone.py` 在三种姿态下实测：升降台 0 mm、升降台 400 mm、升降台 400 mm + 双臂 `joint2=1.2 rad` 抬起（关节角均已核实到位）。

**结论一：本体不构成盲区。** 三种姿态下 **0 条射线打到机器人自身** —— 雷达装在整车最高点，底盘、升降台、机械臂安装面都在 ±20° 视场之下，立柱顶面又落在 0.5 m 最小量程之内。所以雷达"看得过"整车，升降台升到顶、双臂抬起都不挡。

**结论二：真正的盲区是近场地面盲环，半径 3.39 m。** 各扫描线地面最近距离的实测值与理论 `h/sinθ` 逐条吻合：

| 扫描线 | 俯仰 | 地面最近（实测 / 理论） |
|---|---|---|
| 0 | −20.00° | **3.39 / 3.44 m** |
| 1 | −17.33° | 3.90 / 3.95 m |
| 2 | −14.67° | 4.59 / 4.64 m |
| 3 | −12.00° | 5.58 / 5.65 m |
| 4 | −9.33° | 7.15 / 7.25 m |
| 5 | −6.67° | 9.94 / 10.13 m |
| 6 | −4.00° | 16.40 / 16.85 m |
| 7 | −1.33° | 47.94 / 50.52 m |

由此得到可视锥面 **h(d) = 1.1755 − 0.364·d**（d = 水平距离）：

| 水平距离 | 能看到的物体高度 |
|---|---|
| 1.0 m | 81 cm 以上 |
| 2.0 m | 45 cm 以上 |
| 3.0 m | 8 cm 以上 |
| ≥ 3.4 m | 地面进入视野 |

换算成"多大的障碍物在多近就看不见"：5 cm → 3.09 m，10 cm → 2.95 m，20 cm → 2.68 m，30 cm → 2.41 m，50 cm → 1.86 m。

**其它盲区**：垂直 ±20° 之外全盲（注意真实 VLP-16 为 ±15°，若按真实值收窄，盲环会扩大到 4.39 m）；0.5 m 最小量程是一个硬性近球。世界演示障碍物（墙/方箱/圆柱）背后的"无地面回波"扇区属正常遮挡，不是雷达缺陷。

**结论三：近场靶标校验（第二种测法）。** 在雷达四周 0.40 / 0.48 / 0.52 / 0.68 m 处各放一块 200×200 mm 靶板（`worlds/lidar_blindzone_targets.sdf`，直接 `gz service` spawn 到世界原点即与 `velodyne_link` 对齐），用 `lidar_target_check.py` 校验：

| 靶板最近面 | 预期 | 实测 | 结果 |
|---|---|---|---|
| 0.40 m | 最小量程内，应无回波 | 靶板处无回波，射线透过打到 1.78 m 处 | ✅ |
| 0.48 m | 最小量程内，应无回波 | 靶板处无回波 | ✅ |
| 0.52 m | 应测到约 0.52 m | **0.527 m** | ✅ |
| 0.68 m | 应测到约 0.68 m | **0.684 m** | ✅ |

这条独立验证了 0.5 m 最小量程边界与近场测距精度（在分辨率 1 cm 量级内吻合）。

**两点提示**：① 该盲环是"顶部装 3D 雷达"方案的固有代价，要覆盖 0–3 m 近场需另加近场传感器（车头 2D 雷达 / 超声波 / ToF）；② 实测点云约 3.7 Hz（配置 10 Hz），因为仿真实时率约 0.54×，做感知算法测试时需注意该频率。

---

## 八、已知问题与待办

### 1. ⚠ `base_link` 的轴约定（最需要决定的一条）

**实测现象**（在 Gazebo 里用世界位姿交叉核对）：

- 四个轮子的轴线沿 `base_link` **x**，轮距 603.8 mm 也是 x 向 → **x 是左右轴**
- 发 `linear.x = +0.3`，车实际沿 `base_link` **+y** 移动，位移大小与轮径完全吻合（无打滑）；而 `diff_drive_controller` 的 odom 按 +x 报 → **odom 方向与真实运动差 90°**
- TF 里 `base_link` 有**两个父节点**：`base_footprint`（robot_state_publisher 静态发）和 `odom`（diff_drive 动态发）→ TF 树歧义

**建议修法**（改哪个都会动到你现在的坐标约定，所以留给你定）：

- 最小改动：`config/ros2_control.yaml` 里 `base_frame_id: base_link` → `base_footprint`，至少消掉双父歧义；
- 彻底符合 REP-103（x=前、y=左）：在 `base_footprint` 与 `base_link` 之间插一个 `cad_link`（把现有 `base_link` 的内容整体挪进去，用固定关节带 −90° yaw），让 `base_link` 变成真正的"前方为 +x"的空帧 —— 模型外观不动、臂的安装点语义也不变，但 `cmd_vel`/odom 就对上了。

### 2. 其它

1. **VLP-16 仍是临时几何和暂定位姿**：扫描链路已接通；正式 mesh 完成后再替换外形并精调安装位姿。相机仍未接入。
2. **质量与惯量为估算**（见第六节），接实际控制器前建议用实测值替换。
3. **升降台没有滑块本体**：CAD 里没有卡在 T 型槽里滑动的滑块零件，所以仿真里升降台是"悬空"滑动的，视觉上不贴合。
4. **立柱不作联动**：几何完整保留，要接关节只需改 `base_frame_joint` 的 `type`。

---

## 九、更新记录（2026-09-25）

| 修的问题 | 原因 | 结果 |
|---|---|---|
| 视觉体整车只有 0.55 mm 大 | 8 个 `.obj` 是米制导出的，却被套上了给毫米制 `.stl` 用的 `mesh_scale = 0.001` | 新增 `mesh_scale_obj = 1 1 1` 专供 `.obj`（现在视觉已改 `.stl`，此属性留作备用） |
| 30 多个网格全部加载失败、模型只剩空壳 | `gazebo.launch.py` 没设 `GZ_SIM_RESOURCE_PATH`，`model://ag_robot_description/...` 无法解析 | launch 内自动追加包的 share 目录，开箱即用 |
| 网格解析/材质赋值一堆告警 | `gazebo.xacro` 用了 `Gazebo/Grey`、`Gazebo/Black` 这类 Ogre 材质脚本名，Harmonic 不支持 | 改为 SDF 原生材质；Ogre 告警 6 → 0 |
| 车体一片白、没有颜色 | `.obj` 网格自带内嵌材质，优先级高于 visual 材质，导致颜色写什么都没用 | 车体视觉改 `.stl` + 显式配色，并调亮/调深了整套调色板 |
| 底盘导轨那一块渲染成纯黑 | 同一 link 的第二个 visual 用 `<material name="..."/>` 引用时，sdformat 转出的 diffuse 会变成 `0 0 0` | `base_link` 的两处改用内联 `<color>` |
| 模型发糊、没有棱角 | 8 个车体 `.stl` 法线全为 0，渲染器平滑平均了法线；默认世界太阳垂直向下也没有明暗层次 | 写入逐面法线 + 新增 `worlds/ag_robot.sdf`（斜射太阳/低环境光/补光/阴影） |
| IMU | — | 新增 `imu_link`（无几何），位于 `base_link` 下方 0.0754 m |
| VLP-16 3D 雷达 | 正式 CAD 尚未完成 | 立柱顶部新增临时圆柱 link；Gazebo 16 线 GPU lidar、ROS 桥接、RViz 点云及演示障碍物已接通 |
| 雷达盲区实测 | 需要确认自遮挡与近场覆盖 | 实测三种姿态下本体遮挡 0 条射线；近场地面盲环半径 3.39 m，附可视锥面公式与实测脚本（见七.7） |

---

## 十、许可与维护

- **License**：[MIT](LICENSE) — Copyright (c) 2026 Sut1313
- **Maintainer**：`Sut1313`（见 `package.xml`）
- **第三方素材署名**：`meshes/piper_*.dae`、`meshes/piper_*.stl` 以及 `urdf/piper_arm.xacro` 里的关节参数
  来自 AgileX Robotics 的 [agx_arm_urdf](https://github.com/agilexrobotics/agx_arm_urdf)，
  同样以 **MIT** 许可分发（Copyright (c) 2026 aalicecc）；本包按 MIT 条款保留其版权与许可声明，
  详见 [`NOTICE`](NOTICE)，网格文件本体未做修改。
