import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# base_footprint 贴地时车轮正好落地, spawn 抬一点点避免初始穿模被弹起来。
SPAWN_Z = '0.003'


def generate_launch_description():
    pkg = get_package_share_directory('ag_robot_description')
    xacro_file = os.path.join(pkg, 'urdf', 'AG_robot.urdf.xacro')
    rviz_cfg = os.path.join(pkg, 'rviz', 'AG_robot.rviz')

    # URDF 里的 package://ag_robot_description/meshes/x.stl 由 sdformat 改写成
    # model://ag_robot_description/meshes/x.stl, 而 gz sim 只会去 GZ_SIM_RESOURCE_PATH
    # 的各个目录下找名为 ag_robot_description 的子目录。不把 share 目录的父目录加进去,
    # 33 个网格会全部报 "Unable to find file with URI [model://...]", 模型只剩一堆空壳。
    share_parent = os.path.dirname(pkg)
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(
        [share_parent, pkg] +
        ([os.environ['GZ_SIM_RESOURCE_PATH']] if os.environ.get('GZ_SIM_RESOURCE_PATH') else []))

    # 树根是 base_footprint(不是 world), 所以 sdformat 不会把模型钉在世界坐标系上,
    # 差速驱动可以直接跑。use_gazebo:=true 只加 gz_ros2_control 的硬件接口和插件。
    # lidar_pitch 与 D435i 参考内参都在 launch 时透传给 xacro；拿到实机标定后
    # 可直接从命令行覆盖，不需要改 URDF 或重新编译。
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_gazebo:=true lidar_pitch:=', LaunchConfiguration('lidar_pitch'),
            ' camera_depth_width:=', LaunchConfiguration('camera_depth_width'),
            ' camera_depth_height:=', LaunchConfiguration('camera_depth_height'),
            ' camera_depth_fx:=', LaunchConfiguration('camera_depth_fx'),
            ' camera_depth_fy:=', LaunchConfiguration('camera_depth_fy'),
            ' camera_depth_cx:=', LaunchConfiguration('camera_depth_cx'),
            ' camera_depth_cy:=', LaunchConfiguration('camera_depth_cy'),
            ' camera_depth_hfov:=', LaunchConfiguration('camera_depth_hfov'),
            ' camera_color_width:=', LaunchConfiguration('camera_color_width'),
            ' camera_color_height:=', LaunchConfiguration('camera_color_height'),
            ' camera_color_fx:=', LaunchConfiguration('camera_color_fx'),
            ' camera_color_fy:=', LaunchConfiguration('camera_color_fy'),
            ' camera_color_cx:=', LaunchConfiguration('camera_color_cx'),
            ' camera_color_cy:=', LaunchConfiguration('camera_color_cy'),
            ' camera_color_hfov:=', LaunchConfiguration('camera_color_hfov'),
            ' camera_clip_near:=', LaunchConfiguration('camera_clip_near'),
            ' camera_clip_far:=', LaunchConfiguration('camera_clip_far'),
            ' camera_update_rate:=', LaunchConfiguration('camera_update_rate'),
            ' camera_imu_update_rate:=', LaunchConfiguration('camera_imu_update_rate'),
        ]),
        value_type=str)

    gazebo_gui = LaunchConfiguration('gazebo_gui')
    rviz = LaunchConfiguration('rviz')

    # 用自己的世界而不是 gz 自带的 empty.sdf: empty.sdf 的太阳垂直向下照, 竖直面只剩
    # 环境光, 整个模型没有明暗层次、看着发糊。worlds/ag_robot.sdf 里太阳是斜的、
    # 环境光压低, 结构棱角才分得出来。
    world = os.path.join(pkg, 'worlds', 'ag_robot.sdf')
    if not os.path.exists(world):
        world = 'empty.sdf'

    gz_sim = ExecuteProcess(
        # Gazebo Harmonic 默认 DART 引擎不支持 SDF mimic constraint。PiPER
        # 平行夹爪的两根手指依赖该约束，因此显式使用 Harmonic
        # 自带且支持 mimic 的 Bullet Featherstone 物理引擎。
        cmd=['gz', 'sim', '-r', '-v', '3',
             '--physics-engine', 'gz-physics-bullet-featherstone-plugin', world],
        condition=IfCondition(gazebo_gui), output='screen')
    gz_sim_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', '-v', '3',
             '--physics-engine', 'gz-physics-bullet-featherstone-plugin', world],
        condition=UnlessCondition(gazebo_gui), output='screen')

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen')

    # GPU lidar 同时发布多层 LaserScan 与 PointCloudPacked。PointCloud2 才能完整表达
    # 16 条垂直扫描线；ROS LaserScan 没有垂直维度字段，仅保留作底层数据调试。
    lidar_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/velodyne/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/velodyne/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/velodyne/scan', '/velodyne_scan'),
            ('/velodyne/scan/points', '/velodyne_points'),
        ],
        output='screen')

    # 独立 RGB camera 与 RGBD depth camera 发布各自 CameraInfo。话题使用
    # realsense2_camera 常见命名，建图、导航和抓取节点可直接订阅。
    depth_camera_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='realsense_d435i_bridge',
        arguments=[
            '/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        remappings=[
            ('/camera/depth/depth_image', '/camera/depth/image_raw'),
        ],
        output='screen')

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'AG_robot', '-z', SPAWN_Z],
        output='screen')

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager', '/controller_manager'],
                    output='screen')

    load_jsb = spawner('joint_state_broadcaster')
    load_drive = spawner('diff_drive_controller')
    load_lift = spawner('lift_controller')
    load_arm1 = spawner('arm1_controller')
    load_arm2 = spawner('arm2_controller')
    load_arm1_gripper = spawner('arm1_gripper_controller')
    load_arm2_gripper = spawner('arm2_gripper_controller')

    return LaunchDescription([
        DeclareLaunchArgument('gazebo_gui', default_value='true',
                              description='false = 只跑 gz sim 服务端, 不开界面'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='true = 打开 RViz 并显示 VLP-16 点云'),
        DeclareLaunchArgument('lidar_pitch', default_value='0',
                              description='雷达朝车头下倾角度(度), 正值=下倾; '
                                          '已验证 0/15/25/35, 任意角度均可, 换角度无需重新编译'),
        # librealsense 官方 D435i 参考设备，848x480@30 Hz。每台实机的标定值
        # 会略有差异，所以全部参数仍可在 launch 命令行覆盖。
        DeclareLaunchArgument('camera_depth_width', default_value='848'),
        DeclareLaunchArgument('camera_depth_height', default_value='480'),
        DeclareLaunchArgument('camera_depth_fx', default_value='418.2646789550781'),
        DeclareLaunchArgument('camera_depth_fy', default_value='418.2646789550781'),
        DeclareLaunchArgument('camera_depth_cx', default_value='424.1576232910156'),
        DeclareLaunchArgument('camera_depth_cy', default_value='238.23983764648438'),
        DeclareLaunchArgument('camera_depth_hfov', default_value='1.5844149256643074'),
        DeclareLaunchArgument('camera_color_width', default_value='848'),
        DeclareLaunchArgument('camera_color_height', default_value='480'),
        DeclareLaunchArgument('camera_color_fx', default_value='605.3924560546875'),
        DeclareLaunchArgument('camera_color_fy', default_value='605.6131591796875'),
        DeclareLaunchArgument('camera_color_cx', default_value='428.64471435546875'),
        DeclareLaunchArgument('camera_color_cy', default_value='241.26548767089844'),
        DeclareLaunchArgument('camera_color_hfov', default_value='1.2219513360956136'),
        DeclareLaunchArgument('camera_clip_near', default_value='0.28',
                              description='D435i 最大分辨率 Min-Z (m)'),
        DeclareLaunchArgument('camera_clip_far', default_value='10.0',
                              description='D435i 仿真远裁剪距离(m)；官方理想距离为 0.3..3 m'),
        DeclareLaunchArgument('camera_update_rate', default_value='30.0',
                              description='D435i Color/Depth 仿真帧率(Hz)'),
        DeclareLaunchArgument('camera_imu_update_rate', default_value='200.0',
                              description='D435i IMU 仿真频率(Hz)，官方支持 200/400'),

        LogInfo(msg=['AG_robot: 雷达安装倾角 lidar_pitch = ',
                     LaunchConfiguration('lidar_pitch'), ' 度 (正值=朝车头下倾)']),

        gz_sim,
        gz_sim_headless,
        clock_bridge,
        lidar_bridge,
        depth_camera_bridge,

        Node(package='rviz2', executable='rviz2',
             arguments=['-d', rviz_cfg],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(rviz), output='screen'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description,
                          'use_sim_time': True}],
             output='screen'),

        spawn,

        # 模型 spawn 出来后的头几秒仿真最慢(重网格 + 雷达渲染启动), 这时候立刻切控制器
        # 会撞上 spawner 内部 5 s 的切换超时, joint_state_broadcaster 会 "Failed to activate",
        # 结果没有 /joint_states、可动关节的 TF 也全缺。所以先等 10 s 再开始装控制器。
        RegisterEventHandler(OnProcessExit(
            target_action=spawn, on_exit=[TimerAction(period=10.0, actions=[load_jsb])])),
        RegisterEventHandler(OnProcessExit(target_action=load_jsb, on_exit=[load_drive])),
        RegisterEventHandler(OnProcessExit(target_action=load_drive, on_exit=[load_lift])),
        RegisterEventHandler(OnProcessExit(target_action=load_lift, on_exit=[load_arm1])),
        RegisterEventHandler(OnProcessExit(target_action=load_arm1, on_exit=[load_arm2])),
        RegisterEventHandler(OnProcessExit(target_action=load_arm2, on_exit=[load_arm1_gripper])),
        RegisterEventHandler(OnProcessExit(target_action=load_arm1_gripper, on_exit=[load_arm2_gripper])),
    ])
