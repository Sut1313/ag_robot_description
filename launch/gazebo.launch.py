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
    # lidar_pitch 透传给 xacro: xacro 在 launch 时才展开, 所以换雷达倾角不需要重新编译。
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_gazebo:=true lidar_pitch:=',
                 LaunchConfiguration('lidar_pitch')]),
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
        cmd=['gz', 'sim', '-r', '-v', '3', world],
        condition=IfCondition(gazebo_gui), output='screen')
    gz_sim_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', '-v', '3', world],
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

    return LaunchDescription([
        DeclareLaunchArgument('gazebo_gui', default_value='true',
                              description='false = 只跑 gz sim 服务端, 不开界面'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='true = 打开 RViz 并显示 VLP-16 点云'),
        DeclareLaunchArgument('lidar_pitch', default_value='0',
                              description='雷达朝车头下倾角度(度), 正值=下倾; '
                                          '已验证 0/15/25/35, 任意角度均可, 换角度无需重新编译'),

        LogInfo(msg=['AG_robot: 雷达安装倾角 lidar_pitch = ',
                     LaunchConfiguration('lidar_pitch'), ' 度 (正值=朝车头下倾)']),

        gz_sim,
        gz_sim_headless,
        clock_bridge,
        lidar_bridge,

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
    ])
