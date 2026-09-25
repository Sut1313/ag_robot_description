import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
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
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_gazebo:=true']),
        value_type=str)

    gazebo_gui = LaunchConfiguration('gazebo_gui')

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

        gz_sim,
        gz_sim_headless,
        clock_bridge,

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description,
                          'use_sim_time': True}],
             output='screen'),

        spawn,

        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[load_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=load_jsb, on_exit=[load_drive])),
        RegisterEventHandler(OnProcessExit(target_action=load_drive, on_exit=[load_lift])),
        RegisterEventHandler(OnProcessExit(target_action=load_lift, on_exit=[load_arm1])),
        RegisterEventHandler(OnProcessExit(target_action=load_arm1, on_exit=[load_arm2])),
    ])
