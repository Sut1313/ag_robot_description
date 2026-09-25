import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('ag_robot_description')
    xacro_file = os.path.join(pkg, 'urdf', 'AG_robot.urdf.xacro')

    # robot_description 必须用 ParameterValue(..., value_type=str) 包一层,
    # 否则 xacro 注释里的"中文: 冒号"会让 launch 报 YAML 解析错。
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str)

    rviz_cfg = os.path.join(pkg, 'rviz', 'AG_robot.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否打开 joint_state_publisher_gui 滑条窗口'),
        DeclareLaunchArgument('rviz', default_value='true'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}],
             output='screen'),

        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(LaunchConfiguration('gui')),
             output='screen'),

        Node(package='rviz2', executable='rviz2',
             arguments=['-d', rviz_cfg] if os.path.exists(rviz_cfg) else [],
             condition=IfCondition(LaunchConfiguration('rviz')),
             output='screen'),
    ])
