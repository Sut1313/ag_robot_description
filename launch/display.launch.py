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
    # lidar_pitch 与 gazebo.launch.py 一致: 单位度, 正值 = 朝车头(base_frame_link -Y)下倾,
    # xacro 在 launch 时才展开, 所以换角度不需要重新 colcon build。
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' lidar_pitch:=',
                 LaunchConfiguration('lidar_pitch')]),
        value_type=str)

    rviz_cfg = os.path.join(pkg, 'rviz', 'AG_robot.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否打开 joint_state_publisher_gui 滑条窗口'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('lidar_pitch', default_value='0',
                              description='雷达朝车头下倾角度(度), 正值=下倾; 已验证 0/15/25/35'),

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
