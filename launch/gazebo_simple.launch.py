"""一行跑简化模型：ros2 launch ag_robot_description gazebo_simple.launch.py [lidar_pitch:=25]

这只是 gazebo.launch.py 的一层薄壳（加 model:=simple），所以不存在"两份启动逻辑要同步维护"
的问题：改 gazebo.launch.py（比如以后接 IMU 桥接、加控制器）对两个模型同时生效。
需要原网格版就照旧用 gazebo.launch.py（默认 model:=full）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('ag_robot_description')
    base_launch = os.path.join(pkg, 'launch', 'gazebo.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('gazebo_gui', default_value='true',
                              description='false = 只跑 gz sim 服务端, 不开界面'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='true = 打开 RViz 并显示 VLP-16 点云'),
        DeclareLaunchArgument('lidar_pitch', default_value='0',
                              description='雷达朝车头下倾角度(度), 正值=下倾; 任意角度均可'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                'model': 'simple',
                'gazebo_gui': LaunchConfiguration('gazebo_gui'),
                'rviz': LaunchConfiguration('rviz'),
                'lidar_pitch': LaunchConfiguration('lidar_pitch'),
            }.items()),
    ])
