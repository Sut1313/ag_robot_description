cmake_minimum_required(VERSION 3.8)
project(ag_robot_description)

find_package(ament_cmake REQUIRED)

# meshes 一定要装进去, 否则 package://ag_robot_description/meshes/... 在 install 空间里找不到
# meshes/*.orig 是改分组前的原始网格备份, 只在源目录留档, 不装进 install
# worlds 是仿真世界(斜射太阳 + 低环境光, 让结构棱角清晰), gazebo.launch.py 直接用它
install(DIRECTORY urdf meshes config launch rviz worlds
        DESTINATION share/${PROJECT_NAME}
        PATTERN "*.orig" EXCLUDE
        PATTERN "__pycache__" EXCLUDE)

ament_package()
