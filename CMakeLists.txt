cmake_minimum_required(VERSION 3.8)
project(ag_robot_description)

find_package(ament_cmake REQUIRED)

# meshes 一定要装进去, 否则 package://ag_robot_description/meshes/... 在 install 空间里找不到
# meshes/*.orig 是改分组前的原始网格备份, 只在源目录留档, 不装进 install
# worlds 是仿真世界(斜射太阳 + 低环境光, 让结构棱角清晰), gazebo.launch.py 直接用它
# scripts/ 是雷达测量脚本, docs/ 是 README 引用的图片 —— 一起装上, 装完也能直接用
install(DIRECTORY urdf meshes config launch rviz worlds scripts docs
        DESTINATION share/${PROJECT_NAME}
        PATTERN "*.orig" EXCLUDE
        PATTERN "__pycache__" EXCLUDE)

# 许可证与第三方署名随包安装 (MIT 要求许可声明随副本一并保留)
install(FILES LICENSE NOTICE DESTINATION share/${PROJECT_NAME})

ament_package()
