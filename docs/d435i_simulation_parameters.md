# Intel RealSense D435i 仿真参数来源

本模型将 D435i 拆成独立 Color camera、RGBD depth camera 和 IMU，并使用 ROS/OpenCV 光学坐标约定（+x 右、+y 下、+z 前）。

## 默认 profile

- Color: 848 x 480 @ 30 Hz
- Depth: 848 x 480 @ 30 Hz
- IMU: 200 Hz
- Depth clip: 0.28–10 m（官方理想工作区间是 0.3–3 m，10 m 是仿真裁剪上限）

848 x 480 @ 30 Hz 同时存在于 librealsense 官方 D435i 的 Color(YUYV) 和 Depth(Z16) profile 列表。

## 参考内参

| stream | fx | fy | cx | cy | distortion |
|---|---:|---:|---:|---:|---|
| Depth | 418.2646789550781 | 418.2646789550781 | 424.1576232910156 | 238.23983764648438 | Brown, all zero in fixture |
| Color | 605.3924560546875 | 605.6131591796875 | 428.64471435546875 | 241.26548767089844 | inverse Brown, all zero in fixture |

这些是 librealsense 测试设备中的官方参考标定，不是所有 D435i 共用的唯一标定。SDF `lens/intrinsics` 用这些值同时设置渲染投影和 `CameraInfo` K/P；拿到实机后应使用该机的出厂内参覆盖 xacro/launch 参数。

## 参考外参

- Color frame 相对 Depth/body frame（已转成 REP-103）：
  - xyz = `-0.000217729917 0.015078110620 0.000010675736` m
  - rpy = `-0.000407617216 0.000331054855 -0.003084785990` rad
- IMU frame 相对 Depth/body frame（已转成 REP-103）：
  - xyz = `-0.011740000 -0.005520000 0.005100000` m
  - rpy = `0 0 0`

D435i 文档说明 Depth↔IMU 刚体外参按机械图纸预标定，SDK 会把 IMU 样本对齐到深度坐标系。Color↔Depth 值仍建议在实机到手后替换为该机实际标定。

## 官方来源

- [D435i 产品页](https://www.realsenseai.com/products/depth-camera-d435i/)：FOV 87° x 58°、Min-Z 约 28 cm、理想距离 0.3–3 m、Depth 最高 1280 x 720 / 90 fps、RGB 1920 x 1080 / 30 fps。
- [librealsense D435i 文档](https://github.com/IntelRealSense/librealsense/blob/master/doc/d435i.md)：BMI055 6 轴 IMU、深度时钟同步、坐标约定与 IMU 外参说明。
- [librealsense D435i 参考设备](https://github.com/IntelRealSense/librealsense/blob/master/unit-tests/dds/d435i.py)：profile、内参、RGB/Depth/IMU 外参。
- [Gazebo RgbdCameraSensor](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/src/RgbdCameraSensor.cc)：`image` / `depth_image` / `points` / `camera_info` 话题和 `optical_frame_id` 行为。

