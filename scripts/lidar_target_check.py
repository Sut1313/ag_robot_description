#!/usr/bin/env python3
"""VLP-16 近场靶标校验 (与 lidar_blind_zone.py 互补的第二种测法)。

思路: 在雷达周围已知方位、已知距离放四个靶板 (worlds/lidar_blindzone_targets.sdf),
      看雷达能不能按预期测到它们的最近面。靶板距雷达 0.40 / 0.48 / 0.52 / 0.68 m,
      正好跨过 0.5 m 最小量程这条边界:
        0.40 / 0.48 m -> 应当没有回波 (在最小量程内, 属于硬性近球盲区)
        0.52 / 0.68 m -> 应当测到 ~0.52 / ~0.68 m 的最近面

用法:
    # 1) 起仿真
    ros2 launch ag_robot_description gazebo.launch.py rviz:=false
    # 2) 按当前雷达位姿摆靶板 —— 自动从 TF 算出光心的世界位姿, 支持任意 lidar_pitch
    python3 scripts/spawn_lidar_targets.py
    # 3) 校验
    python3 scripts/lidar_target_check.py
"""
import argparse
import math
import statistics
import struct
import sys
import time

# (标签, 靶板方位 rad, 最近面到雷达的距离 m)
TARGETS = [
    ("0.40 m", 0.0, 0.40),
    ("0.48 m", math.pi / 2, 0.48),
    ("0.52 m", math.pi, 0.52),
    ("0.68 m", -math.pi / 2, 0.68),
]
R_MIN = 0.5          # 与 gazebo.xacro 的 <range><min> 一致
Z_CENTER = 0.010     # 靶板中心在传感器系下的高度 (与 .sdf 的 pose 对应)
Z_TOL = 0.14         # 高度筛选容差
ANG_TOL = 0.24       # 方位筛选容差 rad
NEAR_TOL = 0.09      # 判定"打到了这个靶"的距离容差


def angle_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def grab(topic, timeout=30.0):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    rclpy.init()
    node = Node('vlp16_target_check')
    box = {}
    node.create_subscription(PointCloud2, topic, lambda m: box.setdefault('m', m),
                             qos_profile_sensor_data)
    t0 = time.time()
    while 'm' not in box and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.3)
    rclpy.shutdown()
    return box.get('m')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/velodyne_points')
    args = ap.parse_args()

    msg = grab(args.topic)
    if msg is None:
        print(f'没收到点云 ({args.topic}) —— 仿真在跑吗?')
        return 1
    offsets = {f.name: f.offset for f in msg.fields}
    endian = '>' if msg.is_bigendian else '<'
    points = []
    for row in range(msg.height):
        base = row * msg.row_step
        for col in range(msg.width):
            pos = base + col * msg.point_step
            try:
                x = struct.unpack_from(endian + 'f', msg.data, pos + offsets['x'])[0]
                y = struct.unpack_from(endian + 'f', msg.data, pos + offsets['y'])[0]
                z = struct.unpack_from(endian + 'f', msg.data, pos + offsets['z'])[0]
            except Exception:
                continue
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                points.append((x, y, z))
    print(f"点云 frame={msg.header.frame_id}  {msg.height}x{msg.width}  有效点 {len(points)}")

    ok = True
    print(f"\n  {'靶板':>8s} {'方位':>7s} {'预期最近面':>10s} {'实测最近':>9s} {'中位':>8s} "
          f"{'命中点数':>8s}  判定")
    for label, exp_ang, face in TARGETS:
        cand = [math.sqrt(x*x + y*y + z*z) for x, y, z in points
                if abs(angle_delta(math.atan2(y, x), exp_ang)) < ANG_TOL
                and abs(z - Z_CENTER) < Z_TOL]
        near = [r for r in cand if abs(r - face) < NEAR_TOL]
        nearest = min(cand) if cand else float('nan')
        median = statistics.median(near) if near else float('nan')
        if face < R_MIN:
            # 低于最小量程的靶板收不到回波, 但射线会透过去打到更远的物体上,
            # 所以判据是"靶板距离附近没有回波", 而不是"这个方位完全没有回波"。
            near_target = [r for r in cand if abs(r - face) < NEAR_TOL]
            good = not near_target
            if good:
                tail = f'; 射线透过, 打到 {nearest:.2f} m 处' if cand else ''
                verdict = f'✅ 靶板处无回波, 符合预期(在最小量程 {R_MIN:.1f} m 内){tail}'
            else:
                verdict = f'❌ 靶板距离附近收到 {len(near_target)} 个回波'
        else:
            good = bool(near) and abs(median - face) < 0.03
            verdict = ('✅ 测到 %.3f m, 与预期吻合' % median if good
                       else '❌ 未按预期测到 %.2f m (±%.2f)' % (face, NEAR_TOL))
        ok = ok and good
        print(f"  {label:>8s} {math.degrees(exp_ang):6.0f}° {face:9.2f} m "
              f"{(f'{nearest:.3f} m' if cand else '—'):>9s} "
              f"{(f'{median:.3f} m' if near else '—'):>8s} {len(cand):8d}  {verdict}")
    print(f"\n  总体: {'✅ 全部符合预期' if ok else '❌ 有不符合预期的项'}")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
