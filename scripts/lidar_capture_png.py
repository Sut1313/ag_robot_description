#!/usr/bin/env python3
"""抓一帧 VLP-16 点云并出图 (俯视图 + 3D 视图), 用来肉眼确认扫描轮廓与盲区。

用法:
    ros2 launch ag_robot_description gazebo.launch.py rviz:=false
    python3 scripts/lidar_capture_png.py                       # 存到 ./vlp16_scan.png
    python3 scripts/lidar_capture_png.py -o /tmp/scan.png --range 6
需要 matplotlib (未安装时会给出提示, 不影响其他脚本)。
"""
import argparse
import sys
import time

import numpy as np


def grab(topic, timeout=20.0):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    rclpy.init()
    node = Node('vlp16_capture')
    box = {}
    node.create_subscription(PointCloud2, topic, lambda m: box.setdefault('m', m),
                             qos_profile_sensor_data)
    t0 = time.time()
    while 'm' not in box and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.3)
    rclpy.shutdown()
    return box.get('m')


def to_xyz(msg):
    """按消息自带字段偏移解析 x/y/z/ring —— 不假设固定布局。"""
    n = msg.width * msg.height
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)
    dtypes = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
    out = {}
    for f in msg.fields:
        dt = dtypes.get(f.datatype)
        if dt is None:
            continue
        out[f.name] = buf[:, f.offset:f.offset + np.dtype(dt).itemsize].copy().view(dt).ravel()
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-o', '--output', default='vlp16_scan.png')
    ap.add_argument('--topic', default='/velodyne_points')
    ap.add_argument('--range', type=float, default=6.0, help='俯视图半径范围 (m)')
    args = ap.parse_args()

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('需要 matplotlib: pip install matplotlib')
        return 1

    msg = grab(args.topic)
    if msg is None:
        print(f'没收到点云 ({args.topic}) —— 仿真在跑吗?')
        return 1
    a = to_xyz(msg)
    if not {'x', 'y', 'z'} <= set(a):
        print(f'点云缺 x/y/z: {list(a)}')
        return 1
    xyz = np.column_stack([a['x'], a['y'], a['z']]).astype(np.float64)
    ok = np.isfinite(xyz).all(axis=1)
    xyz = xyz[ok]
    ring = a['ring'][ok] if 'ring' in a else np.zeros(len(xyz))
    print(f"frame={msg.header.frame_id}  {msg.width}x{msg.height}  有效点 {len(xyz):,}")

    plt.style.use('dark_background')
    fig = plt.figure(figsize=(15, 7.5), constrained_layout=True)
    ax1 = fig.add_subplot(1, 2, 1)
    ax2 = fig.add_subplot(1, 2, 2, projection='3d')
    R = args.range
    ax1.scatter(xyz[:, 0], xyz[:, 1], c=ring.astype(float), s=1.3, cmap='turbo', alpha=0.9)
    ax1.scatter([0], [0], c='white', s=55, marker='x', linewidths=2, label='VLP-16')
    ax1.set_aspect('equal', adjustable='box')
    ax1.set_xlim(-R, R); ax1.set_ylim(-R, R)
    ax1.set_xlabel('X / m'); ax1.set_ylabel('Y / m')
    ax1.set_title('Top view - 360 deg scan (colored by ring; blank near field = blind zone)')
    ax1.grid(alpha=0.2); ax1.legend(loc='upper right')
    Z = 1.7
    ax2.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c=xyz[:, 2], s=1.2, cmap='viridis', alpha=0.9)
    ax2.scatter([0], [0], [0], c='red', s=50, marker='x')
    ax2.set_xlim(-R, R); ax2.set_ylim(-R, R); ax2.set_zlim(-Z, Z)
    ax2.set_xlabel('X / m'); ax2.set_ylabel('Y / m'); ax2.set_zlabel('Z / m')
    ax2.set_title('3D view - 16 vertical scan lines')
    ax2.view_init(elev=24, azim=-56)
    stamp = f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
    fig.suptitle(f'AG_robot simulated Velodyne VLP-16 | frame={msg.header.frame_id} | '
                 f'{len(xyz):,} valid returns / {msg.width * msg.height:,} rays | t={stamp}s',
                 fontsize=13)
    fig.savefig(args.output, dpi=110)
    print(f"已保存 {args.output}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
