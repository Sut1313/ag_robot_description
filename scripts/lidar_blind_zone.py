#!/usr/bin/env python3
"""VLP-16 盲区实测脚本 (在仿真运行时执行, 抓一帧点云做分类统计)。

用法:
    # 先跑起仿真并 source 好环境
    ros2 launch ag_robot_description gazebo.launch.py rviz:=false
    python3 scripts/lidar_blind_zone.py                 # 用当前机器人姿态测
    python3 scripts/lidar_blind_zone.py --npz out.npz   # 顺便存原始点云

原理: 世界是空的(只有地面 + 机器人 + worlds/ag_robot.sdf 里的演示障碍物),
所以每一条射线的回波只有四种归属, 分类判据很干净:
    · 命中地面   : 传感器坐标系下 z ≈ -H  (H = 雷达光心离地高度)
    · 本体自遮挡 : 水平距离 < SELF_R (演示障碍物都在 1.8 m 以外, 不会误判)
    · 环境障碍   : 其余有回波
    · 无回波     : 朝天 / 超出量程 / 近于最小量程
盲区的三个来源: 本体自遮挡、最小量程内的近球、以及垂直视场外的近场地面。
"""
import argparse
import math
import sys
import time

import numpy as np

# ── 这些参数随安装位置/型号变化, 换硬件时只改这里 ────────────────────────
H_SENSOR = 1.1754858   # 雷达光心离地高度 (m): base_footprint→base_link 0.2184858
                       #   + velodyne_joint 0.947 + sensor <pose> 0.010
R_MIN = 0.5            # 最小量程 (m), 与 gazebo.xacro 的 <range><min> 一致
R_MAX = 200.0          # 最大量程 (m)
VFOV_DEG = 20.0        # 垂直半视场 (deg), 与 <vertical> 的 min/max 一致
SELF_R = 0.8           # 水平距离小于此值判为机器人本体 (m)
GROUND_TOL = 0.06      # 地面判定容差 (m), 需大于噪声 σ 的若干倍

DTYPES = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
          5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def grab(topic, timeout=30.0):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    rclpy.init()
    node = Node('lidar_blind_zone')
    box = {}
    node.create_subscription(PointCloud2, topic, lambda m: box.setdefault('m', m), 10)
    t0 = time.time()
    while 'm' not in box and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.3)
    rclpy.shutdown()
    return box.get('m')


def fields(msg):
    n = msg.width * msg.height
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)
    out = {}
    for f in msg.fields:
        d = DTYPES.get(f.datatype)
        if d is None:
            continue
        it = np.dtype(d).itemsize
        out[f.name] = buf[:, f.offset:f.offset + it].copy().view(d).ravel().astype(np.float64)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/velodyne_points')
    ap.add_argument('--npz', default=None, help='把原始点云也存一份到该文件')
    args = ap.parse_args()

    m = grab(args.topic)
    if m is None:
        print(f'没收到点云 ({args.topic}) —— 仿真在跑吗? 话题名对吗?')
        return 1
    a = fields(m)
    if not {'x', 'y', 'z'} <= set(a):
        print(f'点云缺 x/y/z 字段: {list(a)}')
        return 1
    x, y, z = a['x'], a['y'], a['z']
    ring = a.get('ring')
    r = np.sqrt(x * x + y * y + z * z)
    horiz = np.sqrt(x * x + y * y)
    fin = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    valid = fin & (r >= R_MIN) & (r <= R_MAX)
    ground = valid & (np.abs(z + H_SENSOR) < GROUND_TOL)
    self_hit = valid & ~ground & (horiz < SELF_R)
    env = valid & ~ground & (horiz >= SELF_R)
    nohit = ~valid
    az = np.degrees(np.arctan2(y, x))
    el = np.degrees(np.arcsin(np.clip(np.divide(z, r, out=np.zeros_like(r), where=valid), -1, 1)))

    print(f"点云 {m.width}x{m.height}  frame_id='{m.header.frame_id}'  总射线 {len(r)}")
    if args.npz:
        np.savez(args.npz, **a, frame=m.header.frame_id)
        print(f"原始点云已存 {args.npz}")
    print(f"  无回波(朝天/超量程/近于 {R_MIN} m) {nohit.sum():6d} ({100*nohit.mean():4.1f}%)")
    print(f"  地面回波                        {ground.sum():6d} ({100*ground.mean():4.1f}%)")
    print(f"  机器人本体自遮挡                {self_hit.sum():6d} ({100*self_hit.mean():4.1f}%)")
    print(f"  环境障碍物回波                  {env.sum():6d} ({100*env.mean():4.1f}%)")

    if ring is not None:
        print(f"\n  逐扫描线:")
        print(f"    {'ring':>4s} {'俯仰':>8s} {'本体':>6s} {'地面':>6s} {'障碍':>6s} {'无回波':>7s} "
              f"{'地面最近':>9s} {'理论 h/sin':>10s}")
        for rr in range(int(ring.max()) + 1):
            s = ring == rr
            if s.sum() == 0:
                continue
            ev = np.median(el[s & valid]) if (s & valid).sum() else float('nan')
            gm = r[s & ground]
            theo = H_SENSOR / math.sin(math.radians(-ev)) if ev == ev and ev < -0.5 else float('nan')
            print(f"    {rr:4d} {ev:8.2f} {self_hit[s].sum():6d} {ground[s].sum():6d} {env[s].sum():6d} "
                  f"{nohit[s].sum():7d} {(f'{gm.min():.2f} m' if gm.size else '—'):>9s} "
                  f"{(f'{theo:.2f} m' if theo == theo else '—'):>10s}")

    if ground.sum():
        print(f"\n  地面可见范围 {r[ground].min():.2f} ~ {r[ground].max():.2f} m"
              f"  → 近场地面盲区半径 ≈ {r[ground].min():.2f} m")
    if self_hit.sum():
        print(f"  本体回波距离 {r[self_hit].min():.2f}~{r[self_hit].max():.2f} m, "
              f"水平 {horiz[self_hit].min():.2f}~{horiz[self_hit].max():.2f} m, "
              f"方位 {az[self_hit].min():.1f}°~{az[self_hit].max():.1f}°")

    bins = np.arange(-180, 181, 5)
    occ = []
    for k in range(len(bins) - 1):
        s = (az >= bins[k]) & (az < bins[k + 1])
        if s.sum() and self_hit[s].mean() > 0.5:
            occ.append([bins[k], bins[k + 1]])
    if occ:
        grp = [occ[0]]
        for lo, hi in occ[1:]:
            if lo == grp[-1][1]:
                grp[-1][1] = hi
            else:
                grp.append([lo, hi])
        print("  本体遮挡方位(>50% 射线打到本体): "
              + ", ".join(f"{lo:+d}°~{hi:+d}°" for lo, hi in grp))
    else:
        print("  本体遮挡方位: 无")

    tan = math.tan(math.radians(VFOV_DEG))
    print(f"\n  盲区锥面: 光心离地 {H_SENSOR:.3f} m, 最低扫描线 -{VFOV_DEG:.0f}°")
    print(f"    水平距离 d 处, 能看到的物体高度 = {H_SENSOR:.4f} - {tan:.4f}·d")
    for d in (0.5, 1.0, 1.5, 2.0, 2.5, 3.0, H_SENSOR / tan):
        h = H_SENSOR - tan * d
        if h > 0:
            print(f"    d={d:4.2f} m → {h*100:5.1f} cm 以上才可见")
        else:
            print(f"    d={d:4.2f} m → 地面已进入视野")
    for oh in (0.05, 0.10, 0.20, 0.30, 0.50):
        print(f"    高 {oh*100:3.0f} cm 的障碍: 近于 {(H_SENSOR-oh)/tan:.2f} m 不可见")
    return 0


if __name__ == '__main__':
    sys.exit(main())
