#!/usr/bin/env python3
"""VLP-16 盲区实测脚本 (在仿真运行时执行, 抓一帧点云做分类统计)。

用法:
    # 先跑起仿真并 source 好环境
    ros2 launch ag_robot_description gazebo.launch.py rviz:=false lidar_pitch:=25
    python3 scripts/lidar_blind_zone.py                 # 倾角/安装高度自动从 TF 读
    python3 scripts/lidar_blind_zone.py --npz out.npz   # 顺便存原始点云
    python3 scripts/lidar_blind_zone.py --roll 25 --h-link 1.1890   # 无 TF 时手动指定

倾角与安装高度**从 TF 的 base_footprint → velodyne_link 读取**, 所以换 lidar_pitch
之后本脚本无需改动; --roll/--h-link 只是没有 TF 时的兜底。

原理: 世界是空的(只有地面 + 机器人 + worlds/ag_robot.sdf 里的演示障碍物),
所以每一条射线的回波只有四种归属:
    · 命中地面   : 代入倾斜后的地面平面方程 (见下)
    · 本体自遮挡 : 水平距离 < SELF_R (演示障碍物都在 1.8 m 以外, 不会误判)
    · 环境障碍   : 其余有回波
    · 无回波     : 朝天 / 超出量程 / 近于最小量程
几何: 雷达朝车头(-Y)下倾 roll 后, 地面在 velodyne_link 系里的平面方程是
    sin(roll)·y + cos(roll)·z + H_LINK = 0     (H_LINK = link 原点离地高度)
把点绕 X 转回水平, 就得到世界水平距离, 从而算出各方向的近地盲区。
"""
import argparse
import math
import sys
import time

import numpy as np

Z_SENSOR = 0.010       # 传感器 <pose> 在 velodyne_link 中的 z 偏移 (m)
R_MIN = 0.5            # 最小量程 (m), 与 gazebo.xacro 的 <range><min> 一致
R_MAX = 200.0          # 最大量程 (m)
VFOV_DEG = 20.0        # 垂直半视场 (deg), 与 <vertical> 的 min/max 一致
SELF_R = 0.8           # 水平距离小于此值判为机器人本体 (m)
GROUND_TOL = 0.06      # 地面判定容差 (m), 需大于噪声 σ 的若干倍

DTYPES = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
          5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def quat_to_rpy(x, y, z, w):
    """四元数 -> ZYX 欧拉角 (roll, pitch, yaw)。"""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def make_node_with_tf(timeout=10.0):
    """起一个 rclpy 节点并等到 base_footprint->velodyne_link 可用; 返回 (node, tf)。"""
    import rclpy
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener
    rclpy.init()
    node = Node('lidar_blind_zone')
    buf = Buffer()
    TransformListener(buf, node)
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if buf.can_transform('base_footprint', 'velodyne_link', rclpy.time.Time()):
            tr = buf.lookup_transform('base_footprint', 'velodyne_link', rclpy.time.Time())
            t, q = tr.transform.translation, tr.transform.rotation
            return node, ((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
    return node, None


def grab(node, topic, timeout=30.0):
    from sensor_msgs.msg import PointCloud2
    box = {}
    node.create_subscription(PointCloud2, topic, lambda m: box.setdefault('m', m), 10)
    import rclpy
    t0 = time.time()
    while 'm' not in box and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.3)
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
    ap.add_argument('--roll', type=float, default=None,
                    help='手动指定下倾角(度); 不给则从 TF 读')
    ap.add_argument('--h-link', type=float, default=None,
                    help='手动指定 velodyne_link 原点离地高度(m); 不给则从 TF 读')
    ap.add_argument('--no-tf', action='store_true', help='跳过 TF, 必须配合 --roll/--h-link')
    args = ap.parse_args()

    node, tf = (None, None) if args.no_tf else make_node_with_tf()
    if tf is not None:
        (tx, ty, tz), (qx, qy, qz, qw) = tf
        roll_rad, pitch_rad, yaw_rad = quat_to_rpy(qx, qy, qz, qw)
        roll_deg = math.degrees(roll_rad)
        h_link = tz
        src = (f"TF base_footprint->velodyne_link: 原点 (%.4f, %.4f, %.4f) m, "
               f"RPY (%.4f, %.4f, %.4f)°" % (tx, ty, tz, roll_deg,
                                             math.degrees(pitch_rad), math.degrees(yaw_rad)))
    elif args.roll is not None and args.h_link is not None:
        roll_deg = float(args.roll)
        h_link = args.h_link
        src = "命令行指定 (--roll / --h-link)"
    else:
        print("拿不到 TF (仿真没跑? 或用了 --no-tf) —— 请用 --roll 和 --h-link 手动指定")
        return 1

    ROLL = math.radians(roll_deg)
    H_LINK = h_link
    H_SENSOR = H_LINK + math.cos(ROLL) * Z_SENSOR
    print(f"雷达位姿来源: {src}")
    print(f"  → 朝车头(-Y)下倾 {roll_deg:.4f}°, velodyne_link 原点离地 {H_LINK:.4f} m, "
          f"光心离地 {H_SENSOR:.4f} m")

    if node is None:
        import rclpy
        from rclpy.node import Node
        rclpy.init()
        node = Node('lidar_blind_zone')
    m = grab(node, args.topic)
    import rclpy
    rclpy.shutdown()
    if m is None:
        print(f'没收到点云 ({args.topic}) —— 仿真在跑吗? 话题名对吗?')
        return 1
    a = fields(m)
    if not {'x', 'y', 'z'} <= set(a):
        print(f'点云缺 x/y/z 字段: {list(a)}')
        return 1
    x, y, z = a['x'], a['y'], a['z']
    ring = a.get('ring')
    # 点云用 velodyne_link 表达；量程和俯仰角应从 sensor <pose> 的光心计算。
    zs = z - Z_SENSOR
    r = np.sqrt(x * x + y * y + zs * zs)
    # 绕 X 轴倾斜后换算到车体/世界水平面。
    with np.errstate(invalid='ignore'):
        horiz = np.sqrt(x * x + (math.cos(ROLL) * y - math.sin(ROLL) * zs) ** 2)
    fin = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    valid = fin & (r >= R_MIN) & (r <= R_MAX)
    # 地面在倾斜 link 坐标系中的平面: sin(roll)*y + cos(roll)*z + H_LINK = 0。
    with np.errstate(invalid='ignore'):
        ground = valid & (np.abs(math.sin(ROLL) * y + math.cos(ROLL) * z + H_LINK) < GROUND_TOL)
    self_hit = valid & ~ground & (horiz < SELF_R)
    env = valid & ~ground & (horiz >= SELF_R)
    nohit = ~valid
    az = np.degrees(np.arctan2(y, x))
    el = np.degrees(np.arcsin(np.clip(np.divide(zs, r, out=np.zeros_like(r), where=valid), -1, 1)))

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
        print(f"    {'ring':>4s} {'传感器俯仰':>10s} {'本体':>6s} {'地面':>6s} {'障碍':>6s} {'无回波':>7s} "
              f"{'地面最近':>9s}")
        for rr in range(int(ring.max()) + 1):
            s = ring == rr
            if s.sum() == 0:
                continue
            ev = np.median(el[s & valid]) if (s & valid).sum() else float('nan')
            gm = r[s & ground]
            print(f"    {rr:4d} {ev:8.2f} {self_hit[s].sum():6d} {ground[s].sum():6d} {env[s].sum():6d} "
                  f"{nohit[s].sum():7d} {(f'{gm.min():.2f} m' if gm.size else '—'):>9s}")

    if ground.sum():
        print(f"\n  地面可见范围 {r[ground].min():.2f} ~ {r[ground].max():.2f} m")
        if ring is not None:
            lowest = ring == int(ring.min())
            print("  最低扫描线的方向性近地盲区（世界水平距离中位）:")
            for name, center in (("前(-Y)", -90), ("左/右(±X)", 0),
                                 ("后(+Y)", 90), ("另一侧(−X)", 180)):
                da = (az - center + 180) % 360 - 180
                s = lowest & ground & (np.abs(da) < 5)
                value = f"{np.median(horiz[s]):.3f} m" if s.any() else "无地面回波"
                print(f"    {name:12s}: {value}")
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

    print(f"\n  倾斜盲区: 光心离地 {H_SENSOR:.3f} m, 朝车头下倾 {roll_deg:.4f}°")
    for name, depression in (("车头", VFOV_DEG + roll_deg),
                             ("左右", math.degrees(math.asin(
                                 math.cos(ROLL) * math.sin(math.radians(VFOV_DEG))))),
                             ("车尾", VFOV_DEG - roll_deg)):
        if depression > 0:
            d = H_SENSOR / math.tan(math.radians(depression))
            print(f"    {name}: 最低线等效下视 {depression:.2f}°, 地面从约 {d:.3f} m 开始可见")
        else:
            print(f"    {name}: 最低线实际上仰 {-depression:.2f}°, 无地面回波")
    if roll_deg > VFOV_DEG:
        edge = math.degrees(math.asin(
            math.tan(math.radians(VFOV_DEG)) / math.tan(ROLL)))
        width = 180.0 - 2.0 * edge
        print(f"    车尾无地面覆盖扇区: 传感器方位 +{edge:.2f}°~+{180-edge:.2f}° "
              f"(宽约 {width:.2f}°)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
