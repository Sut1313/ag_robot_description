#!/usr/bin/env python3
"""把近场盲区靶板按**当前雷达位姿**摆到世界中 (替代手工改靶板 SDF 里的 pose)。

靶板几何取自 worlds/lidar_blindzone_targets.sdf, 本脚本只把它的 <pose> 换成
"当前雷达光心的世界位姿", 写到 /tmp 再 spawn —— 所以无论 lidar_pitch 是 0 还是 35,
靶板都与雷达扫描原点对齐, 近场校验(lidar_target_check.py)始终成立。

用法:
    ros2 launch ag_robot_description gazebo.launch.py rviz:=false lidar_pitch:=25
    python3 scripts/spawn_lidar_targets.py          # 摆靶板
    python3 scripts/lidar_target_check.py           # 校验
    python3 scripts/spawn_lidar_targets.py --remove # 撤掉靶板
"""
import argparse
import math
import os
import re
import subprocess
import sys

import numpy as np

Z_SENSOR = 0.010   # 传感器 <pose> 在 velodyne_link 里的 z 偏移


def sh(cmd, timeout=15):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True,
                          timeout=timeout).stdout


def quat_to_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def mat_to_rpy(R):
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(R[2, 0]) < 1 - 1e-9:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def world_pose_of_lidar():
    """返回雷达光心的世界位姿 (xyz, rpy)。"""
    out = sh("timeout 12 gz model -m AG_robot -p 2>/dev/null")
    nums = []
    for ln in out.splitlines():
        ln = ln.strip().strip('[]')
        if ln and all(c in '0123456789.eE+- ' for c in ln):
            nums += [float(v) for v in ln.split()]
    if len(nums) < 6:
        print("读不到机器人世界位姿 (gz model -p) —— 仿真在跑吗?")
        return None
    bf_xyz, bf_rpy = np.array(nums[0:3]), nums[3:6]

    import rclpy
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener
    rclpy.init()
    node = Node('spawn_lidar_targets')
    buf = Buffer()
    TransformListener(buf, node)
    import time
    t0 = time.time()
    tf = None
    while time.time() - t0 < 10 and tf is None:
        rclpy.spin_once(node, timeout_sec=0.2)
        if buf.can_transform('base_footprint', 'velodyne_link', rclpy.time.Time()):
            tr = buf.lookup_transform('base_footprint', 'velodyne_link', rclpy.time.Time())
            t, q = tr.transform.translation, tr.transform.rotation
            tf = (np.array([t.x, t.y, t.z]), quat_to_mat(q.x, q.y, q.z, q.w))
    rclpy.shutdown()
    if tf is None:
        print("拿不到 TF base_footprint->velodyne_link —— 仿真在跑吗?")
        return None
    link_xyz, link_R = tf

    R_world = rpy_to_mat(*bf_rpy)
    world_R = R_world @ link_R                       # 雷达在世界中的姿态
    world_xyz = bf_xyz + R_world @ link_xyz          # 雷达 link 原点在世界中的位置
    world_xyz = world_xyz + world_R @ np.array([0, 0, Z_SENSOR])   # 光心
    return world_xyz, mat_to_rpy(world_R)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default='ag_robot')
    ap.add_argument('--name', default='vlp16_blindzone_targets')
    ap.add_argument('--sdf', default=None, help='靶板 SDF 模板 (默认包内 worlds/)')
    ap.add_argument('--remove', action='store_true')
    args = ap.parse_args()

    if args.remove:
        print(sh(f"timeout 12 gz service -s /world/{args.world}/remove "
                 f"--reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 6000 "
                 f"--req 'name: \"{args.name}\" type: MODEL' 2>&1 | tail -1"))
        return 0

    tmpl = args.sdf
    if tmpl is None:
        pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        tmpl = os.path.join(pkg, 'worlds', 'lidar_blindzone_targets.sdf')
    if not os.path.exists(tmpl):
        print(f"找不到靶板 SDF: {tmpl}")
        return 1

    pose = world_pose_of_lidar()
    if pose is None:
        return 1
    xyz, (roll, pitch, yaw) = pose
    posed = f"{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f} {roll:.10f} {pitch:.10f} {yaw:.10f}"
    print(f"雷达光心世界位姿: {posed}")
    print(f"  → 水平位置 ({xyz[0]:.4f}, {xyz[1]:.4f}) m, 离地 {xyz[2]:.4f} m, "
          f"roll {math.degrees(roll):.3f}°")

    sdf = open(tmpl).read()
    # 只替换 <model ...> 之后的第一处 <pose>
    m = re.search(r'(<model\b[^>]*>\s*(?:<static>[^<]*</static>\s*)?)<pose>[^<]*</pose>', sdf)
    if not m:
        print("模板里没找到 <pose> —— 检查 SDF 结构")
        return 1
    out = sdf[:m.start()] + m.group(1) + f"<pose>{posed}</pose>" + sdf[m.end():]
    tmp = '/tmp/vlp16_blindzone_targets_auto.sdf'
    open(tmp, 'w').write(out)
    print(f"已生成 {tmp}")

    print(sh(f"timeout 15 gz service -s /world/{args.world}/create "
             f"--reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 8000 "
             f"--req 'sdf_filename: \"{tmp}\"' 2>&1 | tail -1"))
    return 0


if __name__ == '__main__':
    sys.exit(main())
