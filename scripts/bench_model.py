#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bench_model.py —— 在**已经在跑的**仿真上做一轮"跑得动吗 + 动得对吗"的实测

它回答两个问题:
  ① 跑得动吗: /clock 速率(=仿真实时率)、/velodyne_points 实际出点频率
  ② 动得对吗: 直行/转向/升降/机械臂 四类指令的实际跟踪情况
原网格版与图元简化版都可用它测, 逐项数值应当一致(这正是"简化没改动力学"的证据)。

前置: 仿真已经在跑(另一个终端 `ros2 launch ag_robot_description gazebo.launch.py ...`)
     且当前终端已 source 好环境。

用法:
    python3 scripts/bench_model.py --label "简化版"
    python3 scripts/bench_model.py --label "原版" --rtf-dur 15 --pc-dur 20

    对拍标准流程(每次约 2 分钟, 一次只能跑一套仿真):
        ros2 launch ag_robot_description gazebo.launch.py rviz:=false &   # 或 gazebo_simple.launch.py
        python3 scripts/bench_model.py --label 原网格版
        (换另一个模型重启仿真)
        python3 scripts/bench_model.py --label 图元简化版
    同一台机器上, 简化版的 /clock 与点云频率应显著更高, 而下面 ③④⑤⑥ 四项数值应一致。

两个"计量口径"的坑(都已在代码里处理, 列在这里免得后人踩):
  · 实时率、点云频率必须用**墙上时钟**量 —— 那是人眼看到的速度(RTF = 仿真秒/墙上秒)。
  · 关节角速度、车体角速度必须用**仿真时钟**量(odom/关节状态的时间戳都是仿真时间)。
    混用会把 RTF 乘进结果: 低实时率下用墙上时钟算出的"rad/s"会小好几倍, 是假象。
"""
import argparse
import math
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, PointCloud2
from trajectory_msgs.msg import JointTrajectoryPoint

WHEEL_R = 0.11349        # 与 config/ros2_control.yaml 的 wheel_radius 一致
WHEEL_L = 0.6038         # wheel_separation
ARM1 = ['arm1_joint%d' % i for i in range(1, 7)]
WHEELS = ['wheel_1_joint', 'wheel_2_joint', 'wheel_3_joint', 'wheel_4_joint']
LIFT_VMAX = 0.15         # lift_joint 的速度限(URDF), 轨迹时长按它的一半算


class Bench(Node):
    def __init__(self):
        super().__init__('ag_bench')
        self.js, self.js_v = {}, {}
        self.yaw = None
        self.odom_log = []          # (仿真时刻, yaw)
        self.clock = []
        self.pc = []
        self.pc_shape = None
        self.create_subscription(JointState, '/joint_states', self._js, 100)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self._odom, 10)
        self.create_subscription(Clock, '/clock',
                                 lambda m: self.clock.append(time.monotonic()), 2000)
        self.create_subscription(PointCloud2, '/velodyne_points', self._pc, 10)
        self.cmd = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.ac_lift = ActionClient(self, FollowJointTrajectory,
                                    '/lift_controller/follow_joint_trajectory')
        self.ac_arm1 = ActionClient(self, FollowJointTrajectory,
                                    '/arm1_controller/follow_joint_trajectory')

    def _js(self, m):
        for n, p, v in zip(m.name, m.position, m.velocity):
            self.js[n], self.js_v[n] = p, v

    def _odom(self, m):
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom_log.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, self.yaw))

    def _pc(self, m):
        self.pc.append(time.monotonic())
        self.pc_shape = (m.width, m.height)

    def spin_for(self, dur, publish=None, rate=20.0):
        t0, nxt = time.monotonic(), 0.0
        while time.monotonic() - t0 < dur:
            if publish is not None and time.monotonic() >= nxt:
                self.cmd.publish(publish)
                nxt = time.monotonic() + 1.0 / rate
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_topics(self, timeout=90.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            self.spin_for(0.5)
            if self.js and self.pc and self.clock:
                return True
        return False

    def rate_of(self, buf, dur):
        """数 buf 在 dur 秒**墙上时间**里收到多少条 → 每秒条数"""
        buf.clear()
        self.spin_for(dur)
        return len(buf) / dur

    def send_traj(self, client, names, positions, secs=2.0):
        """与 ~/ztools/ag_lift_test.sh 同样的发法: 只给 positions + time_from_start。
           **不要**加 velocities=[0]: 那会让 JTC 生成末端零速轨迹, 在 2 s 内走完 0.30 m
           会超出 path 容差 0.05 而被 Aborted —— 原网格版与简化版都是如此(实测)。"""
        client.wait_for_server(timeout_sec=20.0)
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = list(names)
        p = JointTrajectoryPoint()
        p.positions = [float(x) for x in positions]
        p.time_from_start.sec = int(secs)
        g.trajectory.points = [p]
        fut = client.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return None, '目标被拒绝'
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=secs + 20.0)
        if not res.done():
            return None, '结果超时'
        r = res.result()
        return r.result.error_code, 'action=%s ec=%d' % (r.status, r.result.error_code)


def main():
    ap = argparse.ArgumentParser(description='AG_robot 仿真性能与运动跟踪实测')
    ap.add_argument('--label', default='model', help='这一轮的名字(打在标题上)')
    ap.add_argument('--rtf-dur', type=float, default=10.0, help='/clock 采样时长(s)')
    ap.add_argument('--pc-dur', type=float, default=15.0, help='点云采样时长(s)')
    ap.add_argument('--no-lift', action='store_true', help='跳过升降/机械臂测试')
    args = ap.parse_args()

    rclpy.init()
    n = Bench()
    print('══ %s ══' % args.label)
    if not n.wait_topics():
        print('❌ 等不到 /joint_states /velodyne_points /clock —— 仿真在跑吗? 环境 source 了吗?')
        return 2

    # ① 实时率(墙上时钟口径)
    hz = n.rate_of(n.clock, args.rtf_dur)
    print('① /clock            : %8.1f Hz  → 实时率 RTF ≈ %.3f (标称 1000 Hz / 1 ms 步长)'
          % (hz, hz / 1000.0))

    # ② 点云(墙上时钟口径)
    hz_pc = n.rate_of(n.pc, args.pc_dur)
    print('② /velodyne_points  : %8.2f Hz  (传感器配置 10 Hz, 实际受实时率限制)  每帧 %s 点'
          % (hz_pc, n.pc_shape))

    # ③ 直行
    tw = Twist()
    tw.linear.x = 0.3
    n.spin_for(3.0, publish=tw)
    n.spin_for(0.5)
    v = [n.js_v.get(j, float('nan')) for j in WHEELS]
    exp = 0.3 / WHEEL_R
    print('③ 直行 cmd 0.3 m/s  : 四轮角速度 %s rad/s  理论 %.4f, 误差 %+.2f%%'
          % (' '.join('%.4f' % x for x in v), exp,
             100 * (sum(v) / len(v) - exp) / exp))
    n.cmd.publish(Twist())
    n.spin_for(1.0)

    # ④ 转向: 角速度必须用仿真时钟算(见文件头说明)
    tw = Twist()
    tw.angular.z = 0.5
    n.spin_for(1.5, publish=tw)
    n.odom_log.clear()
    n.spin_for(3.0, publish=tw)
    pts = list(n.odom_log)
    rate = float('nan')
    if len(pts) >= 4:
        ys = [p[1] for p in pts]
        ys = [y - 2 * math.pi * round((y - ys[0]) / (2 * math.pi)) for y in ys]   # 解缠绕
        if pts[-1][0] > pts[0][0]:
            rate = (ys[-1] - ys[0]) / (pts[-1][0] - pts[0][0])
    wl = (n.js_v.get('wheel_1_joint', 0.0) + n.js_v.get('wheel_3_joint', 0.0)) / 2
    wr = (n.js_v.get('wheel_2_joint', 0.0) + n.js_v.get('wheel_4_joint', 0.0)) / 2
    kin = (wr - wl) * WHEEL_R / WHEEL_L
    print('④ 转向 cmd 0.5 rad/s: 仿真时钟下稳态角速度 %+.4f rad/s (期望 ±0.5)  四轮 %s'
          % (rate, ' '.join('%+.4f' % n.js_v.get(j, float('nan'))
                            for j in ('wheel_1_joint', 'wheel_2_joint',
                                      'wheel_3_joint', 'wheel_4_joint'))))
    print('   左右轮平均 %+.4f / %+.4f rad/s → 运动学角速度 %+.4f rad/s' % (wl, wr, kin))
    n.cmd.publish(Twist())
    n.spin_for(1.0)

    if args.no_lift:
        n.destroy_node()
        rclpy.shutdown()
        return 0

    # ⑤ 升降: 目标用"一半速度限"的时长, 这样能真正走到位
    print('⑤ 升降 lift_joint   :')
    for tgt in (0.15, 0.30, 0.0):
        cur = n.js.get('lift_joint', 0.0)
        secs = max(2.0, abs(tgt - cur) / (LIFT_VMAX / 2))
        ec, info = n.send_traj(n.ac_lift, ['lift_joint'], [tgt], secs)
        n.spin_for(1.0)
        got = n.js.get('lift_joint', float('nan'))
        ok = ec == 0 and abs(got - tgt) < 0.005
        print('     %.2f → %.5f m   %s   (%s)' % (tgt, got, '✅' if ok else '❌', info))

    # ⑥ 机械臂
    tgt = [n.js.get(j, 0.0) for j in ARM1]
    tgt[0] = 0.5
    ec, info = n.send_traj(n.ac_arm1, ARM1, tgt, 3.0)
    n.spin_for(0.8)
    got = [n.js.get(j, float('nan')) for j in ARM1]
    err = max(abs(a - b) for a, b in zip(got, tgt))
    print('⑥ arm1 关节角       : 目标 joint1=0.5 → 实测 [%s]  %s   (%s)'
          % (' '.join('%.4f' % x for x in got), '✅' if err < 0.01 else '❌', info))
    print('   最大关节误差 %.5f rad' % err)

    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
