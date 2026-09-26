#!/bin/bash
# 四种（或任意组）雷达倾角依次起仿真 + 测盲区, 输出一张对比表。
#
# 用法（**先在当前终端 source 好环境**）:
#     source /opt/ros/humble/setup.bash
#     source <gz_ros2_control 工作区>/install/setup.bash
#     source <本包所在工作区>/install/setup.bash
#     bash scripts/lidar_pose_sweep.sh              # 默认 0 15 25 35
#     bash scripts/lidar_pose_sweep.sh 10 20        # 或指定任意角度
#
# 模型用环境变量选（默认原网格版）:
#     AG_MODEL=simple bash scripts/lidar_pose_sweep.sh     # 图元简化版
#     两个模型跑出来的表应当逐项吻合 —— 这是"简化没改到传感器/整体几何"的直接验证。
#
# 注意: 它会**杀掉当前正在跑的仿真**（一次只能用一套 gz 服务）, 每轮约 1 分钟。
set -u

PITCHES="${*:-0 15 25 35}"
MODEL="${AG_MODEL:-full}"
LOGDIR=/tmp/lidar_sweep
mkdir -p "$LOGDIR"

command -v ros2 >/dev/null || { echo "先 source ROS 环境再跑本脚本"; exit 1; }
PKG_LAUNCH=$(ros2 pkg prefix ag_robot_description 2>/dev/null) \
  || { echo "找不到 ag_robot_description 包 —— 先 source 本包的工作区"; exit 1; }

stop_sim() {
  pgrep -f 'ros2 launch ag_robot_description' | while read -r p; do kill "$p" 2>/dev/null; done
  pgrep -f 'gz sim' | while read -r p; do kill "$p" 2>/dev/null; done
  sleep 5
  pgrep -f 'gz sim' | while read -r p; do kill -9 "$p" 2>/dev/null; done
  sleep 2
}

echo "模型: $MODEL   (AG_MODEL=simple 可切到图元简化版)"
printf '%-8s %-10s %-12s %-14s %-14s %-16s %s\n' \
  "倾角" "光心离地" "车头近地" "左右近地" "车尾近地" "车尾无地面扇区" "本体遮挡"
printf '%s\n' "--------------------------------------------------------------------------------------------------"

for p in $PITCHES; do
  stop_sim
  LOG="$LOGDIR/${MODEL}_pitch_$p.log"
  nohup ros2 launch ag_robot_description gazebo.launch.py rviz:=false \
        model:="$MODEL" lidar_pitch:="$p" > "$LOG" 2>&1 &
  # 等点云话题出数据
  ok=0
  for _ in $(seq 1 40); do
    sleep 3
    if timeout 6 ros2 topic hz /velodyne_points 2>/dev/null | grep -q 'average rate'; then ok=1; break; fi
  done
  if [ "$ok" != 1 ]; then
    printf '%-8s %s\n' "${p}°" "❌ 起仿真失败 (看 $LOG)"
    continue
  fi
  # 等仿真稳定一点再测
  sleep 5
  # 撤掉演示障碍物: 它们是给 RViz 看的, 但会在某些方位挡住地面回波
  # (0° 时车头方向有墙、左右有方箱), 不撤掉四种倾角就没法公平对比。
  # 注意: 不要用 `gz model --list` 判断有没有撤掉 —— 仿真刚起时这条命令会超时
  # 返回空, 会被误判成"已经撤掉了"。这里直接幂等调用 remove 三次。
  remove_obstacles() {
    for _ in 1 2 3; do
      timeout 12 gz service -s "/world/ag_robot/remove" --reqtype gz.msgs.Entity \
          --reptype gz.msgs.Boolean --timeout 8000 \
          --req 'name: "lidar_demo_obstacles" type: MODEL' >/dev/null 2>&1
      sleep 3
    done
  }
  remove_obstacles
  OUT=$(timeout 150 python3 "$(dirname "$0")/lidar_blind_zone.py" 2>&1)
  # 自查: "环境障碍物回波"占比过高说明障碍物没撤干净, 再撤一次重测
  ENVPCT=$(echo "$OUT" | grep '环境障碍物回波' | grep -oE '[0-9.]+%' | head -1 | tr -d '%')
  if awk "BEGIN{exit !(${ENVPCT:-0} > 5)}"; then
    echo "  (障碍物回波 ${ENVPCT}% 偏高, 重新撤障碍物并复测)"
    remove_obstacles
    OUT=$(timeout 150 python3 "$(dirname "$0")/lidar_blind_zone.py" 2>&1)
  fi
  echo "$OUT" > "$LOGDIR/${MODEL}_blindzone_$p.txt"

  # 顺带出一张点云图 (需要 matplotlib; 失败不影响主流程)
  if timeout 150 python3 "$(dirname "$0")/lidar_capture_png.py" \
       -o "$LOGDIR/${MODEL}_scan_${p}deg.png" --range 6 >/dev/null 2>&1; then
    echo "  (已出图 $LOGDIR/${MODEL}_scan_${p}deg.png)"
  fi

  get() { echo "$OUT" | grep -E "$1" | head -1 | sed -E 's/.*: *//'; }
  HEIGHT=$(echo "$OUT" | grep -oE '光心离地 [0-9.]+ m' | head -1 | awk '{print $2" m"}')
  FRONT=$(echo "$OUT" | grep -E '前\(-Y\)' | head -1 | sed -E 's/.*: *//')
  SIDE=$(echo "$OUT"  | grep -E '左/右' | head -1 | sed -E 's/.*: *//')
  BACK=$(echo "$OUT"  | grep -E '后\(\+Y\)' | head -1 | sed -E 's/.*: *//')
  SECTOR=$(echo "$OUT" | grep -E '车尾无地面覆盖扇区' | head -1 | sed -E 's/.*: *//')
  SELF=$(echo "$OUT" | grep -E '机器人本体自遮挡' | head -1 | awk '{print $2}')
  printf '%-8s %-10s %-12s %-14s %-14s %-16s %s\n' \
    "${p}°" "${HEIGHT:-—}" "${FRONT:-—}" "${SIDE:-—}" "${BACK:-—}" "${SECTOR:-无}" "${SELF:-—}"
  stop_sim
done

echo
echo "明细: $LOGDIR/${MODEL}_blindzone_<倾角>.txt   仿真日志: $LOGDIR/${MODEL}_pitch_<倾角>.log"
