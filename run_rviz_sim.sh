#!/usr/bin/env bash
# 天工 Dex 双臂双手垂直平面形状 demo —— 一键启动 RViz2 仿真
#
#   ./run_rviz_sim.sh                          # 右臂食指，full 模式（耦合全链），circle
#   ./run_rviz_sim.sh triangle --mode fixed    # 手指定值 + 臂 7-DOF；形状可换 arc / line
#   ./run_rviz_sim.sh --mode tcp               # 原 demo：right_tcp_link 走轨迹
#   ./run_rviz_sim.sh --arm both --mode full   # 双臂镜像协同（左手画右手的镜像）
#   ./run_rviz_sim.sh --arm left --finger-joint thumb   # 左臂 + 拇指
#   ./run_rviz_sim.sh --finger 0.9             # 手指弯曲程度 (rad, 四指 0~1.41 / 拇指 0~1.03)
#   ./run_rviz_sim.sh --offset 0.02 0 0.05     # 平移预览图形/轨迹中心 (m, pelvis 系)
#   ./run_rviz_sim.sh --waist free             # 只放腰 yaw（水平旋转，扩工作空间；both 时腰归右臂）
#   ./run_rviz_sim.sh --waist all              # 腰 yaw+roll+pitch 全放（弯腰抓取等，慎用）
#   ./run_rviz_sim.sh --fingers index,thumb    # 多指联动：主指画轨迹，副指保持手型随动
#
# tesseract 现场规划 + 机器人动画 + 指尖历史轨迹，不产生任何中间文件。
# 启动三样东西：robot_state_publisher / demo 节点 / rviz2
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
URDF="$HERE/tienkung_dex/urdf/tienkung_dex.hand.urdf"
CONDA_PY=/home/qingxiangliu/miniconda3/envs/dex/bin/python
CONDA_LIB=/home/qingxiangliu/miniconda3/envs/dex/lib

source /opt/ros/jazzy/setup.bash

# 1) robot_state_publisher：加载 URDF，发布 /robot_description 与 TF
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat "$URDF")" &
RSP=$!

# 2) demo 节点：tesseract 现场规划 + 发布轨迹话题
#    只有它需要 conda 环境的库，所以用环境变量前缀单独设置，不影响 rviz2
env LD_LIBRARY_PATH="${CONDA_LIB}:${LD_LIBRARY_PATH:-}" \
  "$CONDA_PY" "$HERE/circle_planning_7axis.py" "$@" &
DEMO=$!

# 3) rviz2
sleep 1
rviz2 -d "$HERE/rviz/tienkung_dex_circle.rviz" &
RVIZ=$!

cleanup() { kill "$RSP" "$DEMO" "$RVIZ" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
wait "$DEMO"
