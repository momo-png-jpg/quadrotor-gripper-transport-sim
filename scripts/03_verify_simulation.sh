#!/bin/bash
# ============================================================
# 验证: PX4 SITL + Gazebo + MAVROS 能正常通信
# 需要开 3 个终端
# ============================================================

echo "===== 请按以下顺序在 3 个终端中执行 ====="
echo ""
echo "---- 终端 1: 启动 PX4 SITL + Gazebo ----"
echo "  cd ~/PX4-Autopilot"
echo "  DONT_RUN=1 make px4_sitl_default gazebo"
echo "  source ~/catkin_ws/devel/setup.bash"
echo "  source Tools/simulation/gazebo-classic/setup_gazebo.bash \$(pwd) \$(pwd)/build/px4_sitl_default"
echo "  export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:\$(pwd):\$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
echo "  roslaunch px4 posix_sitl.launch"
echo ""
echo "---- 终端 2: 启动 MAVROS ----"
echo "  source ~/catkin_ws/devel/setup.bash"
echo "  roslaunch mavros px4.launch fcu_url:='udp://:14540@127.0.0.1:14557'"
echo ""
echo "---- 终端 3: 检查话题 ----"
echo "  rostopic list | grep mavros"
echo "  rostopic echo /mavros/state -n 1"
echo "  # 应该看到 connected: True"
echo ""
echo "如果看到 Gazebo 中有一架 Iris 四旋翼, MAVROS connected: True → 环境正常 ✅"
