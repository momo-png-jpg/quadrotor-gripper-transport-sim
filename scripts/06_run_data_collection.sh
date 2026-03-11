#!/bin/bash
# ============================================================
# 数据采集操作 (按顺序在不同终端执行)
# ============================================================

echo "在 4 个终端中分别执行以下命令:"
echo ""
echo "==== 终端 1: 启动仿真 ===="
echo "  cd ~/catkin_ws"
echo "  source devel/setup.bash"
echo "  roslaunch quadrotor_gripper_sim sitl_gripper.launch"
echo ""
echo "==== 终端 2: 启动数据记录器 ===="
echo "  cd ~/catkin_ws"
echo "  source devel/setup.bash"
echo "  rosrun quadrotor_gripper_sim data_recorder.py _save_dir:=$(pwd)/data"
echo ""
echo "==== 终端 3: 执行自动飞行采集 ===="
echo "  (等终端1的 Gazebo 完全启动后再运行)"
echo "  cd ~/catkin_ws"
echo "  source devel/setup.bash"
echo "  rosrun quadrotor_gripper_sim auto_flight_collector.py"
echo ""
echo "==== 终端 4: 监控 (可选) ===="
echo "  rostopic hz /mavros/imu/data"
echo "  rostopic echo /mavros/state"
echo ""
echo "数据采集大约需要 15-20 分钟"
echo "完成后数据保存在 ./data/ 目录下"
