#!/bin/bash
# ============================================================
# 编译项目 + 注册模型路径
# ============================================================

# 1. 把自定义模型链接到 Gazebo 默认模型目录
ln -sf ~/catkin_ws/src/quadrotor_gripper_sim/models/x500_gripper \
       ~/.gazebo/models/x500_gripper

# 2. 编译 catkin 工作空间
cd ~/catkin_ws
catkin_make

# 3. source 环境
source devel/setup.bash

# 4. 设置 PX4 环境变量
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/quadrotor_gripper_sim/models

echo "✅ 编译完成, 模型路径已设置"
echo ""
echo "测试启动:"
echo "  roslaunch quadrotor_gripper_sim sitl_gripper.launch"
