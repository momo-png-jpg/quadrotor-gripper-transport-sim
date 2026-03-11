#!/bin/bash
cd ~/catkin_ws/src

catkin_create_pkg quadrotor_gripper_sim \
    rospy roscpp std_msgs geometry_msgs sensor_msgs \
    mavros_msgs nav_msgs tf2_ros

cd quadrotor_gripper_sim
mkdir -p models/x500_gripper
mkdir -p worlds
mkdir -p launch
mkdir -p scripts
mkdir -p config
mkdir -p data
mkdir -p trained_models

cd ~/catkin_ws
catkin_make
source devel/setup.bash

echo "✅ ROS 包创建完成"
