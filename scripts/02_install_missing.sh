#!/bin/bash
# ============================================================
# 仅安装你可能缺失的组件
# 已有的部分可以跳过
# ============================================================

# ---- 1. MAVROS (如果没装) ----
sudo apt update
sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras
# 安装 GeographicLib 数据集 (MAVROS 必需)
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

# ---- 2. PX4 (如果没克隆) ----
if [ ! -d ~/PX4-Autopilot ]; then
    cd ~
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive -b v1.14.3
    cd PX4-Autopilot
    bash ./Tools/setup/ubuntu.sh --no-sim-tools
fi

# ---- 3. Python 依赖 ----
pip3 install numpy scipy matplotlib scikit-learn onnxruntime

# ---- 4. Gazebo 模型路径相关 ----
sudo apt install -y ros-noetic-gazebo-ros ros-noetic-gazebo-plugins

# ---- 5. 创建 catkin 工作空间 (如果没有) ----
if [ ! -d ~/catkin_ws ]; then
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin_make
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
fi

source ~/.bashrc
echo "✅ 安装完成"
