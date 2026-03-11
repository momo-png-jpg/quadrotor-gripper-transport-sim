#!/bin/bash
echo "============================================"
echo "  环境检查脚本"
echo "============================================"

# 1. Ubuntu 版本
echo -n "Ubuntu: "
lsb_release -d | cut -f2

# 2. ROS
echo -n "ROS:    "
rosversion -d 2>/dev/null || echo "未安装"

# 3. Gazebo
echo -n "Gazebo: "
gazebo --version 2>/dev/null | head -1 || echo "未安装"

# 4. PX4
echo -n "PX4:    "
if [ -d ~/PX4-Autopilot ]; then
    echo "已克隆 ($(cd ~/PX4-Autopilot && git describe --tags 2>/dev/null))"
else
    echo "未找到"
fi

# 5. MAVROS
echo -n "MAVROS: "
rospack find mavros 2>/dev/null && echo "" || echo "未安装"

# 6. Python
echo -n "Python: "
python3 --version

# 7. pip 包
echo "Python 包:"
pip3 list 2>/dev/null | grep -E "numpy|torch|onnxruntime|scipy|sklearn" || echo "  需要安装"

echo "============================================"
