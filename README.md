# 四旋翼夹持负载运输仿真 — ASMC + 神经网络抗扰控制器

> Quadrotor UAV with Gripper Payload Transport — Adaptive Sliding Mode Control + Neural Network Disturbance Compensation

## 项目简介

本项目在 **Ubuntu 20.04 + ROS Noetic + PX4 + Gazebo Classic** 环境下，实现了一个带底部夹持爪的四旋翼无人机运输仿真系统。

核心控制器采用 **自适应滑模控制 (ASMC) + 1D-CNN 神经网络扰动补偿** 的混合架构，神经网络通过离线训练学习夹持负载引起的扰动模式，在线实时补偿扰动力矩。

## 系统架构

```
期望轨迹 → 外环位置PID → 期望姿态 → 内环ASMC+NN → 控制力矩 → 电机混控 → 四旋翼+夹爪+负载
                                         ↑
                                   NN扰动估计 d̂(t)
                                   (1D-CNN, ONNX推理)
```

## 技术栈

| 组件 | 版本/工具 |
|------|----------|
| OS | Ubuntu 20.04 |
| ROS | Noetic (ROS1) |
| 仿真器 | Gazebo Classic 11 |
| 飞控 | PX4 Autopilot (SITL) |
| 通信桥 | MAVROS |
| 神经网络训练 | PyTorch (GPU工作站) |
| 神经网络推理 | ONNX Runtime (机载CPU) |

## 目录结构

```
quadrotor-gripper-transport-sim/
├── README.md
├── models/
│   └── x500_gripper/
│       ├── model.sdf              # 四旋翼+夹爪+负载 SDF模型
│       └── model.config           # Gazebo 模型配置
├── worlds/
│   └── transport_world.world      # 仿真世界 (起飞/降落点)
├── launch/
│   └── sitl_gripper.launch        # 一键启动 launch 文件
├── scripts/
│   ├── data_recorder.py           # 扰动数据采集节点
│   ├── auto_flight_collector.py   # 自动飞行数据采集
│   ├── asmc_nn_controller.py      # ASMC+NN 集成控制器
│   └── comparison_flight.py       # 对比实验脚本
├── training/
│   └── train_and_export.py        # 神经网络训练+ONNX导出
├── config/
│   └── controller_params.yaml     # 控制器参数配置
├── scripts/shell/
│   ├── 01_check_environment.sh    # 环境检查
│   ├── 02_install_missing.sh      # 安装缺失组件
│   └── 03_run_data_collection.sh  # 数据采集流程
├── data/                          # 采集的扰动数据 (.npy)
└── trained_models/                # 训练好的模型 (.onnx)
```

## 快速开始

### 1. 环境准备

```bash
# 检查环境
bash scripts/shell/01_check_environment.sh

# 安装缺失组件
bash scripts/shell/02_install_missing.sh
```

### 2. 编译

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3. 启动仿真

```bash
roslaunch quadrotor_gripper_sim sitl_gripper.launch
```

### 4. 数据采集

```bash
# 终端2: 数据记录器
rosrun quadrotor_gripper_sim data_recorder.py

# 终端3: 自动飞行
rosrun quadrotor_gripper_sim auto_flight_collector.py
```

### 5. 训练神经网络 (GPU工作站)

```bash
python3 training/train_and_export.py
```

### 6. 运行 ASMC+NN 控制器

```bash
rosrun quadrotor_gripper_sim asmc_nn_controller.py \
  _nn_model_path:=trained_models/disturbance_cnn.onnx
```

## 控制器原理

### 控制律

```
u_total = u_eq + u_smc + u_nn

u_eq  = J·(φ̈_d + λ·ė) - 交叉耦合项     (标称模型等效控制)
u_smc = K·sat(s/ε)                       (滑模鲁棒项)
u_nn  = -d̂(t)                            (NN扰动补偿, 取反抵消)
```

### 神经网络

- **结构**: 1D-CNN (3层卷积 + 全局平均池化 + 2层全连接)
- **输入**: 最近20帧飞行状态 (12维 × 20步)
- **输出**: 3维扰动力矩估计 [d_Tx, d_Ty, d_Tz]
- **参数量**: ~12K (CPU推理 < 0.3ms)

## 参考文献

1. Physics-Informed Neural Network for Multirotor Slung Load Systems (ICRA 2024)
2. Adaptive Neural Network-Based Fault-Tolerant Control for Quadrotor Slung Load
3. Disturbance Observer and Adaptive Control for Quadrotor: A Survey (2024)

## License

MIT License