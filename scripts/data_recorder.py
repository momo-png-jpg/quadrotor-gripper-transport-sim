#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase 3: 扰动数据采集节点 (ROS1 + MAVROS 版)

订阅 MAVROS 话题获取飞行数据, 动力学反算得到扰动标签
用于后续训练神经网络

使用方法:
  rosrun quadrotor_gripper_sim data_recorder.py _save_dir:=./data
"""
import rospy
import numpy as np
import os
import time
from collections import deque

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import ActuatorControl, State, AttitudeTarget
from nav_msgs.msg import Odometry
import tf.transformations as tf_trans


class DataRecorder:
    def __init__(self):
        rospy.init_node('data_recorder', anonymous=True)

        # ---- 参数 ----
        self.save_dir = rospy.get_param('~save_dir', 'data/')
        self.record_rate = rospy.get_param('~record_rate', 200.0)
        self.max_samples = rospy.get_param('~max_samples', 500000)
        os.makedirs(self.save_dir, exist_ok=True)

        # ---- 状态变量 ----
        self.euler = np.zeros(3)          # [roll, pitch, yaw]
        self.omega = np.zeros(3)          # [p, q, r]  角速度
        self.velocity = np.zeros(3)       # [vx, vy, vz]
        self.acceleration = np.zeros(3)   # [ax, ay, az]
        self.position = np.zeros(3)       # [x, y, z]
        self.actuator_controls = np.zeros(4)  # 归一化电机输入
        self.connected = False

        # ---- 角加速度估计 ----
        self.prev_omega = np.zeros(3)
        self.omega_dot = np.zeros(3)
        self.alpha_lpf = 0.3  # 低通滤波

        # ---- 动力学参数 (与模型 SDF 一致) ----
        self.Ixx = 0.029125
        self.Iyy = 0.029125
        self.Izz = 0.055225
        self.arm_length = 0.18 * np.sqrt(2)  # 对角线距离
        self.k_f = 8.54858e-06
        self.k_m = 0.06

        # ---- 数据缓冲 ----
        self.data_buffer = []
        self.sample_count = 0

        # ---- 订阅 MAVROS 话题 ----
        rospy.Subscriber('/mavros/state', State, self.state_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, self.vel_cb)
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_cb)
        rospy.Subscriber('/mavros/target_actuator_control', ActuatorControl, self.actuator_cb)

        # ---- 定时器 ----
        dt = 1.0 / self.record_rate
        rospy.Timer(rospy.Duration(dt), self.record_callback)

        rospy.loginfo(f"数据记录器启动 | 采样率: {self.record_rate}Hz | "
                      f"保存路径: {self.save_dir}")

    # ============================================================
    # 回调函数
    # ============================================================
    def state_cb(self, msg):
        self.connected = msg.connected

    def pose_cb(self, msg):
        p = msg.pose.position
        self.position = np.array([p.x, p.y, p.z])

        q = msg.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        euler = tf_trans.euler_from_quaternion(quat)
        self.euler = np.array(euler)  # [roll, pitch, yaw]

    def vel_cb(self, msg):
        v = msg.twist.linear
        self.velocity = np.array([v.x, v.y, v.z])

        w = msg.twist.angular
        self.omega = np.array([w.x, w.y, w.z])

    def imu_cb(self, msg):
        a = msg.linear_acceleration
        self.acceleration = np.array([a.x, a.y, a.z])

    def actuator_cb(self, msg):
        # PX4 actuator_control group 0: [roll, pitch, yaw, thrust]
        self.actuator_controls = np.array(msg.controls[:4])

    # ============================================================
    # 核心: 记录数据 + 反算扰动
    # ============================================================
    def record_callback(self, event):
        if not self.connected:
            return
        if abs(self.position[2]) < 0.3:  # 还没起飞
            return
        if self.sample_count >= self.max_samples:
            return

        dt = 1.0 / self.record_rate

        # ---- 估计角加速度 ----
        omega_dot_raw = (self.omega - self.prev_omega) / dt
        self.omega_dot = (self.alpha_lpf * omega_dot_raw +
                          (1 - self.alpha_lpf) * self.omega_dot)
        self.prev_omega = self.omega.copy()

        # ---- 动力学反算扰动力矩 ----
        p, q, r = self.omega
        p_dot, q_dot, r_dot = self.omega_dot

        # 从角加速度和已知惯量反算"实际需要的力矩"
        tau_needed_x = self.Ixx * p_dot - (self.Iyy - self.Izz) * q * r
        tau_needed_y = self.Iyy * q_dot - (self.Izz - self.Ixx) * p * r
        tau_needed_z = self.Izz * r_dot - (self.Ixx - self.Iyy) * p * q

        # PX4 actuator_controls 是归一化值 [-1, 1]
        # 近似: 控制器施加的力矩 ≈ actuator × 最大力矩
        max_tau_xy = 0.5   # 估计值, 需根据电机参数调
        max_tau_z = 0.2
        tau_cmd_x = self.actuator_controls[0] * max_tau_xy
        tau_cmd_y = self.actuator_controls[1] * max_tau_xy
        tau_cmd_z = self.actuator_controls[2] * max_tau_z

        # ★ 扰动 = 实际需要的力矩 - 控制器施加的力矩
        d_tx = tau_needed_x - tau_cmd_x
        d_ty = tau_needed_y - tau_cmd_y
        d_tz = tau_needed_z - tau_cmd_z

        # 力扰动 (简化)
        m_total = 2.18  # 总质量估计
        d_fx = 0.0  # 力扰动较难精确反算, 后续可以用 NN 直接估
        d_fy = 0.0
        d_fz = 0.0

        # ---- 组装样本: 12维输入 + 6维标签 ----
        sample = np.concatenate([
            self.euler,             # [0:3]
            self.omega,             # [3:6]
            self.velocity,          # [6:9]
            self.acceleration,      # [9:12]
            np.array([d_fx, d_fy, d_fz, d_tx, d_ty, d_tz])  # [12:18]
        ])

        self.data_buffer.append(sample)
        self.sample_count += 1

        if self.sample_count % 10000 == 0:
            rospy.loginfo(f"已采集 {self.sample_count} / {self.max_samples} 样本")

        if self.sample_count % 100000 == 0:
            self._save(intermediate=True)

    def _save(self, intermediate=False):
        data = np.array(self.data_buffer, dtype=np.float32)
        ts = int(time.time())
        tag = '_inter' if intermediate else '_final'
        fname = f'disturbance_data_{ts}{tag}.npy'
        fpath = os.path.join(self.save_dir, fname)
        np.save(fpath, data)
        rospy.loginfo(f"💾 保存: {fpath} | shape={data.shape}")

    def on_shutdown(self):
        if len(self.data_buffer) > 0:
            self._save(intermediate=False)
        rospy.loginfo(f"✅ 记录结束, 共 {self.sample_count} 样本")


if __name__ == '__main__':
    try:
        recorder = DataRecorder()
        rospy.on_shutdown(recorder.on_shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
