#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自动化飞行数据采集 (ROS1 + MAVROS Offboard)

让无人机自动执行多种轨迹, 配合 data_recorder.py 同时运行
"""
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
import time


class AutoFlightCollector:
    def __init__(self):
        rospy.init_node('auto_flight_collector')

        # ---- 发布者/订阅者 ----
        self.setpoint_pub = rospy.Publisher(
            '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        rospy.Subscriber('/mavros/state', State, self.state_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)

        self.current_state = State()
        self.current_position = np.zeros(3)

        # ---- 服务客户端 ----
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        # ---- 等待 MAVROS 连接 ----
        rospy.loginfo("等待 MAVROS 连接...")
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.current_state.connected:
            rate.sleep()
        rospy.loginfo("✅ MAVROS 已连接")

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        p = msg.pose.position
        self.current_position = np.array([p.x, p.y, p.z])

    def send_setpoint(self, x, y, z, yaw=0.0):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # 四元数 (只设偏航)
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = np.sin(yaw / 2)
        pose.pose.orientation.w = np.cos(yaw / 2)
        self.setpoint_pub.publish(pose)

    def arm_and_offboard(self):
        """解锁 + 切换到 Offboard 模式"""
        rate = rospy.Rate(20)

        # 先发送一些 setpoint (PX4 要求在切 Offboard 之前就有 setpoint 流)
        rospy.loginfo("预发送 setpoint...")
        for _ in range(100):
            self.send_setpoint(0, 0, 1.5)
            rate.sleep()

        # 切换到 Offboard
        rospy.loginfo("切换到 OFFBOARD 模式...")
        self.set_mode_client(custom_mode="OFFBOARD")
        rospy.sleep(1)

        # 解锁
        rospy.loginfo("解锁...")
        self.arming_client(True)
        rospy.sleep(1)

        rospy.loginfo("🚁 已起飞准备!")

    def fly_to(self, x, y, z, duration=5.0, yaw=0.0):
        """飞向目标点并保持一段时间"""
        rate = rospy.Rate(20)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - t0).to_sec()
            if elapsed > duration:
                break
            self.send_setpoint(x, y, z, yaw)
            rate.sleep()

    def fly_trajectory(self, traj_func, duration=15.0):
        """按轨迹函数飞行"""
        rate = rospy.Rate(20)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            t = (rospy.Time.now() - t0).to_sec()
            if t > duration:
                break
            pos = traj_func(t)
            self.send_setpoint(pos[0], pos[1], pos[2])
            rate.sleep()

    def run_all_missions(self):
        """执行全部数据采集任务"""
        self.arm_and_offboard()

        missions = [
            # (名称, 轨迹函数, 持续时间)
            ("悬停 2m", lambda t: [0, 0, 2.0], 15.0),
            ("悬停 3m", lambda t: [0, 0, 3.0], 15.0),

            ("直飞 x+ 慢速", lambda t: [0.5*t, 0, 2.0], 12.0),
            ("直飞 x+ 快速", lambda t: [1.5*t, 0, 2.0], 8.0),
            ("侧飞 y+",      lambda t: [0, 0.8*t, 2.0], 10.0),

            ("圆形 R=1.5", lambda t: [
                1.5 * np.cos(0.4*t), 1.5 * np.sin(0.4*t), 2.0], 20.0),
            ("圆形 R=2.5", lambda t: [
                2.5 * np.cos(0.3*t), 2.5 * np.sin(0.3*t), 2.5], 25.0),

            ("8字形", lambda t: [
                2.0 * np.sin(0.3*t), 2.0 * np.sin(0.6*t), 2.0], 25.0),

            ("上下运动", lambda t: [0, 0, 2.0 + 1.0*np.sin(0.4*t)], 20.0),

            ("综合运输", self._transport_trajectory, 25.0),
        ]

        for i, (name, traj, dur) in enumerate(missions):
            rospy.loginfo(f"📋 任务 {i+1}/{len(missions)}: {name} ({dur}s)")

            # 每个任务开始前先回到悬停
            if i > 0:
                rospy.loginfo("  回到悬停点...")
                self.fly_to(0, 0, 2.0, duration=8.0)

            self.fly_trajectory(traj, duration=dur)
            rospy.loginfo(f"  ✅ {name} 完成")

        # 降落
        rospy.loginfo("📍 所有任务完成, 降落中...")
        self.fly_to(0, 0, 0.3, duration=10.0)
        self.set_mode_client(custom_mode="AUTO.LAND")
        rospy.loginfo("✅ 全部数据采集完成!")

    def _transport_trajectory(self, t):
        """综合运输轨迹"""
        if t < 5:
            return [0, 0, 2.0 * min(t/5, 1)]
        elif t < 15:
            f = (t - 5) / 10
            return [5*f, 3*f, 2.0]
        elif t < 20:
            f = (t - 15) / 5
            return [5, 3, 2.0 - 1.5*f]
        else:
            return [5, 3, 0.5]


if __name__ == '__main__':
    try:
        collector = AutoFlightCollector()
        collector.run_all_missions()
    except rospy.ROSInterruptException:
        pass
