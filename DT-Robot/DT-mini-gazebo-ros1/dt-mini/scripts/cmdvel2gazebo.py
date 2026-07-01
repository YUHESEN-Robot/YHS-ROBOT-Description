#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math

class DifferentialDriveController:
    def __init__(self):
        rospy.init_node('differential_drive_controller', anonymous=True)
        
        rospy.Subscriber('/cmd_vel', Twist, self.callback, queue_size=1)

        # 速度控制器发布器
        self.front_left_pub = rospy.Publisher('/dt_mini/front_left_velocity_controller/command', Float64, queue_size=1)
        self.front_right_pub = rospy.Publisher('/dt_mini/front_right_velocity_controller/command', Float64, queue_size=1)
        self.rear_left_pub = rospy.Publisher('/dt_mini/rear_left_velocity_controller/command', Float64, queue_size=1)
        self.rear_right_pub = rospy.Publisher('/dt_mini/rear_right_velocity_controller/command', Float64, queue_size=1)

        # 车辆参数
        self.wheel_radius = 0.1  # 车轮半径 (m)
        self.wheel_separation = 0.448  # 左右轮间距 (m)
        
        # 安全参数
        self.timeout = rospy.Duration.from_sec(0.2)  # 200ms超时
        self.lastMsg = rospy.Time.now()

        # 控制变量初始化
        self.linear_vel = 0.0  # 线速度 (m/s)
        self.angular_vel = 0.0  # 角速度 (rad/s)

        rate = rospy.Rate(100)  # 100Hz控制频率
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def callback(self, data):
        # 记录最新的速度命令
        self.linear_vel = data.linear.x
        self.angular_vel = data.angular.z
        self.lastMsg = rospy.Time.now()

    def publish(self):
        # 死锁检测 - 超过200ms未收到新指令则停止
        if (rospy.Time.now() - self.lastMsg) > self.timeout:
            self.stop_all_wheels()
            return

        # 差速驱动机器人运动学模型
        # 计算左右轮线速度
        left_speed = self.linear_vel - (self.angular_vel * self.wheel_separation) / 2.0
        right_speed = self.linear_vel + (self.angular_vel * self.wheel_separation) / 2.0
        
        # 转换为轮子角速度 (rad/s)
        left_wheel_ang_vel = left_speed / self.wheel_radius
        right_wheel_ang_vel = right_speed / self.wheel_radius
        
        # 发布所有轮子的速度指令
        # 对于差速驱动，同侧轮子速度相同
        self.front_left_pub.publish(Float64(left_wheel_ang_vel))
        self.rear_left_pub.publish(Float64(left_wheel_ang_vel))
        self.front_right_pub.publish(Float64(right_wheel_ang_vel))
        self.rear_right_pub.publish(Float64(right_wheel_ang_vel))

    def stop_all_wheels(self):
        # 发布零速度停止所有轮子
        self.front_left_pub.publish(Float64(0))
        self.front_right_pub.publish(Float64(0))
        self.rear_left_pub.publish(Float64(0))
        self.rear_right_pub.publish(Float64(0))
        rospy.logdebug("No cmd_vel received for 200ms - stopping robot")

if __name__ == '__main__':
    try:
        DifferentialDriveController()
    except rospy.ROSInterruptException:
        pass