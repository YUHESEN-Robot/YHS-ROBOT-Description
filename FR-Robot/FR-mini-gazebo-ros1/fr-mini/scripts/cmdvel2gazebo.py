#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math

class CmdVel2Gazebo:
    def __init__(self):
        rospy.init_node('cmdvel2gazebo', anonymous=True)
        
        rospy.Subscriber('/cmd_vel', Twist, self.callback, queue_size=1)

        # 发布器初始化
        self.pub_steerL = rospy.Publisher('/fr_mini/front_left_steering_position_controller/command', Float64, queue_size=1)
        self.pub_steerR = rospy.Publisher('/fr_mini/front_right_steering_position_controller/command', Float64, queue_size=1)
        self.pub_rearL = rospy.Publisher('/fr_mini/rear_left_velocity_controller/command', Float64, queue_size=1)
        self.pub_rearR = rospy.Publisher('/fr_mini/rear_right_velocity_controller/command', Float64, queue_size=1)

        # 车辆参数
        self.L = 0.4            # 轴距 (m)
        self.T_front = 0.312    # 前轮轮距 (m)
        self.T_rear = 0.312     # 后轮轮距 (m)
        self.wheel_radius = 0.07  # 车轮半径 (m)
        self.maxsteer = math.radians(30)  # 中间轮胎最大转向角30度

        # 安全参数
        self.timeout = rospy.Duration.from_sec(0.2)
        self.lastMsg = rospy.Time.now()

        # 控制变量初始化
        self.x = 0.0  # 线速度控制量
        self.z = 0.0  # 转向角控制量

        rate = rospy.Rate(10)  # 10Hz循环
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def callback(self, data):
        # 线速度转换：v(线速度) → ω(角速度) = v / r
        self.x = data.linear.x / self.wheel_radius
        
        # 转向角限幅（直接限制中间轮胎转向角）
        self.z = max(-self.maxsteer, min(self.maxsteer, data.angular.z))
        
        self.lastMsg = rospy.Time.now()

    def publish(self):
        # 死锁检测
        if (rospy.Time.now() - self.lastMsg) > self.timeout:
            self.x = 0.0
            self.z = 0.0
            self.pub_rearL.publish(Float64(0))
            self.pub_rearR.publish(Float64(0))
            self.pub_steerL.publish(Float64(0))
            self.pub_steerR.publish(Float64(0))
            return

        # 直行处理
        if self.z == 0:
            self.pub_rearL.publish(Float64(self.x))
            self.pub_rearR.publish(Float64(self.x))
            self.pub_steerL.publish(Float64(0))
            self.pub_steerR.publish(Float64(0))
            return

        # 阿克曼转向计算
        try:
            # 计算理想转弯半径
            r = self.L / math.tan(abs(self.z))
            
            # 计算各轮胎转弯半径
            sign = 1 if self.z > 0 else -1
            rL_rear = r - sign * self.T_rear/2
            rR_rear = r + sign * self.T_rear/2
            rL_front = r - sign * self.T_front/2
            rR_front = r + sign * self.T_front/2

            # 后轮差速计算
            msgRearL = Float64()
            msgRearR = Float64()
            msgRearL.data = self.x * rL_rear / r
            msgRearR.data = self.x * rR_rear / r
            self.pub_rearL.publish(msgRearL)
            self.pub_rearR.publish(msgRearR)

            # 前轮转向角计算
            msgSteerL = Float64()
            msgSteerR = Float64()
            msgSteerL.data = math.atan(self.L / rL_front) * sign
            msgSteerR.data = math.atan(self.L / rR_front) * sign
            
            # 内侧轮胎机械限幅检查
            theta_inner = min(abs(msgSteerL.data), abs(msgSteerR.data))
            if theta_inner > 0.52333333:  # 机械限幅0.52333333 rad (约30°)
                rospy.logwarn(f"内侧轮胎转向角超过机械限制: {math.degrees(theta_inner):.1f}°")
                # 等比例缩小角度
                scale = 0.52333333 / theta_inner
                msgSteerL.data *= scale
                msgSteerR.data *= scale

            self.pub_steerL.publish(msgSteerL)
            self.pub_steerR.publish(msgSteerR)

        except ZeroDivisionError:
            rospy.logerr("零除错误：转向角计算异常")

if __name__ == '__main__':
    try:
        CmdVel2Gazebo()
    except rospy.ROSInterruptException:
        pass