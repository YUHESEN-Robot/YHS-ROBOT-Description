#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math

class CmdVel2Gazebo:
    def __init__(self):
        rospy.init_node('cmdvel2gazebo_4ws4wd', anonymous=True)
        
        # 订阅者
        rospy.Subscriber('/cmd_vel', Twist, self.callback, queue_size=1)
        
        # 四轮转向和驱动发布器
        self.steer_pubs = [
            rospy.Publisher(n, Float64, queue_size=1) for n in [
                '/fw_mini/front_right_steering_position_controller/command',
                '/fw_mini/front_left_steering_position_controller/command',
                '/fw_mini/rear_right_steering_position_controller/command',
                '/fw_mini/rear_left_steering_position_controller/command'
            ]
        ]
        
        self.drive_pubs = [
            rospy.Publisher(n, Float64, queue_size=1) for n in [
                '/fw_mini/front_left_velocity_controller/command',
                '/fw_mini/front_right_velocity_controller/command',
                '/fw_mini/rear_left_velocity_controller/command',
                '/fw_mini/rear_right_velocity_controller/command'
            ]
        ]

        # 车辆参数
        self.L = 0.35             # 轴距 (m)
        self.T = 0.25            # 轮距 (m)
        self.wheel_radius = 0.07  # 车轮半径 (m)
        self.base_max_steer = math.radians(45)  # 最大转向角
        self.gear = rospy.get_param('~gear', 6)

        if self.gear == 7 :
            self.base_max_steer = math.radians(90)  # 最大转向角
        else :
            self.base_max_steer = math.radians(45)
        
        # 控制参数
        self.steer_rate_limit = 1.0    # 转向速率限制 (rad/s)
        self.low_speed_threshold = 0.5 # 低速阈值 (m/s)
        self.low_speed_steer_boost = math.radians(10) # 低速转向补偿

        
        # 状态变量
        self.last_time = rospy.Time.now()
        self.last_steer = [0.0] * 4
        self.cmd_vel = (0.0, 0.0)      # (linear.x, angular.z)
        self.current_speed = 0.0
        
        # 安全参数
        self.timeout = rospy.Duration(0.2)
        
        # 主循环
        self.control_loop()

    def callback(self, msg):
        self.cmd_vel = (msg.linear.x, msg.angular.z)
        self.last_time = rospy.Time.now()

    def control_loop(self):
        rate = rospy.Rate(30)  # 30Hz控制频率
        while not rospy.is_shutdown():
            self.process_control()
            rate.sleep()

    def process_control(self):
        # 超时检测
        if (rospy.Time.now() - self.last_time) > self.timeout:
            self._stop_vehicle()
            return

        # 解析指令
        linear_vel = self.cmd_vel[0]
        steer_cmd = self.cmd_vel[1]

        max_speed = 1.5  # 最大允许速度 (m/s)
        if linear_vel > max_speed :
            linear_vel = max_speed
        if linear_vel < -max_speed :
            linear_vel = -max_speed
        # 动态转向限制
        self.current_speed = abs(linear_vel)
        effective_max_steer = self.base_max_steer
        if self.current_speed < self.low_speed_threshold:
            effective_max_steer += self.low_speed_steer_boost
        
        # 转向指令限幅
        steer_cmd = max(-effective_max_steer, min(effective_max_steer, steer_cmd))
        
        # 运动学计算
        if abs(steer_cmd) > 1e-3:
            self.handle_steering(linear_vel, steer_cmd)
        else:
            self.handle_straight(linear_vel)

    def handle_steering(self, linear_vel, steer_cmd):
        sign = 1 if steer_cmd > 0 else -1
        abs_steer = abs(steer_cmd)
        
        #横移-档
        if self.gear == 7:
        # 原地旋转特殊处理
            try:
                angles = [
                    steer_cmd,   # FL
                    steer_cmd,    # FR
                    steer_cmd,  # RL反向
                    steer_cmd    # RR反向
                ]
                speeds = [
                        linear_vel,
                        linear_vel,
                        linear_vel,
                        linear_vel
                    ]
            except ZeroDivisionError:
                rospy.logwarn("转向计算异常，紧急停止")
                self._stop_vehicle()
                return
            wheel_speeds = [s / self.wheel_radius for s in speeds]
            self.publish_commands(angles, wheel_speeds)
        


        #4T4D-档
        if self.gear == 6:
        # 原地旋转特殊处理
         if linear_vel == 0.0 :
         
           # R = self.L / math.tan(abs_steer) if abs_steer > 1e-3 else float('inf')
            try:
              if steer_cmd < 0.0:
                angles = [
                    0.7845341782147769,   # FL
                    -0.7845341782147769,    # FR
                    -0.7845341782147769,  # RL反向
                    0.7845341782147769    # RR反向
                ]
                speeds = [
                        0.5,
                        -0.5,
                        0.5,
                        -0.5
                    ]
              if steer_cmd > 0.0:
                angles = [
                    0.7845341782147769,   # FL
                    -0.7845341782147769,    # FR
                    -0.7845341782147769,  # RL反向
                    0.7845341782147769    # RR反向
                ]
                speeds = [
                        -0.5,
                        0.5,
                        -0.5,
                        0.5
                    ]
            except ZeroDivisionError:
                rospy.logwarn("转向计算异常，紧急停止")
                self._stop_vehicle()
                return
            wheel_speeds = [s / self.wheel_radius for s in speeds]#ω=v/r rad/s
            self.publish_commands(angles, wheel_speeds)
        
         else:
            # 正常阿克曼转向计算
            R = self.L / math.tan(abs_steer) if abs_steer > 1e-3 else float('inf')
            try:
                angles = [
                    math.atan(self.L/(R - self.T/2)) * sign,   # FL
                    math.atan(self.L/(R + self.T/2)) * sign,    # FR
                    -math.atan(self.L/(R - self.T/2)) * sign,  # RL反向
                    -math.atan(self.L/(R + self.T/2)) * sign    # RR反向
                ]
                
                # 轮速计算
                if R != 0:
                    speeds = [
                        linear_vel * (R - self.T/2 * sign) / R,
                        linear_vel * (R + self.T/2 * sign) / R,
                        linear_vel * (R - self.T/2 * sign) / R,
                        linear_vel * (R + self.T/2 * sign) / R
                    ]
                else:
                    speeds = [0.0] * 4

            except ZeroDivisionError:
                rospy.logwarn("转向计算异常，紧急停止")
                self._stop_vehicle()
                return
        
        # 转向速率限制和限幅
            angles = self.apply_steer_rate_limit(angles)
            angles = [max(-self.base_max_steer, min(a, self.base_max_steer)) for a in angles]
        
        # 转换到角速度
            wheel_speeds = [s / self.wheel_radius for s in speeds]
        
        # 发布指令
            self.publish_commands(angles, wheel_speeds)

    def handle_straight(self, linear_vel):
        wheel_speed = linear_vel / self.wheel_radius
        self.publish_commands([0.0]*4, [wheel_speed]*4)

    def apply_steer_rate_limit(self, target_angles):
        dt = (rospy.Time.now() - self.last_time).to_sec()
        max_delta = self.steer_rate_limit * dt
        
        limited_angles = []
        for i in range(4):
            delta = target_angles[i] - self.last_steer[i]
            delta = max(-max_delta, min(delta, max_delta))
            limited_angles.append(self.last_steer[i] + delta)
        
        self.last_steer = limited_angles.copy()
        return limited_angles

    def publish_commands(self, angles, speeds):
        for pub, angle in zip(self.steer_pubs, angles):
            pub.publish(Float64(angle))
        for pub, speed in zip(self.drive_pubs, speeds):
            pub.publish(Float64(speed))

    def _stop_vehicle(self):
        self.publish_commands([0.0]*4, [0.0]*4)

if __name__ == '__main__':
    try:
        CmdVel2Gazebo()
    except rospy.ROSInterruptException:
        pass