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
                '/fw_mid/front_right_steering_position_controller/command',
                '/fw_mid/front_left_steering_position_controller/command',
                '/fw_mid/rear_right_steering_position_controller/command',
                '/fw_mid/rear_left_steering_position_controller/command'
            ]
        ]
        
        self.drive_pubs = [
            rospy.Publisher(n, Float64, queue_size=1) for n in [
                '/fw_mid/front_left_velocity_controller/command',
                '/fw_mid/front_right_velocity_controller/command',
                '/fw_mid/rear_left_velocity_controller/command',
                '/fw_mid/rear_right_velocity_controller/command'
            ]
        ]

        # 车辆参数
        self.L = 0.4             # 轴距 (m)
        self.T = 0.42            # 轮距 (m)
        self.wheel_radius = 0.12  # 车轮半径 (m)
        self.base_max_steer = math.radians(45)  # 基础最大转向角
        
        # 控制参数
        self.steer_rate_limit = 1.0    # 转向速率限制 (rad/s)
        self.low_speed_threshold = 0.5 # 低速模式阈值 (m/s)
        self.low_speed_steer_boost = math.radians(10) # 低速额外转向
        
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
        
        # 计算实际转向限制
        self.current_speed = abs(linear_vel)
        effective_max_steer = self.base_max_steer
        if self.current_speed < self.low_speed_threshold:
            effective_max_steer += self.low_speed_steer_boost
        
        # 转向指令处理
        steer_cmd = max(-effective_max_steer, min(effective_max_steer, steer_cmd))
        
        # 运动学计算
        if abs(steer_cmd) > 1e-3:  # 转向模式
            self.handle_steering(linear_vel, steer_cmd)
        else:                      # 直行模式
            self.handle_straight(linear_vel)

    def handle_steering(self, linear_vel, steer_cmd):
        # 计算基础参数
        sign = 1 if steer_cmd > 0 else -1
        abs_steer = abs(steer_cmd)
        R = self.L / math.tan(abs_steer) if abs_steer > 1e-3 else float('inf')
        
        try:
            # 四轮转向角计算
            angles = [
                math.atan(self.L/(R - self.T/2)) * sign,   # FL
                math.atan(self.L/(R + self.T/2)) * sign,   # FR
                -math.atan(self.L/(R - self.T/2)) * sign,  # RL
                -math.atan(self.L/(R + self.T/2)) * sign   # RR
            ]
            
            # 应用速率限制
            angles = self.apply_steer_rate_limit(angles)
            
            # 转向角限幅
            angles = [max(-self.base_max_steer, min(a, self.base_max_steer)) for a in angles]
            
            # 轮速计算
            if R != 0:
                speeds = [
                    linear_vel * (R - self.T/2 * sign) / R,
                    linear_vel * (R + self.T/2 * sign) / R,
                    linear_vel * (R - self.T/2 * sign) / R,
                    linear_vel * (R + self.T/2 * sign) / R
                ]
            else:  # 原地转向
                speeds = [linear_vel * sign] * 4
            
            # 转换到角速度
            wheel_speeds = [s / self.wheel_radius for s in speeds]
            
            # 发布指令
            self.publish_commands(angles, wheel_speeds)
            
        except ZeroDivisionError:
            rospy.logwarn("转向计算异常，紧急停止")
            self._stop_vehicle()

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