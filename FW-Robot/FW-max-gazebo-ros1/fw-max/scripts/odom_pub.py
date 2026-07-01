#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from math import sin, cos, pi

class OdometryPublisher:
    def __init__(self):
        rospy.init_node('odom_publisher')
        
        # 初始化变量
        self.x = 0.0      # X坐标（米）
        self.y = 0.0      # Y坐标（米）
        self.th = 0.0     # 航向角（弧度）
        self.last_time = rospy.Time.now()
        
        # 创建发布器和TF广播器
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # 订阅/cmd_vel
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        
    def cmd_vel_callback(self, msg):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        
        # 从Twist消息获取速度
        vx = msg.linear.x   # 线速度（m/s）
        vy = msg.linear.y   # 横向速度（通常为0）
        vth = msg.angular.z # 角速度（rad/s）
        
        # 积分计算位置和航向
        delta_th = vth * dt
        delta_x = (vx * cos(self.th) - vy * sin(self.th)) * dt
        delta_y = (vx * sin(self.th) + vy * cos(self.th)) * dt
        
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        # 将航向角限制在[-pi, pi]
        self.th = (self.th + pi) % (2 * pi) - pi
        
        # 创建四元数
        quat = Quaternion()
        quat.x, quat.y, quat.z, quat.w = tf.transformations.quaternion_from_euler(0, 0, self.th)
        
        # 发布Odometry消息
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = quat
        
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        
        self.odom_pub.publish(odom)
        
        # 广播TF变换
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0),
            (quat.x, quat.y, quat.z, quat.w),
            current_time,
            "base_link",
            "odom"
        )
        
        self.last_time = current_time

if __name__ == '__main__':
    try:
        odom_publisher = OdometryPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass