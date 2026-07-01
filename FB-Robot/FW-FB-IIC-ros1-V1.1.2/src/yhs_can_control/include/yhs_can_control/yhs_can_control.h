#ifndef __CANCONTROL_NODE_H__
#define __CANCONTROL_NODE_H__

// ROS Core
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

// YHS CAN Messages (FB_FW_IIC)
#include "yhs_can_msgs/ctrl_cmd.h"
#include "yhs_can_msgs/front_free_ctrl_cmd.h"
#include "yhs_can_msgs/rear_free_ctrl_cmd.h"
#include "yhs_can_msgs/io_cmd.h"
#include "yhs_can_msgs/ctrl_fb.h"
#include "yhs_can_msgs/lf_wheel_fb.h"
#include "yhs_can_msgs/lr_wheel_fb.h"
#include "yhs_can_msgs/rr_wheel_fb.h"
#include "yhs_can_msgs/rf_wheel_fb.h"
#include "yhs_can_msgs/io_fb.h"
#include "yhs_can_msgs/bms_fb.h"
#include "yhs_can_msgs/bms_flag_fb.h"
#include "yhs_can_msgs/ultrasonic.h"
#include "yhs_can_msgs/error_fb.h" // 对应 Veh_Diag_fb
#include "yhs_can_msgs/drive_motor_current_fb.h"
#include "yhs_can_msgs/steering_motor_current_fb.h"
#include "yhs_can_msgs/front_angle_free_ctrl_cmd.h"
#include "yhs_can_msgs/rear_angle_free_ctrl_cmd.h"
#include "yhs_can_msgs/front_velocity_free_ctrl_cmd.h"
#include "yhs_can_msgs/rear_velocity_free_ctrl_cmd.h"
#include "yhs_can_msgs/motor_cmd.h"
#include "yhs_can_msgs/motor_fb.h"
#include "yhs_can_msgs/steering_ctrl_cmd.h"

// System & Network
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <string>
#include <sstream>
#include <vector>
#include <mutex>

// Linux CAN
#include <linux/can.h>
#include <linux/can/raw.h>

// TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

namespace yhs_tool {

// 1. 定义控制模式，防止指令冲突
  enum ControlMode {
    MODE_CTRL_CMD = 0,      // 整体运动控制 (默认)
    MODE_STEERING_CTRL,     // 转向独立控制
    MODE_FREE_CTRL_FRONT,   // 前轮自由控制
    MODE_FREE_CTRL_REAR     // 后轮自由控制
  };

class CanControl
{
public:
    CanControl();
    ~CanControl();
    
    void run();

private:
    ros::NodeHandle nh_;

    // Publishers
    ros::Publisher ctrl_fb_pub_;
    ros::Publisher lf_wheel_fb_pub_;
    ros::Publisher lr_wheel_fb_pub_;
    ros::Publisher rr_wheel_fb_pub_;
    ros::Publisher rf_wheel_fb_pub_;
    ros::Publisher io_fb_pub_;
    ros::Publisher bms_fb_pub_;
    ros::Publisher bms_flag_fb_pub_;
    ros::Publisher ultrasonic_pub_;
    ros::Publisher error_pub_;
    ros::Publisher drive_motor_current_fb_pub_;
    ros::Publisher steering_motor_current_fb_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher motor_fb_pub_;
    ros::Publisher steering_ctrl_fb_pub_;

    // Subscribers
    ros::Subscriber ctrl_cmd_sub_;
    ros::Subscriber io_cmd_sub_;
    ros::Subscriber motor_cmd_sub_;
    ros::Subscriber steering_ctrl_cmd_sub_;
    ros::Subscriber front_angle_free_ctrl_cmd_sub_;
    ros::Subscriber front_velocity_free_ctrl_cmd_sub_;
    ros::Subscriber rear_angle_free_ctrl_cmd_sub_;
    ros::Subscriber rear_velocity_free_ctrl_cmd_sub_;  
    ros::Subscriber imu_sub_;

    ros::Time last_imu_time_;
    ros::Time last_ctrl_cmd_time_;

    boost::mutex cmd_mutex_;
    std::mutex mutex_;

    // IMU Data
    double imu_roll_ = 0.0;
    double imu_pitch_ = 0.0;
    double imu_yaw_ = 0.0;

    int dev_handler_;
    can_frame send_frames_[2];
    can_frame recv_frames_[1];
    
    // 发送缓冲区
    unsigned char sendData_u_io_[8] = {0};
    unsigned char sendData_u_vel_[8] = {0};

    std::vector<int> ultrasonic_number_;
    std::string odomFrame_, baseFrame_;
    std::string if_name_;
    bool tfUsed_;

    bool motor_cmd_drive_enable_lf_;
    bool motor_cmd_drive_enable_lr_;
    bool motor_cmd_drive_enable_rf_;
    bool motor_cmd_drive_enable_rr_;
    bool motor_cmd_steering_enable_lf_;
    bool motor_cmd_steering_enable_lr_;
    bool motor_cmd_steering_enable_rf_;
    bool motor_cmd_steering_enable_rr_;
    bool motor_cmd_power_restart_;

    bool io_param_lamp_ctrl_;       // 灯控权限
    bool io_param_unlock_;          // 解锁
    bool io_param_low_power_enable_; // 低功耗使能
    bool io_param_lower_beam_;
    bool io_param_upper_beam_;
    int  io_param_turn_lamp_;
    bool io_param_braking_lamp_;
    bool io_param_clearance_lamp_;
    bool io_param_fog_lamp_;
    bool io_param_speaker_;
    int  io_param_low_power_ratio_; // 低功耗比例

    ControlMode current_control_mode_ = MODE_CTRL_CMD;

    // --- 指令缓存 ---
    yhs_can_msgs::io_cmd current_io_cmd_;
    yhs_can_msgs::ctrl_cmd current_ctrl_cmd_;
    yhs_can_msgs::motor_cmd current_motor_cmd_;
    yhs_can_msgs::steering_ctrl_cmd current_steering_ctrl_cmd_;
    yhs_can_msgs::front_angle_free_ctrl_cmd current_front_angle_cmd_;
    yhs_can_msgs::front_velocity_free_ctrl_cmd current_front_vel_cmd_;
    yhs_can_msgs::rear_angle_free_ctrl_cmd current_rear_angle_cmd_;
    yhs_can_msgs::rear_velocity_free_ctrl_cmd current_rear_vel_cmd_;

    // --- 定时器 ---
    ros::Timer timer_;

    // Callbacks
    void io_cmdCallBack(const yhs_can_msgs::io_cmd::ConstPtr& msg);
    void ctrl_cmdCallBack(const yhs_can_msgs::ctrl_cmd::ConstPtr& msg);
    
    // 独立控制回调
    void front_velocity_free_ctrl_cmdCallBack(const yhs_can_msgs::front_velocity_free_ctrl_cmd msg);
    void rear_velocity_free_ctrl_cmdCallBack(const yhs_can_msgs::rear_velocity_free_ctrl_cmd msg);
    void front_angle_free_ctrl_cmdCallBack(const yhs_can_msgs::front_angle_free_ctrl_cmd msg);
    void rear_angle_free_ctrl_cmdCallBack(const yhs_can_msgs::rear_angle_free_ctrl_cmd msg);
    void motor_cmdCallBack(const yhs_can_msgs::motor_cmd msg);
    void steering_ctrl_cmdCallBack(const yhs_can_msgs::steering_ctrl_cmd msg);   
    
    void ImuDataCallBack(const sensor_msgs::Imu::ConstPtr &imu_msg);
    
    // OdomPub 升级为支持全向移动 (x, y, angular)
    void OdomPub(const float linear_x, const float linear_y, const float angular_z);
    
    void recvData();

    // --- 发送函数 ---
    void sendIoCommand();
    void sendCtrlCommand(); 
    void sendMotorCmd();
    void sendSteeringCtrlCmd();
    void sendFrontFreeCtrlCmd(); // 合并发送前轮的角度和速度
    void sendRearFreeCtrlCmd();  // 合并发送后轮的角度和速度
    void SendAndKeepAlive(); // 封装单次发送逻辑
    void SendControlBurst(int repeat_count); // 连续发送N次
    void timerCallBack(const ros::TimerEvent& event);
};

}

#endif