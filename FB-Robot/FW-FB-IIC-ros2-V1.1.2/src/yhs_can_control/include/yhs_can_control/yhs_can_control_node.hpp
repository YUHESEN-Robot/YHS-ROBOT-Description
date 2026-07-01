#ifndef __YHS_CANCONTROL_NODE_H__
#define __YHS_CANCONTROL_NODE_H__

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <vector>
#include <algorithm>
#include <climits>
#include <cstring>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "yhs_can_interfaces/msg/io_cmd.hpp"
#include "yhs_can_interfaces/msg/motor_cmd.hpp"
#include "yhs_can_interfaces/msg/ctrl_cmd.hpp"
#include "yhs_can_interfaces/msg/front_angle_free_ctrl_cmd.hpp"
#include "yhs_can_interfaces/msg/front_velocity_free_ctrl_cmd.hpp"
#include "yhs_can_interfaces/msg/rear_angle_free_ctrl_cmd.hpp"
#include "yhs_can_interfaces/msg/rear_velocity_free_ctrl_cmd.hpp"
#include "yhs_can_interfaces/msg/steering_ctrl_cmd.hpp"
#include "yhs_can_interfaces/msg/steering_ctrl_fb.hpp"
#include "yhs_can_interfaces/msg/chassis_info_fb.hpp"

#define READ_PARAM(TYPE, NAME, VAR, VALUE)  \
  VAR = VALUE;                              \
  node->declare_parameter<TYPE>(NAME, VAR); \
  node->get_parameter(NAME, VAR);

namespace yhs
{
  class CanControl
  {

  public:
    CanControl(rclcpp::Node::SharedPtr);
    ~CanControl();

    bool run();
    void stop();

  private:
    rclcpp::Node::SharedPtr node_;

    std::string if_name_;
    int can_socket_;
    std::thread thread_;

    std::vector<int64_t> ultrasonic_number_;

    rclcpp::Subscription<yhs_can_interfaces::msg::IoCmd>::SharedPtr io_cmd_subscriber_;
    rclcpp::Subscription<yhs_can_interfaces::msg::MotorCmd>::SharedPtr motor_cmd_subscriber_;
    rclcpp::Subscription<yhs_can_interfaces::msg::CtrlCmd>::SharedPtr ctrl_cmd_subscriber_;
    rclcpp::Subscription<yhs_can_interfaces::msg::SteeringCtrlCmd>::SharedPtr steering_ctrl_cmd_subscriber_;
    rclcpp::Subscription<yhs_can_interfaces::msg::FrontAngleFreeCtrlCmd>::SharedPtr front_angle_free_ctrl_cmd_subscriber_;
    rclcpp::Subscription<yhs_can_interfaces::msg::FrontVelocityFreeCtrlCmd>::SharedPtr front_velocity_free_ctrl_cmd_subscriber_;
    rclcpp::Subscription<yhs_can_interfaces::msg::RearAngleFreeCtrlCmd>::SharedPtr rear_angle_free_ctrl_cmd_subscriber_;
    rclcpp::Subscription<yhs_can_interfaces::msg::RearVelocityFreeCtrlCmd>::SharedPtr rear_velocity_free_ctrl_cmd_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

    rclcpp::Publisher<yhs_can_interfaces::msg::ChassisInfoFb>::SharedPtr chassis_info_fb_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    rclcpp::TimerBase::SharedPtr keep_alive_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

    // odom 参数
    std::string odom_frame_;
    std::string base_frame_;
    bool tf_used_;

    // io_cmd 参数
    bool io_cmd_low_power_enable_;
    bool io_cmd_lamp_ctrl_;
    bool io_cmd_unlock_;
    bool io_cmd_lower_beam_headlamp_;
    bool io_cmd_upper_beam_headlamp_;
    bool io_cmd_braking_lamp_;
    bool io_cmd_clearance_lamp_;
    bool io_cmd_fog_lamp_;
    bool io_cmd_speaker_;
    int io_cmd_turn_lamp_;
    int io_cmd_low_power_ratio_;

    // motor_cmd 参数
    bool motor_cmd_drive_enable_lf_;
    bool motor_cmd_drive_enable_lr_;
    bool motor_cmd_drive_enable_rf_;
    bool motor_cmd_drive_enable_rr_;
    bool motor_cmd_steering_enable_lf_;
    bool motor_cmd_steering_enable_lr_;
    bool motor_cmd_steering_enable_rf_;
    bool motor_cmd_steering_enable_rr_;
    bool motor_cmd_power_restart_;

    // odom 解算变量
    std::mutex mutex_;
    double imu_yaw_;
    double imu_roll_;
    double imu_pitch_;
    double odom_x_;
    double odom_y_;
    double odom_yaw_;
    double fb_linear_vel_;
    double fb_angular_vel_;
    unsigned char current_gear_;
    double linear_scale_x_;
    double linear_scale_y_;
    double angular_scale_z_;

    // 电机脉冲odom相关
    std::vector<int> current_pulses_;
    std::vector<int> last_pulses_;
    std::vector<bool> updated_;
    std::vector<double> steering_angles_;
    bool all_initialized_;
    rclcpp::Time last_odom_time_;

    // 转向反馈存储
    yhs_can_interfaces::msg::SteeringCtrlFb last_steering_fb_;

    void io_cmd_callback(const yhs_can_interfaces::msg::IoCmd::SharedPtr io_cmd_msg);
    void io_cmd_send(const yhs_can_interfaces::msg::IoCmd & msg);

    void motor_cmd_callback(const yhs_can_interfaces::msg::MotorCmd::SharedPtr motor_cmd_msg);
    void motor_cmd_send(const yhs_can_interfaces::msg::MotorCmd & msg);

    void ctrl_cmd_callback(const yhs_can_interfaces::msg::CtrlCmd::SharedPtr ctrl_cmd_msg);

    void steering_ctrl_cmd_callback(const yhs_can_interfaces::msg::SteeringCtrlCmd::SharedPtr ctrl_cmd_msg);

    void front_angle_free_ctrl_cmd_callback(const yhs_can_interfaces::msg::FrontAngleFreeCtrlCmd::SharedPtr front_angle_free_ctrl_cmd_msg);
    void front_velocity_free_ctrl_cmd_callback(const yhs_can_interfaces::msg::FrontVelocityFreeCtrlCmd::SharedPtr front_velocity_free_ctrl_cmd_msg);
    void rear_angle_free_ctrl_cmd_callback(const yhs_can_interfaces::msg::RearAngleFreeCtrlCmd::SharedPtr rear_angle_free_ctrl_cmd_msg);
    void rear_velocity_free_ctrl_cmd_callback(const yhs_can_interfaces::msg::RearVelocityFreeCtrlCmd::SharedPtr rear_velocity_free_ctrl_cmd_msg);

    void imu_data_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);

    bool wait_for_can_frame();

    void can_data_recv_callback();

    void publish_odom(const double linear_vel, const double angular_vel, const unsigned char gear, const double slipangle);
    void update_revolutions();
    int calculate_pulse_diff(int current_pulse, int last_pulse);
    bool all_wheels_initialized();

    void timer_callback();
    void send_and_keep_alive();
    void send_control_burst(int repeat_count);
  };

}

#endif
