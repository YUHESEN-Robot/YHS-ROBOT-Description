#include "yhs_can_control/yhs_can_control_node.hpp"

namespace yhs
{

  CanControl::CanControl(rclcpp::Node::SharedPtr node)
      : node_(node), if_name_("can0"), can_socket_(-1),
        imu_yaw_(88.0), imu_roll_(0.0), imu_pitch_(0.0),
        odom_x_(0.0), odom_y_(0.0), odom_yaw_(0.0),
        fb_linear_vel_(0.0), fb_angular_vel_(0.0), current_gear_(6),
        all_initialized_(false)
  {

    READ_PARAM(std::string, "can_name", (if_name_), "can0");

    node_->declare_parameter<std::vector<int64_t>>("ultrasonic_number", std::vector<int64_t>{});
    node_->get_parameter("ultrasonic_number", ultrasonic_number_);

    // odom 参数
    READ_PARAM(std::string, "odom_frame", odom_frame_, "odom");
    READ_PARAM(std::string, "base_link_frame", base_frame_, "base_link");
    READ_PARAM(bool, "tf_used", tf_used_, false);
    READ_PARAM(double, "linear_scale_x", linear_scale_x_, 1.0);
    READ_PARAM(double, "linear_scale_y", linear_scale_y_, 1.0);
    READ_PARAM(double, "angular_scale_z", angular_scale_z_, 1.0);

    // io_cmd 参数
    READ_PARAM(bool, "io_cmd.io_cmd_low_power_enable", io_cmd_low_power_enable_, true);
    READ_PARAM(bool, "io_cmd.io_cmd_lamp_ctrl", io_cmd_lamp_ctrl_, false);
    READ_PARAM(bool, "io_cmd.io_cmd_unlock", io_cmd_unlock_, false);
    READ_PARAM(bool, "io_cmd.io_cmd_lower_beam_headlamp", io_cmd_lower_beam_headlamp_, true);
    READ_PARAM(bool, "io_cmd.io_cmd_upper_beam_headlamp", io_cmd_upper_beam_headlamp_, false);
    READ_PARAM(int, "io_cmd.io_cmd_turn_lamp", io_cmd_turn_lamp_, 0);
    READ_PARAM(bool, "io_cmd.io_cmd_braking_lamp", io_cmd_braking_lamp_, true);
    READ_PARAM(bool, "io_cmd.io_cmd_clearance_lamp", io_cmd_clearance_lamp_, false);
    READ_PARAM(bool, "io_cmd.io_cmd_fog_lamp", io_cmd_fog_lamp_, false);
    READ_PARAM(bool, "io_cmd.io_cmd_speaker", io_cmd_speaker_, false);
    READ_PARAM(int, "io_cmd.io_cmd_low_power_ratio", io_cmd_low_power_ratio_, 0);

    // motor_cmd 参数
    READ_PARAM(bool, "motor_cmd.motor_cmd_drive_enable_lf", motor_cmd_drive_enable_lf_, true);
    READ_PARAM(bool, "motor_cmd.motor_cmd_drive_enable_lr", motor_cmd_drive_enable_lr_, true);
    READ_PARAM(bool, "motor_cmd.motor_cmd_drive_enable_rf", motor_cmd_drive_enable_rf_, true);
    READ_PARAM(bool, "motor_cmd.motor_cmd_drive_enable_rr", motor_cmd_drive_enable_rr_, true);
    READ_PARAM(bool, "motor_cmd.motor_cmd_steering_enable_lf", motor_cmd_steering_enable_lf_, true);
    READ_PARAM(bool, "motor_cmd.motor_cmd_steering_enable_lr", motor_cmd_steering_enable_lr_, true);
    READ_PARAM(bool, "motor_cmd.motor_cmd_steering_enable_rf", motor_cmd_steering_enable_rf_, true);
    READ_PARAM(bool, "motor_cmd.motor_cmd_steering_enable_rr", motor_cmd_steering_enable_rr_, true);
    READ_PARAM(bool, "motor_cmd.motor_cmd_power_restart", motor_cmd_power_restart_, false);

    // 初始化电机脉冲odom变量
    current_pulses_.resize(4, 0);
    last_pulses_.resize(4, -1);
    steering_angles_.resize(4, 0.0);
    updated_.resize(4, false);
    last_odom_time_ = node_->now();
    last_steering_fb_.steering_ctrl_fb_steering = 0.0;
    last_steering_fb_.steering_ctrl_fb_velocity = 0.0;

    io_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::IoCmd>(
        "io_cmd",
        1,
        std::bind(&CanControl::io_cmd_callback, this, std::placeholders::_1));

    motor_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::MotorCmd>(
        "motor_cmd",
        1,
        std::bind(&CanControl::motor_cmd_callback, this, std::placeholders::_1));

    ctrl_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::CtrlCmd>(
        "ctrl_cmd",
        1,
        std::bind(&CanControl::ctrl_cmd_callback, this, std::placeholders::_1));

    steering_ctrl_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::SteeringCtrlCmd>(
        "steering_ctrl_cmd",
        1,
        std::bind(&CanControl::steering_ctrl_cmd_callback, this, std::placeholders::_1));

    front_velocity_free_ctrl_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::FrontVelocityFreeCtrlCmd>(
        "front_velocity_free_ctrl_cmd",
        1,
        std::bind(&CanControl::front_velocity_free_ctrl_cmd_callback, this, std::placeholders::_1));     

    rear_velocity_free_ctrl_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::RearVelocityFreeCtrlCmd>(
        "rear_velocity_free_ctrl_cmd",
        1,
        std::bind(&CanControl::rear_velocity_free_ctrl_cmd_callback, this, std::placeholders::_1));   

    front_angle_free_ctrl_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::FrontAngleFreeCtrlCmd>(
        "front_angle_free_ctrl_cmd",
        1,
        std::bind(&CanControl::front_angle_free_ctrl_cmd_callback, this, std::placeholders::_1));   
        
    rear_angle_free_ctrl_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::RearAngleFreeCtrlCmd>(
        "rear_angle_free_ctrl_cmd",
        1,
        std::bind(&CanControl::rear_angle_free_ctrl_cmd_callback, this, std::placeholders::_1));           

    imu_subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "imu_data",
        5,
        std::bind(&CanControl::imu_data_callback, this, std::placeholders::_1));

    chassis_info_fb_publisher_ = node_->create_publisher<yhs_can_interfaces::msg::ChassisInfoFb>("chassis_info_fb", 1);

    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 5);

    odom_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  }

  void CanControl::io_cmd_callback(const yhs_can_interfaces::msg::IoCmd::SharedPtr io_cmd_msg)
  {
    io_cmd_send(*io_cmd_msg);
  }

  void CanControl::io_cmd_send(const yhs_can_interfaces::msg::IoCmd & msg)
  {
    static unsigned char count = 0;

    unsigned char sendDataTemp[8] = {0};

    if (msg.io_cmd_lamp_ctrl)
      sendDataTemp[0] |= 0x01;
    if (msg.io_cmd_unlock)
      sendDataTemp[0] |= 0x02;

    if (msg.io_cmd_low_power_enable)
      sendDataTemp[0] |= 0x04;

    if (msg.io_cmd_lower_beam_headlamp)
      sendDataTemp[1] |= 0x01;
    if (msg.io_cmd_upper_beam_headlamp)
      sendDataTemp[1] |= 0x02;

    if (msg.io_cmd_turn_lamp == 0)
      sendDataTemp[1] |= 0x00;
    if (msg.io_cmd_turn_lamp == 1)
      sendDataTemp[1] |= 0x04;
    if (msg.io_cmd_turn_lamp == 2)
      sendDataTemp[1] |= 0x08;
    if (msg.io_cmd_turn_lamp == 3)
      sendDataTemp[1] |= 0x0C;

    if (msg.io_cmd_braking_lamp)
      sendDataTemp[1] |= 0x10;
    if (msg.io_cmd_clearance_lamp)
      sendDataTemp[1] |= 0x20;
    if (msg.io_cmd_fog_lamp)
      sendDataTemp[1] |= 0x40;
    if (msg.io_cmd_speaker)
      sendDataTemp[1] |= 0x80;

    sendDataTemp[2] = msg.io_cmd_low_power_ratio;

    count++;
    if (count == 16)
      count = 0;

    sendDataTemp[6] = count << 4;

    sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

    can_frame send_frame;

    send_frame.can_id = 0x18C4D7D0 | CAN_EFF_FLAG;
    send_frame.can_dlc = 8;

    memcpy(send_frame.data, sendDataTemp, 8);

    int ret = write(can_socket_, &send_frame, sizeof(send_frame));
    if (ret <= 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to send message: " << strerror(errno));
    }
  }

  void CanControl::motor_cmd_callback(const yhs_can_interfaces::msg::MotorCmd::SharedPtr motor_cmd_msg)
  {
    motor_cmd_send(*motor_cmd_msg);
  }

  void CanControl::motor_cmd_send(const yhs_can_interfaces::msg::MotorCmd & msg)
  {
    static unsigned char count = 0;

    unsigned char sendDataTemp[8] = {0};

    if (msg.motor_cmd_drive_enable_lf)
      sendDataTemp[0] |= 0x01;
    if (msg.motor_cmd_drive_enable_lr)
      sendDataTemp[0] |= 0x02;
    if (msg.motor_cmd_drive_enable_rf)
      sendDataTemp[0] |= 0x04;
    if (msg.motor_cmd_drive_enable_rr)
      sendDataTemp[0] |= 0x08;

    if (msg.motor_cmd_steering_enable_lf)
      sendDataTemp[0] |= 0x10;
    if (msg.motor_cmd_steering_enable_lr)
      sendDataTemp[0] |= 0x20;
    if (msg.motor_cmd_steering_enable_rf)
      sendDataTemp[0] |= 0x40;
    if (msg.motor_cmd_steering_enable_rr)
      sendDataTemp[0] |= 0x80;

    if (msg.motor_cmd_power_restart)
      sendDataTemp[1] |= 0x01;

    count++;
    if (count == 16)
      count = 0;

    sendDataTemp[6] = count << 4;

    sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

    can_frame send_frame;

    send_frame.can_id = 0x18C4D8D0 | CAN_EFF_FLAG;
    send_frame.can_dlc = 8;

    memcpy(send_frame.data, sendDataTemp, 8);

    int ret = write(can_socket_, &send_frame, sizeof(send_frame));
    if (ret <= 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to send message: " << strerror(errno));
    }
  }

  void CanControl::ctrl_cmd_callback(const yhs_can_interfaces::msg::CtrlCmd::SharedPtr ctrl_cmd_msg)
  {
    yhs_can_interfaces::msg::CtrlCmd msg = *ctrl_cmd_msg;
    const short ctrl_cmd_x_linear = msg.ctrl_cmd_x_linear * 1000;
    const short ctrl_cmd_z_angular = msg.ctrl_cmd_z_angular * 100;
    const short ctrl_cmd_y_linear = msg.ctrl_cmd_y_linear * 1000;
    const unsigned char gear = msg.ctrl_cmd_gear;

    static unsigned char count = 0;
    unsigned char sendDataTemp[8] = {0};

    sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);

    sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((ctrl_cmd_x_linear & 0x0f) << 4));

    sendDataTemp[1] = (ctrl_cmd_x_linear >> 4) & 0xff;

    sendDataTemp[2] = sendDataTemp[2] | (0x0f & (ctrl_cmd_x_linear >> 12));

    sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((ctrl_cmd_z_angular & 0x0f) << 4));

    sendDataTemp[3] = (ctrl_cmd_z_angular >> 4) & 0xff;

    sendDataTemp[4] = sendDataTemp[4] | (0x0f & (ctrl_cmd_z_angular >> 12));

    sendDataTemp[4] = sendDataTemp[4] | (0xf0 & ((ctrl_cmd_y_linear & 0x0f) << 4));

    sendDataTemp[5] = (ctrl_cmd_y_linear >> 4) & 0xff;

    sendDataTemp[6] = sendDataTemp[6] | (0x0f & (ctrl_cmd_y_linear >> 12));

    count++;

    if (count == 16)
      count = 0;

    sendDataTemp[6] = sendDataTemp[6] | (count << 4);

    sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

    can_frame send_frame;

    send_frame.can_id = 0x18C4D1D0 | CAN_EFF_FLAG;
    send_frame.can_dlc = 8;

    memcpy(send_frame.data, sendDataTemp, 8);

    int ret = write(can_socket_, &send_frame, sizeof(send_frame));
    if (ret <= 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to send message: " << strerror(errno));
    }
  }

  void CanControl::steering_ctrl_cmd_callback(const yhs_can_interfaces::msg::SteeringCtrlCmd::SharedPtr steering_ctrl_cmd_msg)
  {
    yhs_can_interfaces::msg::SteeringCtrlCmd msg = *steering_ctrl_cmd_msg;
    const short steering_ctrl_cmd_velocity = msg.steering_ctrl_cmd_velocity * 1000;
    const short steering_ctrl_cmd_steering = msg.steering_ctrl_cmd_steering * 100;
    const unsigned char gear = msg.ctrl_cmd_gear;

    static unsigned char count = 0;
    unsigned char sendDataTemp[8] = {0};

    sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);

    sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((steering_ctrl_cmd_velocity & 0x0f) << 4));

    sendDataTemp[1] = (steering_ctrl_cmd_velocity >> 4) & 0xff;

    sendDataTemp[2] = sendDataTemp[2] | (0x0f & (steering_ctrl_cmd_velocity >> 12));

    sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((steering_ctrl_cmd_steering & 0x0f) << 4));

    sendDataTemp[3] = (steering_ctrl_cmd_steering >> 4) & 0xff;

    sendDataTemp[4] = sendDataTemp[4] | (0x0f & (steering_ctrl_cmd_steering >> 12));

    count++;
    if (count == 16)
      count = 0;

    sendDataTemp[6] = count << 4;

    sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

    can_frame send_frame;

    send_frame.can_id = 0x18C4D2D0 | CAN_EFF_FLAG;
    send_frame.can_dlc = 8;

    memcpy(send_frame.data, sendDataTemp, 8);

    int ret = write(can_socket_, &send_frame, sizeof(send_frame));
    if (ret <= 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to send message: " << strerror(errno));
    }
  }

  void CanControl::front_velocity_free_ctrl_cmd_callback(const yhs_can_interfaces::msg::FrontVelocityFreeCtrlCmd::SharedPtr front_velocity_free_ctrl_cmd_msg)
  {
    yhs_can_interfaces::msg::FrontVelocityFreeCtrlCmd msg = *front_velocity_free_ctrl_cmd_msg;
    const short velocity_lf = msg.free_ctrl_cmd_velocity_lf * 1000;
    const short velocity_rf = msg.free_ctrl_cmd_velocity_rf * 1000;
    const unsigned char gear = msg.ctrl_cmd_gear;

    static unsigned char count = 0;
    unsigned char sendDataTemp[8] = {0};

    sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);

    sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((velocity_lf & 0x0f) << 4));

    sendDataTemp[1] = (velocity_lf >> 4) & 0xff;

    sendDataTemp[2] = sendDataTemp[2] | (0x0f & (velocity_lf >> 12));

    sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((velocity_rf & 0x0f) << 4));

    sendDataTemp[3] = (velocity_rf >> 4) & 0xff;

    sendDataTemp[4] = sendDataTemp[4] | (0x0f & (velocity_rf >> 12));

    count++;
    if (count == 16)
      count = 0;

    sendDataTemp[6] = count << 4;

    sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

    can_frame send_frame;

    send_frame.can_id = 0x18C4D3D0 | CAN_EFF_FLAG;
    send_frame.can_dlc = 8;

    memcpy(send_frame.data, sendDataTemp, 8);

    int ret = write(can_socket_, &send_frame, sizeof(send_frame));
    if (ret <= 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to send message: " << strerror(errno));
    }
  }

  void CanControl::rear_velocity_free_ctrl_cmd_callback(const yhs_can_interfaces::msg::RearVelocityFreeCtrlCmd::SharedPtr rear_velocity_free_ctrl_cmd_msg)
  {
    yhs_can_interfaces::msg::RearVelocityFreeCtrlCmd msg = *rear_velocity_free_ctrl_cmd_msg;
    const short velocity_lr = msg.free_ctrl_cmd_velocity_lr * 1000;
    const short velocity_rr = msg.free_ctrl_cmd_velocity_rr * 1000;
    const unsigned char gear = msg.ctrl_cmd_gear;

    static unsigned char count = 0;
    unsigned char sendDataTemp[8] = {0};

    sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);

    sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((velocity_lr & 0x0f) << 4));

    sendDataTemp[1] = (velocity_lr >> 4) & 0xff;

    sendDataTemp[2] = sendDataTemp[2] | (0x0f & (velocity_lr >> 12));

    sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((velocity_rr & 0x0f) << 4));

    sendDataTemp[3] = (velocity_rr >> 4) & 0xff;

    sendDataTemp[4] = sendDataTemp[4] | (0x0f & (velocity_rr >> 12));

    count++;
    if (count == 16)
      count = 0;

    sendDataTemp[6] = count << 4;

    sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

    can_frame send_frame;

    send_frame.can_id = 0x18C4D4D0 | CAN_EFF_FLAG;
    send_frame.can_dlc = 8;

    memcpy(send_frame.data, sendDataTemp, 8);

    int ret = write(can_socket_, &send_frame, sizeof(send_frame));
    if (ret <= 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to send message: " << strerror(errno));
    }
  }

  void CanControl::front_angle_free_ctrl_cmd_callback(const yhs_can_interfaces::msg::FrontAngleFreeCtrlCmd::SharedPtr front_angle_free_ctrl_cmd_msg)
  {
    yhs_can_interfaces::msg::FrontAngleFreeCtrlCmd msg = *front_angle_free_ctrl_cmd_msg;
    const short angle_lf = msg.free_ctrl_cmd_angle_lf * 100;
    const short angle_rf = msg.free_ctrl_cmd_angle_rf * 100;
    const unsigned char gear = msg.ctrl_cmd_gear;

    static unsigned char count = 0;
    unsigned char sendDataTemp[8] = {0};

    sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);

    sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((angle_lf & 0x0f) << 4));

    sendDataTemp[1] = (angle_lf >> 4) & 0xff;

    sendDataTemp[2] = sendDataTemp[2] | (0x0f & (angle_lf >> 12));

    sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((angle_rf & 0x0f) << 4));

    sendDataTemp[3] = (angle_rf >> 4) & 0xff;

    sendDataTemp[4] = sendDataTemp[4] | (0x0f & (angle_rf >> 12));

    count++;
    if (count == 16)
      count = 0;

    sendDataTemp[6] = count << 4;

    sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

    can_frame send_frame;

    send_frame.can_id = 0x18C4D5D0 | CAN_EFF_FLAG;
    send_frame.can_dlc = 8;

    memcpy(send_frame.data, sendDataTemp, 8);

    int ret = write(can_socket_, &send_frame, sizeof(send_frame));
    if (ret <= 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to send message: " << strerror(errno));
    }
  }

  void CanControl::rear_angle_free_ctrl_cmd_callback(const yhs_can_interfaces::msg::RearAngleFreeCtrlCmd::SharedPtr rear_angle_free_ctrl_cmd_msg)
  {
    yhs_can_interfaces::msg::RearAngleFreeCtrlCmd msg = *rear_angle_free_ctrl_cmd_msg;
    const short angle_lr = msg.free_ctrl_cmd_angle_lr * 100;
    const short angle_rr = msg.free_ctrl_cmd_angle_rr * 100;
    const unsigned char gear = msg.ctrl_cmd_gear;

    static unsigned char count = 0;
    unsigned char sendDataTemp[8] = {0};

    sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);

    sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((angle_lr & 0x0f) << 4));

    sendDataTemp[1] = (angle_lr >> 4) & 0xff;

    sendDataTemp[2] = sendDataTemp[2] | (0x0f & (angle_lr >> 12));

    sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((angle_rr & 0x0f) << 4));

    sendDataTemp[3] = (angle_rr >> 4) & 0xff;

    sendDataTemp[4] = sendDataTemp[4] | (0x0f & (angle_rr >> 12));

    count++;
    if (count == 16)
      count = 0;

    sendDataTemp[6] = count << 4;

    sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

    can_frame send_frame;

    send_frame.can_id = 0x18C4D6D0 | CAN_EFF_FLAG;
    send_frame.can_dlc = 8;

    memcpy(send_frame.data, sendDataTemp, 8);

    int ret = write(can_socket_, &send_frame, sizeof(send_frame));
    if (ret <= 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to send message: " << strerror(errno));
    }
  }

  bool CanControl::wait_for_can_frame()
  {
    struct timeval tv;
    fd_set rdfs;
    FD_ZERO(&rdfs);
    FD_SET(can_socket_, &rdfs);
    tv.tv_sec = 0;
    tv.tv_usec = 500000; // 10ms

    int ret = select(can_socket_ + 1, &rdfs, NULL, NULL, &tv);
    if (ret == -1)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Error waiting for CAN frame: " << std::strerror(errno));
      return false;
    }
    else if (ret == 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Timeout waiting for CAN frame! Please check whether the can0 setting is correct,\
whether the can line is connected correctly, and whether the chassis is powered on.");
      return false;
    }
    else
    {
      return true;
    }
    return false;
  }

  void CanControl::can_data_recv_callback()
  {
    can_frame recv_frame;
    yhs_can_interfaces::msg::ChassisInfoFb chassis_info_msg;

    while (rclcpp::ok())
    {
      if (!wait_for_can_frame())
        continue;

      if (read(can_socket_, &recv_frame, sizeof(recv_frame)) >= 0)
      {
        switch (recv_frame.can_id)
        {
        case 0x18C4D1EF | CAN_EFF_FLAG:
        {
          yhs_can_interfaces::msg::CtrlFb msg;
          msg.ctrl_fb_gear = 0x0f & recv_frame.data[0];

          msg.ctrl_fb_x_linear = static_cast<float>(static_cast<short>((recv_frame.data[2] & 0x0f) << 12 | recv_frame.data[1] << 4 | (recv_frame.data[0] & 0xf0) >> 4)) / 1000;

          msg.ctrl_fb_z_angular = static_cast<float>(static_cast<short>((recv_frame.data[4] & 0x0f) << 12 | recv_frame.data[3] << 4 | (recv_frame.data[2] & 0xf0) >> 4)) / 100;

          msg.ctrl_fb_y_linear = static_cast<float>(static_cast<short>((recv_frame.data[6] & 0x0f) << 12 | recv_frame.data[5] << 4 | (recv_frame.data[4] & 0xf0) >> 4)) / 100;

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if (crc == recv_frame.data[7])
          {
            chassis_info_msg.header.stamp = node_->get_clock()->now();
            chassis_info_msg.ctrl_fb = msg;
            chassis_info_fb_publisher_->publish(chassis_info_msg);

            fb_angular_vel_ = msg.ctrl_fb_z_angular * M_PI / 180.0;
            double current_slip_angle = 0.0;
            double velocity_magnitude = 0.0;

            {
              std::lock_guard<std::mutex> lock(mutex_);
              current_slip_angle = last_steering_fb_.steering_ctrl_fb_steering * M_PI / 180.0;
              velocity_magnitude = last_steering_fb_.steering_ctrl_fb_velocity;
            }

            double calc_linear_vel = 0.0;
            if (msg.ctrl_fb_gear == 6)
            {
              calc_linear_vel = msg.ctrl_fb_x_linear;
              fb_linear_vel_ = msg.ctrl_fb_x_linear;
            }
            else if (msg.ctrl_fb_gear == 7 || msg.ctrl_fb_gear == 8)
            {
              calc_linear_vel = velocity_magnitude;
              fb_linear_vel_ = velocity_magnitude * cos(current_slip_angle);
            }
            else
            {
              calc_linear_vel = msg.ctrl_fb_x_linear;
              fb_linear_vel_ = msg.ctrl_fb_x_linear;
            }

              publish_odom(calc_linear_vel, fb_angular_vel_, msg.ctrl_fb_gear, current_slip_angle);
          }

          break;
        }

        case 0x18C4D2EF | CAN_EFF_FLAG:
        {
          yhs_can_interfaces::msg::SteeringCtrlFb msg;
          msg.steering_ctrl_fb_gear = 0x0f & recv_frame.data[0];

          msg.steering_ctrl_fb_velocity = static_cast<float>(static_cast<short>((recv_frame.data[2] & 0x0f) << 12 | recv_frame.data[1] << 4 | (recv_frame.data[0] & 0xf0) >> 4)) / 1000;

          msg.steering_ctrl_fb_steering = static_cast<float>(static_cast<short>((recv_frame.data[4] & 0x0f) << 12 | recv_frame.data[3] << 4 | (recv_frame.data[2] & 0xf0) >> 4)) / 100;

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if (crc == recv_frame.data[7])
          {
            std::lock_guard<std::mutex> lock(mutex_);
            last_steering_fb_ = msg;
            chassis_info_msg.steering_ctrl_fb = msg;
          }

          break;
        }

        case 0x18C4D6EF | CAN_EFF_FLAG:
        {
          yhs_can_interfaces::msg::LfWheelFb msg;
          msg.lf_wheel_fb_velocity = static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000;

          msg.lf_wheel_fb_pulse = static_cast<int>(recv_frame.data[5] << 24 | recv_frame.data[4] << 16 | recv_frame.data[3] << 8 | recv_frame.data[2]);

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if (crc == recv_frame.data[7])
          {
            chassis_info_msg.lf_wheel_fb = msg;

            current_pulses_[0] = msg.lf_wheel_fb_pulse;
updated_[0] = true;
          }

          break;
        }

        case 0x18C4D7EF | CAN_EFF_FLAG:
        {
          yhs_can_interfaces::msg::LrWheelFb msg;
          msg.lr_wheel_fb_velocity = static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000;

          msg.lr_wheel_fb_pulse = static_cast<int>(recv_frame.data[5] << 24 | recv_frame.data[4] << 16 | recv_frame.data[3] << 8 | recv_frame.data[2]);

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if (crc == recv_frame.data[7])
          {
            chassis_info_msg.lr_wheel_fb = msg;

            current_pulses_[1] = msg.lr_wheel_fb_pulse;
updated_[1] = true;
          }

          break;
        }

        case 0x18C4D8EF | CAN_EFF_FLAG:
        {
          yhs_can_interfaces::msg::RrWheelFb msg;
          msg.rr_wheel_fb_velocity = static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000;

          msg.rr_wheel_fb_pulse = static_cast<int>(recv_frame.data[5] << 24 | recv_frame.data[4] << 16 | recv_frame.data[3] << 8 | recv_frame.data[2]);

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if (crc == recv_frame.data[7])
          {
            chassis_info_msg.rr_wheel_fb = msg;

            current_pulses_[2] = msg.rr_wheel_fb_pulse;
updated_[2] = true;
          }

          break;
        }

        case 0x18C4D9EF | CAN_EFF_FLAG:
        {
          yhs_can_interfaces::msg::RfWheelFb msg;
          msg.rf_wheel_fb_velocity = static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000;

          msg.rf_wheel_fb_pulse = static_cast<int>(recv_frame.data[5] << 24 | recv_frame.data[4] << 16 | recv_frame.data[3] << 8 | recv_frame.data[2]);

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if (crc == recv_frame.data[7])
          {
            chassis_info_msg.rf_wheel_fb = msg;

            current_pulses_[3] = msg.rf_wheel_fb_pulse;
            updated_[3] = true;
          }

          break;
        }

        case 0x18C4DAEF | CAN_EFF_FLAG:
        {
          yhs_can_interfaces::msg::IoFb msg;
          msg.io_fb_lamp_ctrl = (recv_frame.data[0] & 0x01) != 0;
          msg.io_fb_unlock = (recv_frame.data[0] & 0x02) != 0;
          msg.io_fb_low_power_enable = (recv_frame.data[0] & 0x04) != 0;
          msg.io_fb_low_power_state = (recv_frame.data[0] & 0x08) != 0;

          msg.io_fb_lower_beam_headlamp = (recv_frame.data[1] & 0x01) != 0;
          msg.io_fb_upper_beam_headlamp = (recv_frame.data[1] & 0x02) != 0;
          msg.io_fb_turn_lamp = (0x0c & recv_frame.data[1]) >> 2;

          msg.io_fb_braking_lamp = (0x10 & recv_frame.data[1]) != 0;
          msg.io_fb_clearance_lamp = (0x20 & recv_frame.data[1]) != 0;
          msg.io_fb_fog_lamp = (0x40 & recv_frame.data[1]) != 0;
          msg.io_fb_speaker = (0x80 & recv_frame.data[1]) != 0;

          msg.io_fb_low_power_ratio = recv_frame.data[2];

          msg.io_fb_fl_impact_sensor = (0x01 & recv_frame.data[3]) != 0;
          msg.io_fb_fm_impact_sensor = (0x02 & recv_frame.data[3]) != 0;
          msg.io_fb_fr_impact_sensor = (0x04 & recv_frame.data[3]) != 0;
          msg.io_fb_rl_impact_sensor = (0x08 & recv_frame.data[3]) != 0;
          msg.io_fb_rm_impact_sensor = (0x10 & recv_frame.data[3]) != 0;
          msg.io_fb_rr_impact_sensor = (0x20 & recv_frame.data[3]) != 0;
          msg.io_fb_fl_drop_sensor = (0x01 & recv_frame.data[4]) != 0;
          msg.io_fb_fm_drop_sensor = (0x02 & recv_frame.data[4]) != 0;
          msg.io_fb_fr_drop_sensor = (0x04 & recv_frame.data[4]) != 0;
          msg.io_fb_rl_drop_sensor = (0x08 & recv_frame.data[4]) != 0;
          msg.io_fb_rm_drop_sensor = (0x10 & recv_frame.data[4]) != 0;
          msg.io_fb_rr_drop_sensor = (0x20 & recv_frame.data[4]) != 0;
          msg.io_fb_estop = (0x01 & recv_frame.data[5]) != 0;
          msg.io_fb_joypad_ctrl = (0x02 & recv_frame.data[5]) != 0;
          // msg.io_fb_charge_state = (0x04 & recv_frame.data[5]) != 0;
          msg.io_fb_wi_charge_ctrl = (0x04 & recv_frame.data[5]) != 0;
          msg.io_fb_wi_charge_state = (0x18 & recv_frame.data[5]) >> 3;


          msg.io_fb_charger_sign = (0x08 & recv_frame.data[5]) != 0;
          msg.io_fb_joypad_first = (0x10 & recv_frame.data[5]) != 0;
          msg.io_fb_joypad_online = (0x20 & recv_frame.data[5]) != 0;

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if (crc == recv_frame.data[7])
          {
            chassis_info_msg.io_fb = msg;
          }

          break;
        }

        case 0x18C4DBEF | CAN_EFF_FLAG:
        {
          yhs_can_interfaces::msg::MotorFb msg;
          msg.motor_cmd_drive_enable_lf = (recv_frame.data[0] & 0x01) != 0;
          msg.motor_cmd_drive_enable_lr = (recv_frame.data[0] & 0x02) != 0;
          msg.motor_cmd_drive_enable_rf = (recv_frame.data[0] & 0x04) != 0;
          msg.motor_cmd_drive_enable_rr = (recv_frame.data[0] & 0x08) != 0;

          msg.motor_cmd_steering_enable_lf = (0x10 & recv_frame.data[0]) != 0;
          msg.motor_cmd_steering_enable_lr = (0x20 & recv_frame.data[0]) != 0;
          msg.motor_cmd_steering_enable_rf = (0x40 & recv_frame.data[0]) != 0;
          msg.motor_cmd_steering_enable_rr = (0x80 & recv_frame.data[0]) != 0;

          msg.motor_cmd_power_restart = (recv_frame.data[1] & 0x01) != 0;

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if (crc == recv_frame.data[7])
          {
            chassis_info_msg.motor_fb = msg;
            chassis_info_fb_publisher_->publish(chassis_info_msg);
          }

          break;
        }

        case 0x18C4DCEF | CAN_EFF_FLAG:
        {
          yhs_can_interfaces::msg::FrontAngleFb msg;
          msg.front_angle_fb_l = static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 100;

          msg.front_angle_fb_r = static_cast<float>(static_cast<short>(recv_frame.data[3] << 8 | recv_frame.data[2])) / 100;

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if (crc == recv_frame.data[7])
          {
            chassis_info_msg.front_angle_fb = msg;
            steering_angles_[0] = msg.front_angle_fb_l / 180.0 * M_PI; // LF
            steering_angles_[3] = msg.front_angle_fb_r / 180.0 * M_PI; // RF
          }

          break;
        }

        case 0x18C4DDEF | CAN_EFF_FLAG:
        {
          yhs_can_interfaces::msg::RearAngleFb msg;
          msg.rear_angle_fb_l = static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 100;

          msg.rear_angle_fb_r = static_cast<float>(static_cast<short>(recv_frame.data[3] << 8 | recv_frame.data[2])) / 100;

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if (crc == recv_frame.data[7])
          {
            chassis_info_msg.rear_angle_fb = msg;
            steering_angles_[1] = msg.rear_angle_fb_l / 180.0 * M_PI; // LR
            steering_angles_[2] = msg.rear_angle_fb_r / 180.0 * M_PI; // RR
          }

          break;
        }

        case 0x18C4E1EF | CAN_EFF_FLAG:
        {
          yhs_can_interfaces::msg::BmsFb msg;
          msg.bms_fb_voltage = static_cast<float>(static_cast<unsigned short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 100;

          msg.bms_fb_current = static_cast<float>(static_cast<short>(recv_frame.data[3] << 8 | recv_frame.data[2])) / 100;

          msg.bms_fb_remaining_capacity = static_cast<float>(static_cast<unsigned short>(recv_frame.data[5] << 8 | recv_frame.data[4])) / 100;

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if (crc == recv_frame.data[7])
          {
            chassis_info_msg.bms_fb = msg;
          }

          break;
        }

        case 0x18C4E2EF | CAN_EFF_FLAG:
        {
          yhs_can_interfaces::msg::BmsFlagFb msg;
          msg.bms_flag_fb_soc = recv_frame.data[0];

          msg.bms_flag_fb_single_ov = (recv_frame.data[1] & 0x01) != 0;
          msg.bms_flag_fb_single_uv = (recv_frame.data[1] & 0x02) != 0;
          msg.bms_flag_fb_ov = (recv_frame.data[1] & 0x04) != 0;
          msg.bms_flag_fb_uv = (recv_frame.data[1] & 0x08) != 0;
          msg.bms_flag_fb_charge_ot = (recv_frame.data[1] & 0x10) != 0;
          msg.bms_flag_fb_charge_ut = (recv_frame.data[1] & 0x20) != 0;
          msg.bms_flag_fb_discharge_ot = (recv_frame.data[1] & 0x40) != 0;
          msg.bms_flag_fb_discharge_ut = (recv_frame.data[1] & 0x80) != 0;

          msg.bms_flag_fb_charge_oc = (recv_frame.data[2] & 0x01) != 0;
          msg.bms_flag_fb_discharge_oc = (recv_frame.data[2] & 0x02) != 0;
          msg.bms_flag_fb_short = (recv_frame.data[2] & 0x04) != 0;
          msg.bms_flag_fb_ic_error = (recv_frame.data[2] & 0x08) != 0;
          msg.bms_flag_fb_lock_mos = (recv_frame.data[2] & 0x10) != 0;
          msg.bms_flag_fb_charge_flag = (recv_frame.data[2] & 0x20) != 0;
          msg.bms_flag_fb_heating_flag = (recv_frame.data[2] & 0x40) != 0;

          msg.bms_flag_fb_hight_temperature = static_cast<float>(static_cast<short>(recv_frame.data[4] << 4 | recv_frame.data[3] >> 4)) / 10;

          msg.bms_flag_fb_low_temperature = static_cast<float>(static_cast<short>((recv_frame.data[6] & 0x0f) << 8 | recv_frame.data[5])) / 10;

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if (crc == recv_frame.data[7])
          {
            chassis_info_msg.bms_flag_fb = msg;
          }

          break;
        }

        case 0x18C4E3EF | CAN_EFF_FLAG:
        {
          yhs_can_interfaces::msg::DriveMotorCurrentFb msg;
          msg.drive_motor_current_fb_lf = (float)((short) (((recv_frame.data[1] & 0x0f) << 8 | recv_frame.data[0]) << 4) >> 4) / 10;
          msg.drive_motor_current_fb_lr = (float)((short) ((recv_frame.data[2] << 4 | (recv_frame.data[1] >> 4)) << 4) >> 4) / 10;

          msg.drive_motor_current_fb_rf = (float)((short) (((recv_frame.data[4] & 0x0f) << 8 | recv_frame.data[3]) << 4) >> 4) / 10;
          msg.drive_motor_current_fb_rr = (float)((short) ((recv_frame.data[5] << 4 | (recv_frame.data[4] >> 4)) << 4) >> 4) / 10;

          if (0x01 & recv_frame.data[6])
            msg.drive_motor_oc_flag_fb_lf = true;
          else
            msg.drive_motor_oc_flag_fb_lf = false;

          if (0x02 & recv_frame.data[6])
            msg.drive_motor_oc_flag_fb_lr = true;
          else
            msg.drive_motor_oc_flag_fb_lr = false;

          if (0x04 & recv_frame.data[6])
            msg.drive_motor_oc_flag_fb_rf = true;
          else
            msg.drive_motor_oc_flag_fb_rf = false;

          if (0x08 & recv_frame.data[6])
            msg.drive_motor_oc_flag_fb_rr = true;
          else
            msg.drive_motor_oc_flag_fb_rr = false;

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if (crc == recv_frame.data[7])
          {
            chassis_info_msg.drive_motor_current_fb = msg;
          }

          break;
        }

        case 0x18C4E4EF | CAN_EFF_FLAG:
        {
          yhs_can_interfaces::msg::SteeringMotorCurrentFb msg;
          msg.steering_motor_current_fb_lf = (float)((short) (((recv_frame.data[1] & 0x0f) << 8 | recv_frame.data[0]) << 4) >> 4) / 10;
          msg.steering_motor_current_fb_lr = (float)((short) ((recv_frame.data[2] << 4 | (recv_frame.data[1] >> 4)) << 4) >> 4) / 10;

          msg.steering_motor_current_fb_rf = (float)((short) (((recv_frame.data[4] & 0x0f) << 8 | recv_frame.data[3]) << 4) >> 4) / 10;
          msg.steering_motor_current_fb_rr = (float)((short) ((recv_frame.data[5] << 4 | (recv_frame.data[4] >> 4)) << 4) >> 4) / 10;

          if (0x01 & recv_frame.data[6])
            msg.steering_motor_oc_flag_fb_lf = true;
          else
            msg.steering_motor_oc_flag_fb_lf = false;

          if (0x02 & recv_frame.data[6])
            msg.steering_motor_oc_flag_fb_lr = true;
          else
            msg.steering_motor_oc_flag_fb_lr = false;

          if (0x04 & recv_frame.data[6])
            msg.steering_motor_oc_flag_fb_rf = true;
          else
            msg.steering_motor_oc_flag_fb_rf = false;

          if (0x08 & recv_frame.data[6])
            msg.steering_motor_oc_flag_fb_rr = true;
          else
            msg.steering_motor_oc_flag_fb_rr = false;
          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if (crc == recv_frame.data[7])
          {
            chassis_info_msg.steering_motor_current_fb = msg;
          }

          break;
        }

          // ultrasonic预留
          static unsigned short ultra_data[8] = {0};
        case 0x18C4E8EF | CAN_EFF_FLAG:
        {
          ultra_data[0] = (unsigned short)((recv_frame.data[1] & 0x0f) << 8 | recv_frame.data[0]);
          ultra_data[1] = (unsigned short)(recv_frame.data[2] << 4 | ((recv_frame.data[1] & 0xf0) >> 4));

          ultra_data[2] = (unsigned short)((recv_frame.data[4] & 0x0f) << 8 | recv_frame.data[3]);
          ultra_data[3] = (unsigned short)(recv_frame.data[5] << 4 | ((recv_frame.data[4] & 0xf0) >> 4));
          break;
        }

        case 0x18C4E9EF | CAN_EFF_FLAG:
        {
          ultra_data[4] = (unsigned short)((recv_frame.data[1] & 0x0f) << 8 | recv_frame.data[0]);
          ultra_data[5] = (unsigned short)(recv_frame.data[2] << 4 | ((recv_frame.data[1] & 0xf0) >> 4));

          ultra_data[6] = (unsigned short)((recv_frame.data[4] & 0x0f) << 8 | recv_frame.data[3]);
          ultra_data[7] = (unsigned short)(recv_frame.data[5] << 4 | ((recv_frame.data[4] & 0xf0) >> 4));

          yhs_can_interfaces::msg::Ultrasonic ultra_msg;

          ultra_msg.front_ultrasonic_fb_1 = ultra_data[ultrasonic_number_[0]];
          ultra_msg.front_ultrasonic_fb_2 = ultra_data[ultrasonic_number_[1]];
          ultra_msg.front_ultrasonic_fb_3 = ultra_data[ultrasonic_number_[2]];
          ultra_msg.front_ultrasonic_fb_4 = ultra_data[ultrasonic_number_[3]];

          ultra_msg.rear_ultrasonic_fb_5 = ultra_data[ultrasonic_number_[4]];
          ultra_msg.rear_ultrasonic_fb_6 = ultra_data[ultrasonic_number_[5]];
          ultra_msg.rear_ultrasonic_fb_7 = ultra_data[ultrasonic_number_[6]];
          ultra_msg.rear_ultrasonic_fb_8 = ultra_data[ultrasonic_number_[7]];

          chassis_info_msg.ultrasonic = ultra_msg;

          break;
        }

        case 0x18C4EAEF | CAN_EFF_FLAG:
        {
          yhs_can_interfaces::msg::ErrorFb msg;
          msg.error_fb_level = recv_frame.data[0];

          msg.error_fb_device_type = recv_frame.data[1];

          msg.error_fb_devive_id = recv_frame.data[2];

          msg.error_fb_emergency_code = recv_frame.data[3];

          msg.error_fb_register_code = (recv_frame.data[5] << 8) | recv_frame.data[4];

          unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

          if (crc == recv_frame.data[7])
          {
            chassis_info_msg.error_fb = msg;
          }

          break;
        }

        default:
          break;
        }
      }
    }
  }

  void CanControl::imu_data_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    double x = imu_msg->orientation.x;
    double y = imu_msg->orientation.y;
    double z = imu_msg->orientation.z;
    double w = imu_msg->orientation.w;

    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    imu_roll_ = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0)
      imu_pitch_ = std::copysign(M_PI / 2.0, sinp);
    else
      imu_pitch_ = std::asin(sinp);

    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    imu_yaw_ = std::atan2(siny_cosp, cosy_cosp);
  }

  

  void CanControl::publish_odom(const double linear_vel, const double angular_vel, const unsigned char gear, const double slipangle)
  {
    static double x = 0.0;
    static double y = 0.0;
    static double th = 0.0;

    rclcpp::Time current_time = node_->now();
    double dt = (current_time - last_odom_time_).seconds();
    if (dt > 1.0) 
      dt = 0.0;

    double actual_slipangle = (gear == 6) ? 0.0 : slipangle;
    double odom_vx = linear_vel * cos(actual_slipangle) * linear_scale_x_;
    double odom_vy = linear_vel * sin(actual_slipangle) * linear_scale_y_;
    double vth = angular_vel * angular_scale_z_;
    double delta_th = vth * dt;
    double mid_th = th + delta_th / 2.0;
    double delta_x = (odom_vx * cos(mid_th) - odom_vy * sin(mid_th)) * dt;
    double delta_y = (odom_vx * sin(mid_th) + odom_vy * cos(mid_th)) * dt;

    x += delta_x;
    y += delta_y;

    if (imu_yaw_ == 88.0) {
      th += delta_th;
    } else {
      std::lock_guard<std::mutex> lock(mutex_);
      th = imu_yaw_;
    }

    while (th > M_PI) 
      th -= 2.0 * M_PI;
    while (th < -M_PI) 
      th += 2.0 * M_PI;

    tf2::Quaternion q;
    q.setRPY(0, 0, th);

    // 构建 TF 消息
    if (tf_used_)
    {
      geometry_msgs::msg::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = odom_frame_;
      odom_trans.child_frame_id = base_frame_;
      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation.x = q.x();
      odom_trans.transform.rotation.y = q.y();
      odom_trans.transform.rotation.z = q.z();
      odom_trans.transform.rotation.w = q.w();
      odom_broadcaster_->sendTransform(odom_trans);
    }

    // 构建 Odom 消息
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = odom_vx;
    odom_msg.twist.twist.linear.y = odom_vy; 
    odom_msg.twist.twist.angular.z = vth;

    // Covariance 
    odom_msg.pose.covariance[0] = 0.1;
    odom_msg.pose.covariance[7] = 0.1;
    odom_msg.pose.covariance[35] = 0.2;
    odom_msg.pose.covariance[14] = 1e10;
    odom_msg.pose.covariance[21] = 1e10;
    odom_msg.pose.covariance[28] = 1e10;

    odom_pub_->publish(odom_msg);

    last_odom_time_ = current_time;
  }

  // 计算脉冲差值并考虑溢出
  int CanControl::calculate_pulse_diff(int current_pulse, int last_pulse)
  {
    if (current_pulse < last_pulse)
    {
      if (last_pulse - current_pulse > INT_MAX / 2)
      {
        return (INT_MAX - last_pulse) + current_pulse + 1;
      }
      else
      {
        return current_pulse - last_pulse;
      }
    }
    else
    {
      return current_pulse - last_pulse;
    }
  }

  // 检查是否4个轮子都已收到过第一帧数据
  bool CanControl::all_wheels_initialized()
  {
    for (int i = 0; i < 4; ++i)
    {
      if (last_pulses_[i] == -1)
      {
        return false;
      }
    }
    return true;
  }

  void CanControl::timer_callback()
  {
    send_and_keep_alive();
  }

  void CanControl::send_and_keep_alive()
  {
    // 组装 IO 消息
    yhs_can_interfaces::msg::IoCmd io_msg;
    io_msg.io_cmd_lamp_ctrl = io_cmd_lamp_ctrl_;
    io_msg.io_cmd_unlock = io_cmd_unlock_;
    // 低功耗模式使用默认值即可，不用持续激活
    // io_msg.io_cmd_lower_beam_headlamp = io_cmd_lower_beam_headlamp_;
    // io_msg.io_cmd_upper_beam_headlamp = io_cmd_upper_beam_headlamp_;
    io_msg.io_cmd_turn_lamp = io_cmd_turn_lamp_;
    io_msg.io_cmd_braking_lamp = io_cmd_braking_lamp_;
    io_msg.io_cmd_clearance_lamp = io_cmd_clearance_lamp_;
    io_msg.io_cmd_fog_lamp = io_cmd_fog_lamp_;
    io_msg.io_cmd_speaker = io_cmd_speaker_;
    io_msg.io_cmd_low_power_ratio = io_cmd_low_power_ratio_;
    io_msg.io_cmd_low_power_enable = io_cmd_low_power_enable_;

    // 组装 Motor 消息
    yhs_can_interfaces::msg::MotorCmd motor_msg;
    motor_msg.motor_cmd_drive_enable_lf = motor_cmd_drive_enable_lf_;
    motor_msg.motor_cmd_drive_enable_lr = motor_cmd_drive_enable_lr_;
    motor_msg.motor_cmd_drive_enable_rf = motor_cmd_drive_enable_rf_;
    motor_msg.motor_cmd_drive_enable_rr = motor_cmd_drive_enable_rr_;
    motor_msg.motor_cmd_steering_enable_lf = motor_cmd_steering_enable_lf_;
    motor_msg.motor_cmd_steering_enable_lr = motor_cmd_steering_enable_lr_;
    motor_msg.motor_cmd_steering_enable_rf = motor_cmd_steering_enable_rf_;
    motor_msg.motor_cmd_steering_enable_rr = motor_cmd_steering_enable_rr_;
    motor_msg.motor_cmd_power_restart = motor_cmd_power_restart_;

    // 发送
    io_cmd_send(io_msg);
    motor_cmd_send(motor_msg);
  }

  void CanControl::send_control_burst(int repeat_count)
  {
    for (int i = 0; i < repeat_count; i++)
    {
      send_and_keep_alive();
      usleep(10000);
    }
  }

  CanControl::~CanControl()
  {
  }

  bool CanControl::run()
  {
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to open socket: " << strerror(errno));
      return false;
    }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, if_name_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to get interface index: " << strerror(errno) << " ==> " << if_name_.c_str());
      return false;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to bind socket: " << strerror(errno));
      return false;
    }

    // 电机重启处理 (下降沿触发)
    if (motor_cmd_power_restart_)
    {
      send_control_burst(20);
    }
    else
    {
      send_control_burst(20);
    }

    if (motor_cmd_power_restart_) 
    {
      motor_cmd_power_restart_ = false;
      RCLCPP_INFO(node_->get_logger(), "Triggered Motor Restart (Falling Edge).");
    }

    // 处理安全停车解锁下降沿 
    if (io_cmd_unlock_) 
    {
      io_cmd_unlock_ = false;
      RCLCPP_INFO(node_->get_logger(), "Triggered Parking Brake Unlock (Falling Edge).");
    }

    // 启动定时器 (20ms周期发送io_cmd和motor_cmd保活)
    keep_alive_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&CanControl::timer_callback, this));

    thread_ = std::thread(&CanControl::can_data_recv_callback, this);

    return true;
  }

  void CanControl::stop()
  {
    if (can_socket_ >= 0)
    {
      close(can_socket_);
      can_socket_ = -1;
    }

    if (thread_.joinable())
    {
      thread_.join();
    }
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("yhs_can_control_node");

  yhs::CanControl cancontrol(node);
  if (!cancontrol.run())
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize yhs_can_control_node");
    return 0;
  }

  RCLCPP_INFO(node->get_logger(), "yhs_can_control_node initialized successfully");

  rclcpp::spin(node);

  cancontrol.stop();
  RCLCPP_INFO(node->get_logger(), "yhs_can_control_node stopped");

  rclcpp::shutdown();

  return 0;
}
