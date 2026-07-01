#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>
#include <math.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "yhs_can_control.h"

namespace yhs_tool
{

  CanControl::CanControl()
  {
    ros::NodeHandle private_node("~");

    std::string ultrasonic_numbers_str;
    private_node.getParam("ultrasonic_number", ultrasonic_numbers_str);
    private_node.param("/yhs_can_control/odom_frame", odomFrame_, std::string("odom"));
    private_node.param("/yhs_can_control/base_link_frame", baseFrame_, std::string("base_link"));
    private_node.param("/yhs_can_control/tfUsed", tfUsed_, false);
    private_node.param("/yhs_can_control/if_name", if_name_, std::string("can0"));

    private_node.param("/yhs_can_control/io_cmd/io_cmd_lamp_ctrl", io_param_lamp_ctrl_, true);
    private_node.param("/yhs_can_control/io_cmd/io_cmd_unlock", io_param_unlock_, false);
    private_node.param("/yhs_can_control/io_cmd/io_cmd_low_power_enable", io_param_low_power_enable_, false);
    private_node.param("/yhs_can_control/io_cmd/io_cmd_lower_beam_headlamp", io_param_lower_beam_, false);
    private_node.param("/yhs_can_control/io_cmd/io_cmd_upper_beam_headlamp", io_param_upper_beam_, false);
    private_node.param("/yhs_can_control/io_cmd/io_cmd_turn_lamp", io_param_turn_lamp_, 0); 
    private_node.param("/yhs_can_control/io_cmd/io_cmd_braking_lamp", io_param_braking_lamp_, false);
    private_node.param("/yhs_can_control/io_cmd/io_cmd_clearance_lamp", io_param_clearance_lamp_, false);
    private_node.param("/yhs_can_control/io_cmd/io_cmd_fog_lamp", io_param_fog_lamp_, false);
    private_node.param("/yhs_can_control/io_cmd/io_cmd_speaker", io_param_speaker_, false);
    private_node.param("/yhs_can_control/io_cmd/io_cmd_low_power_ratio", io_param_low_power_ratio_, 0);

    private_node.param("/yhs_can_control/motor_cmd/motor_cmd_drive_enable_lf", motor_cmd_drive_enable_lf_, true);
    private_node.param("/yhs_can_control/motor_cmd/motor_cmd_drive_enable_lr", motor_cmd_drive_enable_lr_, true);
    private_node.param("/yhs_can_control/motor_cmd/motor_cmd_drive_enable_rf", motor_cmd_drive_enable_rf_, true);
    private_node.param("/yhs_can_control/motor_cmd/motor_cmd_drive_enable_rr", motor_cmd_drive_enable_rr_, true);
    private_node.param("/yhs_can_control/motor_cmd/motor_cmd_steering_enable_lf", motor_cmd_steering_enable_lf_, true);
    private_node.param("/yhs_can_control/motor_cmd/motor_cmd_steering_enable_lr", motor_cmd_steering_enable_lr_, true);
    private_node.param("/yhs_can_control/motor_cmd/motor_cmd_steering_enable_rf", motor_cmd_steering_enable_rf_, true);
    private_node.param("/yhs_can_control/motor_cmd/motor_cmd_steering_enable_rr", motor_cmd_steering_enable_rr_, true);
    private_node.param("/yhs_can_control/motor_cmd/motor_cmd_power_restart", motor_cmd_power_restart_, false);

    current_io_cmd_.io_cmd_lamp_ctrl = io_param_lamp_ctrl_;
    current_io_cmd_.io_cmd_unlock = io_param_unlock_;
    current_io_cmd_.io_cmd_low_power_enable = io_param_low_power_enable_;
    current_io_cmd_.io_cmd_lower_beam_headlamp = io_param_lower_beam_;
    current_io_cmd_.io_cmd_upper_beam_headlamp = io_param_upper_beam_;
    current_io_cmd_.io_cmd_turn_lamp = io_param_turn_lamp_;
    current_io_cmd_.io_cmd_braking_lamp = io_param_braking_lamp_;
    current_io_cmd_.io_cmd_clearance_lamp = io_param_clearance_lamp_;
    current_io_cmd_.io_cmd_fog_lamp = io_param_fog_lamp_;
    current_io_cmd_.io_cmd_speaker = io_param_speaker_;
    current_io_cmd_.io_cmd_low_power_ratio = io_param_low_power_ratio_;

    current_motor_cmd_.motor_cmd_drive_enable_lf = true;
    current_motor_cmd_.motor_cmd_drive_enable_lr = true;
    current_motor_cmd_.motor_cmd_drive_enable_rf = true;
    current_motor_cmd_.motor_cmd_drive_enable_rr = true;
    current_motor_cmd_.motor_cmd_steering_enable_lf = true;
    current_motor_cmd_.motor_cmd_steering_enable_lr = true;
    current_motor_cmd_.motor_cmd_steering_enable_rf = true;
    current_motor_cmd_.motor_cmd_steering_enable_rr = true;
    current_motor_cmd_.motor_cmd_power_restart = false;

    current_motor_cmd_.motor_cmd_drive_enable_lf = motor_cmd_drive_enable_lf_;
    current_motor_cmd_.motor_cmd_drive_enable_lr = motor_cmd_drive_enable_lr_;
    current_motor_cmd_.motor_cmd_drive_enable_rf = motor_cmd_drive_enable_rf_;
    current_motor_cmd_.motor_cmd_drive_enable_rr = motor_cmd_drive_enable_rr_;
    current_motor_cmd_.motor_cmd_steering_enable_lf = motor_cmd_steering_enable_lf_;
    current_motor_cmd_.motor_cmd_steering_enable_lr = motor_cmd_steering_enable_lr_;
    current_motor_cmd_.motor_cmd_steering_enable_rf = motor_cmd_steering_enable_rf_;
    current_motor_cmd_.motor_cmd_steering_enable_rr = motor_cmd_steering_enable_rr_;
    current_motor_cmd_.motor_cmd_power_restart = motor_cmd_power_restart_;

    last_imu_time_ = ros::Time(0);

    std::istringstream iss(ultrasonic_numbers_str);
    int number;
    while (iss >> number)
    {
      ultrasonic_number_.push_back(number);
    }
  }

  CanControl::~CanControl()
  {
    close(dev_handler_);
  }

  void CanControl::sendIoCommand()
  {
    static unsigned char count_io = 0;
    memset(sendData_u_io_, 0, 8);

    if (current_io_cmd_.io_cmd_lamp_ctrl)        sendData_u_io_[0] |= 0x01;
    if (current_io_cmd_.io_cmd_unlock)           sendData_u_io_[0] |= 0x02;
    if (current_io_cmd_.io_cmd_low_power_enable) sendData_u_io_[0] |= 0x04;

    if (current_io_cmd_.io_cmd_lower_beam_headlamp) sendData_u_io_[1] |= 0x01;
    if (current_io_cmd_.io_cmd_upper_beam_headlamp) sendData_u_io_[1] |= 0x02;
    sendData_u_io_[1] |= (current_io_cmd_.io_cmd_turn_lamp & 0x03) << 2; 
    if (current_io_cmd_.io_cmd_braking_lamp)        sendData_u_io_[1] |= 0x10;
    if (current_io_cmd_.io_cmd_clearance_lamp)      sendData_u_io_[1] |= 0x20;
    if (current_io_cmd_.io_cmd_fog_lamp)            sendData_u_io_[1] |= 0x40;
    if (current_io_cmd_.io_cmd_speaker)             sendData_u_io_[1] |= 0x80;

    sendData_u_io_[2] = (unsigned char)current_io_cmd_.io_cmd_low_power_ratio;

    if (current_io_cmd_.io_cmd_disCharge) sendData_u_io_[5] |= 0x01;

    count_io++;
    if (count_io > 15) count_io = 0;
    sendData_u_io_[6] = count_io << 4;

    sendData_u_io_[7] = sendData_u_io_[0] ^ sendData_u_io_[1] ^ sendData_u_io_[2] ^ 
                        sendData_u_io_[3] ^ sendData_u_io_[4] ^ sendData_u_io_[5] ^ sendData_u_io_[6];

    send_frames_[0].can_id = 0x18C4D7D0 | CAN_EFF_FLAG; // 0x18C4D7D0
    send_frames_[0].can_dlc = 8;
    memcpy(send_frames_[0].data, sendData_u_io_, 8);

    int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) ROS_ERROR_THROTTLE(1, "Send IO CMD failed: %d", ret);
  }

  void CanControl::io_cmdCallBack(const yhs_can_msgs::io_cmd::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(cmd_mutex_);
    current_io_cmd_ = *msg;
  }

  void CanControl::sendCtrlCommand()
  {
    static unsigned char count_ctrl = 0;
    memset(sendData_u_vel_, 0, 8);

    short linear_x = (short)(current_ctrl_cmd_.ctrl_cmd_x_linear * 1000);
    
    short linear_y = (short)(current_ctrl_cmd_.ctrl_cmd_y_linear * 1000);

    // short angular_z = (short)(current_ctrl_cmd_.ctrl_cmd_z_angular * 180.0f / M_PI * 100.0f);
    short angular_z = (short)(current_ctrl_cmd_.ctrl_cmd_z_angular * 100.0f);

    sendData_u_vel_[0] = (0x0f & current_ctrl_cmd_.ctrl_cmd_gear) | (0xf0 & ((linear_x & 0x0f) << 4));

    sendData_u_vel_[1] = (linear_x >> 4) & 0xff;

    sendData_u_vel_[2] = (0x0f & (linear_x >> 12)) | (0xf0 & ((angular_z & 0x0f) << 4));

    sendData_u_vel_[3] = (angular_z >> 4) & 0xff;

    sendData_u_vel_[4] = (0x0f & (angular_z >> 12)) | (0xf0 & ((linear_y & 0x0f) << 4));

    sendData_u_vel_[5] = (linear_y >> 4) & 0xff;

    sendData_u_vel_[6] = (0x0f & (linear_y >> 12));
    
    count_ctrl++;
    if (count_ctrl > 15) count_ctrl = 0;
    sendData_u_vel_[6] |= (count_ctrl << 4);

    sendData_u_vel_[7] = sendData_u_vel_[0] ^ sendData_u_vel_[1] ^ sendData_u_vel_[2] ^ 
                         sendData_u_vel_[3] ^ sendData_u_vel_[4] ^ sendData_u_vel_[5] ^ sendData_u_vel_[6];

    send_frames_[0].can_id = 0x18C4D1D0 | CAN_EFF_FLAG;
    send_frames_[0].can_dlc = 8;
    memcpy(send_frames_[0].data, sendData_u_vel_, 8);

    int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) ROS_ERROR_THROTTLE(1, "Send Ctrl CMD failed: %d", ret);
  }

  void CanControl::ctrl_cmdCallBack(const yhs_can_msgs::ctrl_cmd::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(cmd_mutex_);
    current_ctrl_cmd_ = *msg;
  }

  void CanControl::sendMotorCmd()
  {
    static unsigned char count_motor = 0;
    unsigned char sendData[8];
    memset(sendData, 0, 8);

    if (current_motor_cmd_.motor_cmd_drive_enable_lf)    
      sendData[0] |= 0x01;
    if (current_motor_cmd_.motor_cmd_drive_enable_lr)    
      sendData[0] |= 0x02;
    if (current_motor_cmd_.motor_cmd_drive_enable_rf)    
      sendData[0] |= 0x04;
    if (current_motor_cmd_.motor_cmd_drive_enable_rr)    
      sendData[0] |= 0x08;
    if (current_motor_cmd_.motor_cmd_steering_enable_lf) 
      sendData[0] |= 0x10;
    if (current_motor_cmd_.motor_cmd_steering_enable_lr) 
      sendData[0] |= 0x20;
    if (current_motor_cmd_.motor_cmd_steering_enable_rf) 
      sendData[0] |= 0x40;
    if (current_motor_cmd_.motor_cmd_steering_enable_rr) 
      sendData[0] |= 0x80;

    if (current_motor_cmd_.motor_cmd_power_restart)      
      sendData[1] |= 0x01;

    count_motor++;
    if (count_motor > 15) count_motor = 0;
    sendData[6] = count_motor << 4;

    sendData[7] = sendData[0] ^ sendData[1] ^ sendData[2] ^ sendData[3] ^ 
                  sendData[4] ^ sendData[5] ^ sendData[6];

    send_frames_[0].can_id = 0x18C4D8D0 | CAN_EFF_FLAG;
    send_frames_[0].can_dlc = 8;
    memcpy(send_frames_[0].data, sendData, 8);

    int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) ROS_ERROR_THROTTLE(1, "Send Motor Cmd failed: %d", ret);
  }

  void CanControl::motor_cmdCallBack(const yhs_can_msgs::motor_cmd msg)
  {
    boost::mutex::scoped_lock lock(cmd_mutex_);
    current_motor_cmd_ = msg;
  }

  void CanControl::sendSteeringCtrlCmd()
  {
    static unsigned char count_str = 0;
    unsigned char sendData[8];
    memset(sendData, 0, 8);

    short velocity = (short)(current_steering_ctrl_cmd_.steering_ctrl_cmd_velocity * 1000);
    short steering = (short)(current_steering_ctrl_cmd_.steering_ctrl_cmd_steering * 100);

    sendData[0] = (0x0f & current_steering_ctrl_cmd_.ctrl_cmd_gear) | ((velocity & 0x0f) << 4);
    
    sendData[1] = (velocity >> 4) & 0xff;

    sendData[2] = ((velocity >> 12) & 0x0f) | ((steering & 0x0f) << 4);

    sendData[3] = (steering >> 4) & 0xff;

    sendData[4] = (steering >> 12) & 0x0f;

    count_str++;
    if (count_str > 15) count_str = 0;
    sendData[6] = count_str << 4;

    sendData[7] = sendData[0] ^ sendData[1] ^ sendData[2] ^ sendData[3] ^ 
                  sendData[4] ^ sendData[5] ^ sendData[6];

    send_frames_[0].can_id = 0x18C4D2D0 | CAN_EFF_FLAG;
    send_frames_[0].can_dlc = 8;
    memcpy(send_frames_[0].data, sendData, 8);

    int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) ROS_ERROR_THROTTLE(1, "Send Steering Cmd failed: %d", ret);
  }

  void CanControl::steering_ctrl_cmdCallBack(const yhs_can_msgs::steering_ctrl_cmd msg)
  {
    boost::mutex::scoped_lock lock(cmd_mutex_);
    current_steering_ctrl_cmd_ = msg;
    current_control_mode_ = MODE_STEERING_CTRL; 
  }

  void CanControl::sendFrontFreeCtrlCmd()
  {
    static unsigned char count_free_f = 0;
    
    unsigned char data_vel[8] = {0};
    short vel_lf = (short)(current_front_vel_cmd_.free_ctrl_cmd_velocity_lf * 1000);
    short vel_rf = (short)(current_front_vel_cmd_.free_ctrl_cmd_velocity_rf * 1000);

    data_vel[0] = (0x0f & current_front_vel_cmd_.ctrl_cmd_gear) | ((vel_lf & 0x0f) << 4);
    data_vel[1] = (vel_lf >> 4) & 0xff;
    data_vel[2] = ((vel_lf >> 12) & 0x0f) | ((vel_rf & 0x0f) << 4);
    data_vel[3] = (vel_rf >> 4) & 0xff;
    data_vel[4] = (vel_rf >> 12) & 0x0f;
    
    data_vel[6] = (count_free_f & 0x0F) << 4; 
    data_vel[7] = data_vel[0]^data_vel[1]^data_vel[2]^data_vel[3]^data_vel[4]^data_vel[5]^data_vel[6];

    send_frames_[0].can_id = 0x18C4D3D0 | CAN_EFF_FLAG;
    send_frames_[0].can_dlc = 8;
    memcpy(send_frames_[0].data, data_vel, 8);
    write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));

    unsigned char data_ang[8] = {0};
    short ang_lf = (short)(current_front_angle_cmd_.free_ctrl_cmd_angle_lf * 100);
    short ang_rf = (short)(current_front_angle_cmd_.free_ctrl_cmd_angle_rf * 100);

    data_ang[0] = (0x0f & current_front_angle_cmd_.ctrl_cmd_gear) | ((ang_lf & 0x0f) << 4);
    data_ang[1] = (ang_lf >> 4) & 0xff;
    data_ang[2] = ((ang_lf >> 12) & 0x0f) | ((ang_rf & 0x0f) << 4);
    data_ang[3] = (ang_rf >> 4) & 0xff;
    data_ang[4] = (ang_rf >> 12) & 0x0f;

    data_ang[6] = (count_free_f & 0x0F) << 4;
    data_ang[7] = data_ang[0]^data_ang[1]^data_ang[2]^data_ang[3]^data_ang[4]^data_ang[5]^data_ang[6];

    send_frames_[0].can_id = 0x18C4D5D0 | CAN_EFF_FLAG;
    send_frames_[0].can_dlc = 8;
    memcpy(send_frames_[0].data, data_ang, 8);
    write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));

    count_free_f++;
    if (count_free_f > 15) count_free_f = 0;
  }

  void CanControl::front_velocity_free_ctrl_cmdCallBack(const yhs_can_msgs::front_velocity_free_ctrl_cmd msg)
  {
    boost::mutex::scoped_lock lock(cmd_mutex_);
    current_front_vel_cmd_ = msg;
    
    current_control_mode_ = MODE_FREE_CTRL_FRONT;
  }

  void CanControl::front_angle_free_ctrl_cmdCallBack(const yhs_can_msgs::front_angle_free_ctrl_cmd msg)
  {
    boost::mutex::scoped_lock lock(cmd_mutex_);
    current_front_angle_cmd_ = msg;
    
    current_control_mode_ = MODE_FREE_CTRL_FRONT;
  }

  void CanControl::sendRearFreeCtrlCmd()
  {
    static unsigned char count_free_r = 0;

    unsigned char data_vel[8] = {0};
    short vel_lr = (short)(current_rear_vel_cmd_.free_ctrl_cmd_velocity_lr * 1000);
    short vel_rr = (short)(current_rear_vel_cmd_.free_ctrl_cmd_velocity_rr * 1000);

    data_vel[0] = (0x0f & current_rear_vel_cmd_.ctrl_cmd_gear) | ((vel_lr & 0x0f) << 4);
    data_vel[1] = (vel_lr >> 4) & 0xff;
    data_vel[2] = ((vel_lr >> 12) & 0x0f) | ((vel_rr & 0x0f) << 4);
    data_vel[3] = (vel_rr >> 4) & 0xff;
    data_vel[4] = (vel_rr >> 12) & 0x0f;
    
    data_vel[6] = (count_free_r & 0x0F) << 4;
    data_vel[7] = data_vel[0]^data_vel[1]^data_vel[2]^data_vel[3]^data_vel[4]^data_vel[5]^data_vel[6];

    send_frames_[0].can_id = 0x18C4D4D0 | CAN_EFF_FLAG;
    send_frames_[0].can_dlc = 8;
    memcpy(send_frames_[0].data, data_vel, 8);
    write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));

    unsigned char data_ang[8] = {0};
    short ang_lr = (short)(current_rear_angle_cmd_.free_ctrl_cmd_angle_lr * 100);
    short ang_rr = (short)(current_rear_angle_cmd_.free_ctrl_cmd_angle_rr * 100);

    data_ang[0] = (0x0f & current_rear_angle_cmd_.ctrl_cmd_gear) | ((ang_lr & 0x0f) << 4);
    data_ang[1] = (ang_lr >> 4) & 0xff;
    data_ang[2] = ((ang_lr >> 12) & 0x0f) | ((ang_rr & 0x0f) << 4);
    data_ang[3] = (ang_rr >> 4) & 0xff;
    data_ang[4] = (ang_rr >> 12) & 0x0f;

    data_ang[6] = (count_free_r & 0x0F) << 4;
    data_ang[7] = data_ang[0]^data_ang[1]^data_ang[2]^data_ang[3]^data_ang[4]^data_ang[5]^data_ang[6];

    send_frames_[0].can_id = 0x18C4D6D0 | CAN_EFF_FLAG;
    send_frames_[0].can_dlc = 8;
    memcpy(send_frames_[0].data, data_ang, 8);
    write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));

    count_free_r++;
    if (count_free_r > 15) count_free_r = 0;
  }

  void CanControl::rear_velocity_free_ctrl_cmdCallBack(const yhs_can_msgs::rear_velocity_free_ctrl_cmd msg)
  {
    boost::mutex::scoped_lock lock(cmd_mutex_);
    current_rear_vel_cmd_ = msg;
    
    current_control_mode_ = MODE_FREE_CTRL_REAR;
  }

  void CanControl::rear_angle_free_ctrl_cmdCallBack(const yhs_can_msgs::rear_angle_free_ctrl_cmd msg)
  {
    boost::mutex::scoped_lock lock(cmd_mutex_);
    current_rear_angle_cmd_ = msg;
    
    current_control_mode_ = MODE_FREE_CTRL_REAR;
  }

  void CanControl::recvData()
  {
    while (ros::ok())
    {
      if (read(dev_handler_, &recv_frames_[0], sizeof(recv_frames_[0])) >= 0)
      {
        for (int j = 0; j < 1; j++)
        {
          switch (recv_frames_[0].can_id)
          {
            case 0x18C4D1EF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::ctrl_fb msg;
              uint8_t* d = recv_frames_[0].data;

              msg.ctrl_fb_gear = recv_frames_[0].data[0] & 0x0f;

              int x_raw = ((recv_frames_[0].data[0] & 0xf0) >> 4) | (recv_frames_[0].data[1] << 4) | ((recv_frames_[0].data[2] & 0x0f) << 12);
              if (x_raw & 0x8000) 
                x_raw |= 0xFFFF0000;
              msg.ctrl_fb_x_linear = (float)x_raw / 1000.0f;

              int z_raw = ((recv_frames_[0].data[2] & 0xf0) >> 4) | (recv_frames_[0].data[3] << 4) | ((recv_frames_[0].data[4] & 0x0f) << 12);
              if (z_raw & 0x8000) 
                z_raw |= 0xFFFF0000;
              msg.ctrl_fb_z_angular = (float)z_raw / 100.0f; // deg/s

              int y_raw = ((recv_frames_[0].data[4] & 0xf0) >> 4) | (recv_frames_[0].data[5] << 4) | ((recv_frames_[0].data[6] & 0x0f) << 12);
              if (y_raw & 0x8000) 
                y_raw |= 0xFFFF0000;
              msg.ctrl_fb_y_linear = (float)y_raw / 1000.0f;

              unsigned char crc = recv_frames_[0].data[0]^recv_frames_[0].data[1]^recv_frames_[0].data[2]^recv_frames_[0].data[3]^recv_frames_[0].data[4]^recv_frames_[0].data[5]^recv_frames_[0].data[6];
              if (crc == recv_frames_[0].data[7])
              {
                ctrl_fb_pub_.publish(msg);
                
                float ang_rad = msg.ctrl_fb_z_angular * M_PI / 180.0f;
                OdomPub(msg.ctrl_fb_x_linear, msg.ctrl_fb_y_linear, ang_rad);
              }
              break;
            }

            // 2. IO FB
            case 0x18C4DAEF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::io_fb msg;
              uint8_t* d = recv_frames_[0].data;

              msg.io_fb_lamp_ctrl           = (recv_frames_[0].data[0] & 0x01);
              msg.io_fb_unlock              = (recv_frames_[0].data[0] & 0x02);
              msg.io_fb_low_power_enable    = (recv_frames_[0].data[0] & 0x04);
              msg.io_fb_low_power_state     = (recv_frames_[0].data[0] & 0x08);

              msg.io_fb_lower_beam_headlamp = (recv_frames_[0].data[1] & 0x01);
              msg.io_fb_upper_beam_headlamp = (recv_frames_[0].data[1] & 0x02);
              msg.io_fb_turn_lamp           = (recv_frames_[0].data[1] & 0x0c) >> 2;
              msg.io_fb_braking_lamp        = (recv_frames_[0].data[1] & 0x10);
              msg.io_fb_clearance_lamp      = (recv_frames_[0].data[1] & 0x20);
              msg.io_fb_fog_lamp            = (recv_frames_[0].data[1] & 0x40);
              msg.io_fb_speaker             = (recv_frames_[0].data[1] & 0x80);

              msg.io_fb_low_power_ratio     = recv_frames_[0].data[2];

              // Sensors
              msg.io_fb_fl_impact_sensor    = (recv_frames_[0].data[3] & 0x01);
              msg.io_fb_fm_impact_sensor    = (recv_frames_[0].data[3] & 0x02);
              msg.io_fb_fr_impact_sensor    = (recv_frames_[0].data[3] & 0x04);
              msg.io_fb_rl_impact_sensor    = (recv_frames_[0].data[3] & 0x08);
              msg.io_fb_rm_impact_sensor    = (recv_frames_[0].data[3] & 0x10);
              msg.io_fb_rr_impact_sensor    = (recv_frames_[0].data[3] & 0x20);

              msg.io_fb_fl_drop_sensor      = (recv_frames_[0].data[4] & 0x01);
              msg.io_fb_fm_drop_sensor      = (recv_frames_[0].data[4] & 0x02);
              msg.io_fb_fr_drop_sensor      = (recv_frames_[0].data[4] & 0x04);
              msg.io_fb_rl_drop_sensor      = (recv_frames_[0].data[4] & 0x08);
              msg.io_fb_rm_drop_sensor      = (recv_frames_[0].data[4] & 0x10);
              msg.io_fb_rr_drop_sensor      = (recv_frames_[0].data[4] & 0x20);

              msg.io_fb_estop               = (recv_frames_[0].data[5] & 0x01);
              msg.io_fb_joypad_ctrl         = (recv_frames_[0].data[5] & 0x02);
              msg.io_fb_charge_state        = (recv_frames_[0].data[5] & 0x04);
              msg.io_fb_charger_sign        = (recv_frames_[0].data[5] & 0x08);
              msg.io_fb_joypad_first        = (recv_frames_[0].data[5] & 0x10);
              msg.io_fb_joypad_online       = (recv_frames_[0].data[5] & 0x20);
              unsigned char crc = recv_frames_[0].data[0]^recv_frames_[0].data[1]^recv_frames_[0].data[2]^recv_frames_[0].data[3]^recv_frames_[0].data[4]^recv_frames_[0].data[5]^recv_frames_[0].data[6];
              if (crc == recv_frames_[0].data[7]) io_fb_pub_.publish(msg);
              break;
            }

            case 0x18C4DBEF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::motor_fb msg;
              if (0x01 & recv_frames_[0].data[0])
                msg.motor_cmd_drive_enable_lf = true;
              else
                msg.motor_cmd_drive_enable_lf = false;

              if (0x02 & recv_frames_[0].data[0])
                msg.motor_cmd_drive_enable_lr = true;
              else
                msg.motor_cmd_drive_enable_lr = false;

              if (0x04 & recv_frames_[0].data[0])
                msg.motor_cmd_drive_enable_rf = true;
              else
                msg.motor_cmd_drive_enable_rf = false;

              if (0x08 & recv_frames_[0].data[0])
                msg.motor_cmd_drive_enable_rr = true;
              else
                msg.motor_cmd_drive_enable_rr = false;

              if (0x10 & recv_frames_[0].data[0])
                msg.motor_cmd_steering_enable_lf = true;
              else
                msg.motor_cmd_steering_enable_lf = false;

              if (0x20 & recv_frames_[0].data[0])
                msg.motor_cmd_steering_enable_lr = true;
              else
                msg.motor_cmd_steering_enable_lr = false;

              if (0x40 & recv_frames_[0].data[0])
                msg.motor_cmd_steering_enable_rf = true;
              else
                msg.motor_cmd_steering_enable_rf = false;

              if (0x80 & recv_frames_[0].data[0])
                msg.motor_cmd_steering_enable_rr = true;
              else
                msg.motor_cmd_steering_enable_rr = false;

              if (0x01 & recv_frames_[0].data[1])
                msg.motor_cmd_power_restart = true;
              else
                msg.motor_cmd_power_restart = false;

              unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

              if (crc == recv_frames_[0].data[7])
              {
                motor_fb_pub_.publish(msg);
              }

              break;
            }

            case 0x18C4E1EF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::bms_fb msg;
              msg.bms_fb_voltage = (float)((unsigned short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 100.0f;
              msg.bms_fb_current = (float)((short)(recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2])) / 100.0f;
              msg.bms_fb_remaining_capacity = (float)((unsigned short)(recv_frames_[0].data[5] << 8 | recv_frames_[0].data[4])) / 100.0f;
              bms_fb_pub_.publish(msg);
              break;
            }

            case 0x18C4E2EF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::bms_flag_fb msg;

              msg.bms_flag_fb_soc = recv_frames_[0].data[0];

              msg.bms_flag_fb_single_ov      = (recv_frames_[0].data[1] >> 0) & 0x01;
              msg.bms_flag_fb_single_uv      = (recv_frames_[0].data[1] >> 1) & 0x01;
              msg.bms_flag_fb_ov             = (recv_frames_[0].data[1] >> 2) & 0x01;
              msg.bms_flag_fb_uv             = (recv_frames_[0].data[1] >> 3) & 0x01;
              msg.bms_flag_fb_charge_ot      = (recv_frames_[0].data[1] >> 4) & 0x01;
              msg.bms_flag_fb_charge_ut      = (recv_frames_[0].data[1] >> 5) & 0x01;
              msg.bms_flag_fb_discharge_ot   = (recv_frames_[0].data[1] >> 6) & 0x01;
              msg.bms_flag_fb_discharge_ut   = (recv_frames_[0].data[1] >> 7) & 0x01;

              msg.bms_flag_fb_charge_oc      = (recv_frames_[0].data[2] >> 0) & 0x01;
              msg.bms_flag_fb_discharge_oc   = (recv_frames_[0].data[2] >> 1) & 0x01;
              msg.bms_flag_fb_short          = (recv_frames_[0].data[2] >> 2) & 0x01;
              msg.bms_flag_fb_ic_error       = (recv_frames_[0].data[2] >> 3) & 0x01;
              msg.bms_flag_fb_lock_mos       = (recv_frames_[0].data[2] >> 4) & 0x01;
              msg.bms_flag_fb_charge_flag    = (recv_frames_[0].data[2] >> 5) & 0x01;
              msg.bms_flag_fb_heating_flag   = (recv_frames_[0].data[2] >> 6) & 0x01;
              
              int16_t h_temp = ((recv_frames_[0].data[3] & 0xF0) >> 4) | (recv_frames_[0].data[4] << 4);
              h_temp = (int16_t)(h_temp << 4) >> 4; 
              msg.bms_flag_fb_hight_temperature = (float)h_temp * 0.1f;

              int16_t l_temp = recv_frames_[0].data[5] | ((recv_frames_[0].data[6] & 0x0F) << 8);
              l_temp = (int16_t)(l_temp << 4) >> 4;
              msg.bms_flag_fb_low_temperature = (float)l_temp * 0.1f;

              unsigned char crc = recv_frames_[0].data[0]^recv_frames_[0].data[1]^recv_frames_[0].data[2]^recv_frames_[0].data[3]^recv_frames_[0].data[4]^recv_frames_[0].data[5]^recv_frames_[0].data[6];
              if (crc == recv_frames_[0].data[7])
              {
                bms_flag_fb_pub_.publish(msg);
              }
              break;
            }

            case 0x18C4D6EF | CAN_EFF_FLAG: {
               yhs_can_msgs::lf_wheel_fb msg;
               short vel = (short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0]);
               msg.lf_wheel_fb_velocity = (float)vel / 1000.0f;
               msg.lf_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);
               lf_wheel_fb_pub_.publish(msg);
               break;
            }

            case 0x18C4D7EF | CAN_EFF_FLAG: {
               yhs_can_msgs::lr_wheel_fb msg;
               short vel = (short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0]);
               msg.lr_wheel_fb_velocity = (float)vel / 1000.0f;
               msg.lr_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);
               lr_wheel_fb_pub_.publish(msg);
               break;
            }

            case 0x18C4D8EF | CAN_EFF_FLAG: {
               yhs_can_msgs::rr_wheel_fb msg;
               short vel = (short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0]);
               msg.rr_wheel_fb_velocity = (float)vel / 1000.0f;
               msg.rr_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);
               rr_wheel_fb_pub_.publish(msg);
               break;
            }

            case 0x18C4D9EF | CAN_EFF_FLAG: {
               yhs_can_msgs::rf_wheel_fb msg;
               short vel = (short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0]);
               msg.rf_wheel_fb_velocity = (float)vel / 1000.0f;
               msg.rf_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);
               rf_wheel_fb_pub_.publish(msg);
               break;
            }

            case 0x18C4E3EF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::drive_motor_current_fb msg;
              msg.drive_motor_current_fb_lf = (float)((short) (((recv_frames_[0].data[1] & 0x0f) << 8 | recv_frames_[0].data[0]) << 4) >> 4) / 10;
              msg.drive_motor_current_fb_lr = (float)((short) ((recv_frames_[0].data[2] << 4 | (recv_frames_[0].data[1] >> 4)) << 4) >> 4) / 10;

              msg.drive_motor_current_fb_rf = (float)((short) (((recv_frames_[0].data[4] & 0x0f) << 8 | recv_frames_[0].data[3]) << 4) >> 4) / 10;
              msg.drive_motor_current_fb_rr = (float)((short) ((recv_frames_[0].data[5] << 4 | (recv_frames_[0].data[4] >> 4)) << 4) >> 4) / 10;
              if (0x01 & recv_frames_[0].data[6])
                msg.drive_motor_oc_flag_fb_lf = true;
              else
                msg.drive_motor_oc_flag_fb_lf = false;

              if (0x02 & recv_frames_[0].data[6])
                msg.drive_motor_oc_flag_fb_lr = true;
              else
                msg.drive_motor_oc_flag_fb_lr = false;

              if (0x04 & recv_frames_[0].data[6])
                msg.drive_motor_oc_flag_fb_rf = true;
              else
                msg.drive_motor_oc_flag_fb_rf = false;

              if (0x08 & recv_frames_[0].data[6])
                msg.drive_motor_oc_flag_fb_rr = true;
              else
                msg.drive_motor_oc_flag_fb_rr = false;

              unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

              if (crc == recv_frames_[0].data[7])
              {

                drive_motor_current_fb_pub_.publish(msg);
              }

              break;
            }

            case 0x18C4E4EF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::steering_motor_current_fb msg;
              msg.steering_motor_current_fb_lf = (float)((short) (((recv_frames_[0].data[1] & 0x0f) << 8 | recv_frames_[0].data[0]) << 4) >> 4) / 10;
              msg.steering_motor_current_fb_lr = (float)((short) ((recv_frames_[0].data[2] << 4 | (recv_frames_[0].data[1] >> 4)) << 4) >> 4) / 10;

              msg.steering_motor_current_fb_rf = (float)((short) (((recv_frames_[0].data[4] & 0x0f) << 8 | recv_frames_[0].data[3]) << 4) >> 4) / 10;
              msg.steering_motor_current_fb_rr = (float)((short) ((recv_frames_[0].data[5] << 4 | (recv_frames_[0].data[4] >> 4)) << 4) >> 4) / 10;
              if (0x01 & recv_frames_[0].data[6])
                msg.steering_motor_oc_flag_fb_lf = true;
              else
                msg.steering_motor_oc_flag_fb_lf = false;

              if (0x02 & recv_frames_[0].data[6])
                msg.steering_motor_oc_flag_fb_lr = true;
              else
                msg.steering_motor_oc_flag_fb_lr = false;

              if (0x04 & recv_frames_[0].data[6])
                msg.steering_motor_oc_flag_fb_rf = true;
              else
                msg.steering_motor_oc_flag_fb_rf = false;

              if (0x08 & recv_frames_[0].data[6])
                msg.steering_motor_oc_flag_fb_rr = true;
              else
                msg.steering_motor_oc_flag_fb_rr = false;
              unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

              if (crc == recv_frames_[0].data[7])
              {

                steering_motor_current_fb_pub_.publish(msg);
              }

              break;
            }

            static unsigned short ultra_data[8] = {0};
            case 0x18C4E8EF | CAN_EFF_FLAG:
            {
              ultra_data[0] = (unsigned short)((recv_frames_[0].data[1] & 0x0f) << 8 | recv_frames_[0].data[0]);
              ultra_data[1] = (unsigned short)(recv_frames_[0].data[2] << 4 | ((recv_frames_[0].data[1] & 0xf0) >> 4));

              ultra_data[2] = (unsigned short)((recv_frames_[0].data[4] & 0x0f) << 8 | recv_frames_[0].data[3]);
              ultra_data[3] = (unsigned short)(recv_frames_[0].data[5] << 4 | ((recv_frames_[0].data[4] & 0xf0) >> 4));
              break;
            }

            case 0x18C4E9EF | CAN_EFF_FLAG:
            {
              ultra_data[4] = (unsigned short)((recv_frames_[0].data[1] & 0x0f) << 8 | recv_frames_[0].data[0]);
              ultra_data[5] = (unsigned short)(recv_frames_[0].data[2] << 4 | ((recv_frames_[0].data[1] & 0xf0) >> 4));

              ultra_data[6] = (unsigned short)((recv_frames_[0].data[4] & 0x0f) << 8 | recv_frames_[0].data[3]);
              ultra_data[7] = (unsigned short)(recv_frames_[0].data[5] << 4 | ((recv_frames_[0].data[4] & 0xf0) >> 4));

              yhs_can_msgs::ultrasonic ultra_msg;

              ultra_msg.front_ultrasonic_fb_1 = ultra_data[ultrasonic_number_[0]];
              ultra_msg.front_ultrasonic_fb_2 = ultra_data[ultrasonic_number_[1]];
              ultra_msg.front_ultrasonic_fb_3 = ultra_data[ultrasonic_number_[2]];
              ultra_msg.front_ultrasonic_fb_4 = ultra_data[ultrasonic_number_[3]];

              ultra_msg.rear_ultrasonic_fb_5 = ultra_data[ultrasonic_number_[4]];
              ultra_msg.rear_ultrasonic_fb_6 = ultra_data[ultrasonic_number_[5]];
              ultra_msg.rear_ultrasonic_fb_7 = ultra_data[ultrasonic_number_[6]];
              ultra_msg.rear_ultrasonic_fb_8 = ultra_data[ultrasonic_number_[7]];

              ultrasonic_pub_.publish(ultra_msg);
            }

            case 0x18C4EAEF | CAN_EFF_FLAG:
            {
              yhs_can_msgs::error_fb msg;
              msg.error_fb_level = recv_frames_[0].data[0];

              msg.error_fb_device_type = recv_frames_[0].data[1];

              msg.error_fb_devive_id = recv_frames_[0].data[2];

              msg.error_fb_emergency_code = recv_frames_[0].data[3];

              msg.error_fb_register_code = (recv_frames_[0].data[5] << 8) | recv_frames_[0].data[4];

              unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

              if (crc == recv_frames_[0].data[7])
              {

                error_pub_.publish(msg);
              }

              break;
            }

            default:
              break;
          }
        }
      }
    }
  }

  void CanControl::ImuDataCallBack(const sensor_msgs::Imu::ConstPtr &imu_data_msg)
  {
      std::lock_guard<std::mutex> lock(mutex_);
      last_imu_time_ = ros::Time::now(); 
      tf2::Quaternion quaternion;
      tf2::fromMsg(imu_data_msg->orientation, quaternion);
      tf2::Matrix3x3(quaternion).getRPY(imu_roll_, imu_pitch_, imu_yaw_);
  }

  void CanControl::OdomPub(const float linear_x, const float linear_y, const float angular_z)
  {
    static double x = 0.0;
    static double y = 0.0;
    static double th = 0.0;
    static tf2_ros::TransformBroadcaster odom_broadcaster;
    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();

    double dt = (current_time - last_time).toSec();

    bool is_imu_active = (current_time - last_imu_time_).toSec() < 0.2;
    if (is_imu_active) {
        std::lock_guard<std::mutex> lock(mutex_);
        th = imu_yaw_; 
    } else {
        th += angular_z * dt;
    }

    // dx_world = vx * cos(th) - vy * sin(th)
    // dy_world = vx * sin(th) + vy * cos(th)
    double delta_x = (linear_x * cos(th) - linear_y * sin(th)) * dt;
    double delta_y = (linear_x * sin(th) + linear_y * cos(th)) * dt;

    x += delta_x;
    y += delta_y;

    tf2::Quaternion quat;
    if (is_imu_active) {
        std::lock_guard<std::mutex> lock(mutex_);
        quat.setRPY(imu_roll_, imu_pitch_, th);
    } else {
        quat.setRPY(0, 0, th);
    }
    geometry_msgs::Quaternion odom_quat = tf2::toMsg(quat);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odomFrame_;
    odom_trans.child_frame_id = baseFrame_;

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    if (tfUsed_) {
        odom_broadcaster.sendTransform(odom_trans);
    }

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odomFrame_;
    odom.child_frame_id = baseFrame_;

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.twist.twist.linear.x = linear_x;
    odom.twist.twist.linear.y = linear_y;
    odom.twist.twist.angular.z = angular_z;

    odom.pose.covariance[0] = 0.1; odom.pose.covariance[7] = 0.1; odom.pose.covariance[35] = 0.2;
    odom.pose.covariance[14] = 1e10; odom.pose.covariance[21] = 1e10; odom.pose.covariance[28] = 1e10;

    odom_pub_.publish(odom);
    last_time = current_time;
  }

  void CanControl::timerCallBack(const ros::TimerEvent& event)
  {
      static int loop_count = 0;
      boost::mutex::scoped_lock lock(cmd_mutex_);

      sendMotorCmd(); 

      switch (current_control_mode_)
      {
          case MODE_CTRL_CMD:
              sendCtrlCommand();
              break;
          
          case MODE_STEERING_CTRL:
              sendSteeringCtrlCmd();
              break;

          case MODE_FREE_CTRL_FRONT:
              sendFrontFreeCtrlCmd(); 
              break;

          case MODE_FREE_CTRL_REAR:
              sendRearFreeCtrlCmd();
              break;
              
          default:
              sendCtrlCommand();
              break;
      }
      // 50HZ发送IO指令
      if (loop_count % 2 == 0) {
          sendIoCommand();
      }

      loop_count++;
  }

  void CanControl::SendAndKeepAlive()
  {
      sendMotorCmd();
      sendIoCommand();
      sendCtrlCommand();
  }

  void CanControl::SendControlBurst(int repeat_count)
  {
      for(int i=0; i<repeat_count; ++i) {
          SendAndKeepAlive();
          usleep(10000); // 10ms
      }
  }

  void CanControl::run()
  {
    ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::ctrl_cmd>("ctrl_cmd", 5, &CanControl::ctrl_cmdCallBack, this);
    io_cmd_sub_ = nh_.subscribe<yhs_can_msgs::io_cmd>("io_cmd", 5, &CanControl::io_cmdCallBack, this);
    motor_cmd_sub_ = nh_.subscribe<yhs_can_msgs::motor_cmd>("motor_cmd", 5, &CanControl::motor_cmdCallBack, this);
    steering_ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::steering_ctrl_cmd>("steering_ctrl_cmd", 5, &CanControl::steering_ctrl_cmdCallBack, this);
    front_velocity_free_ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::front_velocity_free_ctrl_cmd>("front_velocity_free_ctrl_cmd", 5, &CanControl::front_velocity_free_ctrl_cmdCallBack, this);
    rear_velocity_free_ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::rear_velocity_free_ctrl_cmd>("rear_velocity_free_ctrl_cmd", 5, &CanControl::rear_velocity_free_ctrl_cmdCallBack, this);
    front_angle_free_ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::front_angle_free_ctrl_cmd>("front_angle_free_ctrl_cmd", 5, &CanControl::front_angle_free_ctrl_cmdCallBack, this);
    rear_angle_free_ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::rear_angle_free_ctrl_cmd>("rear_angle_free_ctrl_cmd", 5, &CanControl::rear_angle_free_ctrl_cmdCallBack, this);
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("imu_data", 5, &CanControl::ImuDataCallBack, this);

    ctrl_fb_pub_ = nh_.advertise<yhs_can_msgs::ctrl_fb>("ctrl_fb", 5);
    io_fb_pub_ = nh_.advertise<yhs_can_msgs::io_fb>("io_fb", 5);
    motor_fb_pub_ = nh_.advertise<yhs_can_msgs::motor_fb>("motor_fb", 5);
    lf_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::lf_wheel_fb>("lf_wheel_fb", 5);
    lr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::lr_wheel_fb>("lr_wheel_fb", 5);
    rf_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::rf_wheel_fb>("rf_wheel_fb", 5);
    rr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::rr_wheel_fb>("rr_wheel_fb", 5);
    bms_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_fb>("bms_fb", 5);
    bms_flag_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_flag_fb>("bms_flag_fb", 5);
    ultrasonic_pub_ = nh_.advertise<yhs_can_msgs::ultrasonic>("ultrasonic", 5);
    error_pub_ = nh_.advertise<yhs_can_msgs::error_fb>("error_fb", 5);
    drive_motor_current_fb_pub_ = nh_.advertise<yhs_can_msgs::drive_motor_current_fb>("drive_motor_current_fb", 5);
    steering_motor_current_fb_pub_ = nh_.advertise<yhs_can_msgs::steering_motor_current_fb>("steering_motor_current_fb", 5);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 5);

    dev_handler_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (dev_handler_ < 0) {
      ROS_ERROR(">>open can deivce error!");
      return;
    }
    struct ifreq ifr;
    strcpy(ifr.ifr_name, if_name_.c_str());
    ioctl(dev_handler_, SIOCGIFINDEX, &ifr);
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(dev_handler_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      ROS_ERROR(">>bind dev_handler error!\r\n");
      return;
    }

    ROS_INFO("Initializing Chassis State...");

    // 建立初始电平
    SendControlBurst(20); 
    bool need_falling_edge = false;

    if (motor_cmd_power_restart_) 
    {
        boost::mutex::scoped_lock lock(cmd_mutex_);
        
        current_motor_cmd_.motor_cmd_power_restart = false;
        
        ROS_INFO("Executing Motor Power Restart (Falling Edge Sequence)...");
        
        SendControlBurst(20);
    }

    boost::thread recvdata_thread(boost::bind(&CanControl::recvData, this));

    // 启动定时器 (10ms)
    timer_ = nh_.createTimer(ros::Duration(0.01), &CanControl::timerCallBack, this);

    ros::spin();
    close(dev_handler_);
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "yhs_can_control_node");
  yhs_tool::CanControl cancontrol;
  cancontrol.run();
  return 0;
}