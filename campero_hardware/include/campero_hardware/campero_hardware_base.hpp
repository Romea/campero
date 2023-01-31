// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef CAMPERO_HARDWARE__CAMPERO_HARDWARE_BASE_HPP_
#define CAMPERO_HARDWARE__CAMPERO_HARDWARE_BASE_HPP_

// std
#include <atomic>
#include <fstream>

// ros2
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

// romea
#include "romea_common_utils/ros_versions.hpp"
#include "romea_mobile_base_hardware/hardware_system_interface.hpp"

namespace romea
{

class CamperoHardwareBase : public HardwareSystemInterface<HardwareInterface4WD>
{
public:
//  RCLCPP_SHARED_PTR_DEFINITIONS(CamperoHardwareBase);

  CamperoHardwareBase();

  virtual ~CamperoHardwareBase() = default;

#if ROS_DISTRO == ROS_GALACTIC
  hardware_interface::return_type read()override;

  hardware_interface::return_type write()override;
#else
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)override;
#endif

protected:
  hardware_interface::return_type connect_() override;

  hardware_interface::return_type disconnect_() override;

  hardware_interface::return_type load_info_(
    const hardware_interface::HardwareInfo & hardware_info) override;

  void joint_states_callback_(const sensor_msgs::msg::JointState::ConstSharedPtr msg);

  virtual void send_command_() = 0;

  void send_null_command_();

  void get_hardware_command_();

  void set_hardware_state_();


#ifndef NDEBUG
  void open_log_file_();
  void write_log_header_();
  void write_log_data_();
#endif

protected:
  float front_wheel_radius_;
  float rear_wheel_radius_;
  double wheelbase_;
  double track_;


  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

  std::atomic<float> front_left_wheel_angular_speed_measure_;
  std::atomic<float> front_right_wheel_angular_speed_measure_;
  std::atomic<float> rear_left_wheel_angular_speed_measure_;
  std::atomic<float> rear_right_wheel_angular_speed_measure_;

  float front_left_wheel_angular_speed_command_;
  float front_right_wheel_angular_speed_command_;
  float rear_left_wheel_angular_speed_command_;
  float rear_right_wheel_angular_speed_command_;

#ifndef NDEBUG
  std::fstream debug_file_;
#endif
};

}  // namespace romea

#endif  // CAMPERO_HARDWARE__CAMPERO_HARDWARE_BASE_HPP_
