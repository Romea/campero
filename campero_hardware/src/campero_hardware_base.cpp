// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// std
#include <limits>
#include <string>
#include <vector>

// romea
#include "romea_common_utils/qos.hpp"
#include "romea_mobile_base_utils/ros2_control/info/hardware_info_common.hpp"

// local
#include "campero_hardware/campero_hardware_base.hpp"

// base_footprint_joint_name : base_footprint_joint
// front_left_wheel_spinning_joint_name: front_left_wheel_joint
// front_right_wheel_spinning_joint_name: front_right_wheel_joint
// rear_left_wheel_spinning_joint_name: back_left_wheel_joint
// rear_right_wheel_spinning_joint_name: back_right_wheel_joint

namespace
{

size_t joint_id(const std::vector<std::string> joint_state_names, const std::string & joint_name)
{
  auto it = std::find(joint_state_names.cbegin(), joint_state_names.cend(), joint_name);

  if (it == joint_state_names.end()) {
    throw std::runtime_error("Cannot find info of " + joint_name + " in joint_states msg");
  }

  return std::distance(joint_state_names.cbegin(), it);
}

const double & velocity(
  const sensor_msgs::msg::JointState & joint_states, const std::string & joint_name)
{
  return joint_states.velocity[joint_id(joint_states.name, joint_name)];
}

}  // namespace

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
CamperoHardwareBase::CamperoHardwareBase()
: HardwareSystemInterface<HardwareInterface4WD>(),
  front_wheel_radius_(0),
  rear_wheel_radius_(0),
  //  front_left_wheel_steering_angle_measure_(0),
  //  front_right_wheel_steering_angle_measure_(0),
  //  front_left_wheel_angular_speed_measure_(0),
  //  front_right_wheel_angular_speed_measure_(0),
  //  rear_left_wheel_angular_speed_measure_(0),
  //  rear_right_wheel_angular_speed_measure_(0),
  front_left_wheel_angular_speed_measure_(std::numeric_limits<double>::quiet_NaN()),
  front_right_wheel_angular_speed_measure_(std::numeric_limits<double>::quiet_NaN()),
  rear_left_wheel_angular_speed_measure_(std::numeric_limits<double>::quiet_NaN()),
  rear_right_wheel_angular_speed_measure_(std::numeric_limits<double>::quiet_NaN()),
  front_left_wheel_angular_speed_command_(0),
  front_right_wheel_angular_speed_command_(0),
  rear_left_wheel_angular_speed_command_(0),
  rear_right_wheel_angular_speed_command_(0)
{
#ifndef NDEBUG
  open_log_file_();
  write_log_header_();
#endif

  // std::string ns = "_" + hardware_info.name;
  // std::replace(ns.begin(), ns.end(), '_', '/');
  // RCLCPP_INFO_STREAM(rclcpp::get_logger("CamperoHardwareBase"), "hardware node name: " << ns);
  // node_ = rclcpp::Node::make_shared("hardware", ns);

  node_ = rclcpp::Node::make_shared("mobile_base_controller_bridge");
  auto callback =
    std::bind(&CamperoHardwareBase::joint_states_callback_, this, std::placeholders::_1);
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "bridge/vehicle_controller/cmd_vel", sensor_data_qos());
  joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "bridge/vehicle_controller/joint_states", best_effort(1), callback);

  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("CamperoHardwareBase"), "logger name: " << node_->get_logger().get_name());
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("CamperoHardwareBase"), "logger dir: " << rclcpp::get_logging_directory());
}

//-----------------------------------------------------------------------------
CamperoHardwareBase::~CamperoHardwareBase()
{
  // force deactive when interface has not been deactivated by controller manager but by ctrl-c
  if (lifecycle_state_.id() == 3) {
    on_deactivate(lifecycle_state_);
  }
}

//-----------------------------------------------------------------------------
hardware_interface::return_type CamperoHardwareBase::connect_()
{
  RCLCPP_INFO(rclcpp::get_logger("CamperoHardwareBase"), "Init communication with robot");

  send_null_command_();
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type CamperoHardwareBase::disconnect_()
{
  RCLCPP_INFO(rclcpp::get_logger("CamperoHardwareBase"), "Close communication with robot");

  send_null_command_();
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type CamperoHardwareBase::load_info_(
  const hardware_interface::HardwareInfo & hardware_info)
{
  try {
    // front_wheel_radius_ = get_parameter<float>(hardware_info, "front_wheel_radius");
    // rear_wheel_radius_ = get_parameter<float>(hardware_info, "rear_wheel_radius");
    // wheelbase_ = get_parameter<float>(hardware_info, "wheelbase");
    // track_ = get_parameter<float>(hardware_info, "track_");
    front_wheel_radius_ = get_front_wheel_radius(hardware_info);
    rear_wheel_radius_ = get_rear_wheel_radius(hardware_info);
    wheelbase_ = get_wheelbase(hardware_info);
    track_ = get_front_track(hardware_info);
    return hardware_interface::return_type::OK;
  } catch (std::runtime_error & e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("CamperoHardwareBase"), e.what());
    return hardware_interface::return_type::ERROR;
  }
}

//-----------------------------------------------------------------------------
void CamperoHardwareBase::send_null_command_()
{
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
}

//-----------------------------------------------------------------------------
#if ROS_DISTRO == ROS_GALACTIC
hardware_interface::return_type CamperoHardwareBase::read()
#else
hardware_interface::return_type CamperoHardwareBase::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
#endif
{
  //    RCLCPP_INFO(rclcpp::get_logger("CamperoHardwareBase"), "Read data from robot");
  rclcpp::spin_some(node_);

  try {
    set_hardware_state_();
#ifndef NDEBUG
    write_log_data_();
#endif
    return hardware_interface::return_type::OK;
  } catch (std::runtime_error & e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("CamperoHardwareBase"), e.what());
    return hardware_interface::return_type::ERROR;
  }
}

//-----------------------------------------------------------------------------
#if ROS_DISTRO == ROS_GALACTIC
hardware_interface::return_type CamperoHardwareBase::write()
#else
hardware_interface::return_type CamperoHardwareBase::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
#endif
{
  //  RCLCPP_INFO(rclcpp::get_logger("CamperoHardwareBase"), "Send command to robot");
  get_hardware_command_();
  send_command_();
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
void CamperoHardwareBase::joint_states_callback_(
  const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  front_left_wheel_angular_speed_measure_ = velocity(*msg, "front_right_wheel_joint");
  front_right_wheel_angular_speed_measure_ = velocity(*msg, "front_left_wheel_joint");
  rear_left_wheel_angular_speed_measure_ = velocity(*msg, "back_left_wheel_joint");
  rear_right_wheel_angular_speed_measure_ = velocity(*msg, "back_right_wheel_joint");
}

//-----------------------------------------------------------------------------
void CamperoHardwareBase::set_hardware_state_()
{
  core::HardwareState4WD state;
  state.frontLeftWheelSpinningMotion.velocity = front_left_wheel_angular_speed_measure_;
  state.frontRightWheelSpinningMotion.velocity = front_right_wheel_angular_speed_measure_;
  state.rearLeftWheelSpinningMotion.velocity = front_left_wheel_angular_speed_measure_;
  state.rearRightWheelSpinningMotion.velocity = front_right_wheel_angular_speed_measure_;
  this->hardware_interface_->set_feedback(state);
}

//-----------------------------------------------------------------------------
void CamperoHardwareBase::get_hardware_command_()
{
  core::HardwareCommand4WD command = hardware_interface_->get_hardware_command();
  front_left_wheel_angular_speed_command_ = command.frontLeftWheelSpinningSetPoint;
  front_right_wheel_angular_speed_command_ = command.frontRightWheelSpinningSetPoint;
  rear_left_wheel_angular_speed_command_ = command.rearLeftWheelSpinningSetPoint;
  rear_right_wheel_angular_speed_command_ = command.rearRightWheelSpinningSetPoint;
}

#ifndef NDEBUG
//-----------------------------------------------------------------------------
void CamperoHardwareBase::open_log_file_()
{
  debug_file_.open(
    std::string("campero.dat").c_str(), std::fstream::in | std::fstream::out | std::fstream::trunc);
}
//-----------------------------------------------------------------------------
void CamperoHardwareBase::write_log_header_()
{
  if (debug_file_.is_open()) {
    debug_file_ << "# time, ";
    debug_file_ << " FLS, " << " FRS, ";
    debug_file_ << " RLS, " << " RRS, ";
    debug_file_ << " FLS_cmd, " << " FRS_cmd, ";
    debug_file_ << " RLS_cmd, " << " RRS_cmd, ";
  }
}

//-----------------------------------------------------------------------------
void CamperoHardwareBase::write_log_data_()
{
  if (debug_file_.is_open()) {
    auto now = std::chrono::system_clock::now();
    auto now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);

    debug_file_ << std::setprecision(10);
    debug_file_ << now_ns.time_since_epoch().count() << " ";
    debug_file_ << front_left_wheel_angular_speed_measure_ * front_wheel_radius_ << " ";
    debug_file_ << front_right_wheel_angular_speed_measure_ * front_wheel_radius_ << " ";
    debug_file_ << rear_left_wheel_angular_speed_measure_ * rear_wheel_radius_ << " ";
    debug_file_ << rear_right_wheel_angular_speed_measure_ * rear_wheel_radius_ << " ";
    debug_file_ << front_left_wheel_angular_speed_command_ * front_wheel_radius_ << " ";
    debug_file_ << front_right_wheel_angular_speed_command_ * front_wheel_radius_ << " ";
    debug_file_ << rear_left_wheel_angular_speed_command_ * rear_wheel_radius_ << " ";
    debug_file_ << rear_right_wheel_angular_speed_command_ * rear_wheel_radius_ << " ";
  }
}
#endif

}  // namespace ros2
}  // namespace romea
