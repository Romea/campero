// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#include "campero_hardware/campero_hardware4WD.hpp"
#include <romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp>

namespace
{

}

namespace romea
{

//-----------------------------------------------------------------------------
CamperoHardware4WD::CamperoHardware4WD()
: CamperoHardwareBase()
{
}

//-----------------------------------------------------------------------------
void CamperoHardware4WD::send_command_()
{
  geometry_msgs::msg::Twist cmd;

  double left_angular_linear_speed_ = (front_left_wheel_angular_speed_command_ +
    rear_left_wheel_angular_speed_command_);

  double right_angular_linear_speed_ = (front_right_wheel_angular_speed_command_ +
    rear_right_wheel_angular_speed_command_);

  cmd.linear.x = 0.5 * (left_angular_linear_speed_ + right_angular_linear_speed_);

  cmd.angular.z = SkidSteeringKinematic::
    computeAngularSpeed(
    left_angular_linear_speed_,
    right_angular_linear_speed_,
    track_);

  cmd_vel_pub_->publish(cmd);
}

}  // namespace romea


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(romea::CamperoHardware4WD, hardware_interface::SystemInterface)
