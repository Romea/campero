// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#include "campero_hardware/campero_hardware4WMD.hpp"
#include <romea_core_mobile_base/kinematic/omni_steering/MecanumWheelSteeringKinematic.hpp>

namespace
{

}

namespace romea
{

//-----------------------------------------------------------------------------
CamperoHardware4WMD::CamperoHardware4WMD()
: CamperoHardwareBase()
{
}

//-----------------------------------------------------------------------------
void CamperoHardware4WMD::send_command_()
{
  geometry_msgs::msg::Twist cmd;

  cmd.linear.x = MecanumWheelSteeringKinematic::
    computeLongitudinalSpeed(
    front_left_wheel_angular_speed_command_,
    front_right_wheel_angular_speed_command_,
    rear_left_wheel_angular_speed_command_,
    rear_right_wheel_angular_speed_command_);

  cmd.linear.y = MecanumWheelSteeringKinematic::
    computeLateralSpeed(
    front_left_wheel_angular_speed_command_,
    front_right_wheel_angular_speed_command_,
    rear_left_wheel_angular_speed_command_,
    rear_right_wheel_angular_speed_command_);

  cmd.angular.z = MecanumWheelSteeringKinematic::
    computeAngularSpeed(
    front_left_wheel_angular_speed_command_,
    front_right_wheel_angular_speed_command_,
    rear_left_wheel_angular_speed_command_,
    rear_right_wheel_angular_speed_command_,
    wheelbase_ / 2.,
    track_ / 2.);

  cmd_vel_pub_->publish(cmd);
}

}  // namespace romea

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(romea::CamperoHardware4WMD, hardware_interface::SystemInterface)
