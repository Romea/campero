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


#include "campero_hardware/campero_hardware4WMD.hpp"
#include "romea_core_mobile_base/kinematic/omni_steering/MecanumWheelSteeringKinematic.hpp"

namespace
{

}

namespace romea
{
namespace ros2
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

  cmd.linear.x = core::MecanumWheelSteeringKinematic::
    computeLongitudinalSpeed(
    front_left_wheel_angular_speed_command_,
    front_right_wheel_angular_speed_command_,
    rear_left_wheel_angular_speed_command_,
    rear_right_wheel_angular_speed_command_);

  cmd.linear.y = core::MecanumWheelSteeringKinematic::
    computeLateralSpeed(
    front_left_wheel_angular_speed_command_,
    front_right_wheel_angular_speed_command_,
    rear_left_wheel_angular_speed_command_,
    rear_right_wheel_angular_speed_command_);

  cmd.angular.z = core::MecanumWheelSteeringKinematic::
    computeAngularSpeed(
    front_left_wheel_angular_speed_command_,
    front_right_wheel_angular_speed_command_,
    rear_left_wheel_angular_speed_command_,
    rear_right_wheel_angular_speed_command_,
    wheelbase_ / 2.,
    track_ / 2.);

  cmd_vel_pub_->publish(cmd);
}

}  // namespace ros2
}  // namespace romea

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(romea::ros2::CamperoHardware4WMD, hardware_interface::SystemInterface)
