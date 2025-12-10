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

#include "campero_hardware/campero_hardware4WD.hpp"

#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp"

namespace
{

}

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
CamperoHardware4WD::CamperoHardware4WD() : CamperoHardwareBase()
{
}

//-----------------------------------------------------------------------------
void CamperoHardware4WD::send_command_()
{
  geometry_msgs::msg::Twist cmd;

  double left_linear_speed_ = 0.5 * (front_left_wheel_angular_speed_command_ * front_wheel_radius_ +
                                     rear_left_wheel_angular_speed_command_ * rear_wheel_radius_);

  double right_linear_speed_ =
    0.5 * (front_right_wheel_angular_speed_command_ * front_wheel_radius_ +
           rear_right_wheel_angular_speed_command_ * rear_wheel_radius_);

  cmd.linear.x = 0.5 * (left_linear_speed_ + right_linear_speed_);

  cmd.angular.z = core::SkidSteeringKinematic::computeAngularSpeed(
    left_linear_speed_, right_linear_speed_, track_);

  cmd_vel_pub_->publish(cmd);
}

}  // namespace ros2
}  // namespace romea

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(romea::ros2::CamperoHardware4WD, hardware_interface::SystemInterface)
