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

#include "campero_bridge.hpp"

// joints:
//   /$(robot)/imu/data: /campero/imu/data
//   /$(robot)/gps/nmea_sentence: /campero/gps/nmea_sentence
//   /$(robot)/vehicle_controller/odom: /campero/robotnik_base_control/odom
//   /$(robot)/vehicle_controller/cmd_vel: /campero/robotnik_base_control/cmd_vel
//   /$(robot)/front_laser/scan: /campero/front_laser/scan
//   /$(robot)/rear_laser/scan: /campero/rear_laser/scan
//   /$(robot)/joint_states: /campero/joint_states
// joints_prefix:
//   $(robot)/: campero_

// base_footprint_joint_name : base_footprint_joint
// front_left_wheel_spinning_joint_name: front_left_wheel_joint
// front_right_wheel_spinning_joint_name: front_right_wheel_joint
// rear_left_wheel_spinning_joint_name: back_left_wheel_joint
// rear_right_wheel_spinning_joint_name: back_right_wheel_joint

const char campero_odom_topic[] = "/campero/robotnik_base_control/odom";
const char campero_cmd_vel_topic[] = "/campero/robotnik_base_control/cmd_vel";
const char campero_joint_states_topic[] = "/campero/joint_states";
const char campero_front_laser_scan_topic[] = "/campero/front_laser/scan";
const char campero_rear_laser_scan_topic[] = "/campero/rear_laser/scan";
const char campero_joy_topic[] = "/campero/joy";

const char bridge_odom_topic[] = "~/vehicle_controller/odom";
const char bridge_cmd_vel_topic[] = "~/vehicle_controller/cmd_vel";
const char bridge_joint_states_topic[] = "~/vehicle_controller/joint_states";
const char bridge_joy_topic[] = "~/joy";
const char bridge_front_laser_scan_topic[] = "~/front_laser/scan";
const char bridge_rear_laser_scan_topic[] = "~/rear_laser/scan";

const rclcpp::QoS data_qos = rclcpp::SensorDataQoS().reliable();
const rclcpp::QoS cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

namespace
{

void convert_laserscan(Ros2ScanMsg & ros2_msg, const Ros1ScanMsg::ConstPtr & ros1_msg)
{
  ros2_msg.header.frame_id = ros1_msg->header.frame_id;
  ros2_msg.angle_min = ros1_msg->angle_min;
  ros2_msg.angle_max = ros1_msg->angle_max;
  ros2_msg.angle_increment = ros1_msg->angle_increment;
  ros2_msg.time_increment = ros1_msg->time_increment;
  ros2_msg.scan_time = ros1_msg->scan_time;
  ros2_msg.range_min = ros1_msg->range_min;
  ros2_msg.range_max = ros1_msg->range_max;
  ros2_msg.ranges = ros1_msg->ranges;
  ros2_msg.intensities = ros1_msg->intensities;
}

}  // namespace

//-----------------------------------------------------------------------------
CamperoBridge::CamperoBridge(Ros1NodePtr ros1_node_ptr, Ros2NodePtr ros2_node_ptr)
: ros1_node_ptr_(ros1_node_ptr), ros2_node_ptr_(ros2_node_ptr)
{
}

//-----------------------------------------------------------------------------
void CamperoBridge::ros2_cmd_vel_callback_(const Ros2TwistMsg::SharedPtr ros2_msg)
{
  Ros1TwistMsg ros1_msg;
  ros1_msg.linear.x = ros2_msg->linear.x;
  ros1_msg.linear.y = ros2_msg->linear.y;
  ros1_msg.linear.z = ros2_msg->linear.z;
  ros1_msg.angular.x = ros2_msg->angular.x;
  ros1_msg.angular.y = ros2_msg->angular.y;
  ros1_msg.angular.z = ros2_msg->angular.z;
  ros1_cmd_vel_pub_.publish(ros1_msg);

  RCLCPP_INFO_STREAM_ONCE(
    ros2_node_ptr_->get_logger(), "ROS2: Received message from " << bridge_cmd_vel_topic);
}

//-----------------------------------------------------------------------------
void CamperoBridge::ros1_odometry_callback_(const Ros1OdomMsg::ConstPtr & ros1_msg)
{
  Ros2OdomMsg ros2_msg;
  ros2_msg.header.stamp = ros2_node_ptr_->get_clock()->now();
  ros2_msg.header.frame_id = ros1_msg->header.frame_id;
  ros2_msg.child_frame_id = ros1_msg->child_frame_id;

  ros2_msg.pose.pose.position.x = ros1_msg->pose.pose.position.x;
  ros2_msg.pose.pose.position.y = ros1_msg->pose.pose.position.y;
  ros2_msg.pose.pose.position.z = ros1_msg->pose.pose.position.z;
  ros2_msg.pose.pose.orientation.x = ros1_msg->pose.pose.orientation.x;
  ros2_msg.pose.pose.orientation.y = ros1_msg->pose.pose.orientation.y;
  ros2_msg.pose.pose.orientation.z = ros1_msg->pose.pose.orientation.z;
  ros2_msg.pose.pose.orientation.w = ros1_msg->pose.pose.orientation.w;

  std::copy(
    ros1_msg->pose.covariance.begin(),
    ros1_msg->pose.covariance.end(),
    ros2_msg.pose.covariance.begin());

  ros2_msg.twist.twist.linear.x = ros1_msg->twist.twist.linear.x;
  ros2_msg.twist.twist.linear.y = ros1_msg->twist.twist.linear.y;
  ros2_msg.twist.twist.linear.z = ros1_msg->twist.twist.linear.z;
  ros2_msg.twist.twist.angular.x = ros1_msg->twist.twist.angular.x;
  ros2_msg.twist.twist.angular.y = ros1_msg->twist.twist.angular.y;
  ros2_msg.twist.twist.angular.z = ros1_msg->twist.twist.angular.z;

  std::copy(
    ros1_msg->twist.covariance.begin(),
    ros1_msg->twist.covariance.end(),
    ros2_msg.twist.covariance.begin());

  ros2_odom_pub_->publish(ros2_msg);

  RCLCPP_INFO_STREAM_ONCE(
    ros2_node_ptr_->get_logger(), "ROS1: Received message from " << campero_odom_topic);
}

//-----------------------------------------------------------------------------
void CamperoBridge::ros1_joint_states_callback_(const Ros1JointStatesMsg::ConstPtr & ros1_msg)
{
  Ros2JointStatesMsg ros2_msg;
  ros2_msg.header.stamp = ros2_node_ptr_->get_clock()->now();
  ros2_msg.header.frame_id = ros1_msg->header.frame_id;
  ros2_msg.name = ros1_msg->name;
  ros2_msg.position = ros1_msg->position;
  ros2_msg.velocity = ros1_msg->velocity;
  ros2_msg.effort = ros1_msg->effort;
  ros2_joint_states_pub_->publish(ros2_msg);

  RCLCPP_INFO_STREAM_ONCE(
    ros2_node_ptr_->get_logger(), "ROS1: Received message from " << campero_joint_states_topic);
}

void CamperoBridge::ros1_joy_callback_(const Ros1JoyMsg::ConstPtr & ros1_msg)
{
  Ros2JoyMsg ros2_msg;
  ros2_msg.header.stamp = ros2_node_ptr_->get_clock()->now();
  ros2_msg.header.frame_id = ros1_msg->header.frame_id;
  ros2_msg.axes = ros1_msg->axes;
  ros2_msg.buttons = ros1_msg->buttons;
  ros2_joy_pub_->publish(ros2_msg);

  RCLCPP_INFO_STREAM_ONCE(
    ros2_node_ptr_->get_logger(), "ROS1: Received message from " << campero_joy_topic);
}

void CamperoBridge::ros1_front_laser_callback_(const Ros1ScanMsg::ConstPtr & ros1_msg)
{
  Ros2ScanMsg ros2_msg;
  ros2_msg.header.stamp = ros2_node_ptr_->get_clock()->now();
  convert_laserscan(ros2_msg, ros1_msg);
  ros2_front_laser_pub_->publish(ros2_msg);

  RCLCPP_INFO_STREAM_ONCE(
    ros2_node_ptr_->get_logger(), "ROS1: Received message from " << campero_front_laser_scan_topic);
}

void CamperoBridge::ros1_rear_laser_callback_(const Ros1ScanMsg::ConstPtr & ros1_msg)
{
  Ros2ScanMsg ros2_msg;
  ros2_msg.header.stamp = ros2_node_ptr_->get_clock()->now();
  convert_laserscan(ros2_msg, ros1_msg);
  ros2_rear_laser_pub_->publish(ros2_msg);

  RCLCPP_INFO_STREAM_ONCE(
    ros2_node_ptr_->get_logger(), "ROS1: Received message from " << campero_rear_laser_scan_topic);
}

//-----------------------------------------------------------------------------
void CamperoBridge::start()
{
  init_ros1_publisher_();
  init_ros2_publishers_();
  init_ros1_subscriptions_();
  init_ros2_subcription_();
}

//-----------------------------------------------------------------------------
void CamperoBridge::init_ros1_publisher_()
{
  ros1_cmd_vel_pub_ = ros1_node_ptr_->advertise<Ros1TwistMsg>(campero_cmd_vel_topic, 1);
}

//-----------------------------------------------------------------------------
void CamperoBridge::init_ros2_publishers_()
{
  ros2_odom_pub_ = ros2_node_ptr_->create_publisher<Ros2OdomMsg>(bridge_odom_topic, data_qos);
  ros2_joint_states_pub_ =
    ros2_node_ptr_->create_publisher<Ros2JointStatesMsg>(bridge_joint_states_topic, data_qos);
  ros2_joy_pub_ = ros2_node_ptr_->create_publisher<Ros2JoyMsg>(bridge_joy_topic, data_qos);
  ros2_front_laser_pub_ =
    ros2_node_ptr_->create_publisher<Ros2ScanMsg>(bridge_front_laser_scan_topic, data_qos);
  ros2_rear_laser_pub_ =
    ros2_node_ptr_->create_publisher<Ros2ScanMsg>(bridge_rear_laser_scan_topic, data_qos);
  RCLCPP_INFO_STREAM(
    ros2_node_ptr_->get_logger(),
    "Create pub 2 <- 1: " << bridge_odom_topic << " <- " << campero_odom_topic);
  RCLCPP_INFO_STREAM(
    ros2_node_ptr_->get_logger(),
    "Create pub 2 <- 1: " << bridge_joint_states_topic << " <- " << campero_joint_states_topic);
  RCLCPP_INFO_STREAM(
    ros2_node_ptr_->get_logger(),
    "Create pub 2 <- 1: " << bridge_joy_topic << " <- " << campero_joy_topic);
  RCLCPP_INFO_STREAM(
    ros2_node_ptr_->get_logger(),
    "Create pub 2 <- 1: " << bridge_front_laser_scan_topic << " <- "
                          << campero_front_laser_scan_topic);
  RCLCPP_INFO_STREAM(
    ros2_node_ptr_->get_logger(),
    "Create pub 2 <- 1: " << bridge_rear_laser_scan_topic << " <- "
                          << campero_rear_laser_scan_topic);
}

//-----------------------------------------------------------------------------
void CamperoBridge::init_ros1_subscriptions_()
{
  ros1_joint_states_sub_ = ros1_node_ptr_->subscribe(
    campero_joint_states_topic, 10, &CamperoBridge::ros1_joint_states_callback_, this);
  ros1_odom_sub_ = ros1_node_ptr_->subscribe(
    campero_odom_topic, 10, &CamperoBridge::ros1_odometry_callback_, this);
  ros1_joy_sub_ =
    ros1_node_ptr_->subscribe(campero_joy_topic, 3, &CamperoBridge::ros1_joy_callback_, this);
  ros1_front_laser_sub_ = ros1_node_ptr_->subscribe(
    campero_front_laser_scan_topic, 3, &CamperoBridge::ros1_front_laser_callback_, this);
  ros1_rear_laser_sub_ = ros1_node_ptr_->subscribe(
    campero_rear_laser_scan_topic, 3, &CamperoBridge::ros1_rear_laser_callback_, this);
}

//-----------------------------------------------------------------------------
void CamperoBridge::init_ros2_subcription_()
{
  rclcpp::SubscriptionOptions options;
  options.ignore_local_publications = true;

  auto callback = std::bind(&CamperoBridge::ros2_cmd_vel_callback_, this, std::placeholders::_1);

  ros2_cmd_vel_sub_ = ros2_node_ptr_->create_subscription<Ros2TwistMsg>(
    bridge_cmd_vel_topic, cmd_qos, callback, options);

  RCLCPP_INFO_STREAM(
    ros2_node_ptr_->get_logger(),
    "Create sub 2 -> 1: " << bridge_cmd_vel_topic << " -> " << campero_cmd_vel_topic);
}
