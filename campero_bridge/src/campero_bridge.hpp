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

#ifndef CAMPERO_BRIDGE_HPP_
#define CAMPERO_BRIDGE_HPP_

// include ROS 1
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

// include ROS 2
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

using Ros1TwistMsg = geometry_msgs::Twist;
using Ros2TwistMsg = geometry_msgs::msg::Twist;
using Ros1OdomMsg = nav_msgs::Odometry;
using Ros2OdomMsg = nav_msgs::msg::Odometry;
using Ros1JointStatesMsg = sensor_msgs::JointState;
using Ros2JointStatesMsg = sensor_msgs::msg::JointState;
using Ros2ScanMsg = sensor_msgs::msg::LaserScan;
using Ros1ScanMsg = sensor_msgs::LaserScan;
using Ros2JoyMsg = sensor_msgs::msg::Joy;
using Ros1JoyMsg = sensor_msgs::Joy;

using Ros1NodePtr = std::shared_ptr<ros::NodeHandle>;
using Ros2NodePtr = std::shared_ptr<rclcpp::Node>;

class CamperoBridge
{
public:
  CamperoBridge(Ros1NodePtr ros1_node_ptr, Ros2NodePtr ros2_node_ptr);

  void start();

private:
  void init_ros1_publisher_();
  void init_ros2_publishers_();
  void init_ros1_subscriptions_();
  void init_ros2_subcription_();

  void ros1_odometry_callback_(const Ros1OdomMsg::ConstPtr & ros1_msg);
  void ros1_joint_states_callback_(const Ros1JointStatesMsg::ConstPtr & ros1_msg);
  void ros1_joy_callback_(const Ros1JoyMsg::ConstPtr & ros1_msg);
  void ros1_front_laser_callback_(const Ros1ScanMsg::ConstPtr & ros1_msg);
  void ros1_rear_laser_callback_(const Ros1ScanMsg::ConstPtr & ros1_msg);

  void ros2_cmd_vel_callback_(const Ros2TwistMsg::SharedPtr ros2_msg);

private:
  Ros1NodePtr ros1_node_ptr_;
  Ros2NodePtr ros2_node_ptr_;

  ros::Publisher ros1_cmd_vel_pub_;
  ros::Subscriber ros1_joint_states_sub_;
  ros::Subscriber ros1_odom_sub_;
  ros::Subscriber ros1_joy_sub_;
  ros::Subscriber ros1_front_laser_sub_;
  ros::Subscriber ros1_rear_laser_sub_;

  rclcpp::Publisher<Ros2OdomMsg>::SharedPtr ros2_odom_pub_;
  rclcpp::Publisher<Ros2JointStatesMsg>::SharedPtr ros2_joint_states_pub_;
  rclcpp::Publisher<Ros2JoyMsg>::SharedPtr ros2_joy_pub_;
  rclcpp::Publisher<Ros2ScanMsg>::SharedPtr ros2_front_laser_pub_;
  rclcpp::Publisher<Ros2ScanMsg>::SharedPtr ros2_rear_laser_pub_;
  rclcpp::Subscription<Ros2TwistMsg>::SharedPtr ros2_cmd_vel_sub_;
};

#endif  // CAMPERO_BRIDGE_HPP_
