#include "campero_bridge.hpp"

//joints:
//  /$(robot)/imu/data: /campero/imu/data
//  /$(robot)/gps/nmea_sentence: /campero/gps/nmea_sentence
//  /$(robot)/vehicle_controller/odom: /campero/robotnik_base_control/odom
//  /$(robot)/vehicle_controller/cmd_vel: /campero/robotnik_base_control/cmd_vel
//  /$(robot)/front_laser/scan: /campero/front_laser/scan
//  /$(robot)/rear_laser/scan: /campero/rear_laser/scan
//  /$(robot)/joint_states: /campero/joint_states
//joints_prefix:
//  $(robot)/: campero_

//base_footprint_joint_name : base_footprint_joint
//front_left_wheel_spinning_joint_name: front_left_wheel_joint
//front_right_wheel_spinning_joint_name: front_right_wheel_joint
//rear_left_wheel_spinning_joint_name: back_left_wheel_joint
//rear_right_wheel_spinning_joint_name: back_right_wheel_joint

const std::string campero_odom_topic = "/campero/robotnik_base_control/odom";
const std::string campero_cmd_vel_topic = "/campero/robotnik_base_control/cmd_vel";
const std::string campero_joint_states_topic = "/campero/joint_states";
const std::string campero_front_laser_scan_topic = "/campero/front_laser/scan";
const std::string campero_rear_laser_scan_topic = "/campero/rear_laser/scan";

const std::string bridge_odom_topic = "/campero_bridge/vehicle_controller/odom";
const std::string bridge_cmd_vel_topic = "/campero_bridge/vehicle_controller/cmd_steer";
const std::string bridge_joint_states_topic = "/campero_bridge/vehicle_controller/joint_states";
const std::string bridge_front_laser_scan_topic = "/campero_bridge/front_laser/scan";
const std::string bridge_rear_laser_scan_topic = "/campero_bridge/rear_laser/scan";

const rclcpp::QoS data_qos= rclcpp::SensorDataQoS().reliable();
const rclcpp::QoS cmd_qos= rclcpp::QoS(rclcpp::KeepLast(1)).
    best_effort().durability_volatile();

//-----------------------------------------------------------------------------
CamperoBridge::CamperoBridge(Ros1NodePtr ros1_node_ptr,
                             Ros2NodePtr ros2_node_ptr):
  ros1_node_ptr_(ros1_node_ptr),
  ros2_node_ptr_(ros2_node_ptr)
{
}

//-----------------------------------------------------------------------------
void CamperoBridge::ros2_cmd_vel_callback_(const Ros2TwistMsg::SharedPtr ros2_msg)
{
  Ros1TwistMsg ros1_msg;
  ros1_msg.linear.x = ros2_msg->linear.x;
  ros1_msg.linear.y = ros2_msg->linear.y;
  ros1_msg.linear.z = ros2_msg->linear.z;
  ros1_cmd_vel_pub_.publish(ros1_msg);
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

  std::copy( ros1_msg->pose.covariance.begin(),
             ros1_msg->pose.covariance.end(),
             ros2_msg.pose.covariance.begin());

  ros2_msg.twist.twist.linear.x = ros1_msg->twist.twist.linear.x;
  ros2_msg.twist.twist.linear.y = ros1_msg->twist.twist.linear.y;
  ros2_msg.twist.twist.linear.z = ros1_msg->twist.twist.linear.z;
  ros2_msg.twist.twist.angular.x = ros1_msg->twist.twist.angular.x;
  ros2_msg.twist.twist.angular.y = ros1_msg->twist.twist.angular.y;
  ros2_msg.twist.twist.angular.z = ros1_msg->twist.twist.angular.z;

  std::copy( ros1_msg->twist.covariance.begin(),
             ros1_msg->twist.covariance.end(),
             ros2_msg.twist.covariance.begin());

  ros2_odom_pub_->publish(ros2_msg);
}

//-----------------------------------------------------------------------------
void CamperoBridge::ros1_joint_states_callback_(const Ros1JointStatesMsg::ConstPtr & ros1_msg)
{
  Ros2JointStatesMsg ros2_msg;
  ros2_msg.name = ros1_msg->name;
  ros2_msg.position = ros1_msg->position;
  ros2_msg.velocity = ros1_msg->velocity;
  ros2_msg.effort = ros1_msg->effort;
  ros2_joint_states_pub_->publish(ros2_msg);
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
  ros1_cmd_vel_pub_ =  ros1_node_ptr_->
      advertise<Ros1TwistMsg>(campero_cmd_vel_topic, 1);
}

//-----------------------------------------------------------------------------
void CamperoBridge::init_ros2_publishers_()
{
  ros2_odom_pub_ = ros2_node_ptr_->
      create_publisher<Ros2OdomMsg>(bridge_odom_topic, data_qos);
  ros2_joint_states_pub_ = ros2_node_ptr_->
      create_publisher<Ros2JointStatesMsg>(bridge_joint_states_topic, data_qos);
}

//-----------------------------------------------------------------------------
void CamperoBridge::init_ros1_subscriptions_()
{
  ros1_joint_states_sub_ = ros1_node_ptr_->subscribe(
        campero_joint_states_topic, 10, &CamperoBridge::ros1_joint_states_callback_,this);
  ros1_odom_sub_ = ros1_node_ptr_->subscribe(
        campero_odom_topic, 10, &CamperoBridge::ros1_odometry_callback_,this);
}

//-----------------------------------------------------------------------------
void CamperoBridge::init_ros2_subcription_()
{
  rclcpp::SubscriptionOptions options;
  options.ignore_local_publications = true;

  auto callback = std::bind(&CamperoBridge::ros2_cmd_vel_callback_,
                            this,std::placeholders::_1);

  ros2_cmd_vel_sub_ = ros2_node_ptr_->create_subscription<Ros2TwistMsg>(
        bridge_cmd_vel_topic, cmd_qos, callback,options);
}
