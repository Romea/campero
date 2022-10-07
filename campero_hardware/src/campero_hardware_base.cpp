#include "campero_hardware/campero_hardware_base.hpp"

#include <romea_common_utils/qos.hpp>
#include <romea_mobile_base_hardware/hardware_info.hpp>
//#include <romea_core_mobile_base/kinematic/omni_steering/MecanumWheelSteeringKinematic.hpp>

//base_footprint_joint_name : base_footprint_joint
//front_left_wheel_spinning_joint_name: front_left_wheel_joint
//front_right_wheel_spinning_joint_name: front_right_wheel_joint
//rear_left_wheel_spinning_joint_name: back_left_wheel_joint
//rear_right_wheel_spinning_joint_name: back_right_wheel_joint

namespace  {

size_t joint_id(const std::vector<std::string> joint_state_names,
                const std::string & joint_name)
{
  auto it = std::find(joint_state_names.cbegin(),
                      joint_state_names.cend(),
                      joint_name);

  if(it==joint_state_names.end())
  {
    throw std::runtime_error("Cannot find info of "+ joint_name + " in joint_states msg");
  }

  return std::distance(joint_state_names.cbegin(),it);
}

const double & velocity(const sensor_msgs::msg::JointState & joint_states,const std::string & joint_name)
{
  return joint_states.velocity[joint_id(joint_states.name,joint_name)];
}

}

namespace romea {

//-----------------------------------------------------------------------------
CamperoHardwareBase::CamperoHardwareBase():
  HardwareSystemInterface<HardwareInterface4WD>(),
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


  node_ = rclcpp::Node::make_shared("mobile_base_controller_bridge");
  auto callback = std::bind(&CamperoHardwareBase::joint_states_callback_,this,std::placeholders::_1);
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/alpo_bridge/vehicle_controller/cmd_vel",sensor_data_qos());
  joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>("/alpo_bridge/vehicle_controller/joint_states",best_effort(1),callback);

  std::cout << " logger name"  <<node_->get_logger().get_name()<<" "<<   rclcpp::get_logging_directory() <<     std::endl;
}



//-----------------------------------------------------------------------------
hardware_interface::return_type CamperoHardwareBase::connect_()
{
  RCLCPP_ERROR(rclcpp::get_logger("CamperoHardwareBase"), "Init communication with robot");

  send_null_command_();
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type CamperoHardwareBase::disconnect_()
{
  RCLCPP_ERROR(rclcpp::get_logger("CamperoHardwareBase"), "Close communication with robot");

  send_null_command_();
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type CamperoHardwareBase::load_info_(
    const hardware_interface::HardwareInfo & hardware_info)
{

  RCLCPP_ERROR_STREAM(rclcpp::get_logger("CamperoHardwareBase"),"load_info");

  try {
    front_wheel_radius_=get_parameter<float>(hardware_info,"front_wheel_radius");
    rear_wheel_radius_=get_parameter<float>(hardware_info,"rear_wheel_radius");
    wheelbase_=get_parameter<float>(hardware_info,"wheelbase");
    track_=get_parameter<float>(hardware_info,"track_");

    return hardware_interface::return_type::OK;
  }
  catch (std::runtime_error &e)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("CamperoHardwareBase"),e.what());
    return hardware_interface::return_type::ERROR;
  }
}


//-----------------------------------------------------------------------------
void CamperoHardwareBase::send_null_command_()
{
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
}

//-----------------------------------------------------------------------------
hardware_interface::return_type CamperoHardwareBase::read()
{
  //    RCLCPP_INFO(rclcpp::get_logger("CamperoHardwareBase"), "Read data from robot");
  rclcpp::spin_some(node_);

  try {
    set_hardware_state_();
#ifndef NDEBUG
    write_log_data_();
#endif
    return hardware_interface::return_type::OK;
  }
  catch (std::runtime_error & e)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("CamperoHardwareBase"), e.what());
    return hardware_interface::return_type::ERROR;
  }
}


//-----------------------------------------------------------------------------
hardware_interface::return_type CamperoHardwareBase::write()
{
  //  RCLCPP_INFO(rclcpp::get_logger("CamperoHardwareBase"), "Send command to robot");
  send_command_();
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
void CamperoHardwareBase::joint_states_callback_(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  front_left_wheel_angular_speed_measure_=
      velocity(*msg,"front_right_wheel_joint");
  front_right_wheel_angular_speed_measure_=
      velocity(*msg,"front_left_wheel_joint");
  rear_left_wheel_angular_speed_measure_=
      velocity(*msg,"back_left_wheel_joint");
  rear_right_wheel_angular_speed_measure_=
      velocity(*msg,"back_right_wheel_joint");
}

//-----------------------------------------------------------------------------
void CamperoHardwareBase::set_hardware_state_()
{
  HardwareState4WD state;
  state.frontLeftWheelSpinningMotion.velocity = front_left_wheel_angular_speed_measure_;
  state.frontRightWheelSpinningMotion.velocity = front_right_wheel_angular_speed_measure_;
  state.rearLeftWheelSpinningMotion.velocity = front_left_wheel_angular_speed_measure_;
  state.rearRightWheelSpinningMotion.velocity = front_right_wheel_angular_speed_measure_;
  this->hardware_interface_->set_state(state);
}

//-----------------------------------------------------------------------------
void CamperoHardwareBase::get_hardware_command_()
{
  HardwareCommand4WD command = hardware_interface_->get_command();
  front_left_wheel_angular_speed_command_ = command.frontLeftWheelSpinningSetPoint;
  front_right_wheel_angular_speed_command_ = command.frontRightWheelSpinningSetPoint;
  rear_left_wheel_angular_speed_command_ = command.rearLeftWheelSpinningSetPoint;
  rear_right_wheel_angular_speed_command_ = command.rearRightWheelSpinningSetPoint;
}

#ifndef NDEBUG
//-----------------------------------------------------------------------------
void CamperoHardwareBase::open_log_file_()
{
  debug_file_.open(std::string("campero.dat").c_str(),
                   std::fstream::in|std::fstream::out|std::fstream::trunc);
}
//-----------------------------------------------------------------------------
void CamperoHardwareBase::write_log_header_()
{
  if(debug_file_.is_open())
  {
    debug_file_ <<"# time, ";
    debug_file_ <<" FLS, "<<" FRS, ";
    debug_file_ <<" RLS, "<<" RRS, ";
    debug_file_ <<" FLS_cmd, "<<" FRS_cmd, ";
    debug_file_ <<" RLS_cmd, "<<" RRS_cmd, ";
  }
}

//-----------------------------------------------------------------------------
void CamperoHardwareBase::write_log_data_()
{
  if(debug_file_.is_open())
  {
    auto now = std::chrono::system_clock::now();
    auto now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);

    debug_file_ << std::setprecision(10);
    debug_file_ << now_ns.time_since_epoch().count()<<" ";
    debug_file_ <<front_left_wheel_angular_speed_measure_*front_wheel_radius_<<" ";
    debug_file_ << front_right_wheel_angular_speed_measure_*front_wheel_radius_<<" ";
    debug_file_ <<rear_left_wheel_angular_speed_measure_*rear_wheel_radius_<<" ";
    debug_file_ << rear_right_wheel_angular_speed_measure_*rear_wheel_radius_<<" ";
    debug_file_ <<front_left_wheel_angular_speed_command_*front_wheel_radius_<<" ";
    debug_file_ << front_right_wheel_angular_speed_command_*front_wheel_radius_<<" ";
    debug_file_ <<rear_left_wheel_angular_speed_command_*rear_wheel_radius_<<" ";
    debug_file_ << rear_right_wheel_angular_speed_command_*rear_wheel_radius_<<" ";
  }
}
#endif

}
