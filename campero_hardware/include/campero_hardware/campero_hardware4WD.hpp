// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef CAMPERO_HARDWARE__CAMPERO_HARDWARE4WD_HPP_
#define CAMPERO_HARDWARE__CAMPERO_HARDWARE4WD_HPP_

// romea
#include "campero_hardware_base.hpp"

namespace romea
{

class CamperoHardware4WD : public CamperoHardwareBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CamperoHardware4WD);

  CamperoHardware4WD();

  virtual ~CamperoHardware4WD() = default;

private:
  virtual void send_command_();
};

}  // namespace romea

#endif  // CAMPERO_HARDWARE__CAMPERO_HARDWARE4WD_HPP_
