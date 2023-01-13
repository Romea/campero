// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef CAMPERO_HARDWARE__CAMPERO_HARDWARE4WMD_HPP_
#define CAMPERO_HARDWARE__CAMPERO_HARDWARE4WMD_HPP_

// romea
#include "campero_hardware_base.hpp"

namespace romea
{

class CamperoHardware4WMD : public CamperoHardwareBase
{
public:
  CamperoHardware4WMD();

  virtual ~CamperoHardware4WMD() = default;

private:
  virtual void send_command_();
};

}  // namespace romea

#endif  // CAMPERO_HARDWARE__CAMPERO_HARDWARE4WMD_HPP_
