#ifndef ALPO_HARDWARE_4WD_HPP_
#define ALPO_HARDWARE_4WD_HPP_

//romea
#include "campero_hardware_base.hpp"

namespace romea
{

class CamperoHardware4WD : public CamperoHardwareBase
{

public :

  RCLCPP_SHARED_PTR_DEFINITIONS(CamperoHardware4WD);

  CamperoHardware4WD();

  virtual ~CamperoHardware4WD()=default;

private :

  virtual void send_command_();

};

}

#endif
