#ifndef ALPO_HARDWARE_4WMD_HPP_
#define ALPO_HARDWARE_4WMD_HPP_

//romea
#include "campero_hardware_base.hpp"

namespace romea
{

class CamperoHardware4WMD : public CamperoHardwareBase
{
public:

  CamperoHardware4WMD();

  virtual ~CamperoHardware4WMD()=default;

private:

  virtual void send_command_();

};

}

#endif
