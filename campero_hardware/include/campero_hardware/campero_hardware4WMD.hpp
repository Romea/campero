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
