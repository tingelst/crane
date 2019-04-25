// Copyright (c) 2019 Norwegian University of Science and Technology
// Use of this source code is governed by the MIT license, see LICENSE

// Author: Lars Tingelstad (NTNU) <lars.tingelstad@ntnu.no>

#ifndef CRANE_HARDWARE_INTERFACE_CRANE_TIP_VELOCITY_STATE_INTERFACE_H
#define CRANE_HARDWARE_INTERFACE_CRANE_TIP_VELOCITY_STATE_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace crane_hw_interface
{
using namespace hardware_interface;

class CraneTipStateHandle
{
public:
  CraneTipStateHandle() : name_(), pos_(0), vel_(0)
  {
  }

  CraneTipStateHandle(const std::string& name, const std::array<double, 2>* pos, const std::array<double, 2>* vel)
    : name_(name), pos_(pos), vel_(vel)
  {
    if (!pos)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Position data pointer is null.");
    }
    if (!vel)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Velocity data pointer is null.");
    }
  }

  std::string getName() const
  {
    return name_;
  }
  std::array<double, 2> getPosition() const
  {
    assert(pos_);
    return *pos_;
  }
  std::array<double, 2> getVelocity() const
  {
    assert(vel_);
    return *vel_;
  }

  const std::array<double, 2>* getPositionPtr() const
  {
    return pos_;
  }
  const std::array<double, 2>* getVelocityPtr() const
  {
    return vel_;
  }

private:
  std::string name_;
  const std::array<double, 2>* pos_;
  const std::array<double, 2>* vel_;
};

class CraneTipStateInterface : public HardwareResourceManager<CraneTipStateHandle>
{
};

}  // namespace crane_hw_interface

#endif