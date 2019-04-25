#ifndef CRANE_HARDWARE_INTERFACE_CRANE_TIP_VELOCITY_COMMAND_INTERFACE_H
#define CRANE_HARDWARE_INTERFACE_CRANE_TIP_VELOCITY_COMMAND_INTERFACE_H

#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <crane_hw_interface/crane_tip_state_interface.h>

namespace crane_hw_interface
{
class CraneTipVelocityHandle : public CraneTipStateHandle
{
public:
  CraneTipVelocityHandle() : CraneTipStateHandle(), cmd_(0)
  {
  }

  CraneTipVelocityHandle(const CraneTipStateHandle& as, std::array<double, 2>* cmd) : CraneTipStateHandle(as), cmd_(cmd)
  {
    if (!cmd_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + as.getName() + "'. Command data pointer is null.");
    }
  }

  void setCommand(std::array<double, 2> command)
  {
    assert(cmd_);
    *cmd_ = command;
  }

  std::array<double, 2> getCommand() const
  {
    assert(cmd_);
    return *cmd_;
  }

  std::array<double, 2>* getCommandPtr()
  {
    return cmd_;
  }

private:
  std::array<double, 2>* cmd_;
};

class CraneTipVelocityCommandInterface : public HardwareResourceManager<CraneTipStateHandle>
{
};

}  // namespace crane_hw_interface

#endif