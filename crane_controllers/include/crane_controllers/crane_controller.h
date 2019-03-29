#pragma once

#include <array>
#include <string>
#include <vector>

#include <boost/function.hpp>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/package.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>

namespace crane_controllers
{
class CraneController
  : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface>
{
public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

private:
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
};
}  // namespace crane_controllers

PLUGINLIB_EXPORT_CLASS(crane_controllers::CraneController, controller_interface::ControllerBase)
