#pragma once

#include <array>
#include <string>
#include <vector>

#include <boost/function.hpp>

#include <ros/ros.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <crane_hw_interface/crane_tip_velocity_command_interface.h>

namespace crane_controllers
{
class CraneTipVelocityForwardController
  : public controller_interface::MultiInterfaceController<crane_hw_interface::CraneTipVelocityCommandInterface>
{
public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  // void command(const geometry_msgs::TwistStamped::ConstPtr& goal);

private:
  crane_hw_interface::CraneTipVelocityCommandInterface* crane_tip_velocity_command_interface_;
  crane_hw_interface::CraneTipVelocityHandle crane_tip_velocity_handle_;

};
}  // namespace crane_controllers

PLUGINLIB_EXPORT_CLASS(crane_controllers::CraneTipVelocityForwardController, controller_interface::ControllerBase)
