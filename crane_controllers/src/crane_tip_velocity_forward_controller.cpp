#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <crane_controllers/crane_tip_velocity_forward_controller.h>

namespace crane_controllers
{
bool CraneTipVelocityForwardController::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle)
{
  crane_tip_velocity_command_interface_ = robot_hardware->get<crane_hw_interface::CraneTipVelocityCommandInterface>();
  if (crane_tip_velocity_command_interface_ == nullptr)
  {
    ROS_ERROR("CraneTipVelocityForwardController: Error getting control interface from hardware!");
    return false;
  }

  crane_tip_velocity_handle_ = crane_tip_velocity_command_interface_->getHandle("crane_tip");

  return true;
}

void CraneTipVelocityForwardController::starting(const ros::Time&)
{
  crane_tip_velocity_handle_.setCommand({ 0.0, 0.0 });
}

void CraneTipVelocityForwardController::update(const ros::Time& now, const ros::Duration& period)
{
}

}  // namespace crane_controllers