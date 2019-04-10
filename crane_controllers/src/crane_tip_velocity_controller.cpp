#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <crane_controllers/crane_tip_velocity_controller.h>

namespace crane_controllers
{
bool CraneTipVelocityController::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle)
{
}

void CraneTipVelocityController::starting(const ros::Time&)
{
}

void CraneTipVelocityController::update(const ros::Time&, const ros::Duration& period)
{
}

}  // namespace crane_controllers