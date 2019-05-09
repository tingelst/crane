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
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <crane_msgs/CraneState.h>
#include <crane_msgs/CraneControl.h>
#include <crane_hw_interface/crane_tip_velocity_command_interface.h>

namespace crane_controllers
{
class LyapunovPendulumDampingController
  : public controller_interface::MultiInterfaceController<crane_hw_interface::CraneTipVelocityCommandInterface>
{
public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void command(const crane_msgs::CraneControl::ConstPtr& command);

private:
  crane_hw_interface::CraneTipVelocityCommandInterface* crane_tip_velocity_command_interface_;
  crane_hw_interface::CraneTipVelocityHandle crane_tip_velocity_handle_;

  ros::Subscriber command_sub_;
  realtime_tools::RealtimeBuffer<crane_msgs::CraneControl> command_buffer_;
  ros::Subscriber pendulum_joint_state_sub_;

  realtime_tools::RealtimeBuffer<std::array<double, 4>> pendulum_joint_state_buffer_;

  std::unique_ptr<realtime_tools::RealtimePublisher<crane_msgs::CraneControl>> command_pub_;

  void pendulumJointStateCB(const sensor_msgs::JointState::ConstPtr& msg)
  {
    pendulum_joint_state_buffer_.writeFromNonRT(
        { msg->position[0], msg->velocity[0], msg->position[1], msg->velocity[1] });
  }
  void commandCB(const crane_msgs::CraneControl::ConstPtr& msg)
  {
    command_buffer_.writeFromNonRT(*msg);
  }
};
}  // namespace crane_controllers

PLUGINLIB_EXPORT_CLASS(crane_controllers::LyapunovPendulumDampingController, controller_interface::ControllerBase)
