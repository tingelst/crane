#pragma once

// C++ standard
#include <array>
#include <string>
#include <vector>

// Boost
#include <boost/function.hpp>

// ROS
#include <ros/ros.h>

// actionlib
#include <actionlib/server/action_server.h>

// ROS Controls
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

// realtime_tools
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_server_goal_handle.h>

// Reflexxes
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

// Crane
#include <crane_msgs/CraneState.h>
#include <crane_msgs/CraneControl.h>
#include <crane_msgs/CraneTrajectoryPoint.h>
#include <crane_msgs/FollowCraneTrajectoryAction.h>
#include <crane_hw_interface/crane_tip_velocity_command_interface.h>

namespace crane_controllers
{
class CraneTipVelocityRMLController
  : public controller_interface::MultiInterfaceController<crane_hw_interface::CraneTipVelocityCommandInterface>
{
public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;

private:
  using ActionServer = actionlib::ActionServer<crane_msgs::FollowCraneTrajectoryAction>;
  using ActionServerPtr = std::shared_ptr<ActionServer>;
  using GoalHandle = ActionServer::GoalHandle;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<crane_msgs::FollowCraneTrajectoryAction>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;
  using CraneTrajectoryConstPtr = crane_msgs::CraneTrajectory::ConstPtr;

  crane_hw_interface::CraneTipVelocityCommandInterface* crane_tip_velocity_command_interface_;
  crane_hw_interface::CraneTipVelocityHandle crane_tip_velocity_handle_;

  // RML
  std::unique_ptr<ReflexxesAPI> rml_;
  std::unique_ptr<RMLPositionInputParameters> rml_input_;
  std::unique_ptr<RMLPositionOutputParameters> rml_output_;
  RMLPositionFlags rml_flags_;

  ros::Subscriber command_sub_;
  ActionServerPtr action_server_;
  realtime_tools::RealtimeBuffer<crane_msgs::CraneTrajectoryPoint> command_buffer_;



  void commandCB(const crane_msgs::CraneTrajectoryPointConstPtr& msg)
  {
    command_buffer_.writeFromNonRT(*msg);
  }

  virtual void goalCB(GoalHandle gh)
  {
    ROS_INFO_STREAM("Received goal!");
  }

  virtual void cancelCB(GoalHandle gh)
  {
    ROS_INFO_STREAM("Cancelled goal!");
  }
};
}  // namespace crane_controllers

PLUGINLIB_EXPORT_CLASS(crane_controllers::CraneTipVelocityRMLController, controller_interface::ControllerBase)
