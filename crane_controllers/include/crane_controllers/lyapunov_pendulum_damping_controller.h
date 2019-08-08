#pragma once

// C++ standard
#include <array>
#include <string>
#include <vector>

// Boost
#include <boost/function.hpp>

// ROS
#include <ros/ros.h>

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

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// Crane
#include <crane_msgs/CraneState.h>
#include <crane_msgs/CraneControl.h>
#include <crane_msgs/CraneTrajectoryPoint.h>
#include <crane_hw_interface/crane_tip_velocity_command_interface.h>

// Casadi
#include "casadi/casadi.hpp"

namespace crane_controllers
{
class LyapunovPendulumDampingController
  : public controller_interface::MultiInterfaceController<crane_hw_interface::CraneTipVelocityCommandInterface>
{
public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;

private:
  crane_hw_interface::CraneTipVelocityCommandInterface* crane_tip_velocity_command_interface_;
  crane_hw_interface::CraneTipVelocityHandle crane_tip_velocity_handle_;

  // RML
  std::unique_ptr<ReflexxesAPI> rml_;
  std::unique_ptr<RMLPositionInputParameters> rml_input_;
  std::unique_ptr<RMLPositionOutputParameters> rml_output_;
  RMLPositionFlags rml_flags_;

  ros::Subscriber command_sub_;
  realtime_tools::RealtimeBuffer<crane_msgs::CraneTrajectoryPoint> command_buffer_;

  ros::Subscriber pendulum_joint_state_sub_;
  realtime_tools::RealtimeBuffer<std::array<double, 4>> pendulum_joint_state_buffer_;

  std::unique_ptr<realtime_tools::RealtimePublisher<crane_msgs::CraneControl>> command_pub_;

  void pendulumJointStateCB(const sensor_msgs::JointState::ConstPtr& msg)
  {
    pendulum_joint_state_buffer_.writeFromNonRT(
        { msg->position[0], msg->velocity[0], msg->position[1], msg->velocity[1] });
  }

  void commandCB(const crane_msgs::CraneTrajectoryPoint::ConstPtr& msg)
  {
    command_buffer_.writeFromNonRT(*msg);
  }

  casadi::Function continuousDynamics(void);
  casadi::Function discreteDynamics(void);

  casadi::Function solver(const std::vector<double>& z, const std::vector<double>& zref,
                          const std::vector<double>& last_g);
  casadi::Function solver_;

  std::vector<double> last_g_;
};
}  // namespace crane_controllers

PLUGINLIB_EXPORT_CLASS(crane_controllers::LyapunovPendulumDampingController, controller_interface::ControllerBase)
