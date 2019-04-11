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

#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

namespace crane_controllers
{
class CraneTipVelocityController
  : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface>
{
public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

  void command(const geometry_msgs::TwistStamped::ConstPtr& goal);

private:
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;

  bool initialized_;

  KDL::Chain kdl_chain_;
  boost::shared_ptr<KDL::ChainIkSolverVel_wdls> solver_;
  boost::shared_ptr<KDL::ChainFkSolverPos_recursive> fksolver_;
  KDL::JntArray tgt_jnt_pos_;
  KDL::JntArray tgt_jnt_vel_;
  KDL::JntArray last_tgt_jnt_vel_;

  ros::Publisher feedback_pub_;
  ros::Subscriber command_sub_;

  boost::mutex mutex_;
  KDL::Twist twist_command_;
  std::string twist_command_frame_;
  ros::Time last_command_time_;
  bool is_active_;
};
}  // namespace crane_controllers

PLUGINLIB_EXPORT_CLASS(crane_controllers::CraneTipVelocityController, controller_interface::ControllerBase)
