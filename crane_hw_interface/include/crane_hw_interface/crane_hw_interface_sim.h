// Copyright (c) 2019 Norwegian University of Science and Technology
// Use of this source code is governed by the MIT license, see LICENSE

// Author: Lars Tingelstad (NTNU) <lars.tingelstad@ntnu.no>

#ifndef CRANE_HW_INTERFACE_SIM
#define CRANE_HW_INTERFACE_SIM

#include <string>
#include <vector>
#include <cmath>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <crane_hw_interface/crane_tip_velocity_command_interface.h>

namespace crane_hw_interface
{
constexpr double PI = 3.14159265358979323846;
constexpr double PI_2 = 1.57079632679489661923;
constexpr double DEG2RAD = 0.017453292519943295;

class CraneHardwareInterfaceSim : public hardware_interface::RobotHW
{
private:
  ros::NodeHandle nh_;

  const unsigned int n_dof_ = 3;

  std::vector<std::string> joint_names_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_velocity_command_;

  std::vector<std::string> actuator_names_;
  std::vector<double> actuator_position_;
  std::vector<double> actuator_velocity_;
  std::vector<double> actuator_effort_;
  std::vector<double> actuator_velocity_command_;

  std::array<double, 2> crane_tip_position_;
  std::array<double, 2> crane_tip_velocity_;
  std::array<double, 2> crane_tip_velocity_command_;

  KDL::Chain kdl_chain_;
  boost::shared_ptr<KDL::ChainIkSolverVel_wdls> solver_;
  boost::shared_ptr<KDL::ChainFkSolverPos_recursive> fksolver_;
  std::string twist_command_frame_{"base_frame"};

  // Timing
  ros::Duration control_period_;
  ros::Duration elapsed_time_;
  double loop_hz_;

  // Interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::ActuatorStateInterface actuator_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  CraneTipStateInterface crane_tip_state_interface_;
  CraneTipVelocityCommandInterface crane_tip_velocity_command_interface_;

public:
  CraneHardwareInterfaceSim();
  ~CraneHardwareInterfaceSim();

  void init();
  void start();
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);
};

}  // namespace crane_hw_interface

#endif  // CRANE_HW_INTERFACE_SIM
