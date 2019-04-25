// Copyright (c) 2019 Norwegian University of Science and Technology
// Use of this source code is governed by the MIT license, see LICENSE

// Author: Lars Tingelstad (NTNU) <lars.tingelstad@ntnu.no>

#ifndef CRANE_HW_INTERFACE
#define CRANE_HW_INTERFACE

#include <string>
#include <vector>
#include <cmath>

#include <mlpiApiLib.h>
#include <mlpiLogicLib.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

namespace crane_hw_interface
{
constexpr double PI = 3.14159265358979323846;
constexpr double PI_2 = 1.57079632679489661923;
constexpr double DEG2RAD = 0.017453292519943295;

class CraneHardwareInterface : public hardware_interface::RobotHW
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

  // Timing
  ros::Duration control_period_;
  ros::Duration elapsed_time_;
  double loop_hz_;

  // Interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::ActuatorStateInterface actuator_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  // MLPI
  MLPIHANDLE mlpi_connection_;

public:
  CraneHardwareInterface();
  ~CraneHardwareInterface();

  void init();
  void start();
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);
};

}  // namespace crane_hw_interface

#endif  // CRANE_HW_INTERFACE
