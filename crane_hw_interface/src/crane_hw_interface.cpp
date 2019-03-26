// Copyright (c) 2019 Norwegian University of Science and Technology
// Use of this source code is governed by the MIT license, see LICENSE

// Author: Lars Tingelstad (NTNU) <lars.tingelstad@ntnu.no>

#include <crane_hw_interface/crane_hw_interface.h>

namespace crane_hw_interface
{
CraneHardwareInterface::CraneHardwareInterface()
  : joint_position_(n_dof_, 0.0)
  , joint_velocity_(n_dof_, 0.0)
  , joint_effort_(n_dof_, 0.0)
  , joint_velocity_command_(n_dof_, 0.0)
  , joint_names_(n_dof_)
  , mlpi_connection_(0)
{
}

CraneHardwareInterface::~CraneHardwareInterface()
{
}

void CraneHardwareInterface::init()
{
  std::string address;
  std::string user;
  std::string password;

  if (nh_.getParam("mlpi/address", address) && nh_.getParam("mlpi/user", user) &&
      nh_.getParam("mlpi/password", password))
  {
    ROS_INFO_STREAM_NAMED("crane_hw_interface", "Connecting to MLPI");

    std::stringstream ss;
    ss << address << " -user=" << user << " -password=" << password;
    std::string ss_str = ss.str();
    std::wstring connection_identifier = std::wstring(ss_str.begin(), ss_str.end());

    MLPIRESULT result = mlpiApiConnect(connection_identifier.c_str(), &mlpi_connection_);
    if (MLPI_FAILED(result))
    {
      ROS_ERROR_NAMED("crane_hw_interface", "Failed to connect to MLPI!");
    }
    else
    {
      ROS_INFO_NAMED("crane_hw_interface", "Successfully connected to MLPI!");
    }
  }

  // Get controller joint names from parameter server
  if (!nh_.getParam("controller_joint_names", joint_names_))
  {
    ROS_ERROR("Cannot find required parameter 'controller_joint_names' on the parameter server.");
    throw std::runtime_error("Cannot find required parameter 'controller_joint_names' on the parameter server.");
  }

  // Create ros_control interfaces (joint state and position joint for all dof's)
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    // Joint state interface
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i],
                                                                               &joint_velocity_[i], &joint_effort_[i]));

    // Joint velocity control interface
    velocity_joint_interface_.registerHandle(hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[i]), &joint_velocity_command_[i]));
  }

  // Register interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);

  ROS_INFO_STREAM_NAMED("crane_hw_interface", "Loaded crane hardware interface");
}

void CraneHardwareInterface::start()
{
  ROS_INFO_NAMED("crane_hw_interface", "Started crane hardware interface...");
}

void CraneHardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
  ULONG num_elements_ret = 0;
  MLPIRESULT result;
  result = mlpiLogicReadVariableBySymbolArrayDouble(mlpi_connection_, L"position", &joint_position_[0], n_dof_,
                                                    &num_elements_ret);
  result = mlpiLogicReadVariableBySymbolArrayDouble(mlpi_connection_, L"velocity", &joint_velocity_[0], n_dof_,
                                                    &num_elements_ret);
  result = mlpiLogicReadVariableBySymbolArrayDouble(mlpi_connection_, L"effort", &joint_effort_[0], n_dof_,
                                                    &num_elements_ret);
}

void CraneHardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
  MLPIRESULT result =
      mlpiLogicWriteVariableBySymbolArrayDouble(mlpi_connection_, L"velocity_cmd", &joint_velocity_[0], n_dof_);
}

}  // namespace crane_hw_interface
