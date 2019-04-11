// Copyright (c) 2019 Norwegian University of Science and Technology
// Use of this source code is governed by the MIT license, see LICENSE

// Author: Lars Tingelstad (NTNU) <lars.tingelstad@ntnu.no>

#include <crane_hw_interface/crane_hw_interface_sim.h>

namespace crane_hw_interface
{
CraneHardwareInterfaceSim::CraneHardwareInterfaceSim()
  : joint_position_(n_dof_, 0.0)
  , joint_velocity_(n_dof_, 0.0)
  , joint_effort_(n_dof_, 0.0)
  , actuator_position_(n_dof_, 0.0)
  , actuator_velocity_(n_dof_, 0.0)
  , actuator_effort_(n_dof_, 0.0)
  , joint_velocity_command_(n_dof_, 0.0)
  , joint_names_(n_dof_)
{
}

CraneHardwareInterfaceSim::~CraneHardwareInterfaceSim()
{
}

void CraneHardwareInterfaceSim::init()
{
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

  ROS_INFO_STREAM_NAMED("crane_hw_interface", "Loaded simulated crane hardware interface");
}

void CraneHardwareInterfaceSim::start()
{
  ROS_INFO_NAMED("crane_hw_interface", "Started simulated crane hardware interface...");
}

void CraneHardwareInterfaceSim::read(const ros::Time& time, const ros::Duration& period)
{
  // No need to read since our write() command populates our state for us
}

void CraneHardwareInterfaceSim::write(const ros::Time& time, const ros::Duration& period)
{
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    joint_velocity_[i] = joint_velocity_command_[i];
    joint_position_[i] += joint_velocity_command_[i] * period.toSec();
  }
}

}  // namespace crane_hw_interface
