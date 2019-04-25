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
  , actuator_position_(n_dof_, 0.0)
  , actuator_velocity_(n_dof_, 0.0)
  , actuator_effort_(n_dof_, 0.0)
  , actuator_names_(n_dof_)
  , joint_names_(n_dof_)
  , mlpi_connection_(MLPI_INVALIDHANDLE)
  , nh_("~")
{
}

CraneHardwareInterface::~CraneHardwareInterface()
{
  MLPIRESULT result = mlpiApiDisconnect(&mlpi_connection_);
  if (MLPI_FAILED(result))
  {
    ROS_ERROR_NAMED("crane_hardware_interface", "Could not disconnect from MLPI!");
  }
}

void CraneHardwareInterface::init()
{
  // Get controller joint names from parameter server
  if (!nh_.getParam("joint_names", joint_names_))
  {
    ROS_ERROR("Cannot find required parameter 'joint_names' on the parameter server.");
    throw std::runtime_error("Cannot find required parameter 'joint_names' on the parameter server.");
  }

  if (!nh_.getParam("actuator_names", actuator_names_))
  {
    ROS_ERROR("Cannot find required parameter 'actuator_names' on the parameter server.");
    throw std::runtime_error("Cannot find required parameter 'actuator_names' on the parameter server.");
  }
  MLPIRESULT result = MLPI_S_OK;

  MlpiVersion versionInfo;
  memset(&versionInfo, 0, sizeof(versionInfo));
  result = mlpiApiGetClientCoreVersion(&versionInfo);
  if (MLPI_FAILED(result))
  {
    ROS_INFO("call of MLPI function failed with 0x%08x!", (unsigned)result);
  }
  else
  {
    ROS_INFO("Version (Major, Minor, Bugfix, Patch): %d.%d.%d.%d Build: %d", versionInfo.major, versionInfo.minor,
             versionInfo.bugfix, versionInfo.patch, versionInfo.build);
  }

  result = mlpiApiConnect(u"192.168.234.234 -user=boschrexroth -password=boschrexroth ", &mlpi_connection_);
  if (MLPI_FAILED(result))
  {
    ROS_ERROR_NAMED("crane_hw_interface", "Failed to connect to MLPI with error code: 0x%08x", result);
  }
  else
  {
    ROS_INFO_NAMED("crane_hw_interface", "Successfully connected to MLPI!");
  }

  // Create ros_control interfaces (joint state and position joint for all dof's)
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    // Joint state interface
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i],
                                                                               &joint_velocity_[i], &joint_effort_[i]));

    // Actuator state interface
    actuator_state_interface_.registerHandle(hardware_interface::ActuatorStateHandle(
        actuator_names_[i], &actuator_position_[i], &actuator_velocity_[i], &actuator_effort_[i]));

    // Joint velocity control interface
    velocity_joint_interface_.registerHandle(hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[i]), &joint_velocity_command_[i]));
  }

  // Register interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&actuator_state_interface_);
  registerInterface(&velocity_joint_interface_);

  ROS_INFO_STREAM_NAMED("crane_hw_interface", "Loaded crane hardware interface");
}  // namespace crane_hw_interface

void CraneHardwareInterface::start()
{
  ROS_INFO_NAMED("crane_hw_interface", "Started crane hardware interface...");
}

void CraneHardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
  std::array<double, 6> vars_cont;
  MLPIRESULT result = mlpiLogicReadVariableBySymbolArrayDouble(mlpi_connection_, u"Application.PlcProg.VARS_cont",
                                                               &vars_cont[0], 6, nullptr);
  if (MLPI_FAILED(result))
  {
    ROS_ERROR("Application.PlcProg.VARS_cont");
  }

  actuator_position_[0] = vars_cont[2];
  actuator_position_[1] = vars_cont[0];
  actuator_position_[2] = vars_cont[1];
  actuator_velocity_[0] = vars_cont[3];
  actuator_velocity_[1] = vars_cont[4];
  actuator_velocity_[2] = vars_cont[5];

  joint_position_[0] = actuator_position_[0] * DEG2RAD;

  double e1 = 0.154236;
  double a1 = 0.550;
  double e2 = 0.130;
  double a2 = 0.600199;
  double l = actuator_position_[1];
  double b1 = sqrt(a1 * a1 + e1 * e1);
  double b2 = sqrt(a2 * a2 + e2 * e2);
  double u = (l * l - b1 * b1 - b2 * b2) / (-2.0 * b1 * b2);
  joint_position_[1] = acos(u) + atan(e1 / a1) + atan(e2 / a2) - PI_2;
  // joint_velocity_[1] = (1.0 / sqrt(1 - u * u)) * (-l / (b1 * b2)) * joint_velocity_[1];

  e1 = 0.160;
  a1 = 0.750;
  e2 = 0.078714;
  a2 = 0.165893;
  l = actuator_position_[2];
  b1 = sqrt(a1 * a1 + e1 * e1);
  b2 = sqrt(a2 * a2 + e2 * e2);
  joint_position_[2] = acos((l * l - b1 * b1 - b2 * b2) / (-2.0 * b1 * b2)) + atan(e1 / a1) + atan(e2 / a2) + PI;

  // joint_velocity_[2] = (1.0 / sqrt(1 - u * u)) * (-l / (b1 * b2)) * joint_velocity_[2];
}

void CraneHardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
  // MLPIRESULT result =
  //     mlpiLogicWriteVariableBySymbolArrayDouble(mlpi_connection_, L"velocity_cmd", &joint_velocity_[0], n_dof_);
}

}  // namespace crane_hw_interface
