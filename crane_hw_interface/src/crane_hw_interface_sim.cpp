// Copyright (c) 2019 Norwegian University of Science and Technology
// Use of this source code is governed by the MIT license, see LICENSE

// Author: Lars Tingelstad (NTNU) <lars.tingelstad@ntnu.no>

#include <Eigen/LU>
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
  , actuator_names_(n_dof_)
  , crane_tip_velocity_command_({ 0.0, 0.0 })
  , nh_("~")
{
}

CraneHardwareInterfaceSim::~CraneHardwareInterfaceSim()
{
}

void CraneHardwareInterfaceSim::init()
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
  if (!nh_.getParam("initial_actuator_position", actuator_position_))
  {
    ROS_ERROR("Cannot find required parameter 'initial_actuator_position' on the parameter server.");
    throw std::runtime_error("Cannot find required parameter 'initial_actuator_position' on the parameter server.");
  }

  // Initialize KDL structures
  std::string tip_link, root_link;
  nh_.param<std::string>("root_name", root_link, "command_frame");
  nh_.param<std::string>("tip_name", tip_link, "tip_link");

  // Load URDF
  urdf::Model model;
  if (!model.initParam("robot_description"))
  {
    ROS_ERROR_STREAM_NAMED("CraneTipVelocityController", "Failed to parse URDF");
    throw std::runtime_error("CraneTipVelocityController: Failed to parse URDF");
  }
  // Load the tree
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
  {
    ROS_ERROR("Could not construct tree from URDF");
    throw std::runtime_error("Could not construct tree from URDF");
  }
  // Populate the Chain
  if (!kdl_tree.getChain(root_link, tip_link, kdl_chain_))
  {
    ROS_ERROR("Could not construct chain from URDF");
    throw std::runtime_error("Could not construct chain from URDF");
  }
  solver_.reset(new KDL::ChainIkSolverVel_wdls(kdl_chain_));
  fksolver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

  J_.resize(n_dof_);

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

  crane_tip_state_interface_.registerHandle(
      CraneTipStateHandle("crane_tip", &crane_tip_position_, &crane_tip_velocity_));

  crane_tip_velocity_command_interface_.registerHandle(
      CraneTipVelocityHandle(crane_tip_state_interface_.getHandle("crane_tip"), &crane_tip_velocity_command_));

  // Register interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&actuator_state_interface_);
  registerInterface(&velocity_joint_interface_);
  registerInterface(&crane_tip_state_interface_);
  registerInterface(&crane_tip_velocity_command_interface_);

  ROS_INFO_STREAM_NAMED("crane_hw_interface", "Loaded simulated crane hardware interface");
}

void CraneHardwareInterfaceSim::start()
{
  ROS_INFO_NAMED("crane_hw_interface", "Started simulated crane hardware interface...");
}

void CraneHardwareInterfaceSim::read(const ros::Time& time, const ros::Duration& period)
{
  joint_position_[0] = actuator_position_[0];

  double e1 = 0.154236;
  double a1 = 0.550;
  double e2 = 0.130;
  double a2 = 0.600199;
  double l = actuator_position_[1];
  double b1 = sqrt(a1 * a1 + e1 * e1);
  double b2 = sqrt(a2 * a2 + e2 * e2);
  double u = (l * l - b1 * b1 - b2 * b2) / (-2.0 * b1 * b2);
  joint_position_[1] = acos(u) + atan(e1 / a1) + atan(e2 / a2) - PI_2;

  e1 = 0.160;
  a1 = 0.750;
  e2 = 0.078714;
  a2 = 0.165893;
  l = actuator_position_[2];
  b1 = sqrt(a1 * a1 + e1 * e1);
  b2 = sqrt(a2 * a2 + e2 * e2);
  u = (l * l - b1 * b1 - b2 * b2) / (-2.0 * b1 * b2);
  joint_position_[2] = acos(u) + atan(e1 / a1) + atan(e2 / a2) - PI;

  KDL::JntArray q(n_dof_);
  KDL::JntArray qd(n_dof_);
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    q(i) = joint_position_[i];
    qd(i) = joint_velocity_[i];
  }

  KDL::Frame X;
  fksolver_->JntToCart(q, X);

  jnt_to_jac_solver_->JntToJac(q, J_);
  Eigen::Matrix<double, 3, 3> J = J_.data.topRows(3);
  Eigen::Matrix<double, 3, 1> xd = J * qd.data;

  for (std::size_t i = 0; i < 2; ++i)
  {
    crane_tip_position_[i] = X.p(i);
    crane_tip_velocity_[i] = xd(i);
  }

}

void CraneHardwareInterfaceSim::write(const ros::Time& time, const ros::Duration& period)
{
  KDL::JntArray joint_position(n_dof_);
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    joint_position(i) = joint_position_[i];
  }

  // Compute Jacobian
  jnt_to_jac_solver_->JntToJac(joint_position, J_);
  Eigen::Matrix<double, 3, 3> J = J_.data.topRows(3);

  // Initialize desired cartesian crane tip velocity
  Eigen::Matrix<double, 3, 1> xd;
  xd << crane_tip_velocity_command_[0], crane_tip_velocity_command_[1], 0.0;

  // Solve for joint velocities
  Eigen::Matrix<double, 3, 1> qd = J.inverse() * xd;

  KDL::JntArray joint_velocity_command(n_dof_);
  for (unsigned i = 0; i < n_dof_; ++i)
  {
    joint_velocity_command(i) = qd(i);
  }

  actuator_velocity_[0] = joint_velocity_command(0);
  actuator_position_[0] += actuator_velocity_[0] * period.toSec();

  double e1 = 0.154236;
  double a1 = 0.550;
  double e2 = 0.130;
  double a2 = 0.600199;
  double l = actuator_position_[1];
  double b1 = sqrt(a1 * a1 + e1 * e1);
  double b2 = sqrt(a2 * a2 + e2 * e2);
  double u = (l * l - b1 * b1 - b2 * b2) / (-2.0 * b1 * b2);
  actuator_velocity_[1] = -(1.0 / ((1.0 / sqrt(1.0 - u * u)) * (l / (-b1 * b2)))) * joint_velocity_command(1);
  actuator_position_[1] += actuator_velocity_[1] * period.toSec();

  e1 = 0.160;
  a1 = 0.750;
  e2 = 0.078714;
  a2 = 0.165893;
  l = actuator_position_[2];
  b1 = sqrt(a1 * a1 + e1 * e1);
  b2 = sqrt(a2 * a2 + e2 * e2);
  u = (l * l - b1 * b1 - b2 * b2) / (-2.0 * b1 * b2);
  actuator_velocity_[2] = -(1.0 / ((1.0 / sqrt(1.0 - u * u)) * (l / (-b1 * b2)))) * joint_velocity_command(2);
  actuator_position_[2] += actuator_velocity_[2] * period.toSec();

  for (unsigned i = 0; i < n_dof_; ++i)
  {
    joint_velocity_[i] = joint_velocity_command(i);
  }
}

}  // namespace crane_hw_interface
