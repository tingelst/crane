#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <crane_controllers/crane_tip_velocity_controller.h>

namespace crane_controllers
{
bool CraneTipVelocityController::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle)
{
  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr)
  {
    ROS_ERROR("CraneTipVelocityController: Error getting velocity joint interface from hardware!");
    return false;
  }

  // Initialize KDL structures
  std::string tip_link, root_link;
  node_handle.param<std::string>("root_name", root_link, "base_link");
  node_handle.param<std::string>("tip_name", tip_link, "tip_link");

  // Load URDF
  urdf::Model model;
  if (!model.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse URDF");
    return -1;
  }

  // Load the tree
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
  {
    ROS_ERROR("Could not construct tree from URDF");
    return -1;
  }

  // Populate the Chain
  if (!kdl_tree.getChain(root_link, tip_link, kdl_chain_))
  {
    ROS_ERROR("Could not construct chain from URDF");
    return -1;
  }

  solver_.reset(new KDL::ChainIkSolverVel_wdls(kdl_chain_));
  fksolver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  unsigned num_joints = kdl_chain_.getNrOfJoints();
  tgt_jnt_pos_.resize(num_joints);
  tgt_jnt_vel_.resize(num_joints);
  last_tgt_jnt_vel_.resize(num_joints);

  // Init Joint Handles
  velocity_joint_handles_.clear();
  for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
  {
    if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
    {
      velocity_joint_handles_.push_back(
          velocity_joint_interface_->getHandle(kdl_chain_.getSegment(i).getJoint().getName()));
    }
  }

  if (velocity_joint_handles_.size() != num_joints)
  {
    ROS_ERROR("Inconsistant joint count %d, %d", num_joints, int(velocity_joint_handles_.size()));
    return false;
  }

  for (unsigned int i = 0; i < num_joints; ++i)
  {
    last_tgt_jnt_vel_(i) = 0.0;
  }

  // Subscribe to command
  command_sub_ = node_handle.subscribe<geometry_msgs::TwistStamped>("command", 1,
                                                           boost::bind(&CraneTipVelocityController::command, this, _1));
  last_command_time_ = ros::Time(0);

  initialized_ = true;

  return true;
}

void CraneTipVelocityController::starting(const ros::Time&)
{
}

void CraneTipVelocityController::update(const ros::Time&, const ros::Duration& period)
{
}

}  // namespace crane_controllers