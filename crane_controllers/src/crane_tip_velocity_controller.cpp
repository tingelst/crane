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
    ROS_ERROR_STREAM_NAMED("CraneTipVelocityController", "Failed to parse URDF");
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

  joint_position_.resize(num_joints);
  joint_velocity_.resize(num_joints);
  joint_velocity_command_.resize(num_joints);
  last_joint_velocity_command_.resize(num_joints);

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
    last_joint_velocity_command_(i) = 0.0;
  }

  // Subscribe to command
  command_sub_ = node_handle.subscribe<geometry_msgs::TwistStamped>(
      "command", 1, boost::bind(&CraneTipVelocityController::command, this, _1));
  last_command_time_ = ros::Time(0);

  return true;
}

void CraneTipVelocityController::starting(const ros::Time&)
{
}

void CraneTipVelocityController::update(const ros::Time& now, const ros::Duration& period)
{
  unsigned num_joints = velocity_joint_handles_.size();
  // Read joint positions from hardware interface
  for (size_t i = 0; i < num_joints; ++i)
  {
    joint_position_(i) = velocity_joint_handles_[i].getPosition();
    joint_velocity_(i) = velocity_joint_handles_[i].getVelocity();
  }

  KDL::Frame cart_pose;
  // Copy desired twist and update time to local var to reduce lock contention
  KDL::Twist twist;
  ros::Time last_command_time;
  {
    boost::mutex::scoped_lock lock(mutex_);
    // FK is used to transform the twist command seen from end-effector frame to the one seen from body frame.
    if (fksolver_->JntToCart(joint_position_, cart_pose) < 0)
    {
      twist.Zero();
      ROS_ERROR_THROTTLE(1.0, "FKsolver solver failed");
    }
    else
    {
      if (twist_command_frame_ == "end_effector_frame")
      {
        twist = cart_pose.M * twist_command_;
      }
      else
      {
        twist = twist_command_;
      }
    }
    last_command_time = last_command_time_;
  }

  // change the twist here
  if (solver_->CartToJnt(joint_position_, twist, joint_velocity_command_) < 0)
  {
    for (unsigned i = 0; i < num_joints; ++i)
    {
      joint_velocity_command_(i) = 0.0;
    }
  }

  // Make sure solver didn't generate any NaNs.
  for (unsigned i = 0; i < num_joints; ++i)
  {
    if (!std::isfinite(joint_velocity_command_(i)))
    {
      ROS_ERROR_THROTTLE(1.0, "Target joint velocity (%d) is not finite : %f", i, joint_velocity_command_(i));
      joint_velocity_command_(i) = 1.0;
    }
  }

  for (size_t i = 0; i < num_joints; ++i)
  {
    velocity_joint_handles_[i].setCommand(joint_velocity_command_(i));
    last_joint_velocity_command_(i) = joint_velocity_command_(i);
  }
}

void CraneTipVelocityController::command(const geometry_msgs::TwistStamped::ConstPtr& goal)
{

  if (goal->header.frame_id.empty())
  {
    return;
  }

  TwistCommand twist_command;
  twist_command(0) = goal->twist_command.linear.x;
  twist_command(1) = goal->twist_command.linear.y;
  twist_command(2) = goal->twist_command.linear.z;
  twist_command(3) = goal->twist_command.angular.x;
  twist_command(4) = goal->twist_command.angular.y;
  twist_command(5) = goal->twist_command.angular.z;

  for (int i = 0; i < 6; ++i)
  {
    if (!std::isfinite(twist_command(i)))
    {
      ROS_ERROR_THROTTLE(1.0, "Twist command value (%d) is not finite : %f", i, twist(i));
      twist(i) = 0.0;
    }
  }


  ros::Time now(ros::Time::now());

  twist_command.


  {
    boost::mutex::scoped_lock lock(mutex_);
    twist_command_frame_ = goal->header.frame_id;
    twist_command_ = twist;
    last_command_time_ = now;
  }
}

}  // namespace crane_controllers