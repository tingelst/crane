#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <crane_controllers/crane_tip_velocity_controller.h>

namespace crane_controllers
{
namespace internal
{
std::vector<std::string> getStrings(const ros::NodeHandle& nh, const std::string& param_name)
{
  using namespace XmlRpc;
  XmlRpcValue xml_array;
  if (!nh.getParam(param_name, xml_array))
  {
    ROS_ERROR_STREAM("Could not find '" << param_name << "' parameter (namespace: " << nh.getNamespace() << ").");
    return std::vector<std::string>();
  }
  if (xml_array.getType() != XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("The '" << param_name << "' parameter is not an array (namespace: " << nh.getNamespace() << ").");
    return std::vector<std::string>();
  }

  std::vector<std::string> out;
  for (int i = 0; i < xml_array.size(); ++i)
  {
    XmlRpc::XmlRpcValue& elem = xml_array[i];
    if (elem.getType() != XmlRpcValue::TypeString)
    {
      ROS_ERROR_STREAM("The '" << param_name << "' parameter contains a non-string element (namespace: "
                               << nh.getNamespace() << ").");
      return std::vector<std::string>();
    }
    out.push_back(static_cast<std::string>(elem));
  }
  return out;
}

urdf::ModelSharedPtr getUrdf(const ros::NodeHandle& nh, const std::string& param_name)
{
  urdf::ModelSharedPtr urdf(new urdf::Model);

  std::string urdf_str;
  // Check for robot_description in proper namespace
  if (nh.getParam(param_name, urdf_str))
  {
    if (!urdf->initString(urdf_str))
    {
      ROS_ERROR_STREAM("Failed to parse URDF contained in '"
                       << param_name << "' parameter (namespace: " << nh.getNamespace() << ").");
      return urdf::ModelSharedPtr();
    }
  }
  // Check for robot_description in root
  else if (!urdf->initParam("robot_description"))
  {
    ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter");
    return urdf::ModelSharedPtr();
  }
  return urdf;
}

std::vector<urdf::JointConstSharedPtr> getUrdfJoints(const urdf::Model& urdf,
                                                     const std::vector<std::string>& joint_names)
{
  std::vector<urdf::JointConstSharedPtr> out;
  for (const auto& joint_name : joint_names)
  {
    urdf::JointConstSharedPtr urdf_joint = urdf.getJoint(joint_name);
    if (urdf_joint)
    {
      out.push_back(urdf_joint);
    }
    else
    {
      ROS_ERROR_STREAM("Could not find joint '" << joint_name << "' in URDF model.");
      return std::vector<urdf::JointConstSharedPtr>();
    }
  }
  return out;
}

}  // namespace internal

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
  command_sub_ = node_handle.subscribe<geometry_msgs::TwistStamped>(
      "command", 1, boost::bind(&CraneTipVelocityController::command, this, _1));
  last_command_time_ = ros::Time(0);

  initialized_ = true;

  return true;
}

void CraneTipVelocityController::starting(const ros::Time&)
{
}

void CraneTipVelocityController::update(const ros::Time& now, const ros::Duration& period)
{
  // Need to initialize KDL structs
  if (!initialized_)
    return;  // Should never really hit this

  unsigned num_joints = velocity_joint_handles_.size();

  // Read joint positions from hardware interface
  for (size_t i = 0; i < num_joints; ++i)
  {
      
    joint_position_map_[joint_names_[i]] = position_joint_handles_[i].getPosition();
  }

  KDL::Frame cart_pose;
  // Copy desired twist and update time to local var to reduce lock contention
  KDL::Twist twist;
  ros::Time last_command_time;
  {
    boost::mutex::scoped_lock lock(mutex_);
    // FK is used to transform the twist command seen from end-effector frame to the one seen from body frame.
    if (fksolver_->JntToCart(tgt_jnt_pos_, cart_pose) < 0)
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


//   if ((now - last_command_time) > ros::Duration(0.5))
//   {
//     manager_->requestStop(getName());
//   }

  // change the twist here
  if (solver_->CartToJnt(tgt_jnt_pos_, twist, tgt_jnt_vel_) < 0)
  {
    for (unsigned ii = 0; ii < num_joints; ++ii)
    {
      tgt_jnt_vel_(ii) = 0.0;
    }
  }

  // Limit joint velocities by scaling all target velocities equally so resulting movement is in same direction
  double max_vel = 0.0;
  for (unsigned ii = 0; ii < num_joints; ++ii)
  {
    max_vel = std::max(std::abs(tgt_jnt_vel_(ii)), max_vel);
  }

  double joint_velocity_limit = 0.5;
  double scale = 1.0;
  if (max_vel > joint_velocity_limit)
  {
    double scale = joint_velocity_limit / max_vel;
    for (unsigned ii = 0; ii < num_joints; ++ii)
    {
      tgt_jnt_vel_(ii) *= scale;
    }
    ROS_DEBUG_THROTTLE(1.0, "Joint velocity limit reached.");
  }

  // Make sure solver didn't generate any NaNs.
  for (unsigned ii = 0; ii < num_joints; ++ii)
  {
    if (!std::isfinite(tgt_jnt_vel_(ii)))
    {
      ROS_ERROR_THROTTLE(1.0, "Target joint velocity (%d) is not finite : %f", ii, tgt_jnt_vel_(ii));
      tgt_jnt_vel_(ii) = 1.0;
    }
  }

  // Limit accelerations while trying to keep same resulting direction
  // somewhere between previous and current value
  scale = 1.0;
  double accel_limit = 1.0;
  double vel_delta_limit = accel_limit * period.toSec();
  for (unsigned ii = 0; ii < num_joints; ++ii)
  {
    double vel_delta = std::abs(tgt_jnt_vel_(ii) - last_tgt_jnt_vel_(ii));
    if (vel_delta > vel_delta_limit)
    {
      scale = std::min(scale, vel_delta_limit / vel_delta);
    }
  }

  if (scale <= 0.0)
  {
    ROS_ERROR_THROTTLE(1.0, "CartesianTwistController: acceleration limit produces non-positive scale %f", scale);
    scale = 0.0;
  }

  // Linear interpolate betwen previous velocity and new target velocity using scale.
  // scale = 1.0  final velocity = new target velocity
  // scale = 0.0  final velocity = previous velocity
  for (unsigned ii = 0; ii < num_joints; ++ii)
  {
    tgt_jnt_vel_(ii) = (tgt_jnt_vel_(ii) - last_tgt_jnt_vel_(ii)) * scale + last_tgt_jnt_vel_(ii);
  }

  // Calculate new target position of joint.  Put target position a few timesteps into the future
  double period_sec = period.toSec();
  for (unsigned ii = 0; ii < num_joints; ++ii)
  {
    tgt_jnt_pos_(ii) += tgt_jnt_vel_(ii) * period_sec;
  }

  // Limit target position of joint
  for (unsigned ii = 0; ii < num_joints; ++ii)
  {
    if (!joints_[ii]->isContinuous())
    {
      if (tgt_jnt_pos_(ii) > joints_[ii]->getPositionMax())
      {
        tgt_jnt_pos_(ii) = joints_[ii]->getPositionMax();
      }
      else if (tgt_jnt_pos_(ii) < joints_[ii]->getPositionMin())
      {
        tgt_jnt_pos_(ii) = joints_[ii]->getPositionMin();
      }
    }
  }

  for (size_t ii = 0; ii < joints_.size(); ++ii)
  {
    joints_[ii]->setPosition(tgt_jnt_pos_(ii), tgt_jnt_vel_(ii), 0.0);
    last_tgt_jnt_vel_(ii) = tgt_jnt_vel_(ii);
  }
}

void CraneTipVelocityController::command(const geometry_msgs::TwistStamped::ConstPtr& goal)
{
  // Need to initialize KDL structs
  if (!initialized_)
  {
    ROS_ERROR("CraneTipVelocityController: Cannot accept goal, controller is not initialized.");
    return;
  }

  if (goal->header.frame_id.empty())
  {
    // manager_->requestStop(getName());
    return;
  }

  KDL::Twist twist;
  twist(0) = goal->twist.linear.x;
  twist(1) = goal->twist.linear.y;
  twist(2) = goal->twist.linear.z;
  twist(3) = goal->twist.angular.x;
  twist(4) = goal->twist.angular.y;
  twist(5) = goal->twist.angular.z;

  for (int i = 0; i < 6; ++i)
  {
    if (!std::isfinite(twist(i)))
    {
      ROS_ERROR_THROTTLE(1.0, "Twist command value (%d) is not finite : %f", i, twist(i));
      twist(i) = 0.0;
    }
  }

  ros::Time now(ros::Time::now());

  {
    boost::mutex::scoped_lock lock(mutex_);
    twist_command_frame_ = goal->header.frame_id;
    twist_command_ = twist;
    last_command_time_ = now;
  }
}

}  // namespace crane_controllers