// Copyright (c) 2019 Norwegian University of Science and Technology
// Use of this source code is governed by the MIT license, see LICENSE

// Author: Lars Tingelstad (NTNU) <lars.tingelstad@ntnu.no>

#include <algorithm>
#include <cstddef>

#include "crane_controllers/crane_state_controller.h"

namespace crane_controllers
{
bool CraneStateController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh)
{
  hardware_interface::JointStateInterface* js = robot_hw->get<hardware_interface::JointStateInterface>();
  if (!js)
  {
    return false;
  }

  // get all joint names from the hardware interface
  const std::vector<std::string>& joint_names = js->getNames();
  num_hw_joints_ = joint_names.size();
  for (unsigned i = 0; i < num_hw_joints_; i++)
  {
    ROS_INFO("Got joint %s", joint_names[i].c_str());
  }

  hardware_interface::ActuatorStateInterface* as = robot_hw->get<hardware_interface::ActuatorStateInterface>();
  if (!as)
  {
    return false;
  }

  crane_hw_interface::CraneTipStateInterface* cs = robot_hw->get<crane_hw_interface::CraneTipStateInterface>();
  if (!cs)
  {
    return false;
  }
  crane_tip_state_ = cs->getHandle("crane_tip");

  // get all actuator names from the hardware interface
  const std::vector<std::string>& actuator_names = as->getNames();
  num_hw_actuators_ = actuator_names.size();
  for (unsigned i = 0; i < num_hw_actuators_; i++)
  {
    ROS_INFO("Got actuator %s", actuator_names[i].c_str());
  }

  // realtime publisher
  joint_realtime_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(root_nh, "joint_states", 4));

  crane_realtime_pub_.reset(new realtime_tools::RealtimePublisher<crane_msgs::CraneState>(root_nh, "crane_states", 4));

  // get joints and allocate message
  for (unsigned i = 0; i < num_hw_joints_; i++)
  {
    joint_state_.push_back(js->getHandle(joint_names[i]));
    joint_realtime_pub_->msg_.name.push_back(joint_names[i]);
    joint_realtime_pub_->msg_.position.push_back(0.0);
    joint_realtime_pub_->msg_.velocity.push_back(0.0);
    joint_realtime_pub_->msg_.effort.push_back(0.0);
  }

  for (unsigned i = 0; i < num_hw_actuators_; i++)
  {
    actuator_state_.push_back(as->getHandle(actuator_names[i]));
  }

  joint_realtime_pub_->msg_.name.push_back("link2_cylinder1_joint");
  joint_realtime_pub_->msg_.position.push_back(0.0);
  joint_realtime_pub_->msg_.velocity.push_back(0.0);
  joint_realtime_pub_->msg_.effort.push_back(0.0);
  joint_realtime_pub_->msg_.name.push_back("actuator2");
  joint_realtime_pub_->msg_.position.push_back(0.0);
  joint_realtime_pub_->msg_.velocity.push_back(0.0);
  joint_realtime_pub_->msg_.effort.push_back(0.0);
  joint_realtime_pub_->msg_.name.push_back("link3_cylinder2_joint");
  joint_realtime_pub_->msg_.position.push_back(0.0);
  joint_realtime_pub_->msg_.velocity.push_back(0.0);
  joint_realtime_pub_->msg_.effort.push_back(0.0);
  joint_realtime_pub_->msg_.name.push_back("actuator3");
  joint_realtime_pub_->msg_.position.push_back(0.0);
  joint_realtime_pub_->msg_.velocity.push_back(0.0);
  joint_realtime_pub_->msg_.effort.push_back(0.0);
  joint_realtime_pub_->msg_.name.push_back("suspension_joint1");
  joint_realtime_pub_->msg_.position.push_back(0.0);
  joint_realtime_pub_->msg_.velocity.push_back(0.0);
  joint_realtime_pub_->msg_.effort.push_back(0.0);
  joint_realtime_pub_->msg_.name.push_back("suspension_joint2");
  joint_realtime_pub_->msg_.position.push_back(0.0);
  joint_realtime_pub_->msg_.velocity.push_back(0.0);
  joint_realtime_pub_->msg_.effort.push_back(0.0);

  return true;
}

void CraneStateController::starting(const ros::Time& time)
{
  // initialize time
  last_publish_time_ = time;
}

void CraneStateController::update(const ros::Time& time, const ros::Duration& /*period*/)
{
  // limit rate of publishing
  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
  {
    if (crane_realtime_pub_->trylock())
    {
      // populate crane state message:
      std::array<double, 2> position = crane_tip_state_.getPosition();
      std::array<double, 2> velocity = crane_tip_state_.getVelocity();
      crane_realtime_pub_->msg_.header.stamp = time;
      crane_realtime_pub_->msg_.x = position[0];
      crane_realtime_pub_->msg_.dx = velocity[0];
      crane_realtime_pub_->msg_.y = position[1];
      crane_realtime_pub_->msg_.dy = velocity[1];
      crane_realtime_pub_->msg_.phix = 0.0;
      crane_realtime_pub_->msg_.dphix = 0.0;
      crane_realtime_pub_->msg_.phiy = 0.0;
      crane_realtime_pub_->msg_.dphiy = 0.0;
      crane_realtime_pub_->unlockAndPublish();
    }

    // try to publish
    if (joint_realtime_pub_->trylock())
    {
      // we're actually publishing, so increment time
      last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);

      // populate joint state message:
      joint_realtime_pub_->msg_.header.stamp = time;
      for (unsigned i = 0; i < num_hw_joints_; i++)
      {
        joint_realtime_pub_->msg_.position[i] = joint_state_[i].getPosition();
        joint_realtime_pub_->msg_.velocity[i] = joint_state_[i].getVelocity();
        joint_realtime_pub_->msg_.effort[i] = joint_state_[i].getEffort();
      }

      // link2_cylinder1_joint
      double e1 = 0.154236;
      double a1 = 0.550;
      double e2 = 0.130;
      double a2 = 0.600199;
      double l = actuator_state_[1].getPosition();
      double b1 = sqrt(a1 * a1 + e1 * e1);
      double b2 = sqrt(a2 * a2 + e2 * e2);
      double u = (l * l - b1 * b1 - b2 * b2) / (-2.0 * b1 * b2);
      joint_realtime_pub_->msg_.position[3] = -acos((b2 * b2 - b1 * b1 - l * l) / (-2.0 * b1 * l)) - atan(a1 / e1) + PI;
      joint_realtime_pub_->msg_.velocity[3] = 0.0;
      joint_realtime_pub_->msg_.effort[3] = 0.0;

      // actuator2
      joint_realtime_pub_->msg_.position[4] = actuator_state_[1].getPosition();
      joint_realtime_pub_->msg_.velocity[4] = actuator_state_[1].getVelocity();
      joint_realtime_pub_->msg_.effort[4] = actuator_state_[1].getEffort();

      // link3_cylinder2_joint
      e1 = 0.160;
      a1 = 0.750;
      e2 = 0.078714;
      a2 = 0.165893;
      l = actuator_state_[2].getPosition();
      b1 = sqrt(a1 * a1 + e1 * e1);
      b2 = sqrt(a2 * a2 + e2 * e2);
      joint_realtime_pub_->msg_.position[5] =
          -acos((b2 * b2 - b1 * b1 - l * l) / (-2.0 * b1 * l)) - atan(a1 / e1) + PI_2;
      joint_realtime_pub_->msg_.velocity[5] = 0.0;
      joint_realtime_pub_->msg_.effort[5] = 0.0;

      // actuator3
      joint_realtime_pub_->msg_.position[6] = actuator_state_[2].getPosition();
      joint_realtime_pub_->msg_.velocity[6] = actuator_state_[2].getVelocity();
      joint_realtime_pub_->msg_.effort[6] = actuator_state_[2].getEffort();

      // suspension_joint1
      joint_realtime_pub_->msg_.position[7] = -joint_state_[1].getPosition() - joint_state_[2].getPosition();

      // suspension_joint2
      joint_realtime_pub_->msg_.position[8] = -joint_state_[0].getPosition();

      joint_realtime_pub_->unlockAndPublish();
    }
  }
}

void CraneStateController::stopping(const ros::Time& /*time*/)
{
}

}  // namespace crane_controllers

PLUGINLIB_EXPORT_CLASS(crane_controllers::CraneStateController, controller_interface::ControllerBase)
