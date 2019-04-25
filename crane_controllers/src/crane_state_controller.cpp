///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2019, Norwegian University of Science and Technology (NTNU)
// Copyright (C) 2012, hiDOF INC.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF Inc nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/*
 * Author: Wim Meeussen, Lars Tingelstad
 */

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

  // get all actuator names from the hardware interface
  const std::vector<std::string>& actuator_names = as->getNames();
  num_hw_actuators_ = actuator_names.size();
  for (unsigned i = 0; i < num_hw_actuators_; i++)
  {
    ROS_INFO("Got actuator %s", actuator_names[i].c_str());
  }

  // realtime publisher
  realtime_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(root_nh, "/joint_states", 4));

  // get joints and allocate message
  for (unsigned i = 0; i < num_hw_joints_; i++)
  {
    joint_state_.push_back(js->getHandle(joint_names[i]));
    realtime_pub_->msg_.name.push_back(joint_names[i]);
    realtime_pub_->msg_.position.push_back(0.0);
    realtime_pub_->msg_.velocity.push_back(0.0);
    realtime_pub_->msg_.effort.push_back(0.0);
  }

  for (unsigned i = 0; i < num_hw_actuators_; i++)
  {
    actuator_state_.push_back(as->getHandle(actuator_names[i]));
  }

  realtime_pub_->msg_.name.push_back("link2_cylinder1_joint");
  realtime_pub_->msg_.position.push_back(0.0);
  realtime_pub_->msg_.velocity.push_back(0.0);
  realtime_pub_->msg_.effort.push_back(0.0);
  realtime_pub_->msg_.name.push_back("actuator2");
  realtime_pub_->msg_.position.push_back(0.0);
  realtime_pub_->msg_.velocity.push_back(0.0);
  realtime_pub_->msg_.effort.push_back(0.0);
  realtime_pub_->msg_.name.push_back("link3_cylinder2_joint");
  realtime_pub_->msg_.position.push_back(0.0);
  realtime_pub_->msg_.velocity.push_back(0.0);
  realtime_pub_->msg_.effort.push_back(0.0);
  realtime_pub_->msg_.name.push_back("actuator3");
  realtime_pub_->msg_.position.push_back(0.0);
  realtime_pub_->msg_.velocity.push_back(0.0);
  realtime_pub_->msg_.effort.push_back(0.0);

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
    // try to publish
    if (realtime_pub_->trylock())
    {
      // we're actually publishing, so increment time
      last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);

      // populate joint state message:
      realtime_pub_->msg_.header.stamp = time;
      for (unsigned i = 0; i < num_hw_joints_; i++)
      {
        realtime_pub_->msg_.position[i] = joint_state_[i].getPosition();
        realtime_pub_->msg_.velocity[i] = joint_state_[i].getVelocity();
        realtime_pub_->msg_.effort[i] = joint_state_[i].getEffort();
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
      realtime_pub_->msg_.position[3] = -acos((b2*b2 - b1*b1 - l*l) / (-2.0*b1*l)) - atan(a1/e1) + PI;
      realtime_pub_->msg_.velocity[3] = 0.0;
      realtime_pub_->msg_.effort[3] = 0.0;

      // actuator2
      realtime_pub_->msg_.position[4] = actuator_state_[1].getPosition();
      realtime_pub_->msg_.velocity[4] = actuator_state_[1].getVelocity();
      realtime_pub_->msg_.effort[4] = actuator_state_[1].getEffort();

      // link3_cylinder2_joint
      e1 = 0.160;
      a1 = 0.750;
      e2 = 0.078714;
      a2 = 0.165893;
      l = actuator_state_[2].getPosition();
      b1 = sqrt(a1 * a1 + e1 * e1);
      b2 = sqrt(a2 * a2 + e2 * e2);
      realtime_pub_->msg_.position[5] = -acos((b2*b2 - b1*b1 - l*l) / (-2.0*b1*l)) - atan(a1/e1) + PI_2;
      realtime_pub_->msg_.velocity[5] = 0.0;
      realtime_pub_->msg_.effort[5] = 0.0;
      
      // actuator3
      realtime_pub_->msg_.position[6] = actuator_state_[2].getPosition();
      realtime_pub_->msg_.velocity[6] = actuator_state_[2].getVelocity();
      realtime_pub_->msg_.effort[6] = actuator_state_[2].getEffort();

      realtime_pub_->unlockAndPublish();
    }
  }
}

void CraneStateController::stopping(const ros::Time& /*time*/)
{
}

}  // namespace crane_controllers

PLUGINLIB_EXPORT_CLASS(crane_controllers::CraneStateController, controller_interface::ControllerBase)
