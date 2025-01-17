///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2019, Norwegian University of Science and Technology (NTNU).
// Copyright (C) 2012, hiDOF INC.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF, Inc. nor the names of its
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

#ifndef JOINT_STATE_CONTROLLER_JOINT_STATE_CONTROLLER_H
#define JOINT_STATE_CONTROLLER_JOINT_STATE_CONTROLLER_H

#include <memory>
#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>

#include <crane_hw_interface/crane_tip_state_interface.h>
#include <crane_msgs/CraneState.h>

namespace crane_controllers
{
constexpr double PI = 3.14159265358979323846;
constexpr double PI_2 = 1.57079632679489661923;
constexpr double DEG2RAD = 0.017453292519943295;

class CraneStateController
  : public controller_interface::MultiInterfaceController<hardware_interface::JointStateInterface,
                                                          hardware_interface::ActuatorStateInterface,
                                                          crane_hw_interface::CraneTipStateInterface>
{
public:
  CraneStateController() : publish_rate_(50.0)
  {
  }

  virtual bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh);
  virtual void starting(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
  virtual void stopping(const ros::Time& /*time*/);

private:
  std::vector<hardware_interface::ActuatorStateHandle> actuator_state_;
  std::vector<hardware_interface::JointStateHandle> joint_state_;
  crane_hw_interface::CraneTipStateHandle crane_tip_state_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > joint_realtime_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<crane_msgs::CraneState> > crane_realtime_pub_;
  ros::Time last_publish_time_;
  double publish_rate_;
  unsigned int num_hw_joints_;
  unsigned int num_hw_actuators_;
};

}  // namespace crane_controllers

#endif
