#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <crane_controllers/lyapunov_pendulum_damping_controller.h>

namespace crane_controllers
{
bool LyapunovPendulumDampingController::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle)
{
  crane_tip_velocity_command_interface_ = robot_hardware->get<crane_hw_interface::CraneTipVelocityCommandInterface>();
  if (crane_tip_velocity_command_interface_ == nullptr)
  {
    ROS_ERROR("LyapunovPendulumDampingController: Error getting control interface from hardware!");
    return false;
  }

  crane_tip_velocity_handle_ = crane_tip_velocity_command_interface_->getHandle("crane_tip");

  command_sub_ = node_handle.subscribe<crane_msgs::CraneControl>("/crane_nmpc_nodelet/command", 1,
                                                                 &LyapunovPendulumDampingController::commandCB, this);

  pendulum_joint_state_sub_ = node_handle.subscribe<sensor_msgs::JointState>(
      "/pendulum_joint_states", 1, &LyapunovPendulumDampingController::pendulumJointStateCB, this);
  pendulum_joint_state_buffer_.writeFromNonRT({ 0.0, 0.0, 0.0, 0.0 });

  command_pub_.reset(new realtime_tools::RealtimePublisher<crane_msgs::CraneControl>(node_handle, "commanded", 3));
  state_pub_.reset(new realtime_tools::RealtimePublisher<crane_msgs::CraneState>(node_handle, "state", 3));

  trajectory_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(node_handle, "zref", 3));
  trajectory_pub_->msg_.name.push_back("x");
  trajectory_pub_->msg_.name.push_back("y");
  for (size_t i = 0; i < 2; ++i)
  {
    trajectory_pub_->msg_.position.push_back(0.0);
    trajectory_pub_->msg_.velocity.push_back(0.0);
  }

  return true;
}

void LyapunovPendulumDampingController::starting(const ros::Time& now)
{
  crane_msgs::CraneControl command;
  command.gx = 0.0;
  command.gy = 0.0;
  command_buffer_.initRT(command);
}

void LyapunovPendulumDampingController::update(const ros::Time& now, const ros::Duration& period)
{
  // Crane state
  std::array<double, 2> position = crane_tip_velocity_handle_.getPosition();
  std::array<double, 2> velocity = crane_tip_velocity_handle_.getVelocity();

  // Pendulum state
  std::array<double, 4> q = *pendulum_joint_state_buffer_.readFromRT();
  double phix = q[0];
  double dphix = q[1];
  double phiy = q[2];
  double dphiy = q[3];

  if (state_pub_->trylock())
  {
    state_pub_->msg_.header.stamp = now;
    state_pub_->msg_.x = position[0];
    state_pub_->msg_.dx = velocity[0];
    state_pub_->msg_.y = position[1];
    state_pub_->msg_.dy = velocity[1];
    state_pub_->msg_.phix = phix;
    state_pub_->msg_.dphix = dphix;
    state_pub_->msg_.phiy = phiy;
    state_pub_->msg_.dphiy = dphiy;
    state_pub_->unlockAndPublish();
  }

  crane_msgs::CraneControl nmpc_command = *command_buffer_.readFromRT();
  double gx = nmpc_command.gx;
  double gy = nmpc_command.gy;

  // Gains
  double kp = 1.0;
  double kd = 2.0;

  double g = 9.81;
  double L = 1.05;

  double cx = cos(phix);
  double sx = sin(phix);
  double cy = cos(phiy);
  double sy = sin(phiy);

  double uy = -L * cy / cx * (kd * dphix + kp * phix) - 2.0 * L * sy / cx + g * sx * sy * sy / cx;
  double ux = L / cy * (kd * dphiy + kp * phiy) - L * sy * dphix * dphix - uy * sx * sy / cy;

  // Velocity loop
  double Tv = 0.2;
  // double dwx = ux + gx;
  // double dwy = uy + gy;

  double dwx = gx;
  double dwy = gy;

  double wx = dwx * period.toSec() + last_wx_; 
  double wy = dwy * period.toSec() + last_wy_;
  double ddx0 = (wx - dx0_) / Tv;
  double ddy0 = (wy - dy0_) / Tv;
  dx0_ = ddx0 * period.toSec() + last_dx0_;
  dy0_ = ddy0 * period.toSec() + last_dy0_;

  last_wx_ = wx;
  last_wy_ = wy;
  last_dx0_ = dx0_;
  last_dy0_ = dy0_;

  if (command_pub_->trylock())
  {
    command_pub_->msg_.header.stamp = now;
    command_pub_->msg_.gx = ddx0;
    command_pub_->msg_.gy = ddy0;
    command_pub_->unlockAndPublish();
  }

  crane_tip_velocity_handle_.setCommand({ dx0_, dy0_ });
}

}  // namespace crane_controllers