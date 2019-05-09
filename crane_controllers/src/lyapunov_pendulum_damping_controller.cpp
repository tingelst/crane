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
  command_sub_ = node_handle.subscribe<crane_msgs::CraneControl>("command", 1,
                                                                 &LyapunovPendulumDampingController::commandCB, this);
  pendulum_joint_state_sub_ = node_handle.subscribe<sensor_msgs::JointState>(
      "/pendulum_joint_states", 1, &LyapunovPendulumDampingController::pendulumJointStateCB, this);

  pendulum_joint_state_buffer_.writeFromNonRT({ 0.0, 0.0, 0.0, 0.0 });

  command_pub_.reset(new realtime_tools::RealtimePublisher<crane_msgs::CraneControl>(node_handle, "command", 3));

  return true;
}

void LyapunovPendulumDampingController::update(const ros::Time& now, const ros::Duration& period)
{
  std::array<double, 4> q = *pendulum_joint_state_buffer_.readFromRT();

  double kp = 1.0;
  double kd = 2.0;
  double phix = q[0];
  double dphix = q[1];
  double phiy = q[2];
  double dphiy = q[3];

  double g = 9.81;
  double L = 1.0;

  double cx = cos(phix);
  double sx = sin(phix);
  double cy = cos(phiy);
  double sy = sin(phiy);

  double uy = -L * cy / cx * (kd * dphix + kp * phix) - 2.0 * L * sy / cx + g * sx * sy * sy / cx;
  double ux = L / cy * (kd * dphiy + kp * phiy) - L * sy * dphix * dphix - uy * sx * sy / cy;

  if (command_pub_->trylock())
  {
    command_pub_->msg_.gx = ux;
    command_pub_->msg_.gy = uy;
    command_pub_->unlockAndPublish();
  }

  crane_tip_velocity_handle_.setCommand({ ux * period.toSec(), uy * period.toSec() });
}

}  // namespace crane_controllers