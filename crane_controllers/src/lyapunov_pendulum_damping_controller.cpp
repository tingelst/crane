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

  ROS_INFO_STREAM(this->continuousDynamics());

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

casadi::Function LyapunovPendulumDampingController::continuousDynamics(void)
{
  using namespace casadi;

  SX kp = SX::sym("kp");
  SX kd = SX::sym("kd");
  SX k = SX::vertcat({ kp, kd });
  SX L = SX::sym("L");
  SX x0 = SX::sym("x0");
  SX dx0 = SX::sym("dx0");
  SX y0 = SX::sym("y0");
  SX dy0 = SX::sym("dy0");
  SX phix = SX::sym("phix");
  SX dphix = SX::sym("dphix");
  SX phiy = SX::sym("phiy");
  SX dphiy = SX::sym("dphiy");
  SX z = SX::vertcat({ x0, dx0, y0, dy0, phix, dphix, phiy, dphiy });

  SX cx = SX::cos(phix);
  SX sx = SX::sin(phix);
  SX cy = SX::cos(phiy);
  SX sy = SX::sin(phiy);

  SX gx = SX::sym("gx");
  SX gy = SX::sym("gy");
  SX g = SX::vertcat({ gx, gy });

  SX dzdt = SX::sym("dzdt", 8);

  SX uy = -L * cy / cx * (kd * dphix + kp * phix) - 2.0 * L * sy / cx * dphix * dphiy + 9.81 * sx * sy * sy / cx;

  //  Eq 51-54 in Ecc
  dzdt(0) = dx0;                                                                                                 // dx0
  dzdt(1) = gx + (L * (kd * dphiy + kp * phiy) - L * cy * sy * dphix * dphix - uy * sx * sy) / cy;               // ddx0
  dzdt(2) = dy0;                                                                                                 // dy0
  dzdt(3) = gy - (L * cy * (kd * dphix + kp * phix) + 2.0 * L * sy * dphix * dphiy - 9.81 * sx * sy * sy) / cx;  // ddy0
  dzdt(4) = z(5);                                                                                 // dphix
  dzdt(5) = (-kd * dphix) - (kp * phix) - (9.81 / L * cy * sx) + (gy * cx / (cy * L));            // ddphix
  dzdt(6) = z(7);                                                                                 // dphiy
  dzdt(7) = (-kd * dphiy) - (kp * phiy) - (9.81 / L * cx * sy) - ((gx * cy + gy * sx * sy) / L);  // ddphiy

  Function cs("cs", { z, g, k, L }, { dzdt }, { "z", "g", "k", "L" }, { "dzdt" });

  return cs;
}

casadi::Function LyapunovPendulumDampingController::discreteDynamics(void)
{
  using namespace casadi;

  SX zk = SX::sym("zk", 8);
  SX zk1 = SX::sym("zk1", 8);
  SX gk = SX::sym("gk", 2);
  SX Ts = SX::sym("Ts");
  SX k = SX::sym("k", 2);
  SX L = SX::sym("L");

  SX cd_out = SX::zeros(8);

  Function cd = continuousDynamics();
  std::vector<SX> input{ { zk1, gk, k, L } };
  std::vector<SX> output;

  SX delta = Ts / 10.0;
  zk1 = zk;
  for (int i = 0; i < 10; ++i)
  {
    std::vector<SX> out = cd(input);
    zk1 = zk1 + delta * output[0];
  }

  Function dd("discreteDynamics", { zk, gk, Ts, k, L }, { zk1 }, { "zk", "gk", "Ts", "k", "L" }, { "zk1" });
  return dd;
}

}  // namespace crane_controllers