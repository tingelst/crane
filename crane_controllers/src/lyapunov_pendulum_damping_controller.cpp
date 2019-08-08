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

  // RML
  rml_.reset(new ReflexxesAPI(2, 0.1));
  rml_input_.reset(new RMLPositionInputParameters(2));
  rml_output_.reset(new RMLPositionOutputParameters(2));

  crane_tip_velocity_handle_ = crane_tip_velocity_command_interface_->getHandle("crane_tip");

  command_sub_ = node_handle.subscribe<crane_msgs::CraneTrajectoryPoint>(
      "command", 1, &LyapunovPendulumDampingController::commandCB, this);

  pendulum_joint_state_sub_ = node_handle.subscribe<sensor_msgs::JointState>(
      "/pendulum_joint_states", 1, &LyapunovPendulumDampingController::pendulumJointStateCB, this);
  pendulum_joint_state_buffer_.writeFromNonRT({ 0.0, 0.0, 0.0, 0.0 });

  command_pub_.reset(new realtime_tools::RealtimePublisher<crane_msgs::CraneControl>(node_handle, "commanded", 3));
  trajectory_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(node_handle, "zref", 3));
  trajectory_pub_->msg_.name.push_back("x");
  trajectory_pub_->msg_.name.push_back("y");
  for (size_t i = 0; i < 2; ++i)
  {
    trajectory_pub_->msg_.position.push_back(0.0);
    trajectory_pub_->msg_.velocity.push_back(0.0);
  }

  last_g_ = std::vector<double>{ { 0.0, 0.0 } };
  last_gopt_ = std::vector<double>{ { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

  return true;
}

void LyapunovPendulumDampingController::starting(const ros::Time& now)
{
  ROS_INFO("LyapunovPendulumDampingController: Starting!");

  std::array<double, 2> position = crane_tip_velocity_handle_.getPosition();
  std::array<double, 2> velocity = crane_tip_velocity_handle_.getVelocity();

  crane_msgs::CraneTrajectoryPoint command;
  for (std::size_t i = 0; i < 2; ++i)
  {
    command.position.push_back(position[i]);
    command.velocity.push_back(0.0);
    command.max_velocity.push_back(0.1);
    command.max_acceleration.push_back(0.1);
  }
  command_buffer_.initRT(command);

  for (std::size_t i = 0; i < 2; ++i)
  {
    rml_input_->SelectionVector->VecData[i] = true;
  }
}

void LyapunovPendulumDampingController::update(const ros::Time& now, const ros::Duration& period)
{
  // Trajectory
  crane_msgs::CraneTrajectoryPoint trajectory_point = *command_buffer_.readFromRT();

  std::array<double, 2> position = crane_tip_velocity_handle_.getPosition();
  std::array<double, 2> velocity = crane_tip_velocity_handle_.getVelocity();

  for (std::size_t i = 0; i < 2; ++i)
  {
    rml_input_->CurrentPositionVector->VecData[i] = position[i];
    rml_input_->CurrentVelocityVector->VecData[i] = velocity[i];

    rml_input_->TargetPositionVector->VecData[i] = trajectory_point.position[i];
    rml_input_->TargetVelocityVector->VecData[i] = trajectory_point.velocity[i];
    rml_input_->MaxVelocityVector->VecData[i] = trajectory_point.max_velocity[i];
    rml_input_->MaxAccelerationVector->VecData[i] = trajectory_point.max_acceleration[i];
  }

  int result = rml_->RMLPosition(*rml_input_.get(), rml_output_.get(), rml_flags_);
  if (result < 0)
  {
    ROS_ERROR_STREAM("RML error: " << result);
  }

  // Damping
  std::array<double, 4> q = *pendulum_joint_state_buffer_.readFromRT();

  double kp = 1.0 / 2.0;
  double kd = 2.0 / 2.0;
  double phix = q[0];
  double dphix = q[1];
  double phiy = q[2];
  double dphiy = q[3];

  double g = 9.81;
  double L = 1.05;

  double cx = cos(phix);
  double sx = sin(phix);
  double cy = cos(phiy);
  double sy = sin(phiy);

  double uy = -L * cy / cx * (kd * dphix + kp * phix) - 2.0 * L * sy / cx + g * sx * sy * sy / cx;
  double ux = L / cy * (kd * dphiy + kp * phiy) - L * sy * dphix * dphix - uy * sx * sy / cy;

  // DEBUG
  ux = 0.0;
  uy = 0.0;

  std::vector<double> z{ position[0], velocity[0], position[1], velocity[1], phix, dphix, phiy, dphiy };
  std::vector<double> zref{ rml_output_->NewPositionVector->VecData[0],
                            rml_output_->NewVelocityVector->VecData[0],
                            rml_output_->NewPositionVector->VecData[1],
                            rml_output_->NewVelocityVector->VecData[1],
                            0.,
                            0.,
                            0.,
                            0. };

  if (trajectory_pub_->trylock())
  {
    trajectory_pub_->msg_.header.stamp = now;
    trajectory_pub_->msg_.position[0] = zref[0];
    trajectory_pub_->msg_.position[1] = zref[2];
    trajectory_pub_->msg_.velocity[0] = zref[1];
    trajectory_pub_->msg_.velocity[1] = zref[3];
    trajectory_pub_->unlockAndPublish();
  }

  // ROS_INFO("%f, %f, %f. %f",   zref[0], zref[1], zref[2], zref[3]);

  // Solve the problem
  std::vector<double> gmin{ -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1 };
  std::vector<double> gmax{ 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 };

  solver_ = solver(z, zref, last_g_);
  std::vector<double> sol;
  solver_({ { "lbx", gmin }, { "ubx", gmax }, { "x0", last_gopt_ } }, { { "x", &sol } });

  double gx = sol[0];
  double gy = sol[1];

  // Store the results for next cycle
  last_gopt_ = sol;
  last_g_ = std::vector<double>{ { gx, gy } };

  double dwx = ux + gx;
  double dwy = uy + gy;

  double Tv = 0.2;
  double ddx0 = (dwx * period.toSec() - dx0_) / Tv;
  double ddy0 = (dwy * period.toSec() - dy0_) / Tv;

  if (command_pub_->trylock())
  {
    command_pub_->msg_.gx = ddx0;
    command_pub_->msg_.gy = ddy0;
    command_pub_->unlockAndPublish();
  }

  dx0_ = ddx0 * period.toSec();
  dy0_ = ddy0 * period.toSec();

  crane_tip_velocity_handle_.setCommand({ dx0_, dy0_ });
  // crane_tip_velocity_handle_.setCommand({ zref[1], zref[3] });

}  // namespace crane_controllers

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
  SX gk = SX::sym("gk", 2);
  SX Ts = SX::sym("Ts");
  SX k = SX::sym("k", 2);
  SX L = SX::sym("L");

  Function cd = continuousDynamics();
  SX delta = Ts / 10.0;

  SX zk1 = zk;
  for (int i = 0; i < 10; ++i)
  {
    zk1 += delta * cd(std::vector<SX>{ zk1, gk, k, L }).at(0);
  }

  Function dd("discreteDynamics", { zk, gk, Ts, k, L }, { zk1 }, { "zk", "gk", "Ts", "k", "L" }, { "zk1" });
  return dd;
}

casadi::Function LyapunovPendulumDampingController::solver(const std::vector<double>& z_in,
                                                           const std::vector<double>& zref_in,
                                                           const std::vector<double>& last_g_in)
{
  using namespace casadi;
  using namespace std;

  SX zk = SX(z_in);
  SX zref = SX(zref_in);
  SX last_g = SX(last_g_in);

  SX zk1;

  SX Ts = 0.2;
  SX N = 4;
  SX L = 1.05;

  SX g = SX::sym("g", 8);
  SX gk = SX::sym("gk", 2);

  SX k = SX::vertcat({ 1.0, 2.0 });

  SX R = SX::diagcat({ 0.1, 0.1 });
  SX Q = SX::diagcat({ 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0 });

  SX J = 0.0;

  // Discrete dynamics
  Function dynamics = discreteDynamics();

  gk = g(Slice(0, 2));
  zk1 = dynamics(vector<SX>{ zk, gk, Ts, k, L }).at(0);
  J = J + SX::mtimes(vector<SX>{ (zk1 - zref).T(), Q, (zk1 - zref) }) +
      SX::mtimes(vector<SX>{ (gk - last_g).T(), R, (gk - last_g) });
  zk = zk1;

  gk = g(Slice(2, 4));
  zk1 = dynamics(vector<SX>{ zk, gk, Ts, k, L }).at(0);
  J = J + SX::mtimes(vector<SX>{ (zk1 - zref).T(), Q, (zk1 - zref) }) +
      SX::mtimes(vector<SX>{ (gk - g(Slice(0, 2))).T(), R, (gk - g(Slice(0, 2))) });
  zk = zk1;

  gk = g(Slice(4, 6));
  zk1 = dynamics(vector<SX>{ zk, gk, Ts, k, L }).at(0);
  J = J + SX::mtimes(vector<SX>{ (zk1 - zref).T(), Q, (zk1 - zref) }) +
      SX::mtimes(vector<SX>{ (gk - g(Slice(2, 4))).T(), R, (gk - g(Slice(2, 4))) });
  zk = zk1;

  gk = g(Slice(6, 8));
  zk1 = dynamics(vector<SX>{ zk, gk, Ts, k, L }).at(0);
  J = J + SX::mtimes(vector<SX>{ (zk1 - zref).T(), Q, (zk1 - zref) }) +
      SX::mtimes(vector<SX>{ (gk - g(Slice(4, 6))).T(), R, (gk - g(Slice(4, 6))) });

  // Create the NLP
  SXDict nlp = { { "x", g }, { "f", J } };

  // NLP solver options
  Dict solver_opts;
  std::string solver_name = "ipopt";
  // std::string solver_name = "sqpmethod";
  solver_opts["verbose"] = false;
  solver_opts["print_time"] = false;
  if (solver_name == "ipopt")
  {
    solver_opts["ipopt.print_level"] = 0;
  }

  // Allocate an NLP solver and buffers
  Function solver = nlpsol("solver", solver_name, nlp, solver_opts);

  return solver;
}

}  // namespace crane_controllers