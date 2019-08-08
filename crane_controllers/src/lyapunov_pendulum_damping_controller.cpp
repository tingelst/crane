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
  rml_.reset(new ReflexxesAPI(2, 0.01));
  rml_input_.reset(new RMLPositionInputParameters(2));
  rml_output_.reset(new RMLPositionOutputParameters(2));

  crane_tip_velocity_handle_ = crane_tip_velocity_command_interface_->getHandle("crane_tip");

  command_sub_ = node_handle.subscribe<crane_msgs::CraneTrajectoryPoint>(
      "command", 1, &LyapunovPendulumDampingController::commandCB, this);

  pendulum_joint_state_sub_ = node_handle.subscribe<sensor_msgs::JointState>(
      "/pendulum_joint_states", 1, &LyapunovPendulumDampingController::pendulumJointStateCB, this);
  pendulum_joint_state_buffer_.writeFromNonRT({ 0.0, 0.0, 0.0, 0.0 });

  command_pub_.reset(new realtime_tools::RealtimePublisher<crane_msgs::CraneControl>(node_handle, "commanded", 1));

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

  double kp = 1.0;
  double kd = 2.0;
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

  std::array<double, 2> command;
  command[0] = rml_output_->NewVelocityVector->VecData[0] + ux * period.toSec();
  command[1] = rml_output_->NewVelocityVector->VecData[1] + ux * period.toSec();

  if (command_pub_->trylock())
  {
    command_pub_->msg_.gx = command[0];
    command_pub_->msg_.gy = command[1];
    command_pub_->unlockAndPublish();
  }

  ROS_INFO("%f, %f", command[0], command[1]);
  crane_tip_velocity_handle_.setCommand({ command[0], command[1] });

  /*

  {
    std::vector<double> z{ 1.23934024e+00,  1.25449097e-02, -7.56690389e-02, 1.94915197e-02,
                           -2.48908109e-04, 1.97056315e-02, -1.45635425e-03, 1.03694469e-03 };
    std::vector<double> zref{ 1.29644275, 0., 0., 0., 0., 0., 0., 0. };
    std::vector<double> last_g{ -0.01027713, 0.01856886 };

    std::vector<double> x0{ 0.01818353,  0.02595379, 0.00907866,  0.03259367,
                            -0.01027713, 0.01856886, -0.02268865, 0.00656219 };

    std::vector<double> gmin{ -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1 };
    std::vector<double> gmax{ 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 };

    solver_ = solver(z, zref, last_g);
    // Solve the problem
    casadi::DMDict arg = {
      { "lbx", gmin },
      { "ubx", gmax },
      { "x0", x0 },
    };

    casadi::DMDict res = solver_(arg);
  }

  std::vector<double> x0{
    0.0040685, -0.02000171, 0.0101947, 0.00103702, 0.00799181, 0.01778638, 0.00498826, 0.0260024
  };

  {
    casadi::DM z(std::vector<double>{ 1.20927357, -0.24379647, -0.12293626, -0.27332213, 0.05360948, 0.002603,
                                      -0.06313162, 0.04556808 });
    casadi::DM g(std::vector<double>{ 0.1, 0.1 });
    casadi::DM k(std::vector<double>{ 1.0, 2.0 });
    casadi::DM L = 1.05;
    casadi::DM Ts = 0.2;

    casadi::Function cd = continuousDynamics();
    casadi::DM res = cd(std::vector<casadi::DM>{ z, g, k, L }).at(0);
    // ROS_INFO_STREAM(res);

    casadi::Function dd = discreteDynamics();
    res = dd(std::vector<casadi::DM>{ z, g, Ts, k, L }).at(0);
    // ROS_INFO_STREAM(res);
  }

  */
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