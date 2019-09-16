// ROS
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "std_msgs/String.h"
#include <crane_msgs/CraneState.h>
#include <crane_msgs/CraneControl.h>
#include <crane_msgs/CraneTrajectoryPoint.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// Casadi
#include "casadi/casadi.hpp"

namespace crane_controllers
{
class CraneNMPCNodelet : public nodelet::Nodelet
{
  virtual void onInit();
  void run();

  std::unique_ptr<realtime_tools::RealtimePublisher<crane_msgs::CraneControl>> command_pub_;
  ros::Subscriber state_sub_;
  ros::Subscriber init_state_sub_;
  ros::Subscriber trajectory_point_sub_;
  realtime_tools::RealtimeBuffer<crane_msgs::CraneState> state_buffer_;
  realtime_tools::RealtimeBuffer<crane_msgs::CraneTrajectoryPoint> trajectory_buffer_;

  casadi::Function continuousDynamics(void);
  casadi::Function discreteDynamics(void);
  casadi::Function solver(const std::vector<double>& z, const std::vector<double>& zref,
                          const std::vector<double>& last_g);
  casadi::Function solver_;
  std::vector<double> last_g_;
  std::vector<double> last_gopt_;
  std::vector<double> sol_;
  double gx_{ 0.0 };
  double gy_{ 0.0 };

  double max_ = 0.1;
  std::vector<double> gmin_;
  std::vector<double> gmax_;

  bool initialized_ = false;

  void stateCB(const crane_msgs::CraneState::ConstPtr& msg)
  {
    state_buffer_.writeFromNonRT(*msg);
  }

  void initStateCB(const crane_msgs::CraneState::ConstPtr& msg)
  {
    if (!initialized_)
    {
      crane_msgs::CraneTrajectoryPoint trajectory_point;
      trajectory_point.position.push_back(msg->x);
      trajectory_point.position.push_back(msg->y);
      trajectory_point.velocity.push_back(msg->dx);
      trajectory_point.velocity.push_back(msg->dy);
      trajectory_buffer_.writeFromNonRT(trajectory_point);

      state_buffer_.writeFromNonRT(*msg);

      initialized_ = true;

      ROS_INFO("MPC: Initialized");
    }
  }

  void trajectoryCB(const crane_msgs::CraneTrajectoryPoint::ConstPtr& msg)
  {
    trajectory_buffer_.writeFromNonRT(*msg);
  }
};

void CraneNMPCNodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();

  command_pub_.reset(new realtime_tools::RealtimePublisher<crane_msgs::CraneControl>(private_nh, "command", 3));
  state_sub_ = nh.subscribe<crane_msgs::CraneState>("/lyapunov_pendulum_damping_controller/state", 1,
                                                    &CraneNMPCNodelet::stateCB, this);

  init_state_sub_ = nh.subscribe<crane_msgs::CraneState>("/crane_state_controller/crane_states", 1,
                                                         &CraneNMPCNodelet::initStateCB, this);

  trajectory_point_sub_ =
      nh.subscribe<crane_msgs::CraneTrajectoryPoint>("/crane_trajectory", 1, &CraneNMPCNodelet::trajectoryCB, this);

  last_g_ = std::vector<double>{ { 0.0, 0.0 } };
  last_gopt_ = std::vector<double>{ { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  sol_ = std::vector<double>{ { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

  gmin_ = std::vector<double>{ { -max_, -max_, -max_, -max_, -max_, -max_, -max_, -max_ } };
  gmax_ = std::vector<double>{ { max_, max_, max_, max_, max_, max_, max_, max_ } };

  run();
}

void CraneNMPCNodelet::run()
{
  ros::Rate rate(10);  // 10 hz
  while (ros::ok())
  {
    if (initialized_)
    {
      crane_msgs::CraneState state = *state_buffer_.readFromRT();

      std::vector<double> z{ state.x, state.dx, state.y, state.dy, state.phix, state.dphix, state.phiy, state.dphiy };

      crane_msgs::CraneTrajectoryPoint trajectory_point = *trajectory_buffer_.readFromRT();

      std::vector<double> zref{ trajectory_point.position[0],
                                trajectory_point.velocity[0],
                                trajectory_point.position[1],
                                trajectory_point.velocity[1],
                                0.0,
                                0.0,
                                0.0,
                                0.0 };

      // std::vector<double> zref{ 0.5, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

      // Solve the problem
      solver_ = solver(z, zref, last_g_);
      solver_({ { "lbx", gmin_ }, { "ubx", gmax_ }, { "x0", last_gopt_ } }, { { "x", &sol_ } });

      gx_ = sol_[0];
      gy_ = sol_[1];

      // // Store the results for next cycle
      last_gopt_ = sol_;
      last_g_[0] = gx_;
      last_g_[1] = gy_;

      // NODELET_INFO("gx: %f, gy: %f", gx_, gy_);

      if (command_pub_->trylock())
      {
        command_pub_->msg_.gx = gx_;
        command_pub_->msg_.gy = gy_;
        command_pub_->unlockAndPublish();
      }
    }

    // rate.sleep();
  }
}

casadi::Function CraneNMPCNodelet::continuousDynamics(void)
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

casadi::Function CraneNMPCNodelet::discreteDynamics(void)
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

casadi::Function CraneNMPCNodelet::solver(const std::vector<double>& z_in, const std::vector<double>& zref_in,
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

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(crane_controllers::CraneNMPCNodelet, nodelet::Nodelet)