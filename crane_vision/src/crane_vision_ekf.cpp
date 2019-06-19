#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>

using namespace std;
using namespace Eigen;

using ProjectionMatrix = Eigen::Matrix<double, 3, 4>;
using Point2d = Eigen::Matrix<double, 2, 1>;
using Vector3d = Eigen::Matrix<double, 3, 1>;

bool dlt(const Point2d& c0, const Point2d& c1, const Point2d& c2, const ProjectionMatrix& P0,
         const ProjectionMatrix& P1, const ProjectionMatrix P2, Vector3d& L)
{
  Eigen::Matrix<double, 6, 4> X;
  X.row(0) = c0(0) * P0.row(2) - P0.row(0);
  X.row(1) = c0(1) * P0.row(2) - P0.row(1);
  X.row(2) = c1(0) * P1.row(2) - P1.row(0);
  X.row(3) = c1(1) * P1.row(2) - P1.row(1);
  X.row(4) = c2(0) * P2.row(2) - P2.row(0);
  X.row(5) = c2(1) * P2.row(2) - P2.row(1);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(X, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd V = svd.matrixV();

  double scale = V(3, 3);
  if (scale > 1e-8)
  {
    L = V.col(3).topRows(3) / scale;
    return true;
  }
  else
  {
    return false;
  }
}

bool findLine(const Point2d& c01, const Point2d& c02, const Point2d& c11, const Point2d& c12, const Point2d& c21,
              const Point2d& c22, const ProjectionMatrix& P0, const ProjectionMatrix& P1, const ProjectionMatrix P2,
              Vector3d& L)
{
  Vector3d X1, X2;
  dlt(c01, c11, c21, P0, P1, P2, X1);
  dlt(c02, c12, c22, P0, P1, P2, X2);
  L = X2 - X1;
  L.normalize();
}

using StateVector = Eigen::Matrix<double, 6, 1>;
using TransitionMatrix = Eigen::Matrix<double, 6, 6>;
using CovarianceMatrix = Eigen::Matrix<double, 6, 6>;
using AccelerationVector = Eigen::Vector2d;

StateVector fk(StateVector x, AccelerationVector u, double w, double L)
{
  double c0 = cos(x(0));
  double c1 = cos(x(1));
  double s0 = sin(x(0));
  double s1 = sin(x(1));

  StateVector x1 = StateVector::Zero();
  x1(0) = x(2);
  x1(1) = x(3);
  x1(2) = (2.0 * x(2) * x(3) * s1 - w * s0 + (u(1) * c0 / L)) / c1;
  x1(3) = -c1 * s1 * x(2) * x(2) - (u(0) * c1 + u(1) * s0 * s1) / L - w * c0 * s1;
  return x1;
}

TransitionMatrix Fk(StateVector x, AccelerationVector u, double w, double L, double dt)
{
  double c0 = cos(x(0));
  double c1 = cos(x(1));
  double s0 = sin(x(0));
  double s1 = sin(x(1));

  TransitionMatrix X = TransitionMatrix::Identity();
  X(0, 2) = dt;
  X(1, 3) = dt;
  X(2, 0) = -(dt * (w * c0 + (u(1) * s0) / L)) / c1;
  X(2, 1) = 2.0 * dt * x(2) * x(3) + (dt * s1 * (2.0 * x(2) * x(3) * s1 - w * s0 + (u(1) * c0) / L)) / (c1 * c1);
  X(2, 2) = (2.0 * dt * x(3) * s1) / c1 + 1.0;
  X(2, 3) = (2.0 * dt * x(2) * s1) / c1;
  X(3, 0) = dt * (w * s0 * s1 - (u[1] * c0 * s1) / L);
  X(3, 1) = -dt * (x(2) * x(2) * c1 * c1 - x(2) * x(2) * s1 * s1 - (u(0) * s1 - u(1) * c1 * s0 / L) + w * c0 * c1);
  X(3, 2) = -2.0 * dt * x(2) * c1 * s1;
  return X;
}

bool ekf(Vector3d Lvec, AccelerationVector uk, MatrixXd hat_Pkm1, VectorXd hat_thetakm1, double r, double dt)
{
  int D = 10;
  double g = 9.81;
  double L = r;
  AccelerationVector u = uk;
  VectorXd x = hat_thetakm1;
  Eigen::Matrix2d R;
  R << 0.00377597, -0.00210312, -0.00210312, 0.00125147;

  CovarianceMatrix Q = CovarianceMatrix::Zero();
  Q.diagonal() << 0.00003, 0.00003, 0.0005, 0.0005, 0.0001, 0.0001;
  Eigen::Matrix<double, 2, 6> H;
  H << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

  TransitionMatrix Fi = Fk(x, u, g / r, L, dt);

  Vector2d zkp1;
  zkp1 << atan2(-Lvec(1), Lvec(2)), atan2(Lvec(0), sqrt(Lvec(1) * Lvec(1) + Lvec(2) * Lvec(2)));

  for (int i = 0; i < D; ++i)
  {
    x = x + fk(x, u, g / r, L) * dt / D;
  }

  Eigen::MatrixXd barP_kp1 = (Fi * hat_Pkm1 * Fi.transpose()) + Q;
  Eigen::MatrixXd K_kp1 = barP_kp1 * H.transpose() * (R + H * barP_kp1 * H.transpose()).inverse();

  Eigen::VectorXd hat_thetak = x + K_kp1 * (zkp1 - H * x);
  Eigen::MatrixXd hat_Pk = (Eigen::Matrix<double, 6, 6>::Identity() - K_kp1 * H) * barP_kp1;

  cout << hat_thetak << endl;

}

int main()
{
  // ProjectionMatrix P0, P1, P2;
  // P0 << 942.0, 0.0, 623.66, 0.0, 0.0, 942.0, 345.69, 0.0, 0.0, 0.0, 1.0, 0.0;
  // P1 << 941.0, 0.0, 637.21, 220005.80000000002, 0.0, 941.0, 349.9, 0.0, 0.0, 0.0, 1.0, 0.0;
  // P2 << 937.0, 0.0, 637.21, 437579.0, 0.0, 937.0, 381.54, 0.0, 0.0, 0.0, 1.0, 0.0;

  // Vector3d L;
  // findLine(Point2d(150.4, 100.6), Point2d(150., 250.), Point2d(300., 100.), Point2d(300., 250.), Point2d(450., 100.),
  //          Point2d(450., 250.), P0, P1, P2, L);

  // StateVector x;
  // x << 1, 2, 3, 4, 5, 6;
  // AccelerationVector u;
  // u << 0, 0;
  // cout << Fk(x, u, 9.81 / 1.05, 1.05, 99) << endl;

  Vector3d Lvec;
  Lvec << 1.0, 2.0, 3.0;

  AccelerationVector u;
  u << 0.1, 0.2;

  Eigen::Matrix<double, 6, 6> hat_Pkm1;
  hat_Pkm1 << 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5,
      6;

  Eigen::Matrix<double, 6, 1> hat_thetakm1;
  hat_thetakm1 << 1, 2, 3, 4, 5, 6;

  ekf(Lvec, u, hat_Pkm1, hat_thetakm1, 1.05, 0.01);
}