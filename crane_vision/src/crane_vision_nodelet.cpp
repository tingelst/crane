// Eigen
#include <Eigen/Dense>
#include <Eigen/SVD>

// OpenCV
#include <opencv2/opencv.hpp>

// ROS
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64MultiArray.h>

#include <crane_msgs/CranePendulumImagePoints.h>

namespace crane_vision
{
using namespace sensor_msgs;
using namespace message_filters::sync_policies;

using ProjectionMatrix = Eigen::Matrix<double, 3, 4>;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using StateVector = Eigen::Matrix<double, 6, 1>;
using TransitionMatrix = Eigen::Matrix<double, 6, 6>;
using CovarianceMatrix = Eigen::Matrix<double, 6, 6>;
using AccelerationVector = Eigen::Vector2d;

Vector3d dlt(const Eigen::Vector2d& c0, const Eigen::Vector2d& c1, const Eigen::Vector2d& c2,
             const ProjectionMatrix& P0, const ProjectionMatrix& P1, const ProjectionMatrix P2)
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

  Eigen::Vector3d L = Eigen::Vector3d::Zero();
  double scale = V(3, 3);
  if (scale > 1e-8)
  {
    L = V.col(3).topRows(3) / scale;
  }
  return L;
}

Vector3d findLine(const Eigen::Vector2d& c01, const Eigen::Vector2d& c02, const Eigen::Vector2d& c11,
                  const Eigen::Vector2d& c12, const Eigen::Vector2d& c21, const Eigen::Vector2d& c22,
                  const ProjectionMatrix& P0, const ProjectionMatrix& P1, const ProjectionMatrix P2)
{
  Vector3d X1 = dlt(c01, c11, c21, P0, P1, P2);
  Vector3d X2 = dlt(c02, c12, c22, P0, P1, P2);
  Vector3d L = X2 - X1;
  L.normalize();
  return L;
}

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

auto ekf(Vector3d Lvec, AccelerationVector uk, Eigen::MatrixXd hat_Pkm1, Eigen::VectorXd hat_thetakm1, double r,
         double dt)
{
  int D = 10;
  double g = 9.81;
  double L = r;
  AccelerationVector u = uk;
  Eigen::VectorXd x = hat_thetakm1;
  Eigen::Matrix2d R;
  R << 0.00377597, -0.00210312, -0.00210312, 0.00125147;

  CovarianceMatrix Q = CovarianceMatrix::Zero();
  Q.diagonal() << 0.00003, 0.00003, 0.0005, 0.0005, 0.0001, 0.0001;
  Eigen::Matrix<double, 2, 6> H;
  H << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

  TransitionMatrix Fi = Fk(x, u, g / r, L, dt);

  Eigen::Vector2d zkp1;
  zkp1 << atan2(-Lvec(1), Lvec(2)), atan2(Lvec(0), sqrt(Lvec(1) * Lvec(1) + Lvec(2) * Lvec(2)));

  for (int i = 0; i < D; ++i)
  {
    x = x + fk(x, u, g / r, L) * dt / D;
  }

  Eigen::MatrixXd barP_kp1 = (Fi * hat_Pkm1 * Fi.transpose()) + Q;
  Eigen::MatrixXd K_kp1 = barP_kp1 * H.transpose() * (R + H * barP_kp1 * H.transpose()).inverse();

  Eigen::VectorXd hat_thetak = x + K_kp1 * (zkp1 - H * x);
  Eigen::MatrixXd hat_Pk = (Eigen::Matrix<double, 6, 6>::Identity() - K_kp1 * H) * barP_kp1;

  return std::make_tuple(hat_thetak, hat_Pk);
}

struct CompareArea
{
  CompareArea(const std::vector<float>& areas) : areas_(&areas)
  {
  }
  bool operator()(int a, int b) const
  {
    return (*areas_)[a] > (*areas_)[b];
  }
  const std::vector<float>* areas_;
};

class CraneVisionEKF : public nodelet::Nodelet {

  virtual void onInit();

};

class CraneVisionNodelet : public nodelet::Nodelet
{
  virtual void onInit();

  bool extractSphereCenters(const sensor_msgs::ImageConstPtr& image_msg,
                            const sensor_msgs::CameraInfoConstPtr& info_msg, const cv::Rect& roi,
                            std::vector<double>& points, bool debug, const std::string& winname);

  void imageCb(const sensor_msgs::ImageConstPtr& image0_msg, const sensor_msgs::CameraInfoConstPtr& info0_msg,
               const sensor_msgs::ImageConstPtr& image1_msg, const sensor_msgs::CameraInfoConstPtr& info1_msg,
               const sensor_msgs::ImageConstPtr& image2_msg, const sensor_msgs::CameraInfoConstPtr& info2_msg);

  boost::shared_ptr<image_transport::ImageTransport> it_;

  /// Subscriptions
  image_transport::SubscriberFilter image0_sub_, image1_sub_, image2_sub_;
  message_filters::Subscriber<CameraInfo> image0_info_sub_, image1_info_sub_, image2_info_sub_;
  typedef ExactTime<Image, CameraInfo, Image, CameraInfo, Image, CameraInfo> ExactPolicy;
  typedef ApproximateTime<Image, CameraInfo, Image, CameraInfo, Image, CameraInfo> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;

  image_transport::CameraSubscriber camera_sub_;
  ros::Publisher pub_;

  int hmin_, hmax_, smin_, smax_, vmin_, vmax_;
  cv::Rect roi0_, roi1_, roi2_;

  Eigen::Matrix<double, 6, 6> hat_Pkm1_;
  StateVector hat_thetakm1_;
  ProjectionMatrix P0_, P1_, P2_;

  bool debug_;
};

void CraneVisionNodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  it_.reset(new image_transport::ImageTransport(nh));

  // Synchronize inputs. Topic subscriptions happen on demand in the connection
  // callback. Optionally do approximate synchronization.
  int queue_size;
  private_nh.param("queue_size", queue_size, 5);
  bool approx;
  private_nh.param("approximate_sync", approx, false);
  if (approx)
  {
    approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size), image0_sub_, image0_info_sub_,
                                                image1_sub_, image1_info_sub_, image2_sub_, image2_info_sub_));
    approximate_sync_->registerCallback(boost::bind(&CraneVisionNodelet::imageCb, this, _1, _2, _3, _4, _5, _6));
  }
  else
  {
    exact_sync_.reset(new ExactSync(ExactPolicy(queue_size), image0_sub_, image0_info_sub_, image1_sub_,
                                    image1_info_sub_, image2_sub_, image2_info_sub_));
    exact_sync_->registerCallback(boost::bind(&CraneVisionNodelet::imageCb, this, _1, _2, _3, _4, _5, _6));
  }

  std::vector<int> roi;
  if (private_nh.getParam("roi0", roi))
  {
    roi0_ = cv::Rect(roi[0], roi[1], roi[2], roi[3]);
  }
  else
  {
    NODELET_ERROR_STREAM("Did not find any roi0 parameters!");
  }
  if (private_nh.getParam("roi1", roi))
  {
    roi1_ = cv::Rect(roi[0], roi[1], roi[2], roi[3]);
  }
  else
  {
    NODELET_ERROR_STREAM("Did not find any roi1 parameters!");
  }
  if (private_nh.getParam("roi2", roi))
  {
    roi2_ = cv::Rect(roi[0], roi[1], roi[2], roi[3]);
  }
  else
  {
    NODELET_ERROR_STREAM("Did not find any roi2 parameters!");
  }

  private_nh.param("debug", debug_, false);

  private_nh.param("hmin", hmin_, 43);
  private_nh.param("hmax", hmax_, 73);
  private_nh.param("smin", smin_, 54);
  private_nh.param("smax", smax_, 250);
  private_nh.param("vmin", vmin_, 86);
  private_nh.param("vmax", vmax_, 255);

  image0_sub_.subscribe(*it_, "/camera0/image_raw", 1);
  image0_info_sub_.subscribe(nh, "/camera0/camera_info", 1);
  image1_sub_.subscribe(*it_, "/camera1/image_raw", 1);
  image1_info_sub_.subscribe(nh, "/camera1/camera_info", 1);
  image2_sub_.subscribe(*it_, "/camera2/image_raw", 1);
  image2_info_sub_.subscribe(nh, "/camera2/camera_info", 1);

  pub_ = private_nh.advertise<crane_msgs::CranePendulumImagePoints>("points", 1);

  P0_ << 942.0, 0.0, 623.66, 0.0, 0.0, 942.0, 345.69, 0.0, 0.0, 0.0, 1.0, 0.0;
  P1_ << 941.0, 0.0, 637.21, 220005.80000000002, 0.0, 941.0, 349.9, 0.0, 0.0, 0.0, 1.0, 0.0;
  P2_ << 937.0, 0.0, 637.21, 437579.0, 0.0, 937.0, 381.54, 0.0, 0.0, 0.0, 1.0, 0.0;

  hat_thetakm1_.setZero();
  // hat_Pkm1_ << 0.0, 0.0, 0.0, 0.0, 0.04, -0.03;
}

bool CraneVisionNodelet::extractSphereCenters(const sensor_msgs::ImageConstPtr& image_msg,
                                              const sensor_msgs::CameraInfoConstPtr& info_msg, const cv::Rect& roi,
                                              std::vector<double>& points, bool debug, const std::string& winname)
{
  using namespace std;
  using namespace cv;

  Mat bgr8_image = cv_bridge::toCvShare(image_msg, "bgr8")->image;

  Mat roi_image = bgr8_image(roi);

  Mat hsv_image;
  cvtColor(roi_image, hsv_image, CV_BGR2HSV);

  Mat thresh_image;
  inRange(hsv_image, Scalar(hmin_, smin_, vmin_), Scalar(hmax_, smax_, vmax_), thresh_image);

  using Contours = vector<vector<Point> >;
  using Hierarchy = vector<Vec4i>;
  Contours contours;
  Hierarchy hierarchy;

  /// Find contours
  findContours(thresh_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
  if (contours.size() > 1)
  {
    /// Get sort index based on contour area
    vector<int> sort_idx(contours.size());
    vector<float> areas(contours.size());
    for (int n = 0; n < (int)contours.size(); n++)
    {
      sort_idx[n] = n;
      areas[n] = contourArea(contours[n], false);
    }
    sort(sort_idx.begin(), sort_idx.end(), CompareArea(areas));

    /// Get the moments
    vector<Moments> mu(contours.size());
    for (int i = 0; i < contours.size(); i++)
    {
      mu[i] = moments(contours[i], false);
    }

    ///  Get the mass centers:
    vector<Point2d> mc(contours.size());
    for (int i = 0; i < contours.size(); i++)
    {
      mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
    }

    /// Swap to get right order
    if (mc[sort_idx[1]].y > mc[sort_idx[0]].y)
    {
      points[0] = mc[sort_idx[1]].x;
      points[1] = mc[sort_idx[1]].y;
      points[2] = mc[sort_idx[0]].x;
      points[3] = mc[sort_idx[0]].y;
    }
    else
    {
      points[0] = mc[sort_idx[0]].x;
      points[1] = mc[sort_idx[0]].y;
      points[2] = mc[sort_idx[1]].x;
      points[3] = mc[sort_idx[1]].y;
    }

    if (debug)
    {
      circle(roi_image, Point2d(points[0], points[1]), 5, Scalar(0, 0, 255), -1, 8, 0);
      circle(roi_image, Point2d(points[2], points[3]), 5, Scalar(255, 0, 0), -1, 8, 0);
      imshow(winname, roi_image);
      waitKey(30);
    }
  }
}

void CraneVisionNodelet::imageCb(const sensor_msgs::ImageConstPtr& image0_msg,
                                 const sensor_msgs::CameraInfoConstPtr& info0_msg,
                                 const sensor_msgs::ImageConstPtr& image1_msg,
                                 const sensor_msgs::CameraInfoConstPtr& info1_msg,
                                 const sensor_msgs::ImageConstPtr& image2_msg,
                                 const sensor_msgs::CameraInfoConstPtr& info2_msg)
{
  using namespace std;
  using namespace Eigen;

  vector<double> points0{ 0.0, 0.0, 0.0, 0.0 };
  vector<double> points1{ 0.0, 0.0, 0.0, 0.0 };
  vector<double> points2{ 0.0, 0.0, 0.0, 0.0 };

  extractSphereCenters(image0_msg, info0_msg, roi0_, points0, debug_, "camera0");
  extractSphereCenters(image1_msg, info1_msg, roi1_, points1, debug_, "camera1");
  extractSphereCenters(image2_msg, info2_msg, roi2_, points2, debug_, "camera2");

  Vector2d center01 = Vector2d(points0[0] + roi0_.x, points0[1] + roi0_.y);
  Vector2d center02 = Vector2d(points0[2] + roi0_.x, points0[3] + roi0_.y);
  Vector2d center11 = Vector2d(points1[0] + roi1_.x, points1[1] + roi1_.y);
  Vector2d center12 = Vector2d(points1[2] + roi1_.x, points1[3] + roi1_.x);
  Vector2d center21 = Vector2d(points2[0] + roi2_.x, points2[1] + roi2_.y);
  Vector2d center22 = Vector2d(points2[2] + roi2_.x, points2[3] + roi2_.y);

  // Vector3d Lc0 = findLine(center01, center02, center11, center12, center21, center22, P0_, P1_, P2_);



  // todo(Lars): Transform to inertial coordinatesA
  // Vector3d Lvec = Lc0;

  // std::tie(hat_thetakm1_, hat_Pkm1_) = ekf(Lvec, AccelerationVector(0.0,0.0), hat_thetakm1_, hat_Pkm1_, 1.05, 0.01);

  crane_msgs::CranePendulumImagePoints msg;
  // Reuse the synchronized time stamp
  msg.header.stamp = image0_msg->header.stamp;
  std::vector<double> data(12);
  msg.points =
      std::vector<double>{ points0[0] + roi0_.x, points0[1] + roi0_.y, points0[2] + roi0_.x, points0[3] + roi0_.y,
                           points1[0] + roi1_.y, points1[1] + roi1_.y, points1[2] + roi1_.y, points1[3] + roi1_.y,
                           points2[0] + roi2_.y, points2[1] + roi2_.y, points2[2] + roi2_.y, points2[3] + roi2_.y };
  pub_.publish(msg);
}

}  // namespace crane_vision

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(crane_vision::CraneVisionNodelet, nodelet::Nodelet)