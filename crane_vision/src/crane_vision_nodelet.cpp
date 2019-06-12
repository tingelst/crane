#include <opencv2/opencv.hpp>

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

namespace crane_vision
{
using namespace sensor_msgs;
using namespace message_filters::sync_policies;

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

class CraneVisionNodelet : public nodelet::Nodelet
{
  virtual void onInit();

  std::vector<double> imageCb(const sensor_msgs::ImageConstPtr& image_msg,
                              const sensor_msgs::CameraInfoConstPtr& info_msg);

  void imageCb2(const sensor_msgs::ImageConstPtr& image0_msg, const sensor_msgs::CameraInfoConstPtr& info0_msg,
                const sensor_msgs::ImageConstPtr& image1_msg, const sensor_msgs::CameraInfoConstPtr& info1_msg);

  boost::shared_ptr<image_transport::ImageTransport> it_;

  /// Subscriptions
  image_transport::SubscriberFilter image0_sub_, image1_sub_, image2_sub_;
  message_filters::Subscriber<CameraInfo> image0_info_sub_, image1_info_sub_, image2_info_sub_;
  typedef ExactTime<Image, CameraInfo, Image, CameraInfo> ExactPolicy;
  typedef ApproximateTime<Image, CameraInfo, Image, CameraInfo> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;

  image_transport::CameraSubscriber camera_sub_;
  ros::Publisher pub_;

  int hmin_, hmax_, smin_, smax_, vmin_, vmax_;
  cv::Rect roi_;
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
                                                image1_sub_, image1_info_sub_));
    approximate_sync_->registerCallback(boost::bind(&CraneVisionNodelet::imageCb2, this, _1, _2, _3, _4));
  }
  else
  {
    exact_sync_.reset(
        new ExactSync(ExactPolicy(queue_size), image0_sub_, image0_info_sub_, image1_sub_, image1_info_sub_));
    exact_sync_->registerCallback(boost::bind(&CraneVisionNodelet::imageCb2, this, _1, _2, _3, _4));
  }

  std::vector<int> roi;
  if (private_nh.getParam("roi0", roi))
  {
    roi_ = cv::Rect(roi[0], roi[1], roi[2], roi[3]);
  }
  else
  {
    NODELET_ERROR_STREAM("Did not find any roi parameters!");
  }

  private_nh.param("hmin", hmin_, 43);
  private_nh.param("hmax", hmax_, 73);
  private_nh.param("smin", smin_, 54);
  private_nh.param("smax", smax_, 250);
  private_nh.param("vmin", vmin_, 86);
  private_nh.param("vmax", vmax_, 255);

  // camera_sub_ = it_->subscribeCamera("image_raw", 3, &CraneVisionNodelet::imageCb, this);

  image0_sub_.subscribe(*it_, "/camera0/image_raw", 1);
  image0_info_sub_.subscribe(nh, "/camera0/camera_info", 1);
  image1_sub_.subscribe(*it_, "/camera1/image_raw", 1);
  image1_info_sub_.subscribe(nh, "/camera1/camera_info", 1);

  pub_ = private_nh.advertise<std_msgs::Float64MultiArray>("points", 1);
}

std::vector<double> CraneVisionNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
                                                const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  using namespace std;
  using namespace cv;

  Mat bgr8_image = cv_bridge::toCvShare(image_msg, "bgr8")->image;

  Mat roi_image = bgr8_image(roi_);

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
      return std::vector<double>{ mc[sort_idx[1]].x, mc[sort_idx[1]].y, mc[sort_idx[0]].x, mc[sort_idx[0]].y };
    }
    else
    {
      return std::vector<double>{ mc[sort_idx[0]].x, mc[sort_idx[0]].y, mc[sort_idx[1]].x, mc[sort_idx[1]].y };
    }
  }
}

void CraneVisionNodelet::imageCb2(const sensor_msgs::ImageConstPtr& image0_msg,
                                  const sensor_msgs::CameraInfoConstPtr& info0_msg,
                                  const sensor_msgs::ImageConstPtr& image1_msg,
                                  const sensor_msgs::CameraInfoConstPtr& info1_msg)
{
  std::vector<double> points0 = imageCb(image0_msg, info0_msg);
  std::vector<double> points1 = imageCb(image0_msg, info0_msg);
  std::vector<double> points2 = imageCb(image0_msg, info0_msg);

  std_msgs::Float64MultiArray msg;
  msg.data = std::vector<double>{ 1.0, 2.0, 3.0, 4.0 };
  pub_.publish(msg);
}

}  // namespace crane_vision

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(crane_vision::CraneVisionNodelet, nodelet::Nodelet)