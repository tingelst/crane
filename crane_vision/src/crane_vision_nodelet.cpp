#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include "cv_bridge/cv_bridge.h"
// #include "sensor_msgs/Image.h"

namespace crane_vision
{
class CraneVisionNodelet : public nodelet::Nodelet
{
  virtual void onInit();

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);

  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber camera_sub_;
};

void CraneVisionNodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  it_.reset(new image_transport::ImageTransport(nh));

  camera_sub_ = it_->subscribeCamera("camera0/image_raw", 3, &CraneVisionNodelet::imageCb, this);
}

void CraneVisionNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
                                 const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  NODELET_INFO_STREAM("Got image");
}

}  // namespace crane_vision

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(crane_vision::CraneVisionNodelet, nodelet::Nodelet)