#include <iostream>

#include <opencv2/opencv.hpp>

#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;

image_transport::Publisher pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  Mat bgr8_image = cv_bridge::toCvShare(msg, "bgr8")->image;

  Mat hsv_image;
  cvtColor(bgr8_image, hsv_image, CV_BGR2HSV);

  Mat thresh_image;
  inRange(hsv_image, Scalar(43, 54, 86), Scalar(73, 250, 255), thresh_image);

  using Contours = vector<vector<Point> >;
  using Hierarchy = vector<Vec4i>;
  Contours contours;
  Hierarchy hierarchy;

  /// Find contours
  findContours(thresh_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

  RNG rng(12345);

  Mat drawing = Mat::zeros(thresh_image.size(), CV_8UC3);
  for (int i = 0; i < contours.size(); i++)
  {
    Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
  }

  sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", drawing).toImageMsg();
  pub.publish(msg2);

//   imshow("image", drawing);
//   waitKey(30);
}

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "cv_example");

  // node handler
  ros::NodeHandle node_handle;

  // subsribe topic

  image_transport::ImageTransport it(node_handle);
  image_transport::Subscriber sub = it.subscribe("camera0/image_raw", 1, imageCallback);

  pub = it.advertise("/contours", 1);

  ros::spin();

  return 0;
}