/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#pragma once

#include <iostream>
#include <thread>
#include <memory>
#include <string>
#include <map>
#include <cmath>
#include <fstream>

#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
// #include <op/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

// #include <opencv4/opencv2/imgcodecs.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pose_estimator/kalman_filter.hpp>
#include <filters/mean.hpp>
#include <filters/median.hpp>

// #include <pose_estimator/

class PoseEstimator : public rclcpp::Node
{
private:
  message_filters::Subscriber<sensor_msgs::msg::Image> subscription_rgb_;
  message_filters::Subscriber<sensor_msgs::msg::Image> subscription_depth_;

  void image_raw_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg);

  cv::Mat cameraMatrix_ =
    (cv::Mat_<double>(3, 3) << 974.724, 0.0, 1024.82, 0.0, 974.456, 773.291, 0, 0, 1.0 );

  // In OpenCV, the distortion coefficients are usually represented as a 1x5 (or sometimes 1x8) matrix:
  // distCoeffs_= [k1, k2, p1, p2, k3] where
  // k1, k2, k3 = radial distortion coefficients
  // p1, p2 = tangential distortion coefficients
  // For Azure kinect, they can be found when running the ROS2 camera node and explained in the following:
  // https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/structk4a__calibration__intrinsic__parameters__t_1_1__param.html

  cv::Mat distCoeffs_ =
    (cv::Mat_<double>(5, 1) << 0.602113, -2.92716, 0.000402061, -0.000392198, 1.63343);

  std::vector<int> markerIds_;
  std::vector<std::vector<cv::Point2f>> markerCorners_, rejectedCandidates_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  using sync_policy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
      sensor_msgs::msg::Image>;

  // Define synchronizer
  std::shared_ptr<message_filters::Synchronizer<sync_policy>> sync_;

  // Kalman filter 
  KalmanFilter kalman_filter_;


  std::shared_ptr<filters::MultiChannelFilterBase<double>> median_filter_ =
    std::make_shared<filters::MultiChannelMedianFilter<double>>();


  std::vector<double> median_filtered_rpy;
  // std::vector<double> raw_rpy;

  // << 0.0, 0.0, 0.0;

  rclcpp::Time last_time_;
  bool last_time_flag_ = false;
  double dt = 0.0001;

  void appendVectorsToCSV(const std::string& filename, std::vector<double> data);

public:
  PoseEstimator();
};
