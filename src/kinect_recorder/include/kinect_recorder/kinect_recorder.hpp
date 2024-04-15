/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#pragma once

#include <iostream>
#include <thread>
#include <memory>
#include <string>
#include <map>

#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
// #include <op/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
// #include <opencv4/opencv2/imgcodecs.hpp>
#include <image_transport/image_transport.hpp>

#include <sensor_msgs/msg/image.hpp>

class KinectRecorder : public rclcpp::Node
{
private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_rgb_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_depth_;

  void rgb_raw_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void depth_raw_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

public:
  KinectRecorder();
};
