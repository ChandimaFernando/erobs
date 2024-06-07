/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

class KalmanFilter
{

private:
  Eigen::Vector3d state_;   // [roll, pitch, yaw]
  Eigen::Matrix3d P_;       // Covariance matrix
  Eigen::Matrix3d Q_;       // Process noise covariance
  Eigen::Matrix3d R_;       // Measurement noise covariance

  Eigen::Matrix3d F_;       // State transition matrix
  Eigen::Matrix3d H_;       // Observation matrix

public:
  KalmanFilter();
  void predict(const Eigen::Vector3d & u, double dt);
  void update(const Eigen::Vector3d & z);

  Eigen::Vector3d getState() const;
};
