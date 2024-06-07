/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#include <pose_estimator/kalman_filter.hpp>

KalmanFilter::KalmanFilter()
{
  state_.setZero();
  P_.setIdentity();
  Q_ = Eigen::Matrix3d::Identity() * 1;
  R_ = Eigen::Matrix3d::Identity() * 1;

  F_.setIdentity();
  H_.setIdentity();
}

void KalmanFilter::predict(const Eigen::Vector3d & u, double dt)
{
  // Predict the state using a simple constant velocity model
  F_(0, 1) = dt;
  F_(1, 2) = dt;

  state_ = F_ * state_ + u * dt;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::Vector3d & z)
{
  Eigen::Vector3d y = z - H_ * state_;   // Innovation
  Eigen::Matrix3d S = H_ * P_ * H_.transpose() + R_;
  Eigen::Matrix3d K = P_ * H_.transpose() * S.inverse();

  state_ = state_ + K * y;
  P_ = (Eigen::Matrix3d::Identity() - K * H_) * P_;
}

Eigen::Vector3d KalmanFilter::getState() const
{
  return state_;
}
