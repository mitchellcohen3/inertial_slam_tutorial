#pragma once

#include <Eigen/Dense>

// Converts an IMU state to an ASL format vector
// Ordering is as follows:
// stamp, position, qw, qx, qy, qz, vel, bg, ba
Eigen::Matrix<double, 17, 1>
toAslFormat(const Eigen::Matrix3d attitude, const Eigen::Vector3d &velocity,
            const Eigen::Vector3d &position, const Eigen::Vector3d &bg,
            const Eigen::Vector3d &ba, double stamp) {
  Eigen::Matrix<double, 17, 1> vec;
  vec(0) = stamp;
  vec.block<3, 1>(1, 0) = position;

  Eigen::Quaterniond quat(attitude);
  vec(4, 0) = quat.w();
  vec(5, 0) = quat.x();
  vec(6, 0) = quat.y();
  vec(7, 0) = quat.z();
  vec.block<3, 1>(8, 0) = velocity;
  vec.block<3, 1>(11, 0) = bg;
  vec.block<3, 1>(14, 0) = ba;
  return vec;
}
