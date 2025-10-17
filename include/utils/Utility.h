#pragma once

#include <Eigen/Dense>

// Converts an IMU state to an ASL format vector
// Ordering is as follows:
// stamp, position, qw, qx, qy, qz, vel, bg, ba
Eigen::Matrix<double, 17, 1>
toAslFormat(const Eigen::Matrix3d attitude, const Eigen::Vector3d &velocity,
            const Eigen::Vector3d &position, const Eigen::Vector3d &bg,
            const Eigen::Vector3d &ba, double stamp);

double roundTo(double value, double precision);
double roundStamp(double value);
