#pragma once

#include <Eigen/Dense>
#include <vector>

struct ImuMessage {
  double timestamp;
  Eigen::Vector3d gyro;
  Eigen::Vector3d accel;

  ImuMessage(double timestamp, Eigen::Vector3d gyro, Eigen::Vector3d accel)
      : timestamp(timestamp), gyro(gyro), accel(accel) {}

  ImuMessage() {
    timestamp = 0.0;
    gyro = Eigen::Vector3d::Zero();
    accel = Eigen::Vector3d::Zero();
  }

  // Sort function to allow sorting of STL containers
  bool operator<(const ImuMessage &other) const {
    return timestamp < other.timestamp;
  }
};

struct GpsMessage {
  double timestamp;
  Eigen::Vector3d meas;
  Eigen::Matrix3d covariance;

  GpsMessage() {
    timestamp = 0.0;
    meas = Eigen::Vector3d::Zero();
    covariance = Eigen::Matrix3d::Zero();
  }

  GpsMessage(double timestamp, const Eigen::Vector3d &meas,
             const Eigen::Matrix3d &cov)
      : timestamp(timestamp), meas(meas), covariance(cov) {}

  // Sort function to allow sorting of STL containers
  bool operator<(const GpsMessage &other) const {
    return timestamp < other.timestamp;
  }
};

struct RelativeFeatureMessage {
  // Timestamp of reading
  double timestamp;

  // Relative position measurement
  Eigen::Vector3d meas;

  // Unique ID of this measurement
  size_t feature_id;

  // Measurement covariance
  Eigen::Matrix3d covariance;

  RelativeFeatureMessage() {
    timestamp = 0.0;
    meas = Eigen::Vector3d::Zero();
    feature_id = 0;
    covariance = Eigen::Matrix3d::Zero();
  }

  RelativeFeatureMessage(double timestamp, const Eigen::Vector3d &meas,
                         size_t feature_id, const Eigen::Matrix3d &cov)
      : timestamp(timestamp), meas(meas), feature_id(feature_id),
        covariance(cov) {}
};