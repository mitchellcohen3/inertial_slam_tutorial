#pragma once

#include "utils/SensorData.h"
#include <Eigen/Dense>
#include <vector>

/**
 * @brief Plain struct for returning the latest IMU navigation state from any
 * estimator implementation.
 */
struct NavStateEstimate {
  double timestamp = -1;
  Eigen::Matrix3d attitude = Eigen::Matrix3d::Identity();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
};

/**
 * @brief Abstract base class for SLAM estimators.
 *
 * Defines the interface shared by EKF-based and iSAM2-based SLAM estimators,
 * allowing the example runner to work with either without modification.
 */
class SlamEstimatorBase {
public:
  virtual ~SlamEstimatorBase() = default;

  virtual void
  initializeIMUState(double stamp,
                     const Eigen::Matrix<double, 5, 5> &nav_state,
                     const Eigen::Vector3d &gyro_bias,
                     const Eigen::Vector3d &accel_bias,
                     const Eigen::Matrix<double, 15, 15> &init_cov) = 0;

  virtual void inputIMU(ImuMessage &imu) = 0;

  virtual void inputRelativeFeatureMeasurements(
      std::vector<RelativeFeatureMessage> &feats, double stamp) = 0;

  virtual NavStateEstimate getLatestState() const = 0;

  virtual Eigen::Matrix<double, 15, 15> getLatestIMUCovariance() const = 0;

  virtual std::vector<Eigen::Vector3d> getEstimatedMap() const = 0;

  virtual double getEstimateTime() const = 0;
};
