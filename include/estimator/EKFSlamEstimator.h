#pragma once

#include "estimator/ImuKinematicsConfig.h"
#include "utils/SensorData.h"
#include "lieutils/SO3.h"

#include <Eigen/Dense>
#include <memory>

class ImuPropagator;
class ImuEKFState;

class EstimatorConfig {
public:
  bool use_fej = false;
  LieDirection lie_direction = LieDirection::left;

  KinematicsConfig kinematics_config;

  EstimatorConfig() {}
};

class EKFState;

/**
 * @brief An Extended Kalman Filter (EKF) based SLAM estimator
 *
 * Contains the logic for propagating the IMU state, and processing
 * 3D SLAM feature measurements to update the state.
 */
class EKFSlamEstimator {
public:
  EKFSlamEstimator(const EstimatorConfig &config_);

  void inputIMU(ImuMessage &imu_data);
  void inputRelativeFeatureMeasurements(
      std::vector<RelativeFeatureMessage> &relative_feat_meas, double stamp);

  void initializeIMUState(double stamp,
                          const Eigen::Matrix<double, 5, 5> &nav_state,
                          const Eigen::Vector3d &gyro_bias,
                          const Eigen::Vector3d &accel_bias,
                          const Eigen::Matrix<double, 15, 15> &init_imu_cov);

  std::shared_ptr<ImuEKFState> getLatestIMUState() const;
  Eigen::Matrix<double, 15, 15> getLatestIMUCovariance() const;
  std::vector<Eigen::Vector3d> getEstimatedMap() const;
  double getEstimateTime() const;

protected:
  void propagateIMUStateToStamp(double stamp);

  void performEKFUpdate(const std::vector<RelativeFeatureMessage> &meas,
                        double stamp);

  void marginalizeOldFeatures(double stamp);

  // The main configuration parameters for the estimator
  EstimatorConfig config_;

  // Our EKF state and covariance, to be manipulated
  // through the EKFStateHelper class
  std::shared_ptr<EKFState> state_;
  std::shared_ptr<ImuPropagator> imu_propagator_;

  /**
   * @brief Stores the first estimates of the landmarks for FEJ
   */
  std::unordered_map<size_t, Eigen::Vector3d> fej_landmarks_;

  // Timing and initialization
  double startup_time_ = -1;
  bool is_initialized_ = false;
  bool first_frame_ = true;
  bool optimized_first_window_ = false;

  double last_imu_time_ = -1;
  size_t frame_id = 0;
};

void computeMeasurementModelJacobians(const Eigen::Matrix3d &C_ab,
                      const Eigen::Vector3d &r_zw_a,
                      const Eigen::Vector3d &r_pw_a, Eigen::Matrix3d &att_jac,
                      Eigen::Matrix3d &pos_jac, Eigen::Matrix3d &feat_jac,
                      LieDirection direction);