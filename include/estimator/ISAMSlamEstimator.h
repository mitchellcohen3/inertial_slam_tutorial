#pragma once

#include "estimator/SlamEstimatorBase.h"
#include "estimator/ImuKinematicsConfig.h"
#include "utils/SensorData.h"

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <unordered_map>

/**
 * @brief iSAM2-based SLAM estimator.
 *
 * Maintains an instance of iSAM2 that is updated incrementally with new IMU and relative feature
 * measurements.
 * 
 * This example is based on the following examples from the GTSAM repository:
 *     - ImuFactorExample.cpp for the IMU preintegration
 *     - VisualiSAM2Example.cpp for how to incrementally update the graph with new measurements
 *     and retrieve the latest state estimate and covariance.
 */
class ISAMSlamEstimator : public SlamEstimatorBase {
public:
  ISAMSlamEstimator(const KinematicsConfig &kinematics_config);

  void initializeIMUState(double stamp,
                          const Eigen::Matrix<double, 5, 5> &nav_state,
                          const Eigen::Vector3d &gyro_bias,
                          const Eigen::Vector3d &accel_bias,
                          const Eigen::Matrix<double, 15, 15> &init_cov) override;

  void inputIMU(ImuMessage &imu) override;

  void inputRelativeFeatureMeasurements(
      std::vector<RelativeFeatureMessage> &feats, double stamp) override;

  NavStateEstimate getLatestState() const override;

  Eigen::Matrix<double, 15, 15> getLatestIMUCovariance() const override;

  std::vector<Eigen::Vector3d> getEstimatedMap() const override;

  double getEstimateTime() const override;

private:
  /**
   * @brief Preintegrate the buffered IMU measurements up to the given timestamp.
   * Also predicts the current state forward to that timestamp using the 
   * preintegrated measurements.
   */
  void integrateMeasurementsToStamp(double stamp);

  /**
   * @brief Propagate forward our latest state estimate by integrating the 
   * IMU measurements
   */
  void predictIMUState(double dt, const Eigen::Vector3d &gyro, const Eigen::Vector3d &accel);

  // iSAM2 object that maintains the factor graph and performs incremental updates
  gtsam::ISAM2 isam_;

  // The new factors and values to be added at the next update step
  // These are cleared after each update
  gtsam::NonlinearFactorGraph new_factors_;
  gtsam::Values new_values_;

  // IMU preintegration object that resets after each update
  // This is what's used to create the preintegrated IMU factor each time we get new measurements
  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> preintegration_;
  boost::shared_ptr<gtsam::PreintegrationCombinedParams> preint_params_;

  // Buffered IMU measurements to be preintegrated
  std::vector<ImuMessage> imu_buffer_;

  // Current best estimate, updated after each call to inputRelativeFeatureMeasurements
  NavStateEstimate current_state_;
  gtsam::imuBias::ConstantBias current_bias_;

  // Increments each time we add a new pose to the graph
  size_t pose_index_ = 0;

  // Maps simulator feature IDs to their ISAM2 key index (L(j))
  std::unordered_map<size_t, size_t> feature_id_to_key_index_;

  bool is_initialized_ = false;
  double last_stamp_ = -1;

  // Gravity vector in the world frame
  Eigen::Vector3d gravity_;
};
