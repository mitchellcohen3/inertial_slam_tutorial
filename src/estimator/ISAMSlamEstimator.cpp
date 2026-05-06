#include "estimator/ISAMSlamEstimator.h"
#include "estimator/RelativeFeatureFactor.h"

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <glog/logging.h>

#include "estimator/ImuPropagator.h"

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

ISAMSlamEstimator::ISAMSlamEstimator(const KinematicsConfig &kinematics_config) {
  // Create the ISam2 object
  gtsam::ISAM2Params isam_params;
  isam_params.relinearizeThreshold = 0.1;
  isam_params.relinearizeSkip = 1;
  isam_ = gtsam::ISAM2(isam_params);

  // Set up IMU preintegration parameters from our kinematics config
  preint_params_ = gtsam::PreintegrationCombinedParams::MakeSharedU(
      kinematics_config.gravity_mag);
  const ImuNoises &n = kinematics_config.imu_noises;
  preint_params_->setAccelerometerCovariance(
      gtsam::I_3x3 * std::pow(n.sigma_accel, 2));
  preint_params_->setGyroscopeCovariance(
      gtsam::I_3x3 * std::pow(n.sigma_gyro, 2));
  preint_params_->setBiasAccCovariance(
      gtsam::I_3x3 * std::pow(n.sigma_accel_bias, 2));
  preint_params_->setBiasOmegaCovariance(
      gtsam::I_3x3 * std::pow(n.sigma_gyro_bias, 2));
  preint_params_->setBiasAccOmegaInit(gtsam::Z_6x6);

  gravity_ = Eigen::Vector3d(0, 0, -kinematics_config.gravity_mag);
}

void ISAMSlamEstimator::initializeIMUState(
    double stamp, const Eigen::Matrix<double, 5, 5> &nav_state,
    const Eigen::Vector3d &gyro_bias, const Eigen::Vector3d &accel_bias,
    const Eigen::Matrix<double, 15, 15> &init_cov) {

  // Extract the initial navigation state from the SE_2(3) matrix
  Eigen::Matrix3d C = nav_state.block<3, 3>(0, 0);
  Eigen::Vector3d vel = nav_state.block<3, 1>(0, 3);
  Eigen::Vector3d pos = nav_state.block<3, 1>(0, 4);

  gtsam::Rot3 R0(C);
  gtsam::Pose3 pose0(R0, gtsam::Point3(pos));
  gtsam::Vector3 vel0 = vel;
  current_bias_ = gtsam::imuBias::ConstantBias(accel_bias, gyro_bias);

  // Add a prior factor on initial pose, velocity, and bias
  auto pose_noise = gtsam::noiseModel::Gaussian::Covariance(
      (gtsam::Matrix66() << init_cov.block<3, 3>(0, 0),
       gtsam::Z_3x3,
       gtsam::Z_3x3,
       init_cov.block<3, 3>(6, 6))
          .finished());
  auto vel_noise = gtsam::noiseModel::Gaussian::Covariance(
      init_cov.block<3, 3>(3, 3));
  auto bias_noise = gtsam::noiseModel::Gaussian::Covariance(
      (gtsam::Matrix66() << init_cov.block<3, 3>(9, 9),
       gtsam::Z_3x3,
       gtsam::Z_3x3,
       init_cov.block<3, 3>(12, 12))
          .finished());

  new_factors_.addPrior(X(0), pose0, pose_noise);
  new_factors_.addPrior(V(0), vel0, vel_noise);
  new_factors_.addPrior(B(0), current_bias_, bias_noise);

  new_values_.insert(X(0), pose0);
  new_values_.insert(V(0), vel0);
  new_values_.insert(B(0), current_bias_);

  isam_.update(new_factors_, new_values_);
  new_factors_.resize(0);
  new_values_.clear();

  // Reset the preintegrator with the initial bias
  preintegration_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(
      preint_params_, current_bias_);

  current_state_.timestamp = stamp;
  current_state_.attitude = C;
  current_state_.velocity = vel;
  current_state_.position = pos;
  current_state_.gyro_bias = gyro_bias;
  current_state_.accel_bias = accel_bias;

  last_stamp_ = stamp;
  is_initialized_ = true;

  LOG(INFO) << "ISAM IMU state initialized at time " << stamp;
}

void ISAMSlamEstimator::inputIMU(ImuMessage &imu) {
  if (!is_initialized_) {
    return;
  }
  imu_buffer_.push_back(imu);
}

void ISAMSlamEstimator::integrateMeasurementsToStamp(double stamp) {
  // Preintegrate all the IMU measurements up until the specified timestamp
  for (const auto &msg : imu_buffer_) {
    if (msg.timestamp > stamp) {
      break;
    }

    double dt = msg.timestamp - last_stamp_;
    if (dt <= 0) {
      LOG(WARNING) << "Non-positive IMU dt: " << dt
                   << " | last_stamp: " << last_stamp_
                   << " | current_stamp: " << msg.timestamp;  
      continue;
    }
    // Printegrate the IMU measurements and predict the state forward
    preintegration_->integrateMeasurement(msg.accel, msg.gyro, dt);
    predictIMUState(dt, msg.gyro, msg.accel);
    last_stamp_ = msg.timestamp;
  }

  // Remove all the IMU messages that we're used from the buffer
  imu_buffer_.erase(
      std::remove_if(imu_buffer_.begin(), imu_buffer_.end(),
                     [&](const ImuMessage &m) { return m.timestamp <= stamp; }),
      imu_buffer_.end());
}

void ISAMSlamEstimator::predictIMUState(double dt, const Eigen::Vector3d &gyro, const Eigen::Vector3d &accel) {
  Eigen::Matrix<double, 5, 5> prev_extended_pose = Eigen::Matrix<double, 5, 5>::Identity();
  prev_extended_pose.block<3, 3>(0, 0) = current_state_.attitude;
  prev_extended_pose.block<3, 1>(0, 3) = current_state_.velocity;
  prev_extended_pose.block<3, 1>(0, 4) = current_state_.position;

  Eigen::Vector3d unbiased_gyro = gyro - current_state_.gyro_bias;
  Eigen::Vector3d unbiased_accel = accel - current_state_.accel_bias;

  Eigen::Matrix<double, 5, 5> G = createGMatrix(gravity_, dt);
  Eigen::Matrix<double, 5, 5> U =
      createUMatrix(unbiased_gyro, unbiased_accel, dt);

  // Propagate forward the pose
  Eigen::Matrix<double, 5, 5> next_extended_pose = G * prev_extended_pose * U;
  current_state_.timestamp += dt;
  current_state_.attitude = next_extended_pose.block<3, 3>(0, 0);
  current_state_.velocity = next_extended_pose.block<3, 1>(0, 3);
  current_state_.position = next_extended_pose.block<3, 1>(0, 4);
}

void ISAMSlamEstimator::inputRelativeFeatureMeasurements(
    std::vector<RelativeFeatureMessage> &feats, double stamp) {
  if (!is_initialized_) {
    LOG(WARNING) << "ISAM estimator not initialized.";
    return;
  }

  // Predict the IMU forward, integrating all measurements up to the current 
  // timestamp
  integrateMeasurementsToStamp(stamp);

  if (std::abs(current_state_.timestamp - stamp) > 1e-5) {
    LOG(WARNING) << "Current state timestamp (" << current_state_.timestamp
                 << ") does not match measurement timestamp (" << stamp
                 << "). Check IMU integration.";
  }

  size_t prev_idx = pose_index_;
  size_t curr_idx = ++pose_index_;

  // Add a preintegrated IMU factor that connects the previous state to the 
  gtsam::CombinedImuFactor imu_factor(X(prev_idx), V(prev_idx), X(curr_idx),
                                      V(curr_idx), B(prev_idx), B(curr_idx),
                                      *preintegration_);
  new_factors_.add(imu_factor);

  // Create the new values 
  gtsam::Rot3 R_curr(current_state_.attitude);
  gtsam::Pose3 pose_curr(R_curr, gtsam::Point3(current_state_.position));
  gtsam::Vector3 vel_curr = current_state_.velocity;
  current_bias_ = gtsam::imuBias::ConstantBias(current_state_.accel_bias,
                                               current_state_.gyro_bias); 
  new_values_.insert(X(curr_idx), pose_curr);
  new_values_.insert(V(curr_idx), vel_curr);
  new_values_.insert(B(curr_idx), current_bias_);

  // Isotropic noise model for body-frame landmark measurements

  for (auto &feat : feats) {
    // Check if we've seen this landmark before
    // if not, add a new variable and initialize the estimate by 
    // inverting the measurement model
    if (feature_id_to_key_index_.find(feat.feature_id) ==
        feature_id_to_key_index_.end()) {
      size_t key_idx = feature_id_to_key_index_.size();
      feature_id_to_key_index_[feat.feature_id] = key_idx;

      // Initial landmark estimate: back-project through predicted pose
      gtsam::Point3 landmark_world =
          current_state_.attitude * feat.meas + current_state_.position;
      new_values_.insert(L(key_idx), landmark_world);
    }

    double meas_cov = feat.covariance(0, 0);  // Assuming isotropic covariance
    auto landmark_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
    
    // Add a factor for this measurement
    size_t key_idx = feature_id_to_key_index_.at(feat.feature_id);
    new_factors_.add(RelativeFeatureFactor(X(curr_idx), L(key_idx),
                                           feat.meas, landmark_noise));
  }

  // Update ISAM2
  isam_.update(new_factors_, new_values_);

  new_factors_.resize(0);
  new_values_.clear();

  // Reset preintegrator with updated bias estimate for the next epoch
  gtsam::Values estimate = isam_.calculateEstimate();
  current_bias_ = estimate.at<gtsam::imuBias::ConstantBias>(B(curr_idx));
  preintegration_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(
      preint_params_, current_bias_);

  gtsam::Pose3 pose_est = estimate.at<gtsam::Pose3>(X(curr_idx));
  gtsam::Vector3 vel_est = estimate.at<gtsam::Vector3>(V(curr_idx));

  current_state_.timestamp = stamp;
  current_state_.attitude = pose_est.rotation().matrix();
  current_state_.position = pose_est.translation();
  current_state_.velocity = vel_est;
  current_state_.gyro_bias = current_bias_.gyroscope();
  current_state_.accel_bias = current_bias_.accelerometer();

  last_stamp_ = stamp;
}

NavStateEstimate ISAMSlamEstimator::getLatestState() const {
  return current_state_;
}

Eigen::Matrix<double, 15, 15>
ISAMSlamEstimator::getLatestIMUCovariance() const {
  // TODO: Extract the covariance from ISAM2
  return Eigen::Matrix<double, 15, 15>::Identity() * 1e-6;
}

std::vector<Eigen::Vector3d> ISAMSlamEstimator::getEstimatedMap() const {
  std::vector<Eigen::Vector3d> map;
  if (feature_id_to_key_index_.empty()) return map;

  gtsam::Values estimate = isam_.calculateEstimate();
  for (const auto &pair : feature_id_to_key_index_) {
    gtsam::Point3 p = estimate.at<gtsam::Point3>(L(pair.second));
    map.push_back(p);
  }
  return map;
}

double ISAMSlamEstimator::getEstimateTime() const {
  return current_state_.timestamp;
}
