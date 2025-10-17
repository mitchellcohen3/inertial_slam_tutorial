#include "estimator/EKFSlamEstimator.h"

#include "ekf/EKFState.h"
#include "ekf/EKFStateHelper.h"
#include "estimator/ImuPropagator.h"
#include "types/ImuEKFState.h"
#include "utils/Utility.h"

#include <glog/logging.h>

EKFSlamEstimator::EKFSlamEstimator(const EstimatorConfig &config_)
    : config_(config_) {
  state_ = std::make_shared<EKFState>();
  imu_propagator_ = std::make_shared<ImuPropagator>(config_.kinematics_config);
  LOG(INFO) << "EKF SLAM Estimator initialized." << std::endl;
}

void EKFSlamEstimator::inputIMU(ImuMessage &imu_data) {
  imu_data.timestamp = roundStamp(imu_data.timestamp);
  imu_propagator_->inputIMU(imu_data);

  if (last_imu_time_ > imu_data.timestamp) {
    LOG(ERROR) << "IMU measurements are out of order!";
    std::exit(EXIT_FAILURE);
  }

  if (last_imu_time_ < 0) {
    last_imu_time_ = imu_data.timestamp;
    return;
  }
}

void EKFSlamEstimator::inputRelativeFeatureMeasurements(
    std::vector<RelativeFeatureMessage> &relative_feat_meas, double stamp) {
  // Feed the measurements to the feature manager

  if (!is_initialized_) {
    LOG(WARNING)
        << "Estimator not initialized, cannot process feature measurements";
    return;
  }

  propagateIMUStateToStamp(stamp);

  // Initialize new features to the state
  for (auto &message : relative_feat_meas) {
    if (state_->slam_features_.find(message.feature_id) ==
        state_->slam_features_.end()) {
      // This feature is not yet in the state, initialize it based on our
      // current estimate and the measurement
      Eigen::Vector3d meas = message.meas;
      std::shared_ptr<ImuEKFState> imu_state = state_->imu_state_;
      Eigen::Matrix3d C = imu_state->attitude();
      Eigen::Vector3d r = imu_state->position();

      Eigen::Vector3d r_pw_a = C * meas + r;
      auto new_feature = std::make_shared<ov_type::Vec>(3);
      new_feature->set_value(r_pw_a);
      // new_feature->set_fej(r_pw_a);

      Eigen::Matrix3d R = message.covariance;
      Eigen::Matrix<double, 3, 15> Hx = Eigen::Matrix<double, 3, 15>::Zero();
      Eigen::Matrix3d att_jac;
      Eigen::Matrix3d pos_jac;
      Eigen::Matrix3d landmark_jac;

      computeMeasurementModelJacobians(
          C, r, r_pw_a, att_jac, pos_jac, landmark_jac,
          config_.lie_direction);

      Hx.block<3, 3>(0, 0) = att_jac;
      Hx.block<3, 3>(0, 6) = pos_jac;

      std::vector<std::shared_ptr<ov_type::Type>> state_order = {
          state_->imu_state_};
      Eigen::Vector3d res = Eigen::Vector3d::Zero();

      EKFStateHelper::initialize_invertible(state_, new_feature, state_order,
                                            Hx, landmark_jac, R, res);
      state_->slam_features_.insert({message.feature_id, new_feature});

      // if (config->use_fej) {
      //   fej_landmarks_.insert({message.feature_id, r_pw_a});
      // }
    }
  }

  // Perform EKF update with all measurements of features at this timestamp
  // performEKFUpdate(relative_feat_meas, stamp);

  // Marginalize out features we no longer need
  // marginalizeOldFeatures(stamp);

  // Clean feature manager and IMU propagator
  imu_propagator_->cleanOldIMUData(stamp - 0.2);

  last_imu_time_ = stamp;
}

// void EKFSlamEstimator::marginalizeOldFeatures(double stamp) {
//   // Check if there are any features that need to be marginaized
//   // std::vector<size_t> old_feature_ids;

//   // Marginalize each old feature
//   for (auto const &feat : old_features) {
//     if (state_->slam_features_.find(feat->feature_id) ==
//         state_->slam_features_.end()) {
//       LOG(INFO) << "Feature " << feat->feature_id
//                 << " not in state, skipping marginalization.";
//       continue;
//     }

//     std::shared_ptr<ov_type::Vec> feature =
//         state_->slam_features_.at(feat->feature_id);
//     EKFStateHelper::marginalize(state_, feature);

//     // Remove from state
//     state_->slam_features_.erase(feat->feature_id);

//     // If we're using FEJ, also remove from fej landmarks
//     if (config->use_fej) {
//       if (fej_landmarks_.find(feat->feature_id) != fej_landmarks_.end()) {
//         fej_landmarks_.erase(feat->feature_id);
//       }
//       else {
//         LOG(WARNING) << "FEJ landmark for feature ID " << feat->feature_id
//                      << " not found during marginalization.";
//       }
//     }
//   }
// }

void EKFSlamEstimator::performEKFUpdate(
    const std::vector<RelativeFeatureMessage> &message_vec, double stamp) {
  // Apply EKF update for each measurement
  for (auto const &message : message_vec) {
    // Only process features that are in the state
    if (state_->slam_features_.find(message.feature_id) ==
        state_->slam_features_.end()) {
      LOG(WARNING) << "Feature " << message.feature_id
                   << " not in state, skipping EKF update.";
      continue;
    }

    std::shared_ptr<ImuEKFState> imu_state = state_->imu_state_;

    if (std::abs(stamp - message.timestamp) > 1e-4) {
      LOG(ERROR) << "Measurement timestamp does not match current state time !";
      std::exit(EXIT_FAILURE);
    }

    // Compute the residual and Jacobian for this measurement
    Eigen::Vector3d meas = message.meas;
    std::shared_ptr<ov_type::Vec> feature =
        state_->slam_features_.at(message.feature_id);

    Eigen::Matrix3d C = imu_state->attitude();
    Eigen::Vector3d r = imu_state->position();
    Eigen::Vector3d y_hat = C.transpose() * (feature->value() - r);
    Eigen::Vector3d res = meas - y_hat;

    // Compute Jacobians
    Eigen::Matrix<double, 3, 18> Hx = Eigen::Matrix<double, 3, 18>::Zero();
    Eigen::Matrix3d att_jac;
    Eigen::Matrix3d pos_jac;
    Eigen::Matrix3d landmark_jac;

    // Possibly evaluate the Jacobian at the groundtruth feature location if
    // if requested
    Eigen::Vector3d r_pw_a_jac = feature->value();

    if (config_.use_fej) {
      if (fej_landmarks_.find(message.feature_id) == fej_landmarks_.end()) {
        LOG(ERROR) << "FEJ landmark not found for feature ID: "
                   << message.feature_id;
      }
      r_pw_a_jac = fej_landmarks_.at(message.feature_id);
    }

    // RelativeLandmarkJacobianHelper::computeJacobians(
    //     C, r, r_pw_a_jac, att_jac, pos_jac, landmark_jac, config->state_rep,
    //     config->direction);
    Hx.block<3, 3>(0, 0) = att_jac;
    Hx.block<3, 3>(0, 6) = pos_jac;
    Hx.block<3, 3>(0, 15) = landmark_jac;

    std::vector<std::shared_ptr<ov_type::Type>> state_order;
    state_order.push_back(state_->imu_state_);
    state_order.push_back(feature);

    EKFStateHelper::EKFUpdate(state_, state_order, Hx, res, message.covariance);
  }
}

void EKFSlamEstimator::propagateIMUStateToStamp(double time1) {
  std::vector<ImuMessage> imu_data;

  double time0 = state_->timestamp_;
  std::vector<ImuMessage> all_imu_data = imu_propagator_->getImuDataBuffer();
  std::vector<ImuMessage> prop_data =
      imu_propagator_->selectIMUData(all_imu_data, time0, time1);

  // Sum up all the state transition matrices, so we can do a single large
  // multiplication at the end
  Eigen::Matrix<double, 15, 15> Phi_summed =
      Eigen::Matrix<double, 15, 15>::Identity();
  Eigen::Matrix<double, 15, 15> Qd_summed =
      Eigen::Matrix<double, 15, 15>::Zero();
  double dt_summed = 0.0;
  if (prop_data.size() > 1) {
    for (size_t i = 0; i < prop_data.size() - 1; i++) {
      double dt = prop_data[i + 1].timestamp - prop_data[i].timestamp;

      Eigen::Matrix<double, 15, 15> F, Qdi;

      imu_propagator_->predictWithJacobians(state_->imu_state_,
                                            prop_data[i].gyro,
                                            prop_data[i].accel, dt, F, Qdi);

      // Sum up the state transition matrix and discrete noise covariance
      Phi_summed = F * Phi_summed;
      Qd_summed = F * Qd_summed * F.transpose() + Qdi;
      Qd_summed = 0.5 * (Qd_summed + Qd_summed.transpose());
      dt_summed += dt;
    }
  }
  // Ensure that our summed dt is correct
  CHECK(std::abs((time1 - time0) - dt_summed) < 1e-4);
  state_->timestamp_ = time1;

  // Perform the EKF prediction step
  std::vector<std::shared_ptr<ov_type::Type>> order = {state_->imu_state_};
  EKFStateHelper::EKFPropagation(state_, order, order, Phi_summed, Qd_summed);
}

void EKFSlamEstimator::initializeIMUState(
    double stamp, const Eigen::Matrix<double, 5, 5> &nav_state,
    const Eigen::Vector3d &gyro_bias, const Eigen::Vector3d &accel_bias,
    const Eigen::Matrix<double, 15, 15> &init_imu_cov) {
  // Initilize the system state
  Eigen::Matrix<double, 21, 1> imu_state = Eigen::Matrix<double, 21, 1>::Zero();
  imu_state.block<9, 1>(0, 0) = SO3::flatten(nav_state.block<3, 3>(0, 0));
  imu_state.block<3, 1>(9, 0) = nav_state.block<3, 1>(0, 3);
  imu_state.block<3, 1>(12, 0) = nav_state.block<3, 1>(0, 4);
  imu_state.block<3, 1>(15, 0) = gyro_bias;
  imu_state.block<3, 1>(18, 0) = accel_bias;
  state_->imu_state_->set_value(imu_state);
  state_->imu_state_->set_fej(imu_state);

  // Initialize the covariance
  std::vector<std::shared_ptr<ov_type::Type>> order = {state_->imu_state_};
  EKFStateHelper::set_initial_covariance(state_, init_imu_cov, order);

  // Set the state timestamp
  state_->timestamp_ = roundStamp(stamp);
  last_imu_time_ = state_->timestamp_;
  is_initialized_ = true;
  LOG(INFO) << "IMU state initialized at time " << state_->timestamp_;
  LOG(INFO) << "IMU state size: " << state_->size();
}

double EKFSlamEstimator::getEstimateTime() const {
  return state_->timestamp_;
}

std::shared_ptr<ImuEKFState> EKFSlamEstimator::getLatestIMUState() const {
  return state_->imu_state_;
}

Eigen::Matrix<double, 15, 15> EKFSlamEstimator::getLatestIMUCovariance() const {
  return EKFStateHelper::get_full_covariance(state_).block<15, 15>(0, 0);
}

std::vector<Eigen::Vector3d> EKFSlamEstimator::getEstimatedMap() const {
  // Get all estimated feature positions
  std::vector<Eigen::Vector3d> map_points;
  for (auto const &feat_pair : state_->slam_features_) {
    map_points.push_back(feat_pair.second->value());
  }
  return map_points;
}

void computeMeasurementModelJacobians(const Eigen::Matrix3d &C_ab,
                      const Eigen::Vector3d &r_zw_a,
                      const Eigen::Vector3d &r_pw_a, Eigen::Matrix3d &att_jac,
                      Eigen::Matrix3d &pos_jac, Eigen::Matrix3d &feat_jac,
                      LieDirection direction) {
  feat_jac = C_ab.transpose();

  Eigen::Vector3d y_check = C_ab.transpose() * (r_pw_a - r_zw_a);
  if (direction == LieDirection::right) {
    att_jac = SO3::cross(y_check);
    pos_jac = -Eigen::Matrix3d::Identity();
  } else if (direction == LieDirection::left) {
    att_jac = C_ab.transpose() * SO3::cross(r_pw_a);
    pos_jac = -C_ab.transpose();
  } else {
    LOG(ERROR) << "Unknown Lie direction!";
  }
}