#include "estimator/EKFSlamEstimator.h"

#include "ekf/EKFState.h"
#include "ekf/EKFStateHelper.h"

// #include "slam_estimator/RelativeLandmarkJacobianHelper.h"

#include <glog/logging.h>

EKFSlamEstimator::EKFSlamEstimator(const EstimatorConfig &config_)
    : config_(config_) {
  state_ = std::make_shared<EKFState>();
  // imu_propagator_ = std::make_shared<ImuPropagator>();
  LOG(INFO) << "EKF SLAM Estimator initialized." << std::endl;
}

// EKFSlamEstimator::EKFSlamEstimator(const std::shared_ptr<VinsConfig> &config_)
//     : SlamEstimatorBase(config_) {
//   state_ = std::make_shared<EKFState>();
//   LOG(INFO) << "EKF SLAM Estimator initialized." << std::endl;
// }

// void EKFSlamEstimator::inputIMU(ImuMessage &imu_data) {
//   imu_data.timestamp = roundStamp(imu_data.timestamp);
//   imu_propagator->inputIMU(imu_data);

//   if (last_imu_time_ > imu_data.timestamp) {
//     LOG(ERROR) << "IMU measurements are out of order!";
//     std::exit(EXIT_FAILURE);
//   }

//   if (last_imu_time_ < 0) {
//     last_imu_time_ = imu_data.timestamp;
//     return;
//   }
// }

// void EKFSlamEstimator::inputRelativeFeatureMeasurements(
//     std::vector<RelativeFeatureMessage> &relative_feat_meas, double stamp) {
//   // Feed the measurements to the feature manager

//   if (!is_initialized_) {
//     LOG(WARNING)
//         << "Estimator not initialized, cannot process feature measurements";
//     return;
//   }

//   stamp = roundStamp(stamp);
//   for (auto &meas : relative_feat_meas) {
//     meas.timestamp = stamp;
//     feature_manager_->updateFeature(meas.feature_id, stamp, meas);
//   }

//   // Propagate the IMU state to the current measurement stamp
//   propagateIMUStateToStamp(stamp);

//   // Initialize new features to the state
//   for (auto &message : relative_feat_meas) {
//     if (state_->slam_features_.find(message.feature_id) ==
//         state_->slam_features_.end()) {
//       // This feature is not yet in the state, initialize it based on our
//       // current estimate and the measurement
//       Eigen::Vector3d meas = message.meas;
//       IMUType imu_state = getLatestIMUState();
//       Eigen::Matrix3d C = imu_state.attitude();
//       Eigen::Vector3d r = imu_state.position();

//       Eigen::Vector3d r_pw_a = C * meas + r;
//       auto new_feature = std::make_shared<ov_type::Vec>(3);
//       new_feature->set_value(r_pw_a);
//       // new_feature->set_fej(r_pw_a);

//       Eigen::Matrix3d R = message.covariance;
//       Eigen::Matrix<double, 3, 15> Hx = Eigen::Matrix<double, 3, 15>::Zero();
//       Eigen::Matrix3d att_jac;
//       Eigen::Matrix3d pos_jac;
//       Eigen::Matrix3d landmark_jac;

//       if (gt_features_.find(message.feature_id) != gt_features_.end()) {
//         LOG(INFO) << "Evaluating feature Jacobian at groundtruth!";
//         r_pw_a = gt_features_.at(message.feature_id);
//       }

//       RelativeLandmarkJacobianHelper::computeJacobians(
//           C, r, r_pw_a, att_jac, pos_jac, landmark_jac, config->state_rep,
//           config->direction);
//       Hx.block<3, 3>(0, 0) = att_jac;
//       Hx.block<3, 3>(0, 6) = pos_jac;

//       std::vector<std::shared_ptr<ov_type::Type>> state_order = {
//           state_->imu_state_};
//       Eigen::Vector3d res = Eigen::Vector3d::Zero();

//       EKFStateHelper::initialize_invertible(state_, new_feature, state_order,
//                                             Hx, landmark_jac, R, res);
//       state_->slam_features_.insert({message.feature_id, new_feature});

//       if (config->use_fej) {
//         fej_landmarks_.insert({message.feature_id, r_pw_a});
//       }
//     }
//   }

//   // Perform EKF update with all measurements of features at this timestamp
//   performEKFUpdate(relative_feat_meas, stamp);

//   // Marginalize out features we no longer need
//   marginalizeOldFeatures(stamp);

//   // Clean feature manager and IMU propagator
//   feature_manager_->cleanupMeasurements(stamp - 1.0);
//   feature_manager_->cleanup();
//   imu_propagator->cleanOldIMUData(stamp - 0.2);

//   writeCurrentState();

//   last_imu_time_ = stamp;
// }

// void EKFSlamEstimator::inputGpsMeasurement(GpsMessage &gps_data, double stamp) {
//   if (!is_initialized_) {
//     LOG(WARNING)
//         << "Estimator not initialized, cannot process GPS measurements";
//     return;
//   }

//   stamp = roundStamp(stamp);
//   gps_data.timestamp = stamp;

//   propagateIMUStateToStamp(stamp);

//   // Perform EKF update with GPS measurement
//   Eigen::Vector3d meas = gps_data.meas;
//   Eigen::Matrix3d R = gps_data.covariance;
//   Eigen::Vector3d y_hat = getLatestIMUState().position();
//   Eigen::Vector3d res = meas - y_hat;

//   std::cout << "Covariance: \n" << R << std::endl;

//   LOG(INFO) << "Residual: " << res.x() << ", " << res.y() << ", " << res.z();
//   pauseUntilEnter();

//   std::vector<std::shared_ptr<ov_type::Type>> state_order = {
//       state_->imu_state_};

//   Eigen::Matrix<double, 3, 15> Hx = Eigen::Matrix<double, 3, 15>::Zero();
//   Hx.block<3, 3>(0, 0) = -SO3::cross(y_hat);
//   Hx.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
//   EKFStateHelper::EKFUpdate(state_, state_order, Hx, res, R);

//   writeCurrentState();

//   // Clean old IMU data
//   imu_propagator->cleanOldIMUData(stamp - 0.2);
// }

// void EKFSlamEstimator::marginalizeOldFeatures(double stamp) {
//   // Find any features that have not been observed in the current frame
//   std::vector<std::shared_ptr<SlamFeature>> old_features =
//       feature_manager_->featuresNotContainingNewer(stamp, true);

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

// void EKFSlamEstimator::performEKFUpdate(
//     const std::vector<RelativeFeatureMessage> &message_vec, double stamp) {
//   // Large Jacobian, residual and measurement noise of all features for this
//   // update
//   size_t max_meas_size = 3 * message_vec.size();
//   size_t max_hx_size = 15 * message_vec.size();

//   // Eigen::VectorXd res_big = Eigen::VectorXd::Zero(max_meas_size);
//   // Eigen::MatrixXd Hx_big = Eigen::MatrixXd::Zero(max_meas_size, max_hx_size);
//   // Eigen::MatrixXd R_big = Eigen::MatrixXd::Zero(max_meas_size,
//   // max_meas_size);

//   // std::unordered_map<std::shared_ptr<ov_type::Type>, size_t> Hx_mapping;
//   // std::vector<std::shared_ptr<ov_type::Type>> Hx_order_big;

//   // Counter for rows and columns of big Jacobian
//   size_t ct_jacob = 0;
//   size_t ct_meas = 0;
//   for (auto const &message : message_vec) {
//     // Only process features that are in the state
//     if (state_->slam_features_.find(message.feature_id) ==
//         state_->slam_features_.end()) {
//       LOG(WARNING) << "Feature " << message.feature_id
//                    << " not in state, skipping EKF update.";
//       continue;
//     }

//     // Ensure that our measurement is at the correct timestamp
//     IMUType imu_state = getLatestIMUState();

//     Eigen::Vector3d imu_pos_pre = imu_state.position();
//     // LOG(INFO) << "IMU Position before update: "
//     //           << imu_pos_pre.x() << ", " << imu_pos_pre.y() << ", "
//     //           << imu_pos_pre.z();
//     if (std::abs(stamp - message.timestamp) > 1e-4) {
//       LOG(ERROR) << "Measurement timestamp does not match current state time!";
//       std::exit(EXIT_FAILURE);
//     }

//     // Compute the residual and Jacobian for this measurement
//     Eigen::Vector3d meas = message.meas;
//     std::shared_ptr<ov_type::Vec> feature =
//         state_->slam_features_.at(message.feature_id);

//     Eigen::Matrix3d C = imu_state.attitude();
//     Eigen::Vector3d r = imu_state.position();
//     Eigen::Vector3d y_hat = C.transpose() * (feature->value() - r);
//     Eigen::Vector3d res = meas - y_hat;

//     // Compute Jacobians
//     Eigen::Matrix<double, 3, 18> Hx = Eigen::Matrix<double, 3, 18>::Zero();
//     Eigen::Matrix3d att_jac;
//     Eigen::Matrix3d pos_jac;
//     Eigen::Matrix3d landmark_jac;

//     // Possibly evaluate the Jacobian at the groundtruth feature location if
//     // if requested
//     Eigen::Vector3d r_pw_a_jac;
//     if (config->eval_feat_at_gt) {
//       LOG(INFO) << "Evaluating feature Jacobian at groundtruth!";
//       r_pw_a_jac = gt_features_.at(message.feature_id);
//     } else {
//       r_pw_a_jac = feature->value();
//     }

//     if (config->use_fej) {
//       if (fej_landmarks_.find(message.feature_id) == fej_landmarks_.end()) {
//         LOG(ERROR) << "FEJ landmark not found for feature ID: "
//                    << message.feature_id;
//       }
//       r_pw_a_jac = fej_landmarks_.at(message.feature_id);
//     }

//     RelativeLandmarkJacobianHelper::computeJacobians(
//         C, r, r_pw_a_jac, att_jac, pos_jac, landmark_jac, config->state_rep,
//         config->direction);
//     Hx.block<3, 3>(0, 0) = att_jac;
//     Hx.block<3, 3>(0, 6) = pos_jac;
//     Hx.block<3, 3>(0, 15) = landmark_jac;

//     std::vector<std::shared_ptr<ov_type::Type>> state_order;
//     state_order.push_back(state_->imu_state_);
//     state_order.push_back(feature);

//     EKFStateHelper::EKFUpdate(state_, state_order, Hx, res, message.covariance);

//     IMUType imu_state_post = getLatestIMUState();
//     Eigen::Vector3d imu_pos_post = imu_state_post.position();

//     Eigen::Vector3d pos_diff = imu_pos_post - imu_pos_pre;

//     Eigen::Vector3d imu_position_check =
//         state_->imu_state_->value().block<3, 1>(12, 0);
//     Eigen::Vector3d pos_diff_check = imu_position_check - imu_pos_post;
//   }
// }

// void EKFSlamEstimator::propagateIMUStateToStamp(double time1) {
//   std::vector<ImuMessage> imu_data;

//   double time0 = state_->_timestamp;
//   std::vector<ImuMessage> all_imu_data = imu_propagator->getImuDataBuffer();
//   std::vector<ImuMessage> prop_data =
//       imu_propagator->selectIMUData(all_imu_data, time0, time1);

//   // Propagate our current IMU state forward in time
//   auto imu_state = std::make_shared<IMUType>(
//       state_->imu_state_->value(), state_->_timestamp, config->direction);

//   // Sum u all the state transition matricies, so we can do a single large
//   // multiplication at the end
//   Eigen::Matrix<double, 15, 15> Phi_summed =
//       Eigen::Matrix<double, 15, 15>::Identity();
//   Eigen::Matrix<double, 15, 15> Qd_summed =
//       Eigen::Matrix<double, 15, 15>::Zero();
//   double dt_summed = 0.0;
//   if (prop_data.size() > 1) {
//     for (size_t i = 0; i < prop_data.size() - 1; i++) {
//       double dt = prop_data[i + 1].timestamp - prop_data[i].timestamp;

//       Eigen::Matrix<double, 15, 15> F, Qdi;

//       imu_propagator->predictWithJacobians(imu_state, prop_data[i].gyro,
//                                            prop_data[i].accel, dt, F, Qdi);

//       // Sum up the state transition matrix and discrete noise covariance
//       Phi_summed = F * Phi_summed;
//       Qd_summed = F * Qd_summed * F.transpose() + Qdi;
//       Qd_summed = 0.5 * (Qd_summed + Qd_summed.transpose());
//       dt_summed += dt;
//     }
//   }

//   double imu_time = imu_state->stamp();

//   CHECK(std::abs(imu_time - time1) < 1e-7) << "Error hit!";

//   // Ensure that our summed dt is correct
//   CHECK(std::abs((time1 - time0) - dt_summed) < 1e-4);

//   // Update the IMU state
//   state_->imu_state_->set_value(imu_state->toFullVector());
//   state_->_timestamp = time1;

//   // Perform the EKF prediction step
//   std::vector<std::shared_ptr<ov_type::Type>> order = {state_->imu_state_};
//   EKFStateHelper::EKFPropagation(state_, order, order, Phi_summed, Qd_summed);
// }

// void EKFSlamEstimator::initializeIMUState(
//     const IMUType &init_imu_state,
//     const Eigen::Matrix<double, 15, 15> &init_imu_cov) {
//   // Initilize the system state
//   Eigen::Matrix<double, 21, 1> imu_state = Eigen::Matrix<double, 21, 1>::Zero();
//   imu_state.block<9, 1>(0, 0) = SO3::flatten(init_imu_state.attitude());
//   imu_state.block<3, 1>(9, 0) = init_imu_state.velocity();
//   imu_state.block<3, 1>(12, 0) = init_imu_state.position();
//   imu_state.block<3, 1>(15, 0) = init_imu_state.gyroBias();
//   imu_state.block<3, 1>(18, 0) = init_imu_state.accelBias();

//   state_->imu_state_->set_value(imu_state);
//   state_->imu_state_->set_fej(imu_state);

//   // Initialize the covariance
//   std::vector<std::shared_ptr<ov_type::Type>> order = {state_->imu_state_};
//   EKFStateHelper::set_initial_covariance(state_, init_imu_cov, order);

//   // Set the state timestamp
//   state_->_timestamp = roundStamp(init_imu_state.stamp());
//   last_imu_time_ = state_->_timestamp;
//   is_initialized_ = true;
//   LOG(INFO) << "IMU state initialized at time " << state_->_timestamp;
//   LOG(INFO) << "IMU state size: " << state_->size();
// }

// IMUType EKFSlamEstimator::getLatestIMUState() const {
//   return IMUType(state_->imu_state_->value(), state_->_timestamp,
//                  config->direction);
// }

// Eigen::Matrix<double, 15, 15> EKFSlamEstimator::getLatestIMUCovariance() const {
//   return EKFStateHelper::get_full_covariance(state_).block<15, 15>(0, 0);
// }

// std::vector<Eigen::Vector3d> EKFSlamEstimator::getEstimatedMap() const {
//   // Get all estimated feature positions
//   std::vector<Eigen::Vector3d> map_points;
//   for (auto const &feat_pair : state_->slam_features_) {
//     map_points.push_back(feat_pair.second->value());
//   }
//   return map_points;
// }