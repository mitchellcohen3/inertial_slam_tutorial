#include "ekf/EKFState.h"

EKFState::EKFState() {
  // Append the imu to the state and covariance
  int current_id = 0;
  imu_state_ = std::make_shared<slam_states::ImuEKFState>(LieDirection::left);
  imu_state_->set_local_id(current_id);

  current_id += imu_state_->size();
  LOG(INFO) << "Current ID: " << current_id << std::endl;
  _variables.push_back(imu_state_);

  _Cov = Eigen::MatrixXd::Identity(current_id, current_id) * 1e-6;

  LOG(INFO) << "EKF State initialized with size: " << size() << std::endl;
}