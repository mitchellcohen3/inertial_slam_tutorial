#pragma once

#include "ImuEKFState.h"
#include "ExtendedPoseEKFState.h"
#include "Type.h"

/**
 * @brief the main state of our filter.
 */
class EKFState {
public:
  EKFState();

  int size() const { return _Cov.rows(); }

  double _timestamp = -1;

  std::shared_ptr<slam_states::ImuEKFState> imu_state_;

  std::unordered_map<size_t, std::shared_ptr<ov_type::Vec>> slam_features_;

protected:

  friend class EKFStateHelper;

  Eigen::MatrixXd _Cov;
  std::vector<std::shared_ptr<ov_type::Type>> _variables;

};