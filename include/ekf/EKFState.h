#pragma once

#include "types/ExtendedPoseEKFState.h"
#include "types/ImuEKFState.h"
#include "types/Type.h"

/**
 * @brief The main state of the filter.
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