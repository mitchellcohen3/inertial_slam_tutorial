#pragma once

#include "ExtendedPoseEKFState.h"

#include "Type.h"
#include "Vec.h"
#include "lie/LieDirection.h"

namespace slam_states {

/**
 * @brief Derived class that implements an IMU state to be used with the EKF
 * estimator.
 *
 * Contains an ExtendedPoseEKFState for the SE_2(3) extended pose, and vector
 * states for gyro bias and accel bias.
 */
class ImuEKFState : public ov_type::Type {
public:
  ImuEKFState(LieDirection direction = LieDirection::left) : ov_type::Type(15) {
    // Create all subvariables
    pose_ = std::make_shared<ExtendedPoseEKFState>(direction);
    gyro_bias = std::make_shared<ov_type::Vec>(3);
    accel_bias = std::make_shared<ov_type::Vec>(3);

    // Create initial value
    Eigen::Matrix<double, 21, 1> imu0 = Eigen::Matrix<double, 21, 1>::Zero();
    imu0.head<15>() = pose_->value();
    set_value(imu0);
  }

  // Update function
  void update(const Eigen::VectorXd &delta_xi) override {
    if (delta_xi.rows() != _size) {
      LOG(ERROR) << "Error: Incorrect size passed to ImuEKFState update. Expected "
                 << _size << " but got " << delta_xi.rows() << std::endl;
      return;
    }

    // Update each subvariable
    pose_->update(delta_xi.segment(0, 9));
    gyro_bias->update(delta_xi.segment(9, 3));
    accel_bias->update(delta_xi.segment(12, 3));    

    // Now, set the full state value
    Eigen::Matrix<double, 21, 1> newX;
    newX.head<15>() = pose_->value();
    newX.segment<3>(15) = gyro_bias->value();
    newX.segment<3>(18) = accel_bias->value();
    set_value(newX);
  }

  LieDirection getDirection() const { return pose_->getDirection(); }

  void set_value(const Eigen::MatrixXd &new_value) override  {
    if (new_value.rows() != 21) {
        LOG(ERROR) << "Error: Incorrect size passed to ImuEKFState set_value. Expected 21 but got "
                     << new_value.rows() << std::endl;
        return;
    }

    pose_->set_value(new_value.block<15, 1>(0, 0));
    gyro_bias->set_value(new_value.block<3, 1>(15, 0));
    accel_bias->set_value(new_value.block<3, 1>(18, 0));
    _value = new_value;
  }

  std::shared_ptr<ov_type::Type> clone() override {
    auto clone = std::make_shared<ImuEKFState>(pose_->getDirection());
    clone->set_value(value());
    return clone;
  }

protected:
  // Pose subvariable
  std::shared_ptr<ExtendedPoseEKFState> pose_;
  std::shared_ptr<ov_type::Vec> gyro_bias;
  std::shared_ptr<ov_type::Vec> accel_bias;
};
} // namespace slam_states