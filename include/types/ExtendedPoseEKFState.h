#pragma once

#include <glog/logging.h>

#include "lieutils/SE23.h"
#include "lieutils/SO3.h"

#include "Type.h"

class ExtendedPoseEKFState : public ov_type::Type {
public:
  ExtendedPoseEKFState(LieDirection direction = LieDirection::left)
      : ov_type::Type(9), direction(direction) {
    Eigen::Matrix<double, 15, 1> init_pose;
    init_pose.setZero();
    Eigen::Matrix3d C_ab = Eigen::Matrix3d::Identity();
    init_pose.head<9>() = SO3::flatten(C_ab);
    init_pose.block<3, 1>(9, 0) = Eigen::Vector3d::Zero();  // velocity
    init_pose.block<3, 1>(12, 0) = Eigen::Vector3d::Zero(); // position

    set_value(init_pose);
    // set_fej_internal(init_pose);
  }

  // The main update function for extended poses
  void update(const Eigen::VectorXd &delta_xi) override {
    // Check that the size is correct
    if (delta_xi.rows() != _size) {
      LOG(ERROR) << "Error: Incorrect size passed to ExtendedPoseEKFState "
                    "update. Expected "
                 << _size << " but got " << delta_xi.rows() << std::endl;
      return;
    }

    Eigen::Matrix<double, 5, 5> T = toMatrix();
    Eigen::Matrix<double, 5, 5> T_new;

    if (direction == LieDirection::left) {
      T_new = SE23::expMap(delta_xi) * T;
    } else if (direction == LieDirection::right) {
      T_new = T * SE23::expMap(delta_xi);
    }

    // Flatten back to state vector
    Eigen::Matrix3d C_ab = T_new.block<3, 3>(0, 0);
    Eigen::Vector3d velocity = T_new.block<3, 1>(0, 3);
    Eigen::Vector3d position = T_new.block<3, 1>(0, 4);

    Eigen::Matrix<double, 15, 1> newX;
    newX.head<9>() = SO3::flatten(C_ab);
    newX.block<3, 1>(9, 0) = velocity;
    newX.block<3, 1>(12, 0) = position;
    set_value(newX);
  }

  std::shared_ptr<ov_type::Type> clone() override {
    // auto clone = std::shared_ptr<ExtendedPoseEKFState>(new
    // ExtendedPoseEKFState());
    auto clone = std::make_shared<ExtendedPoseEKFState>(direction);
    clone->set_value(value());
    return clone;
  }

  Eigen::Matrix<double, 5, 5> toMatrix() const {
    Eigen::Matrix<double, 15, 1> state = value();
    Eigen::Matrix3d C_ab = SO3::unflatten(state.head<9>());
    Eigen::Vector3d velocity = state.block<3, 1>(9, 0);
    Eigen::Vector3d position = state.block<3, 1>(12, 0);

    Eigen::Matrix<double, 5, 5> T = Eigen::Matrix<double, 5, 5>::Identity();
    T.block<3, 3>(0, 0) = C_ab;
    T.block<3, 1>(0, 3) = velocity;
    T.block<3, 1>(0, 4) = position;
    return T;
  }

  LieDirection getDirection() const { return direction; }

  Eigen::Matrix3d attitude() const {
    Eigen::Matrix<double, 15, 1> state = value();
    return SO3::unflatten(state.head<9>());
  }

  Eigen::Vector3d velocity() const {
    Eigen::Matrix<double, 15, 1> state = value();
    return state.block<3, 1>(9, 0);
  }

  Eigen::Vector3d position() const {
    Eigen::Matrix<double, 15, 1> state = value();
    return state.block<3, 1>(12, 0);
  }
protected:
  LieDirection direction = LieDirection::left;
};