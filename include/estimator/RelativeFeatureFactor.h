#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

/**
 * @brief Factor for a landmark resolved in the body frame.
 * This factor relates a robot pose (pose3) and a landmark position (point3) 
 * through the following measurement model:
 * 
 * y = C_ab.T * (p_a - r_a) + noise,
 * where C_ab is the attitude of the robot, p_a is the landmark position
 * and r_a is the position of the robot, in the world frame.
 */
class RelativeFeatureFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> {
 public:
  using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3>;

  RelativeFeatureFactor(gtsam::Key pose_key, gtsam::Key landmark_key,
                        const gtsam::Point3& measurement,
                        const gtsam::SharedNoiseModel& noise_model)
      : Base(noise_model, pose_key, landmark_key), measurement_(measurement) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& T_ab, const gtsam::Point3& p_a,
      boost::optional<gtsam::Matrix&> jac_pose = boost::none,
      boost::optional<gtsam::Matrix&> jac_landmark = boost::none) const override {
    return T_ab.transformTo(p_a, jac_pose, jac_landmark) - measurement_;
  }

 private:
  gtsam::Point3 measurement_;
};
