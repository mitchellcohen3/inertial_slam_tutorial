#pragma once

#include "config/VinsConfig.h"
#include "imu/IMUIncrement.h"
#include "types/ImuType.h"
#include "utils/SensorData.h"

#include <Eigen/Dense>
#include <memory>
#include <mutex>

void getUnbiasedImu(const ImuMessage &input,
                    const std::shared_ptr<IMUType> &state,
                    Eigen::Vector3d &unbiased_gyro,
                    Eigen::Vector3d &unbiased_accel);

/**
 * @brief Performs propagation of an IMU state and an RMI, along with their
 * corresponding covariances. Also has tools for selecting which measurements we
 * should propagate with.
 */
class ImuPropagator {
public:
  ImuPropagator(
      const KinematicsConfig &config_,
      LieDirection direction = LieDirection::left,
      ExtendedPoseRepresentation pose_rep = ExtendedPoseRepresentation::SE23)
      : direction_(direction) {
    config = config_;

    // Set initial IMU increment
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();
    imu_increment = std::make_shared<ceres_nav::IMUIncrement>(
        config.imu_noises.Q_ct, bg, ba, 0.0, config.gravity, direction,
        pose_rep);
  }

  /**
   * @brief Predicts the IMU state forward one timestep.
   */
  void predict(std::shared_ptr<IMUType> state, const Eigen::Vector3d &gyro,
               const Eigen::Vector3d &accel, double dt);

  /**
   * @brief Predicts the IMU state forward one timestep.
   *
   * Also computes the discrete-time Jacobians of the process model with respect
   * to the state and the noise.
   */
  void predictWithJacobians(std::shared_ptr<IMUType> state,
                            const Eigen::Vector3d &gyro,
                            const Eigen::Vector3d &accel, double dt,
                            Eigen::Matrix<double, 15, 15> &Ad,
                            Eigen::Matrix<double, 15, 15> &Qd);

  void inputIMU(const ImuMessage &imu_message) {
    imu_data.push_back(imu_message);
  }

  /**
   * @brief Get the IMU message between two times.
   */
  static std::vector<ImuMessage>
  getIMUBetweenTimes(const std::vector<ImuMessage> &imu_data,
                     const double &start_time, const double &end_time);

  /**
   * @brief Select IMU data between two times.
   */
  static std::vector<ImuMessage>
  selectIMUData(const std::vector<ImuMessage> &imu_data,
                const double &start_time, const double &end_time,
                bool warn = false);

  /**
   * Interpolates between two IMU messages.
   */
  static ImuMessage interpolateIMUData(const ImuMessage &imu_data1,
                                       const ImuMessage &imu_data2,
                                       const double &time);

  /**
   * @brief Propagate IMU state and RMI to current timestamp using IMU
   * measurements. Here, we first should call inputIMU() to add the IMU
   * measurements to the buffer, and then call this function to propagate until
   * the desired time.
   */
  void propagate(std::shared_ptr<IMUType> state, double timestamp);

  void setImuIncrementDirection(const LieDirection &direction) {
    imu_increment->direction = direction;
  }

  void resetImuIncrement(const Eigen::Vector3d &bg, const Eigen::Vector3d &ba,
                         const double &stamp) {
    imu_increment->reset(stamp, bg, ba);
  }

  void cleanOldIMUData(double oldest_time) {
    if (oldest_time < 0) {
      return;
    }
    auto it0 = imu_data.begin();
    while (it0 != imu_data.end()) {
      if (it0->timestamp < oldest_time) {
        it0 = imu_data.erase(it0);
      } else {
        it0++;
      }
    }
  }

  std::vector<ImuMessage> getImuDataBuffer() {
    // std::lock_guard<std::mutex> lock(imu_data_mtx);
    return imu_data;
  }

  size_t getImuDataBufferSize() {
    // std::lock_guard<std::mutex> lock(imu_data_mtx);
    return imu_data.size();
  }

  // Our latest RMI
  std::shared_ptr<ceres_nav::IMUIncrement> imu_increment;

protected:
  // Our configuration which contains the noise properties,
  // gravity, etc.
  KinematicsConfig config;

  // History of IMU messages
  std::vector<ImuMessage> imu_data;
  std::mutex imu_data_mtx;
  LieDirection direction_;
};

// IMU data helper functions
void discretizeSystem(const Eigen::MatrixXd &A_ct, const Eigen::MatrixXd &L_ct,
                      const Eigen::MatrixXd &Q_ct, double dt,
                      Eigen::MatrixXd &A_d, Eigen::MatrixXd &Q_d);