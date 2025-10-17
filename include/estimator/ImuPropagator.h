#pragma once

#include <Eigen/Dense>
#include <memory>
#include <mutex>

#include "estimator/ImuKinematicsConfig.h"
#include "types/ImuEKFState.h"
#include "utils/SensorData.h"

/**
 * @brief Class to propagate the IMU state based on incoming IMU measurements.
 */
class ImuPropagator {
public:
  ImuPropagator(const KinematicsConfig &config_) : config(config_) {}

  /**
   * @brief Inputs an IMU measurement into the propagator.
   */
  void inputIMU(const ImuMessage &imu_message) {
    imu_data.push_back(imu_message);
  }

  /**
   * @brief Predicts the IMU state forward one timestep.
   */
  void predict(std::shared_ptr<ImuEKFState> state, const Eigen::Vector3d &gyro,
               const Eigen::Vector3d &accel, double dt);

  /**
   * @brief Predicts the IMU state forward one timestep.
   *
   * Also computes the discrete-time Jacobians of the process model with respect
   * to the state and the noise.
   */
  void predictWithJacobians(std::shared_ptr<ImuEKFState> state,
                            const Eigen::Vector3d &gyro,
                            const Eigen::Vector3d &accel, double dt,
                            Eigen::Matrix<double, 15, 15> &Ad,
                            Eigen::Matrix<double, 15, 15> &Qd);

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

Eigen::Matrix<double, 5, 5> createGMatrix(const Eigen::Vector3d &gravity,
                                          double dt);
Eigen::Matrix<double, 5, 5> createUMatrix(const Eigen::Vector3d &omega,
                                          const Eigen::Vector3d &accel,
                                          double dt);
Eigen::Matrix3d createNMatrix(const Eigen::Vector3d &phi_vec);
  