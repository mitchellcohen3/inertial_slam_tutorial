#pragma once
#include <glog/logging.h>

#include <Eigen/Dense>

class ImuNoises {
public:
  ImuNoises() {}

  // Gyroscope whiten noise
  double sigma_gyro = 0.01;
  // Gyroscope random walk
  double sigma_gyro_bias = 0.01;
  // Accelerometer white noise
  double sigma_accel = 0.0001;
  // Accelerometer random walk
  double sigma_accel_bias = 0.00001;

  Eigen::Matrix<double, 12, 12> Q_ct;

  bool
  load(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) override {
    if (parser != nullptr) {
      parser->parse_external("relative_config_imu", "imu0",
                             "gyroscope_noise_density", sigma_gyro);
      parser->parse_external("relative_config_imu", "imu0",
                             "gyroscope_random_walk", sigma_gyro_bias);
      parser->parse_external("relative_config_imu", "imu0",
                             "accelerometer_noise_density", sigma_accel);
      parser->parse_external("relative_config_imu", "imu0",
                             "accelerometer_random_walk", sigma_accel_bias);
    }

    // Set the continuous time Q Matrix
    Q_ct = Eigen::Matrix<double, 12, 12>::Identity() * 1e-7;
    Q_ct.block<3, 3>(0, 0) =
        Eigen::Matrix3d::Identity() * sigma_gyro * sigma_gyro;
    Q_ct.block<3, 3>(3, 3) =
        Eigen::Matrix3d::Identity() * sigma_accel * sigma_accel;
    Q_ct.block<3, 3>(6, 6) =
        Eigen::Matrix3d::Identity() * sigma_gyro_bias * sigma_gyro_bias;
    Q_ct.block<3, 3>(9, 9) =
        Eigen::Matrix3d::Identity() * sigma_accel_bias * sigma_accel_bias;
    return true;
  }

  void print() const override {
    LOG(INFO) << "IMU Noise Parameters:";
    LOG(INFO) << "  - gyroscope_noise_density: " << sigma_gyro;
    LOG(INFO) << "  - accelerometer_noise_density: " << sigma_accel;
    LOG(INFO) << "  - gyroscope_random_walk: " << sigma_gyro_bias;
    LOG(INFO) << "  - accelerometer_random_walk: " << sigma_accel_bias;
  }
};

class KinematicsConfig {
public:
  KinematicsConfig() = default;

  // Gravity vector
  Eigen::Vector3d gravity;

  // Gravity magnitude
  double gravity_mag = 9.81;

  // Jacobian method
  std::string jacobian_method = "continuous";
  // Discretization method
  std::string discritization_method = "euler";

  // Whether or not to propagate with the average measurements
  bool average_meas = true;

  // IMU noise parameters
  ImuNoises imu_noises;

  bool
  load(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) override {
    if (parser != nullptr) {
      parser->parse_config("gravity_mag", gravity_mag);
      gravity = Eigen::Vector3d(0.0, 0.0, -gravity_mag);

      parser->parse_config("jacobian_method", jacobian_method);
      parser->parse_config("discritization_method", discritization_method);
      parser->parse_config("average_meas", average_meas);
      // parser->parse_config("rmi_cov_prop_method", rmi_cov_prop_method);
    }

    imu_noises.load(parser);
    return true;
  }

  void print() const override {
    LOG(INFO) << "Kinematics Parameters: ";
    LOG(INFO) << "  - gravity: " << gravity.transpose();
    LOG(INFO) << "  - jacobian_method: " << jacobian_method;
    LOG(INFO) << "  - discritization_method: " << discritization_method;
    LOG(INFO) << "  - average_meas: " << average_meas;
    imu_noises.print();
  }
};