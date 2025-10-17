#pragma once

#include <glog/logging.h>
#include <Eigen/Dense>

#include <yaml-cpp/yaml.h>

class ImuNoises {
public:
  ImuNoises() {
    Q_ct = Eigen::Matrix<double, 12, 12>::Identity() * 1e-7;
    Q_ct.block<3, 3>(0, 0) =
        Eigen::Matrix3d::Identity() * sigma_gyro * sigma_gyro;
    Q_ct.block<3, 3>(3, 3) =
        Eigen::Matrix3d::Identity() * sigma_accel * sigma_accel;
    Q_ct.block<3, 3>(6, 6) =
        Eigen::Matrix3d::Identity() * sigma_gyro_bias * sigma_gyro_bias;
    Q_ct.block<3, 3>(9, 9) =
        Eigen::Matrix3d::Identity() * sigma_accel_bias * sigma_accel_bias;
  }

  // Gyroscope whiten noise
  double sigma_gyro = 1.69e-4;
  // Gyroscope random walk
  double sigma_gyro_bias = 1.93e-5;
  // Accelerometer white noise
  double sigma_accel = 2.00e-3;
  // Accelerometer random walk
  double sigma_accel_bias = 3.00e-3;

  // The continuous-time IMU noise covariance
  Eigen::Matrix<double, 12, 12> Q_ct;

  bool load (const std::string &config_file) {
    YAML::Node config = YAML::LoadFile(config_file);
    sigma_gyro = config["sigma_gyro"].as<double>();
    sigma_accel = config["sigma_accel"].as<double>();
    sigma_gyro_bias = config["sigma_gyro_bias"].as<double>();
    sigma_accel_bias = config["sigma_accel_bias"].as<double>();

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

  void print() const {
    LOG(INFO) << "IMU Noise Parameters:";
    LOG(INFO) << "  - gyroscope_noise_density: " << sigma_gyro;
    LOG(INFO) << "  - accelerometer_noise_density: " << sigma_accel;
    LOG(INFO) << "  - gyroscope_random_walk: " << sigma_gyro_bias;
    LOG(INFO) << "  - accelerometer_random_walk: " << sigma_accel_bias;
  }
};

class KinematicsConfig {
public:
  KinematicsConfig() {
    gravity = Eigen::Vector3d(0, 0, -gravity_mag);
  }

  // Gravity vector
  Eigen::Vector3d gravity;

  // Gravity magnitude
  double gravity_mag = 9.81;

  // IMU noise parameters
  ImuNoises imu_noises;

  bool load(const std::string &config_file) {
    YAML::Node config = YAML::LoadFile(config_file);
    gravity_mag = config["gravity_mag"].as<double>();
    imu_noises.load(config_file);

    gravity = Eigen::Vector3d(0, 0, -gravity_mag);
    return true;
  }

  void print() const {
    LOG(INFO) << "Kinematics Parameters: ";
    LOG(INFO) << "  - gravity: " << gravity.transpose();
    imu_noises.print();
  }
};