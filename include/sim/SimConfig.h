#pragma once

#include <Eigen/Dense>
#include <memory>

#include "lieutils/LieDirection.h"
#include "estimator/ImuKinematicsConfig.h"

class SimConfig {

public:
  SimConfig() {
    if (!noise_active) {
      imu_noises.sigma_gyro = 0.0;
      imu_noises.sigma_accel = 0.0;
      imu_noises.sigma_gyro_bias = 0.0;
      imu_noises.sigma_accel_bias = 0.0;

      sigma_feature_meas_3d = 0.0;
    }
  }

  bool load();
  void print() const;

  // 3D SLAM feature config
  double sigma_feature_meas_3d = 1.0; // [m]

  // IMU noises
  ImuNoises imu_noises;

  // Simulator parameters
  bool noise_active = true;

  // Sensor frequencies
  int sim_freq_imu = 400; // Hz
  int sim_freq_meas = 10; // Hz

  bool sim_do_imu_perturbation = false;

  double sim_distance_threshold = 1.1;

  double sim_min_feature_gen_dist = 5.0;  // [m]
  double sim_max_feature_gen_dist = 10.0; // [m]

  Eigen::Vector3d sim_init_gyro_bias;
  Eigen::Vector3d sim_init_accel_bias;

  ///// Seeds for various random number generators
  // Seed for initial states (i.e., random feature 3D positions in the map)
  int sim_seed_state_init = 0;

  // Seed for our IMU state perburbation
  int sim_seed_imu_perturbation = 0;

  // Sim seed measurements. This should be incremented for each Monte-Carlo run
  // to generate the same measurements, but with different noise realizations.
  int sim_seed_measurements = 0;

  //// How much we should perturb the initial state
  double sigma_init_att = 0.1;
  double sigma_init_vel = 0.1;
  double sigma_init_bg = 0.01;
  double sigma_init_ba = 0.001;

  double gravity_mag = 9.81; // [m/s^2]

  double max_sensing_range_slam = 3; // [m]
  int num_slam_features = 20;  // Number of SLAM features to generate

  double t_end = 200.0; // [s] Simulation end time
};
