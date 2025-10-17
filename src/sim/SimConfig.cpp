#include "sim/SimConfig.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

bool SimConfig::load(const std::string &config_file) {
  YAML::Node config = YAML::LoadFile(config_file);

  noise_active = config["noise_active"].as<bool>();
  sim_freq_imu = config["sim_freq_imu"].as<int>();
  sim_freq_meas = config["sim_freq_meas"].as<int>();
  imu_noises.load(config_file);

  LOG(INFO) << "Loaded IMU noise parameters from config file.";
  sim_do_imu_perturbation = config["sim_do_imu_perturbation"].as<bool>();
  LOG(INFO) << "Loaded sim_do_imu_perturbation from config file.";

  sigma_feature_meas_3d = config["sigma_feature_meas_3d"].as<double>();

  // Set all noises to zero if noise is inactive
  if (!noise_active) {
    imu_noises.sigma_gyro = 0.0;
    imu_noises.sigma_accel = 0.0;
    imu_noises.sigma_gyro_bias = 0.0;
    imu_noises.sigma_accel_bias = 0.0;
    imu_noises.Q_ct = Eigen::Matrix<double, 12, 12>::Identity() * 1e-7;
    sigma_feature_meas_3d = 0.0;
  }

  LOG(INFO) << "Loading seeds and initial perturbation parameters.";
  // Seeds 
  sim_seed_state_init = config["sim_seed_state_init"].as<int>();
  sim_seed_imu_perturbation = config["sim_seed_imu_perturbation"].as<int>();
  sim_seed_measurements = config["sim_seed_measurements"].as<int>();

  LOG(INFO) << "Loading initial state perturbation parameters.";
  // How much we should perturb the initial state
  sigma_init_att = config["sigma_init_att"].as<double>();
  sigma_init_vel = config["sigma_init_vel"].as<double>();
  sigma_init_bg = config["sigma_init_bg"].as<double>();
  sigma_init_ba = config["sigma_init_ba"].as<double>();

  t_end = config["t_end"].as<double>();

  return true;
}

void SimConfig::print() const {
  LOG(INFO) << "Simulator Configuration:";
  LOG(INFO) << "  - noise_active: " << noise_active;
  LOG(INFO) << "  - sim_freq_imu: " << sim_freq_imu;
  LOG(INFO) << "  - sim_freq_meas: " << sim_freq_meas;
  LOG(INFO) << "  - sim_do_imu_perturbation: " << sim_do_imu_perturbation;
  imu_noises.print();

  LOG(INFO) << "  - sigma_feature_meas_3d: " << sigma_feature_meas_3d;
  LOG(INFO) << "  - sim_seed_state_init: " << sim_seed_state_init;
  LOG(INFO) << "  - sim_seed_imu_perturbation: " << sim_seed_imu_perturbation;
  LOG(INFO) << "  - sim_seed_measurements: " << sim_seed_measurements;
  LOG(INFO) << "  - sigma_init_att: " << sigma_init_att;
  LOG(INFO) << "  - sigma_init_vel: " << sigma_init_vel;
  LOG(INFO) << "  - sigma_init_bg: " << sigma_init_bg;
  LOG(INFO) << "  - sigma_init_ba: " << sigma_init_ba;
  LOG(INFO) << "  - t_end: " << t_end << "  [s]";
}