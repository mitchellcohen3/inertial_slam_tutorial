#include "SimConfig.h"

#include "cam/CamEqui.h"
#include "cam/CamRadtan.h"

#include "utils/opencv_yaml_parse.h"

#include <glog/logging.h>

bool SimConfig::load(const std::shared_ptr<ov_core::YamlParser> &parser) {
  if (parser != nullptr) {
    parser->parse_config("max_cameras", num_cameras);
    parser->parse_config("noise_active", noise_active);
    parser->parse_config("sim_freq_imu", sim_freq_imu);
    parser->parse_config("sim_freq_meas", sim_freq_meas);
    parser->parse_config("sim_distance_threshold", sim_distance_threshold);
    parser->parse_config("sim_min_feature_gen_dist", sim_min_feature_gen_dist);
    parser->parse_config("sim_max_feature_gen_dist", sim_max_feature_gen_dist);
    parser->parse_config("sim_do_imu_perturbation", sim_do_imu_perturbation);
    parser->parse_config("sim_do_calib_perturbation",
                         sim_do_calib_perturbation);
    parser->parse_config("num_pts", num_pts);
    CameraCalibration calib = loadCameraParameters(parser, num_cameras);
    cam_intrinsics = calib.camera_intrinsics;
    camera_extrinsics = calib.camera_extrinsics;
    calib_camimu_dt = calib.calib_camimu_dt;

    // Load IMU noises
    imu_noises.load(parser);

    // Load measurement noises
    parser->parse_config("sigma_gps", sigma_gps);
    parser->parse_config("sigma_feat", sigma_pix);
    parser->parse_config("sigma_feature_meas_3d", sigma_feature_meas_3d);

    if (!noise_active) {
      imu_noises.sigma_gyro = 0.0;
      imu_noises.sigma_gyro_bias = 0.0;
      imu_noises.sigma_accel = 0.0;
      imu_noises.sigma_accel_bias = 0.0;
      sigma_gps = 0.0;
      sigma_pix = 0.0;
      sigma_feature_meas_3d = 0.0;
    }
    parser->parse_config("num_slam_features", num_slam_features);

    // Load initial biases
    std::vector<double> bias_g = {0, 0, 0};
    std::vector<double> bias_a = {0, 0, 0};
    parser->parse_config("sim_init_gyro_bias", bias_g);
    parser->parse_config("sim_init_accel_bias", bias_a);
    sim_init_gyro_bias << bias_g[0], bias_g[1], bias_g[2];
    sim_init_accel_bias << bias_a[0], bias_a[1], bias_a[2];

    parser->parse_config("sim_seed_state_init", sim_seed_state_init);
    parser->parse_config("sim_seed_perturb", sim_seed_perturb);
    parser->parse_config("sim_seed_measurements", sim_seed_measurements);

    std::string direction_str;
    parser->parse_config("lie_direction", direction_str);
    if (direction_str == "left") {
      direction = LieDirection::left;
    } else if (direction_str == "right") {
      direction = LieDirection::right;
    } else {
      LOG(ERROR) << "Invalid LieDirection specified: " << direction_str;
      LOG(ERROR) << "Defaulting to left direction.";
      direction = LieDirection::left;
    }

    parser->parse_config("sigma_init_att", sigma_init_att);
    parser->parse_config("sigma_init_vel", sigma_init_vel);
    parser->parse_config("sigma_init_bg", sigma_init_bg);
    parser->parse_config("sigma_init_ba", sigma_init_ba);

    parser->parse_config("gravity_mag", gravity_mag);
    parser->parse_config("t_end", t_end);

    //
    parser->parse_config("sigma_init_cam_ext_att", sigma_init_cam_ext_att);
    parser->parse_config("sigma_init_cam_ext_pos", sigma_init_cam_ext_pos);
    parser->parse_config("max_sensing_range_slam", max_sensing_range_slam);
  }

  return true;
}

void SimConfig::print() const {
  LOG(INFO) << "SimConfig:";
  LOG(INFO) << "  - num_cameras: " << num_cameras;
  LOG(INFO) << "  - noise_active: " << noise_active;
  LOG(INFO) << "  - sim_freq_imu: " << sim_freq_imu;
  LOG(INFO) << "  - sim_freq_meas: " << sim_freq_meas;
  LOG(INFO) << "  - sim_distance_threshold: " << sim_distance_threshold;
  LOG(INFO) << "  - sim_min_feature_gen_dist: " << sim_min_feature_gen_dist;
  LOG(INFO) << "  - sim_max_feature_gen_dist: " << sim_max_feature_gen_dist;
  LOG(INFO) << "  - sim_do_imu_perturbation: " << sim_do_imu_perturbation;
  LOG(INFO) << "  - sim_do_calib_perturbation: " << sim_do_calib_perturbation;
  LOG(INFO) << "  - calib_camimu_dt: " << calib_camimu_dt;
  LOG(INFO) << "  - sim_init_gyro_bias: " << sim_init_gyro_bias.transpose();
  LOG(INFO) << "  - sim_init_accel_bias: " << sim_init_accel_bias.transpose();
  LOG(INFO) << "  - cam_intrinsics: ";
  for (const auto &cam : cam_intrinsics) {
    LOG(INFO) << "    - Camera ID: " << cam.first;
  }

  LOG(INFO) << "Seeds: ";
  LOG(INFO) << "  - sim_seed_state_init: " << sim_seed_state_init;
  LOG(INFO) << "  - sim_seed_perturb: " << sim_seed_perturb;
  LOG(INFO) << "  - sim_seed_measurements: " << sim_seed_measurements;

  if (sim_do_imu_perturbation) {
    LOG(INFO) << "Perturbing the initial IMU state.";
    LOG(INFO) << "  - sigma_init_att: " << sigma_init_att;
    LOG(INFO) << "  - sigma_init_vel: " << sigma_init_vel;
    LOG(INFO) << "  - sigma_init_bg: " << sigma_init_bg;
    LOG(INFO) << "  - sigma_init_ba: " << sigma_init_ba;
  }

  LOG(INFO) << "Gravity magnitude: " << gravity_mag;

  LOG(INFO) << "Lie direction: "
            << (direction == LieDirection::left ? "left" : "right");

  LOG(INFO) << "Sim IMU noises: ";
  LOG(INFO) << "  - sigma_gyro: " << imu_noises.sigma_gyro;
  LOG(INFO) << "  - sigma_gyro_bias: " << imu_noises.sigma_gyro_bias;
  LOG(INFO) << "  - sigma_accel: " << imu_noises.sigma_accel;
  LOG(INFO) << "  - sigma_accel_bias: " << imu_noises.sigma_accel_bias;

  LOG(INFO) << "  - sigma_feature_meas_3d: " << sigma_feature_meas_3d;
  LOG(INFO) << "  - max_sensing_range_slam: " << max_sensing_range_slam;
  LOG(INFO) << "End time: " << t_end << " seconds";
}

void SimConfig::writeToYaml(const std::string &output_path) const {
  cv::FileStorage file(output_path, cv::FileStorage::WRITE);
  if (!file.isOpened()) {
    LOG(ERROR) << "Failed to open file for writing: " << output_path;
    return;
  }

  // Write some data to the YAML file
  if (direction == LieDirection::left) {
    file << "lie_direction"
         << "left";
  } else {
    file << "lie_direction"
         << "right";
  }

  file << "num_cameras" << num_cameras;
  file << "noise_active" << noise_active;
  file << "sim_freq_imu" << sim_freq_imu;
  file << "sim_freq_meas" << sim_freq_meas;
  file << "sim_distance_threshold" << sim_distance_threshold;
  file << "sim_min_feature_gen_dist" << sim_min_feature_gen_dist;
  file << "sim_max_feature_gen_dist" << sim_max_feature_gen_dist;
  file << "sim_do_imu_perturbation" << sim_do_imu_perturbation;
  file << "sim_do_calib_perturbation" << sim_do_calib_perturbation;
  file << "calib_camimu_dt" << calib_camimu_dt;
  file << "sim_seed_state_init" << sim_seed_state_init;
  file << "sim_seed_perturb" << sim_seed_perturb;
  file << "sim_seed_measurements" << sim_seed_measurements;
  file << "sigma_init_att" << sigma_init_att;
  file << "sigma_init_vel" << sigma_init_vel;
  file << "sigma_init_bg" << sigma_init_bg;
  file << "sigma_init_ba" << sigma_init_ba;
  file << "gravity_mag" << gravity_mag;
  file << "t_end" << t_end;

  // Write the IMU noises
  file << "gyro_white_noise" << imu_noises.sigma_gyro;
  file << "gyro_bias_random_walk" << imu_noises.sigma_gyro_bias;
  file << "accel_white_noise" << imu_noises.sigma_accel;
  file << "accel_bias_random_walk" << imu_noises.sigma_accel_bias;

  file << "sigma_pix" << sigma_pix;
}