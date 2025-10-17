#include "sim/Simulator.h"

#include "lieutils/LieDirection.h"
#include "lieutils/SE3.h"
#include "lieutils/SO3.h"

#include "utils/FileAccess.h"

void load_simulated_trajectory(std::string path,
                               std::vector<Eigen::VectorXd> &traj_data) {
  // Try to open our groundtruth file
  std::ifstream file;
  file.open(path);
  if (!file) {
    LOG(ERROR) << "ERROR: Unable to open simulation trajectory file...";
    std::exit(EXIT_FAILURE);
  }

  // Loop through each line of this file
  std::string base_filename = path.substr(path.find_last_of("/\\") + 1);
  std::string current_line;
  while (std::getline(file, current_line)) {

    // Skip if we start with a comment
    if (!current_line.find("#"))
      continue;

    // Loop variables
    int i = 0;
    std::istringstream s(current_line);
    std::string field;
    Eigen::Matrix<double, 8, 1> data;

    // Loop through this line (timestamp(s) tx ty tz qx qy qz qw)
    while (std::getline(s, field, ' ')) {
      // Skip if empty
      if (field.empty() || i >= data.rows())
        continue;
      // save the data to our vector
      data(i) = std::atof(field.c_str());
      i++;
    }

    // Only a valid line if we have all the parameters
    if (i > 7) {
      traj_data.push_back(data);

      // std::stringstream ss;
      // ss << std::setprecision(15) << data.transpose() << std::endl;
      // PRINT_DEBUG(ss.str().c_str());
    }
  }

  // Finally close the file
  file.close();

  // Error if we don't have any data
  if (traj_data.empty()) {
    LOG(ERROR) << "ERROR: No valid data found in simulation trajectory file...";
    std::exit(EXIT_FAILURE);
  }
}

Simulator::Simulator(SimConfig &params_, const std::string &imu_traj_path)
    : params(params_) {
  // Load the groundtruth trajectory and its spline
  LOG(INFO) << "Loading trajectory from: " << imu_traj_path;
  load_simulated_trajectory(imu_traj_path, traj_data);
  spline = std::make_shared<ov_core::BsplineSE3>();
  spline->feed_trajectory(traj_data);

  // Set all our timestamps as starting from the minimum spline time
  timestamp = spline->get_start_time();
  timestamp_last_imu = spline->get_start_time();
  timestamp_last_feat = spline->get_start_time();

  // Get the pose at the current timestep
  Eigen::Matrix3d R_GtoI_init;
  Eigen::Vector3d p_IinG_init;
  bool success_pose_init =
      spline->get_pose(timestamp, R_GtoI_init, p_IinG_init);
  if (!success_pose_init) {
    LOG(ERROR) << "Unable to get the pose at the first timestamp";
    std::exit(EXIT_FAILURE);
  }

  // Find the timestamp that we move enough to be considered "moved"
  double distance = 0.0;
  double distancethreshold = params.sim_distance_threshold;
  LOG(INFO) << "Distance threshold to start: " << distancethreshold << " m";
  while (true) {

    // Get the pose at the current timestep
    Eigen::Matrix3d R_GtoI;
    Eigen::Vector3d p_IinG;
    bool success_pose = spline->get_pose(timestamp, R_GtoI, p_IinG);

    // Check if it fails
    if (!success_pose) {
      LOG(ERROR)
          << "Unable to find a jolt in the groundtruth data to initialize at";
      std::exit(EXIT_FAILURE);
    }

    // Append to our scalar distance
    distance += (p_IinG - p_IinG_init).norm();
    p_IinG_init = p_IinG;

    // Now check if we have an acceleration, else move forward in time
    if (distance > distancethreshold) {
      break;
    } else {
      timestamp += 1.0 / params.sim_freq_meas;
      timestamp_last_imu += 1.0 / params.sim_freq_meas;
      timestamp_last_feat += 1.0 / params.sim_freq_meas;
    }
  }

  LOG(INFO)
      << "Moved " << timestamp - spline->get_start_time()
      << " seconds to find a jolt in the groundtruth data to initialize at";

  // Set the initial bias values
  true_bias_gyro = params_.sim_init_gyro_bias;
  true_bias_accel = params_.sim_init_accel_bias;

  LOG(INFO) << "Sim freq IMU: " << params.sim_freq_imu
            << ", sim freq meas: " << params.sim_freq_meas;
  // Append the current true bias to our history
  hist_true_bias_time.push_back(timestamp_last_imu - 1.0 / params.sim_freq_imu);
  hist_true_bias_accel.push_back(true_bias_accel);
  hist_true_bias_gyro.push_back(true_bias_gyro);
  hist_true_bias_time.push_back(timestamp_last_imu);
  hist_true_bias_accel.push_back(true_bias_accel);
  hist_true_bias_gyro.push_back(true_bias_gyro);
  hist_true_bias_time.push_back(timestamp_last_imu + 1.0 / params.sim_freq_imu);
  hist_true_bias_accel.push_back(true_bias_accel);
  hist_true_bias_gyro.push_back(true_bias_gyro);

  // Our simulation is running
  is_running = true;

  //===============================================================
  //===============================================================

  // Load the seeds for the random number generators
  gen_state_init = std::mt19937(params.sim_seed_state_init);
  gen_state_init.seed(params.sim_seed_state_init);
  gen_meas_imu = std::mt19937(params.sim_seed_measurements);
  gen_meas_imu.seed(params.sim_seed_measurements);
  gen_slam_feat_meas = std::mt19937(params.sim_seed_measurements);
  gen_slam_feat_meas.seed(params.sim_seed_measurements);

  // Create synthetic camera frames and ensure that each has enough features
  generateGroundtruthSlamFeatures();
}

void Simulator::generateGroundtruthSlamFeatures() {
  // Get the min and max x, y, and z values for the trajectory
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double min_z = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  double max_z = std::numeric_limits<double>::lowest();

  double timestamp = spline->get_start_time();
  double dt = 1.0;

  while (true) {
    Eigen::Matrix3d R_GtoI;
    Eigen::Vector3d p_IinG;
    bool success_pose = spline->get_pose(timestamp, R_GtoI, p_IinG);

    if (!success_pose) {
      break;
    }

    // Generate measurements from the current position
    size_t num_measurements = 0;
    for (auto const &feat : slam_featmap) {
      Eigen::Vector3d r_pw_a = feat.second;
      Eigen::Vector3d r_pz_b = R_GtoI * (r_pw_a - p_IinG);

      if (r_pz_b.norm() > params.max_sensing_range_slam) {
        continue;
      }
      num_measurements++;
    }

    // At each timestamp, we want to be able to see at least
    // num_slam_features features
    if (num_measurements < params.num_slam_features) {

      // Generate more features around this position
      double min_x = p_IinG.x() - params.max_sensing_range_slam;
      double max_x = p_IinG.x() + params.max_sensing_range_slam;
      double min_y = p_IinG.y() - params.max_sensing_range_slam;
      double max_y = p_IinG.y() + params.max_sensing_range_slam;
      double min_z = p_IinG.z() - params.max_sensing_range_slam;
      double max_z = p_IinG.z() + params.max_sensing_range_slam;
      std::uniform_real_distribution<double> gen_x(min_x, max_x);
      std::uniform_real_distribution<double> gen_y(min_y, max_y);
      std::uniform_real_distribution<double> gen_z(min_z, max_z);

      size_t num_feat_to_generate = params.num_slam_features - num_measurements;
      for (size_t i = 0; i < num_feat_to_generate; i++) {
        Eigen::Vector3d r_pw_a;
        r_pw_a.x() = gen_x(gen_state_init);
        r_pw_a.y() = gen_y(gen_state_init);
        r_pw_a.z() = gen_z(gen_state_init);
        slam_featmap.insert({id_slam_features, r_pw_a});
        id_slam_features++;
      }
    }

    timestamp += dt;
  }

  LOG(INFO) << "Generated " << slam_featmap.size()
            << " total SLAM features for the trajectory";
}

bool Simulator::getNextImu(double &time_imu, Eigen::Vector3d &wm,
                           Eigen::Vector3d &am) {

  // Return if the camera measurement should go before us
  if (timestamp_last_feat + 1.0 / params.sim_freq_meas <
      timestamp_last_imu + 1.0 / params.sim_freq_imu)
    return false;

  // Else lets do a new measurement!!!
  timestamp_last_imu += 1.0 / params.sim_freq_imu;
  timestamp = timestamp_last_imu;
  time_imu = timestamp_last_imu;

  // Current state values
  Eigen::Matrix3d R_GtoI;
  Eigen::Vector3d p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG;

  // Get the pose, velocity, and acceleration
  // NOTE: we get the acceleration between our two IMU
  // NOTE: this is because we are using a constant measurement model for
  // integration bool success_accel =
  // spline->get_acceleration(timestamp+0.5/freq_imu, R_GtoI, p_IinG, w_IinI,
  // v_IinG, alpha_IinI, a_IinG);
  bool success_accel = spline->get_acceleration(
      timestamp, R_GtoI, p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG);

  // If failed, then that means we don't have any more spline
  // Thus we should stop the simulation
  if (!success_accel) {
    is_running = false;
    return false;
  }

  // Transform omega and linear acceleration into imu frame
  Eigen::Vector3d gravity;
  gravity << 0.0, 0.0, params.gravity_mag;
  Eigen::Vector3d accel_inI = R_GtoI * (a_IinG + gravity);
  Eigen::Vector3d omega_inI = w_IinI;

  // Get our imu intrinsic parameters
  //  - kalibr: lower triangular of the matrix is used
  //  - rpng: upper triangular of the matrix is used
  // Eigen::Matrix3d Dw = State::Dm(params.state_options.imu_model,
  // params.vec_dw); Eigen::Matrix3d Da =
  // State::Dm(params.state_options.imu_model, params.vec_da); Eigen::Matrix3d
  // Tg = State::Tg(params.vec_tg);

  // // Get the readings with the imu intrinsic "distortion"
  // Eigen::Matrix3d Tw =
  // Dw.colPivHouseholderQr().solve(Eigen::Matrix3d::Identity());
  // Eigen::Matrix3d Ta =
  // Da.colPivHouseholderQr().solve(Eigen::Matrix3d::Identity());
  // Eigen::Vector3d omega_inGYRO = Tw *
  // quat_2_Rot(params.q_GYROtoIMU).transpose() * omega_inI + Tg * accel_inI;
  // Eigen::Vector3d accel_inACC = Ta *
  // quat_2_Rot(params.q_ACCtoIMU).transpose() * accel_inI;

  Eigen::Vector3d omega_inGYRO = omega_inI;
  Eigen::Vector3d accel_inACC = accel_inI;

  // Calculate the bias values for this IMU reading
  // NOTE: we skip the first ever bias since we have already appended it
  double dt = 1.0 / params.sim_freq_imu;
  std::normal_distribution<double> w(0, 1);
  if (has_skipped_first_bias) {

    double sigma_wb = params.imu_noises.sigma_gyro_bias;
    double sigma_ab = params.imu_noises.sigma_accel_bias;

    // Move the biases forward in time
    true_bias_gyro(0) += sigma_wb * std::sqrt(dt) * w(gen_meas_imu);
    true_bias_gyro(1) += sigma_wb * std::sqrt(dt) * w(gen_meas_imu);
    true_bias_gyro(2) += sigma_wb * std::sqrt(dt) * w(gen_meas_imu);
    true_bias_accel(0) += sigma_ab * std::sqrt(dt) * w(gen_meas_imu);
    true_bias_accel(1) += sigma_ab * std::sqrt(dt) * w(gen_meas_imu);
    true_bias_accel(2) += sigma_ab * std::sqrt(dt) * w(gen_meas_imu);

    // Append the current true bias to our history
    hist_true_bias_time.push_back(timestamp_last_imu);
    hist_true_bias_gyro.push_back(true_bias_gyro);
    hist_true_bias_accel.push_back(true_bias_accel);
  }
  has_skipped_first_bias = true;

  double sigma_w = params.imu_noises.sigma_gyro;
  double sigma_a = params.imu_noises.sigma_accel;
  // Now add noise to these measurements
  wm(0) = omega_inGYRO(0) + true_bias_gyro(0) +
          sigma_w / std::sqrt(dt) * w(gen_meas_imu);
  wm(1) = omega_inGYRO(1) + true_bias_gyro(1) +
          sigma_w / std::sqrt(dt) * w(gen_meas_imu);
  wm(2) = omega_inGYRO(2) + true_bias_gyro(2) +
          sigma_w / std::sqrt(dt) * w(gen_meas_imu);
  am(0) = accel_inACC(0) + true_bias_accel(0) +
          sigma_a / std::sqrt(dt) * w(gen_meas_imu);
  am(1) = accel_inACC(1) + true_bias_accel(1) +
          sigma_a / std::sqrt(dt) * w(gen_meas_imu);
  am(2) = accel_inACC(2) + true_bias_accel(2) +
          sigma_a / std::sqrt(dt) * w(gen_meas_imu);

  // Return success
  return true;
}

bool Simulator::getNextRelativeFeatures(
    double &time_cam, std::vector<std::pair<size_t, Eigen::Vector3d>> &meas) {
  // Return if the IMU should go before us
  // Return if the imu measurement should go before us
  if (timestamp_last_imu + 1.0 / params.sim_freq_imu <
      timestamp_last_feat + 1.0 / params.sim_freq_meas)
    return false;

  // Else lets do a new measurement!!!
  timestamp_last_feat += 1.0 / params.sim_freq_meas;
  timestamp = timestamp_last_feat;
  time_cam = timestamp_last_feat;

  // Get the pose at the current timestep
  Eigen::Matrix3d R_GtoI;
  Eigen::Vector3d p_IinG;
  bool success_pose = spline->get_pose(timestamp, R_GtoI, p_IinG);

  if (!success_pose) {
    is_running = false;
    return false;
  }

  // Generate measurements to each of our SLAM features
  for (const auto &feat : slam_featmap) {
    Eigen::Vector3d r_pw_a = feat.second;
    Eigen::Vector3d r_pz_b = R_GtoI * (r_pw_a - p_IinG);

    if (r_pz_b.norm() > params.max_sensing_range_slam) {
      // LOG(INFO) << "Feature " << feat.first
      //           << " is out of range: " << r_pz_b.norm() << " m";
      continue;
    }

    // Add noise
    std::normal_distribution<double> w(0, 1);
    Eigen::Vector3d meas_value = r_pz_b;
    meas_value(0) += params.sigma_feature_meas_3d * w(gen_slam_feat_meas);
    meas_value(1) += params.sigma_feature_meas_3d * w(gen_slam_feat_meas);
    meas_value(2) += params.sigma_feature_meas_3d * w(gen_slam_feat_meas);

    meas.push_back({feat.first, meas_value});
  }

  return true;
}

bool Simulator::getState(double desired_time, IMUState &imu_state){
  // Current state values
  Eigen::Matrix3d R_GtoI;
  Eigen::Vector3d p_IinG, w_IinI, v_IinG;

  // Get the pose, velocity, and acceleration
  bool success_vel =
      spline->get_velocity(desired_time, R_GtoI, p_IinG, w_IinI, v_IinG);

  // Find the bounding bias values
  bool success_bias = false;
  size_t id_loc = 0;
  for (size_t i = 0; i < hist_true_bias_time.size() - 1; i++) {
    if (hist_true_bias_time.at(i) < desired_time &&
        hist_true_bias_time.at(i + 1) >= desired_time) {
      id_loc = i;
      success_bias = true;
      break;
    }
  }

  if (!success_bias) {
    LOG(INFO) << "Unable to find bias at time " << desired_time;
    return false;
  }

  // If failed, then that means we don't have any more spline or bias
  if (!success_vel || !success_bias) {
    return false;
  }

  // Interpolate our biases (they will be at every IMU timestep)
  double lambda =
      (desired_time - hist_true_bias_time.at(id_loc)) /
      (hist_true_bias_time.at(id_loc + 1) - hist_true_bias_time.at(id_loc));
  Eigen::Vector3d true_bg_interp =
      (1 - lambda) * hist_true_bias_gyro.at(id_loc) +
      lambda * hist_true_bias_gyro.at(id_loc + 1);
  Eigen::Vector3d true_ba_interp =
      (1 - lambda) * hist_true_bias_accel.at(id_loc) +
      lambda * hist_true_bias_accel.at(id_loc + 1);

  // Update struct
  imu_state.timestamp = desired_time;
  imu_state.attitude = R_GtoI.transpose();
  imu_state.position = p_IinG;
  imu_state.velocity = v_IinG;
  imu_state.gyro_bias = true_bg_interp;
  imu_state.accel_bias = true_ba_interp;
  return true;
}

// IMUType
// Simulator::generateInitialImuState(IMUType x0,
//                                        Eigen::Matrix<double, 15, 15>
//                                        &init_cov, ExtendedPoseRepresentation
//                                        &pose_rep) {

//   std::normal_distribution<double> white_noise(0, 1);
//   x0.setDirection(params.direction);

//   if (params.sim_do_imu_perturbation) {
//     PRINT_DEBUG(RED "Generating perturbed initial state\n" RESET);
//     double sigma_init_att = params.sigma_init_att;
//     double sigma_init_vel = params.sigma_init_vel;
//     double sigma_init_bg = params.sigma_init_bg;
//     double sigma_init_ba = params.sigma_init_ba;

//     LOG(INFO) << "Perturbing initial IMU state with: "
//               << "sigma_init_att: " << sigma_init_att
//               << ", sigma_init_vel: " << sigma_init_vel
//               << ", sigma_init_bg: " << sigma_init_bg
//               << ", sigma_init_ba: " << sigma_init_ba;

//     // Perturb the initial state by a small amount
//     init_cov = Eigen::Matrix<double, 15, 15>::Identity() * 1e-7;
//     init_cov.block<2, 2>(0, 0) =
//         Eigen::Matrix2d::Identity() * sigma_init_att * sigma_init_att;
//     init_cov.block<3, 3>(3, 3) =
//         Eigen::Matrix3d::Identity() * sigma_init_vel * sigma_init_vel;
//     init_cov.block<3, 3>(9, 9) =
//         Eigen::Matrix3d::Identity() * sigma_init_bg * sigma_init_bg;
//     init_cov.block<3, 3>(12, 12) =
//         Eigen::Matrix3d::Identity() * sigma_init_ba * sigma_init_ba;

//     Eigen::Matrix<double, 15, 1> delta_xi;
//     delta_xi.setZero();
//     delta_xi(0) = sigma_init_att * white_noise(gen_state_perturb);
//     delta_xi(1) = sigma_init_att * white_noise(gen_state_perturb);
//     delta_xi(3) = sigma_init_vel * white_noise(gen_state_perturb);
//     delta_xi(4) = sigma_init_vel * white_noise(gen_state_perturb);
//     delta_xi(5) = sigma_init_vel * white_noise(gen_state_perturb);
//     delta_xi(9) = sigma_init_bg * white_noise(gen_state_perturb);
//     delta_xi(10) = sigma_init_bg * white_noise(gen_state_perturb);
//     delta_xi(11) = sigma_init_bg * white_noise(gen_state_perturb);
//     delta_xi(12) = sigma_init_ba * white_noise(gen_state_perturb);
//     delta_xi(13) = sigma_init_ba * white_noise(gen_state_perturb);
//     delta_xi(14) = sigma_init_ba * white_noise(gen_state_perturb);

//     if (pose_rep == ExtendedPoseRepresentation::SE23) {
//       IMUType x0_perturbed(x0.direction());
//       x0_perturbed.setNavState(x0.navState());
//       x0_perturbed.setGyroBias(x0.gyroBias());
//       x0_perturbed.setAccelBias(x0.accelBias());
//       x0_perturbed.setStamp(x0.stamp());
//       x0_perturbed.update(delta_xi);
//       return x0_perturbed;
//     } else if (pose_rep == ExtendedPoseRepresentation::Decoupled) {
//       Eigen::Matrix3d C_new =
//           x0.attitude() * SO3::expMap(delta_xi.block<3, 1>(0, 0));
//       Eigen::Vector3d v_new = x0.velocity() + delta_xi.block<3, 1>(3, 0);
//       Eigen::Vector3d r_new = x0.position() + delta_xi.block<3, 1>(6, 0);
//       Eigen::Vector3d bg_new = x0.gyroBias() + delta_xi.block<3, 1>(9, 0);
//       Eigen::Vector3d ba_new = x0.accelBias() + delta_xi.block<3, 1>(12, 0);
//       IMUType x0_perturbed(x0.direction());
//       x0_perturbed.setAllStates(C_new, v_new, r_new, bg_new, ba_new);
//       x0_perturbed.setStamp(x0.stamp());
//       return x0_perturbed;
//     }
//   } else {
//     init_cov = Eigen::Matrix<double, 15, 15>::Identity() * 1e-6;
//     // init_cov = Eigen::Matrix<double, 15, 15>::Identity();
//     // init_cov = std::pow(0.02, 2) * Eigen::Matrix<double, 15,
//     15>::Identity();
//     // init_cov.block<3, 3>(0, 0) =
//     //     std::pow(0.017, 2) * Eigen::Matrix3d::Identity();
//     // init_cov(2, 2) = 1e-5;
//     // init_cov.block<3, 3>(3, 3) =
//     //     std::pow(0.01, 2) * Eigen::Matrix3d::Identity();
//     // init_cov.block<3, 3>(6, 6) =
//     //     std::pow(0.001, 2) * Eigen::Matrix3d::Identity();
//     return x0;
//   }
// }

// void Simulator::perturbCalibrationParameters(
//     std::shared_ptr<VinsConfig> &vins_config) {
//   if (params.sim_do_calib_perturbation) {
//     LOG(INFO) << "Perturbing calibration parameters";

//     double sigma_att = params.sigma_init_cam_ext_att;
//     double sigma_pos = params.sigma_init_cam_ext_pos;
//     LOG(INFO) << "Perturbations for camera extrinsics:";
//     LOG(INFO) << "sigma_att: " << sigma_att << ", sigma_pos: " << sigma_pos;
//     for (auto const &extrinsics : vins_config->camera_extrinsics) {
//       std::normal_distribution<double> perturb_dist(0, 1);

//       Eigen::Matrix<double, 6, 1> delta_xi;
//       delta_xi.setZero();
//       delta_xi(0) = sigma_att * perturb_dist(gen_state_perturb);
//       delta_xi(1) = sigma_att * perturb_dist(gen_state_perturb);
//       delta_xi(2) = sigma_att * perturb_dist(gen_state_perturb);
//       delta_xi(3) = sigma_pos * perturb_dist(gen_state_perturb);
//       delta_xi(4) = sigma_pos * perturb_dist(gen_state_perturb);
//       delta_xi(5) = sigma_pos * perturb_dist(gen_state_perturb);

//       // Update the extrinsics
//       Eigen::Matrix4d new_extrinsics =
//           extrinsics.second * SE3::expMap(delta_xi);
//       vins_config->camera_extrinsics[extrinsics.first] = new_extrinsics;
//     }
//   }
// }