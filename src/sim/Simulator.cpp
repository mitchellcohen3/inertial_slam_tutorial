#include "VinsSimulator.h"

#include "cam/CamEqui.h"
#include "cam/CamRadtan.h"

#include "lie/LieDirection.h"
#include "lie/SE3.h"
#include "lie/SO3.h"

#include "types/ImuType.h"
#include "types/PoseType.h"

#include "utils/FileAccess.h"
#include "utils/print.h"
#include "utils/utility.h"

#include "utils/Drawing.h"
#include "utils/dataset_reader.h"

#include "utils/quat_ops.h"

#include "config/VinsConfig.h"

VinsSimulator::VinsSimulator(SimConfig &params_,
                             const std::string &imu_traj_path)
    : params(params_) {
  LOG(INFO) << "Visual-Inertial simulator started!";

  // Load the groundtruth trajectory and its spline
  LOG(INFO) << "Loading trajectory from: " << imu_traj_path;
  ov_core::DatasetReader::load_simulated_trajectory(imu_traj_path, traj_data);
  spline = std::make_shared<ov_core::BsplineSE3>();
  spline->feed_trajectory(traj_data);

  // Set all our timestamps as starting from the minimum spline time
  timestamp = spline->get_start_time();
  timestamp_last_imu = spline->get_start_time();
  timestamp_last_cam = spline->get_start_time();

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
      timestamp_last_cam += 1.0 / params.sim_freq_meas;
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
  gen_state_perturb = std::mt19937(params.sim_seed_perturb);
  gen_state_perturb.seed(params.sim_seed_perturb);
  gen_meas_imu = std::mt19937(params.sim_seed_measurements);
  gen_meas_imu.seed(params.sim_seed_measurements);
  gen_3d_slam_feature_meas = std::mt19937(params.sim_seed_measurements);
  gen_3d_slam_feature_meas.seed(params.sim_seed_measurements);

  // Create generator for our camera
  for (int i = 0; i < params.num_cameras; i++) {
    gen_meas_cams.push_back(std::mt19937(params.sim_seed_measurements));
    gen_meas_cams.at(i).seed(params.sim_seed_measurements);
  }

  // Create synthetic camera frames and ensure that each has enough features

  double dt = 0.25;
  size_t mapsize = featmap.size();
  LOG(INFO) << "Generating " << params.num_pts
            << " features per frame for each camera";

  // Loop through each camera
  // NOTE: we loop through cameras here so that the feature map for camera 1
  // will always be the same NOTE: thus when we add more cameras the first
  // camera should get the same measurements
  for (int i = 0; i < params.num_cameras; i++) {

    // Reset the start time
    double time_synth = spline->get_start_time();

    // Loop through each pose and generate our feature map in them!!!!
    while (true) {

      // Get the pose at the current timestep
      Eigen::Matrix3d R_GtoI;
      Eigen::Vector3d p_IinG;
      bool success_pose = spline->get_pose(time_synth, R_GtoI, p_IinG);

      // We have finished generating features
      if (!success_pose)
        break;

      // Get the uv features for this frame
      std::vector<std::pair<size_t, Eigen::VectorXf>> uvs =
          projectPointCloud(R_GtoI, p_IinG, i, featmap);
      // If we do not have enough, generate more
      if ((int)uvs.size() < params.num_pts) {
        generatePoints(R_GtoI, p_IinG, i, featmap,
                       params.num_pts - (int)uvs.size());
      }

      // Move forward in time
      time_synth += dt;
    }

    // Debug print
    PRINT_DEBUG("[SIM]: Generated %d map features in total over %d frames "
                "(camera %d)\n",
                (int)(featmap.size() - mapsize),
                (int)((time_synth - spline->get_start_time()) / dt), i);
    mapsize = featmap.size();
  }

  generateGroundtruthSlamFeatures();

  // Initialize random number generators
  gen_gps = std::mt19937(params.sim_seed_measurements);
  gen_gps.seed(params.sim_seed_measurements);
  gen_relative_pose_meas = std::mt19937(params.sim_seed_measurements);
  gen_relative_pose_meas.seed(params.sim_seed_measurements);

  // Initialize last sensor simulation times
  timestamp_last_gps = timestamp_last_cam;
  timestamp_last_relative_feat_meas = timestamp_last_cam;
}

void VinsSimulator::generateGroundtruthSlamFeatures() {
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

      LOG(INFO) << "Number of features to generate at time " << timestamp
                << ": " << params.num_slam_features - num_measurements;
      size_t num_feat_to_generate = params.num_slam_features - num_measurements;
      for (size_t i = 0; i < num_feat_to_generate; i++){
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

std::vector<std::pair<size_t, Eigen::VectorXf>>
VinsSimulator::projectPointCloud(
    const Eigen::Matrix3d &R_GtoI, const Eigen::Vector3d &p_IinG, int camid,
    const std::unordered_map<size_t, Eigen::Vector3d> &feats) {

  // Assert we have good camera
  assert(camid < params.state_options.num_cameras);
  assert((int)params.camera_intrinsics.size() ==
         params.state_options.num_cameras);
  assert((int)params.camera_extrinsics.size() ==
         params.state_options.num_cameras);

  // Grab our extrinsic and intrinsic values
  Eigen::Matrix<double, 3, 3> C_bc =
      params.camera_extrinsics.at(camid).block<3, 3>(0, 0);
  Eigen::Vector3d r_cz_b = params.camera_extrinsics.at(camid).block<3, 1>(0, 3);
  std::shared_ptr<ov_core::CamBase> camera = params.cam_intrinsics.at(camid);

  // Our projected uv true measurements
  std::vector<std::pair<size_t, Eigen::VectorXf>> uvs;

  // Loop through our map
  for (const auto &feat : feats) {

    // Transform feature into current camera frame
    Eigen::Vector3d p_FinI = R_GtoI * (feat.second - p_IinG);
    Eigen::Vector3d p_FinC = C_bc.transpose() * (p_FinI - r_cz_b);

    // Skip cloud if too far away
    if (p_FinC(2) > params.sim_max_feature_gen_dist || p_FinC(2) < 0.1)
      continue;

    // Project to normalized coordinates
    Eigen::Vector2f uv_norm;
    uv_norm << (float)(p_FinC(0) / p_FinC(2)), (float)(p_FinC(1) / p_FinC(2));

    // Distort the normalized coordinates
    Eigen::Vector2f uv_dist = camera->distort_f(uv_norm);

    // Check that it is inside our bounds
    if (uv_dist(0) < 0 || uv_dist(0) > camera->w() || uv_dist(1) < 0 ||
        uv_dist(1) > camera->h()) {
      continue;
    }

    // Else we can add this as a good projection
    uvs.push_back({feat.first, uv_dist});
  }

  // Return our projections
  return uvs;
}

void VinsSimulator::generatePoints(
    const Eigen::Matrix3d &R_GtoI, const Eigen::Vector3d &p_IinG, size_t camid,
    std::unordered_map<size_t, Eigen::Vector3d> &feats, int numpts) {

  // Assert we have good camera
  assert(camid < params.state_options.num_cameras);
  assert((int)params.camera_intrinsics.size() ==
         params.state_options.num_cameras);
  assert((int)params.camera_extrinsics.size() ==
         params.state_options.num_cameras);

  // Grab our extrinsic and intrinsic values
  Eigen::Matrix<double, 3, 3> C_bc =
      params.camera_extrinsics.at(camid).block<3, 3>(0, 0);
  Eigen::Vector3d r_cz_b = params.camera_extrinsics.at(camid).block<3, 1>(0, 3);
  std::shared_ptr<ov_core::CamBase> camera = params.cam_intrinsics.at(camid);

  // Generate the desired number of features
  for (int i = 0; i < numpts; i++) {

    // Uniformly randomly generate within our fov
    std::uniform_real_distribution<double> gen_u(0, camera->w());
    std::uniform_real_distribution<double> gen_v(0, camera->h());
    double u_dist = gen_u(gen_state_init);
    double v_dist = gen_v(gen_state_init);

    // Convert to opencv format
    cv::Point2f uv_dist((float)u_dist, (float)v_dist);

    // Undistort this point to our normalized coordinates
    cv::Point2f uv_norm = camera->undistort_cv(uv_dist);

    // Generate a random depth
    std::uniform_real_distribution<double> gen_depth(
        params.sim_min_feature_gen_dist, params.sim_max_feature_gen_dist);
    double depth = gen_depth(gen_state_init);

    // Get the 3d point
    Eigen::Vector3d bearing;
    bearing << uv_norm.x, uv_norm.y, 1;
    Eigen::Vector3d p_FinC;
    p_FinC = depth * bearing;

    // Move to the global frame of reference
    Eigen::Vector3d p_FinI = C_bc * p_FinC + r_cz_b;
    Eigen::Vector3d p_FinG = R_GtoI.transpose() * p_FinI + p_IinG;

    // Append this as a new feature
    featmap.insert({id_map, p_FinG});
    id_map++;
  }
}

bool VinsSimulator::getNextImu(double &time_imu, Eigen::Vector3d &wm,
                               Eigen::Vector3d &am) {

  // Return if the camera measurement should go before us
  if (timestamp_last_cam + 1.0 / params.sim_freq_meas <
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

bool VinsSimulator::getNextCam(
    double &time_cam, std::vector<int> &camids,
    std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> &feats) {

  // Return if the imu measurement should go before us
  if (timestamp_last_imu + 1.0 / params.sim_freq_imu <
      timestamp_last_cam + 1.0 / params.sim_freq_meas)
    return false;

  // Else lets do a new measurement!!!
  timestamp_last_cam += 1.0 / params.sim_freq_meas;
  timestamp = timestamp_last_cam;
  time_cam = timestamp_last_cam - params.calib_camimu_dt;

  // Get the pose at the current timestep
  Eigen::Matrix3d R_GtoI;
  Eigen::Vector3d p_IinG;
  bool success_pose = spline->get_pose(timestamp, R_GtoI, p_IinG);

  // We have finished generating measurements
  if (!success_pose) {
    is_running = false;
    return false;
  }

  // Loop through each camera
  for (int i = 0; i < params.num_cameras; i++) {

    // Get the uv features for this frame
    std::vector<std::pair<size_t, Eigen::VectorXf>> uvs =
        projectPointCloud(R_GtoI, p_IinG, i, featmap);

    // If we do not have enough, generate more
    if ((int)uvs.size() < params.num_pts) {
      // PRINT_WARNING(YELLOW "[SIM]: cam %d was unable to generate enough "
      //                      "features (%d < %d projections)\n" RESET,
      //               (int)i, (int)uvs.size(), params.num_pts);
      LOG(WARNING) << "[SIM] Camera " << i
                   << " was unable to generate enough features (" << uvs.size()
                   << " < " << params.num_pts << " projections)";
    }

    // If greater than only select the first set
    if ((int)uvs.size() > params.num_pts) {
      uvs.erase(uvs.begin() + params.num_pts, uvs.end());
    }

    // Append the map size so all cameras have unique features in them (but the
    // same map) Only do this if we are not enforcing stereo constraints between
    // all our cameras for (size_t f = 0; f < uvs.size() && !params.use_stereo;
    // f++) {
    //   uvs.at(f).first += i * featmap.size();
    // }

    // Loop through and add noise to each uv measurement
    std::normal_distribution<double> w(0, 1);
    for (size_t j = 0; j < uvs.size(); j++) {
      uvs.at(j).second(0) += params.sigma_pix * w(gen_meas_cams.at(i));
      uvs.at(j).second(1) += params.sigma_pix * w(gen_meas_cams.at(i));
    }

    // Push back for this camera
    feats.push_back(uvs);
    camids.push_back(i);
  }

  // Return success
  return true;
}

bool VinsSimulator::getNextRelativeFeatures(
    double &time_cam, std::vector<std::pair<size_t, Eigen::Vector3d>> &meas) {
  // Return if the IMU should go before us
  // Return if the imu measurement should go before us
  if (timestamp_last_imu + 1.0 / params.sim_freq_imu <
      timestamp_last_cam + 1.0 / params.sim_freq_meas)
    return false;

  // Else lets do a new measurement!!!
  timestamp_last_cam += 1.0 / params.sim_freq_meas;
  timestamp = timestamp_last_cam;
  time_cam = timestamp_last_cam - params.calib_camimu_dt;

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
    meas_value(0) += params.sigma_feature_meas_3d * w(gen_relative_pose_meas);
    meas_value(1) += params.sigma_feature_meas_3d * w(gen_relative_pose_meas);
    meas_value(2) += params.sigma_feature_meas_3d * w(gen_relative_pose_meas);

    meas.push_back({feat.first, meas_value});
  }

  return true;
}

// VinsSimulator::VinsSimulator(ov_msckf::VioManagerOptions &imu_traj_params,
//                              VinsConfig &estimator_params)
//     : Simulator(imu_traj_params), estimator_params_(estimator_params) {

//   // Initialize random number generators
//   gen_gps = std::mt19937(estimator_params_.sim_seed_measurements);
//   gen_gps.seed(estimator_params_.sim_seed_measurements);
//   gen_relative_pose_meas =
//   std::mt19937(estimator_params_.sim_seed_measurements);
//   gen_relative_pose_meas.seed(estimator_params_.sim_seed_measurements);
//   gen_init_state = std::mt19937(estimator_params_.sim_seed_state_init);
//   gen_init_state.seed(estimator_params_.sim_seed_state_init);

//   // Initialize last sensor simulation times
//   timestamp_last_gps = timestamp_last_cam;
//   timestamp_last_relative_feat_meas = timestamp_last_cam;

//   PRINT_DEBUG("Timestamp last cam: %.2f\n", timestamp_last_cam);
//   PRINT_DEBUG("Timestamp last GPS: %.2f\n", timestamp_last_gps);
// }

bool VinsSimulator::getNextGpsMessage(GpsMessage &gps_data) {
  // Return if the  IMU should go before us
  if (timestamp_last_imu + 1.0 / params.sim_freq_imu <
      timestamp_last_cam + 1.0 / params.sim_freq_meas) {
    return false;
  }

  // Else lets do a measurement!
  timestamp_last_cam += 1.0 / params.sim_freq_meas;
  timestamp = timestamp_last_cam;

  // Get the pose at the current time
  Eigen::Matrix3d R_GtoI;
  Eigen::Vector3d p_IinG;
  bool success_pose = spline->get_pose(timestamp, R_GtoI, p_IinG);
  if (!success_pose) {
    LOG(ERROR) << "Unable to get the pose at the GPS timestamp";
    LOG(ERROR) << "Timestamp: " << timestamp;
    is_running = false;
    return false;
  }

  // Generate a noisy position measurement
  Eigen::Vector3d meas_value = p_IinG;
  double sigma_gps = params.sigma_gps;

  std::normal_distribution<double> w(0, 1);
  meas_value(0) += sigma_gps * w(gen_gps);
  meas_value(1) += sigma_gps * w(gen_gps);
  meas_value(2) += sigma_gps * w(gen_gps);

  Eigen::Matrix3d meas_cov = Eigen::Matrix3d::Identity() * 1e-7;
  if (params.noise_active) {
    double sigma_gps = params.sigma_gps;
    meas_cov = Eigen::Matrix3d::Identity() * sigma_gps * sigma_gps;
  }
  gps_data.covariance = meas_cov;
  gps_data.meas = meas_value;
  gps_data.timestamp = timestamp;

  return true;
}

bool VinsSimulator::getState(double desired_time,
                             Eigen::Matrix<double, 17, 1> &imustate) {

  // Set to default state
  imustate.setZero();
  imustate(4) = 1;

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
    // LOG(INFO) << "Bias not found at time " << desired_time
    //           << ", trying to find the nearest bias";
    // auto it = std::min_element(
    //     hist_true_bias_time.begin(), hist_true_bias_time.end(),
    //     [desired_time](double a, double b) {
    //       return std::abs(a - desired_time) < std::abs(b - desired_time);
    //     });

    // int index = std::distance(hist_true_bias_time.begin(), it);
    // id_loc = index;
    // LOG(INFO) << "Using bias at time "
    //           << hist_true_bias_time.at(id_loc);
    // success_bias = true;
    return false;
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

  // Finally lets create the current state
  imustate(0, 0) = desired_time;
  imustate.block(1, 0, 4, 1) = ov_core::rot_2_quat(R_GtoI);
  imustate.block(5, 0, 3, 1) = p_IinG;
  imustate.block(8, 0, 3, 1) = v_IinG;
  imustate.block(11, 0, 3, 1) = true_bg_interp;
  imustate.block(14, 0, 3, 1) = true_ba_interp;
  return true;
}
bool VinsSimulator::getImuState(IMUType &imu_state, double stamp) {
  // Get the current IMU state
  Eigen::Matrix<double, 17, 1> cur_imu_state_mat;
  bool valid_state = getState(stamp, cur_imu_state_mat);
  if (!valid_state) {
    return false;
  }

  // Place into our holder
  double qw = cur_imu_state_mat(4, 0);
  double qx = cur_imu_state_mat(1, 0);
  double qy = cur_imu_state_mat(2, 0);
  double qz = cur_imu_state_mat(3, 0);

  // Note, that when we use Eigen to convert the quaternion to a rotation
  // matrix, it gives us C_ab
  Eigen::Quaterniond q(qw, qx, qy, qz);
  Eigen::Matrix3d C_ab = q.toRotationMatrix();
  Eigen::Vector3d r = cur_imu_state_mat.block<3, 1>(5, 0);
  Eigen::Vector3d v = cur_imu_state_mat.block<3, 1>(8, 0);
  Eigen::Vector3d bg = cur_imu_state_mat.block<3, 1>(11, 0);
  Eigen::Vector3d ba = cur_imu_state_mat.block<3, 1>(14, 0);
  imu_state.setDirection(params.direction);
  imu_state.setAllStates(C_ab, v, r, bg, ba);
  imu_state.setStamp(stamp);

  return true;
}

IMUType
VinsSimulator::generateInitialImuState(IMUType x0,
                                       Eigen::Matrix<double, 15, 15> &init_cov,
                                       ExtendedPoseRepresentation &pose_rep) {

  std::normal_distribution<double> white_noise(0, 1);
  x0.setDirection(params.direction);

  if (params.sim_do_imu_perturbation) {
    PRINT_DEBUG(RED "Generating perturbed initial state\n" RESET);
    double sigma_init_att = params.sigma_init_att;
    double sigma_init_vel = params.sigma_init_vel;
    double sigma_init_bg = params.sigma_init_bg;
    double sigma_init_ba = params.sigma_init_ba;

    LOG(INFO) << "Perturbing initial IMU state with: "
              << "sigma_init_att: " << sigma_init_att
              << ", sigma_init_vel: " << sigma_init_vel
              << ", sigma_init_bg: " << sigma_init_bg
              << ", sigma_init_ba: " << sigma_init_ba;

    // Perturb the initial state by a small amount
    init_cov = Eigen::Matrix<double, 15, 15>::Identity() * 1e-7;
    init_cov.block<2, 2>(0, 0) =
        Eigen::Matrix2d::Identity() * sigma_init_att * sigma_init_att;
    init_cov.block<3, 3>(3, 3) =
        Eigen::Matrix3d::Identity() * sigma_init_vel * sigma_init_vel;
    init_cov.block<3, 3>(9, 9) =
        Eigen::Matrix3d::Identity() * sigma_init_bg * sigma_init_bg;
    init_cov.block<3, 3>(12, 12) =
        Eigen::Matrix3d::Identity() * sigma_init_ba * sigma_init_ba;

    Eigen::Matrix<double, 15, 1> delta_xi;
    delta_xi.setZero();
    delta_xi(0) = sigma_init_att * white_noise(gen_state_perturb);
    delta_xi(1) = sigma_init_att * white_noise(gen_state_perturb);
    delta_xi(3) = sigma_init_vel * white_noise(gen_state_perturb);
    delta_xi(4) = sigma_init_vel * white_noise(gen_state_perturb);
    delta_xi(5) = sigma_init_vel * white_noise(gen_state_perturb);
    delta_xi(9) = sigma_init_bg * white_noise(gen_state_perturb);
    delta_xi(10) = sigma_init_bg * white_noise(gen_state_perturb);
    delta_xi(11) = sigma_init_bg * white_noise(gen_state_perturb);
    delta_xi(12) = sigma_init_ba * white_noise(gen_state_perturb);
    delta_xi(13) = sigma_init_ba * white_noise(gen_state_perturb);
    delta_xi(14) = sigma_init_ba * white_noise(gen_state_perturb);

    if (pose_rep == ExtendedPoseRepresentation::SE23) {
      IMUType x0_perturbed(x0.direction());
      x0_perturbed.setNavState(x0.navState());
      x0_perturbed.setGyroBias(x0.gyroBias());
      x0_perturbed.setAccelBias(x0.accelBias());
      x0_perturbed.setStamp(x0.stamp());
      x0_perturbed.update(delta_xi);
      return x0_perturbed;
    } else if (pose_rep == ExtendedPoseRepresentation::Decoupled) {
      Eigen::Matrix3d C_new =
          x0.attitude() * SO3::expMap(delta_xi.block<3, 1>(0, 0));
      Eigen::Vector3d v_new = x0.velocity() + delta_xi.block<3, 1>(3, 0);
      Eigen::Vector3d r_new = x0.position() + delta_xi.block<3, 1>(6, 0);
      Eigen::Vector3d bg_new = x0.gyroBias() + delta_xi.block<3, 1>(9, 0);
      Eigen::Vector3d ba_new = x0.accelBias() + delta_xi.block<3, 1>(12, 0);
      IMUType x0_perturbed(x0.direction());
      x0_perturbed.setAllStates(C_new, v_new, r_new, bg_new, ba_new);
      x0_perturbed.setStamp(x0.stamp());
      return x0_perturbed;
    }
  } else {
    init_cov = Eigen::Matrix<double, 15, 15>::Identity() * 1e-6;
    // init_cov = Eigen::Matrix<double, 15, 15>::Identity();
    // init_cov = std::pow(0.02, 2) * Eigen::Matrix<double, 15, 15>::Identity();
    // init_cov.block<3, 3>(0, 0) =
    //     std::pow(0.017, 2) * Eigen::Matrix3d::Identity();
    // init_cov(2, 2) = 1e-5;
    // init_cov.block<3, 3>(3, 3) =
    //     std::pow(0.01, 2) * Eigen::Matrix3d::Identity();
    // init_cov.block<3, 3>(6, 6) =
    //     std::pow(0.001, 2) * Eigen::Matrix3d::Identity();
    return x0;
  }
}

void VinsSimulator::perturbCalibrationParameters(
    std::shared_ptr<VinsConfig> &vins_config) {
  if (params.sim_do_calib_perturbation) {
    LOG(INFO) << "Perturbing calibration parameters";

    double sigma_att = params.sigma_init_cam_ext_att;
    double sigma_pos = params.sigma_init_cam_ext_pos;
    LOG(INFO) << "Perturbations for camera extrinsics:";
    LOG(INFO) << "sigma_att: " << sigma_att << ", sigma_pos: " << sigma_pos;
    for (auto const &extrinsics : vins_config->camera_extrinsics) {
      std::normal_distribution<double> perturb_dist(0, 1);

      Eigen::Matrix<double, 6, 1> delta_xi;
      delta_xi.setZero();
      delta_xi(0) = sigma_att * perturb_dist(gen_state_perturb);
      delta_xi(1) = sigma_att * perturb_dist(gen_state_perturb);
      delta_xi(2) = sigma_att * perturb_dist(gen_state_perturb);
      delta_xi(3) = sigma_pos * perturb_dist(gen_state_perturb);
      delta_xi(4) = sigma_pos * perturb_dist(gen_state_perturb);
      delta_xi(5) = sigma_pos * perturb_dist(gen_state_perturb);

      // Update the extrinsics
      Eigen::Matrix4d new_extrinsics =
          extrinsics.second * SE3::expMap(delta_xi);
      vins_config->camera_extrinsics[extrinsics.first] = new_extrinsics;
    }
  }
}