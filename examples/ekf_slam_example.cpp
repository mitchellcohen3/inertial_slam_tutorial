#include <boost/program_options.hpp>
#include <glog/logging.h>
#include <iostream>

#include "estimator/EKFSlamEstimator.h"
#include "sim/SimConfig.h"
#include "sim/Simulator.h"

#include "types/ImuEKFState.h"

#include "lieutils/LieDirection.h"
#include "utils/FileAccess.h"
#include "utils/SensorData.h"
#include "utils/Utility.h"

namespace po = boost::program_options;

po::variables_map handle_args(int argc, const char *argv[]) {
  po::options_description options("Allowed options");

  // clang-format off
  options.add_options()
  ("help", "produce help message")
  ("trajectory_path", po::value<std::string>()->required(), "Path to the IMU trajectory file")
  ("state_gt_path", po::value<std::string>()->default_value("state_gt.txt"), "Path to save groundtruth states")
  ("state_est_path", po::value<std::string>()->default_value("state_est.txt"), "Path to save estimated states")
  ("cov_est_path", po::value<std::string>()->default_value("cov_est.txt"), "Path to save estimated covariances")
  ("feature_map_path", po::value<std::string>()->default_value("feature_map.txt"), "Path to save estimated feature map");

  // clang-format on
  po::variables_map var_map;
  po::store(po::parse_command_line(argc, argv, options), var_map);

  if (var_map.count("help") || argc == 1) {
    std::cout << "Main entry point for EKF-SLAM example." << std::endl;
    std::cout << options << std::endl;
    exit(0);
  }

  po::notify(var_map);
  return var_map;
}

int main(int argc, const char **argv) {
  // Initialize Google's logging library
  google::InitGoogleLogging(argv[0]);
  FLAGS_colorlogtostderr = true;
  FLAGS_alsologtostderr = true;
  FLAGS_v = 0;

  auto args = handle_args(argc, argv);

  std::string traj_path = args["trajectory_path"].as<std::string>();
  std::string state_gt_path = args["state_gt_path"].as<std::string>();
  std::string state_est_path = args["state_est_path"].as<std::string>();
  std::string cov_est_path = args["cov_est_path"].as<std::string>();
  std::string feature_map_path = args["feature_map_path"].as<std::string>();
  LOG(INFO) << "Using trajectory path: " << traj_path;

  // Create output files
  createNewFile(state_gt_path);
  createNewFile(state_est_path);
  createNewFile(cov_est_path);
  createNewFile(feature_map_path);

  LOG(INFO) << "Created output files: ";
  LOG(INFO) << " - " << state_gt_path;
  LOG(INFO) << " - " << state_est_path;
  LOG(INFO) << " - " << cov_est_path;
  LOG(INFO) << " - " << feature_map_path;

  // Initialize the simulation
  SimConfig config;
  std::shared_ptr<Simulator> sim =
      std::make_shared<Simulator>(config, traj_path);
  LOG(INFO) << "Simulation initialized.";

  // Save the landmarks
  std::vector<Eigen::Vector3d> gt_landmarks = sim->getSlamFeatures();
  for (auto const &lm : gt_landmarks) {
    writeDataToFile(feature_map_path, lm, true);
  }
  
  // Create the estimator
  EstimatorConfig est_config;
  std::shared_ptr<EKFSlamEstimator> estimator =
      std::make_shared<EKFSlamEstimator>(est_config);

  // Main run loop
  double dt = 1.0 / config.sim_freq_imu;
  double next_imu_time = sim->currentTimestamp() + dt;
  double end_time = sim->currentTimestamp() + config.t_end;

  Eigen::Matrix<double, 15, 15> init_cov =
      Eigen::Matrix<double, 15, 15>::Identity() * 1e-7;
  IMUState init_imu_state;
  sim->getState(next_imu_time, init_imu_state);
  Eigen::Matrix<double, 5, 5> nav_state =
      Eigen::Matrix<double, 5, 5>::Identity();
  nav_state.block<3, 3>(0, 0) = init_imu_state.attitude;
  nav_state.block<3, 1>(0, 3) = init_imu_state.velocity;
  nav_state.block<3, 1>(0, 4) = init_imu_state.position;
  estimator->initializeIMUState(init_imu_state.timestamp, nav_state,
                                init_imu_state.gyro_bias,
                                init_imu_state.accel_bias, init_cov);


  LOG(INFO) << "Starting main simulation loop...";
  double buffer_timefeat = -1;
  std::vector<RelativeFeatureMessage> buffer_rel_feat;
  while (sim->ok()) {
    if (sim->currentTimestamp() > end_time) {
      LOG(INFO) << "Reached end of simulation time.";
      break;
    }

    // Get the IMU measurement at the next timestamp
    ImuMessage imu_meas;
    if (sim->getNextImu(imu_meas.timestamp, imu_meas.gyro, imu_meas.accel)) {
      estimator->inputIMU(imu_meas);
    } else {
      LOG(ERROR) << "Failed to get IMU measurement at time: " << next_imu_time;
    }

    // Check if this is a measurement time
    double time_feat;
    std::vector<std::pair<size_t, Eigen::Vector3d>> feat_meas;
    if (sim->getNextRelativeFeatures(time_feat, feat_meas)) {
      // Convert to our message type
      std::vector<RelativeFeatureMessage> relative_feat_meas;
      for (const auto &feat : feat_meas) {
        RelativeFeatureMessage msg;
        msg.timestamp = time_feat;
        msg.feature_id = feat.first;
        msg.meas = feat.second;

        if (!config.noise_active) {
          msg.covariance = 1e-5 * Eigen::Matrix3d::Identity();
        } else {
          msg.covariance = Eigen::Matrix3d::Identity() *
                           config.sigma_feature_meas_3d *
                           config.sigma_feature_meas_3d;
        }
        relative_feat_meas.push_back(msg);
      }

      if (buffer_timefeat != -1) {
        estimator->inputRelativeFeatureMeasurements(buffer_rel_feat,
                                                    buffer_timefeat);
      }

      buffer_timefeat = time_feat;
      buffer_rel_feat = relative_feat_meas;

      double estimator_timestamp = estimator->getEstimateTime();

      // Log the groundtruth state, estimated state, and covariance
      IMUState gt_state;
      if (sim->getState(estimator_timestamp, gt_state)) {
        Eigen::Matrix<double, 17, 1> gt_state_vec = toAslFormat(
            gt_state.attitude, gt_state.velocity, gt_state.position,
            gt_state.gyro_bias, gt_state.accel_bias, gt_state.timestamp);
        writeDataToFile(state_gt_path, gt_state_vec, true);
      }

      // Retrieve the estimated state
      std::shared_ptr<ImuEKFState> est_imu_state =
          estimator->getLatestIMUState();
      Eigen::Matrix<double, 17, 1> est_state_vec =
          toAslFormat(est_imu_state->attitude(), est_imu_state->velocity(),
                      est_imu_state->position(), est_imu_state->gyroBias(),
                      est_imu_state->accelBias(), estimator_timestamp);
      writeDataToFile(state_est_path, est_state_vec, true);

      Eigen::Matrix<double, 15, 15> est_cov =
          estimator->getLatestIMUCovariance();

      // Write the covariance as a flattened vector
      Eigen::Map<const Eigen::VectorXd> est_cov_vec(est_cov.data(),
                                                    est_cov.size());
      Eigen::VectorXd cov_with_stamp(est_cov_vec.size() + 1);
      cov_with_stamp(0) = estimator_timestamp;
      cov_with_stamp.tail(est_cov_vec.size()) = est_cov_vec;
      writeDataToFile(cov_est_path, cov_with_stamp, true);
    }
  }

  LOG(INFO) << "Simulation completed successfully.";
  return 0;
}