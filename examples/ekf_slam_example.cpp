#include <boost/program_options.hpp>
#include <glog/logging.h>
#include <iostream>

#include "estimator/EKFSlamEstimator.h"
#include "sim/SimConfig.h"
#include "sim/Simulator.h"

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

  // Create the estimator
  EstimatorConfig est_config;
  std::shared_ptr<EKFSlamEstimator> estimator =
      std::make_shared<EKFSlamEstimator>(est_config);

  // Main run loop
  LOG(INFO) << "Starting main simulation loop...";
  double dt = 1.0 / config.sim_freq_imu;
  double next_imu_time = sim->currentTimestamp() + dt;
  double end_time = sim->currentTimestamp() + config.t_end;
  while (sim->ok()) {
    if (sim->currentTimestamp() > end_time) {
      LOG(INFO) << "Reached end of simulation time.";
      break;
    }

    // Get the IMU measurement at the next timestamp
    ImuMessage imu_meas;
    if (sim->getNextImu(imu_meas.timestamp, imu_meas.gyro, imu_meas.accel)) {
      // LOG(INFO) << "Received IMU measurement at time: " <<
      // imu_meas.timestamp; estimator->inputIMU(imu_meas);
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
        msg.covariance =
            Eigen::Matrix3d::Identity() * 0.01; // Example covariance
        relative_feat_meas.push_back(msg);
      }

      LOG(INFO) << "Received " << relative_feat_meas.size()
                << " relative feature measurements at time: " << time_feat;
      // Get the groundtruth state
      IMUState gt_state;
      if (sim->getState(imu_meas.timestamp, gt_state)) {
        LOG(INFO) << "Received groundtruth state at time: " << time_feat;
        Eigen::Matrix<double, 17, 1> gt_state_vec = toAslFormat(
            gt_state.attitude, gt_state.velocity, gt_state.position,
            gt_state.gyro_bias, gt_state.accel_bias, gt_state.timestamp);
        writeDataToFile(state_gt_path, gt_state_vec, true);
      }
    }
  }

  LOG(INFO) << "Simulation completed successfully.";
  return 0;
}