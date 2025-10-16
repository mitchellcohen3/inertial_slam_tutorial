#pragma once

#include <random>
#include <unordered_map>

#include "config/SimConfig.h"
#include "sim/BsplineSE3.h"
#include "utils/SensorData.h"

class IMUType;
class PoseType;

/**
 * A class that simulates a visual-inertial system. Very similar to Simulator
 * class from OpenVins, but does not depend on VioManagerOptions.
 */
class VinsSimulator {
public:
  /**
   * @brief Default constructor
   */
  VinsSimulator(SimConfig &sim_config, const std::string &imu_traj_path);

  // Returns if we are actively simulating
  bool ok() const { return is_running; }

  // Gets the timestamp we have simulated up to
  double currentTimestamp() const { return timestamp; }

  /**
   * @brief Gets the simulation state at a given time.
   * @param desired_time Timestamp we want to get the state at
   * @param[out] imustate The IMU state at the desired time in MSCKF ordering:
   * [time (sec), q_GtoI, p_IinG, v_IinG, b_gyro, b_accel]
   * @return True if we have a state at the desired time, false otherwise.
   */
  bool getState(double desired_time, Eigen::Matrix<double, 17, 1> &imustate);

  /**
   * @brief Gets the next IMU measurement if available
   */
  bool getNextImu(double &time_imu, Eigen::Vector3d &wm, Eigen::Vector3d &am);

  /**
   * @brief Gets the next camera measurement if available
   */
  bool getNextCam(
      double &time_cam, std::vector<int> &camids,
      std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> &feats);

  /**
   * @brief Gets the next set of 3D SLAM features if available
   *
   * NOTE: This will update the camera timestamp, and is meant to be called for
   * simulations of SLAM systems that use 3D features measurements.
   */
  bool getNextRelativeFeatures(
      double &time_cam, std::vector<std::pair<size_t, Eigen::Vector3d>> &feats);

  std::unordered_map<size_t, Eigen::Vector3d> getMap() const { return featmap; }

  std::vector<Eigen::Vector3d> getMapVec() {
    std::vector<Eigen::Vector3d> map_vec;
    for (const auto &pair : featmap) {
      map_vec.push_back(pair.second);
    }
    return map_vec;
  }

  /**
   * @brief Returns the simulation configuration parameters
   */
  SimConfig getSimConfig() const { return params; }

  /**
   * @brief Gets the simulated IMU state from the spline at a given timestamp.
   */
  bool getImuState(IMUType &imu_state, double timestamp);

  /**
   * @brief Gets the next simulated GPS message if available
   */
  bool getNextGpsMessage(GpsMessage &gps_data);

  /**
   * @brief Generates the initial IMU state.
   */
  IMUType generateInitialImuState(IMUType x0,
                                  Eigen::Matrix<double, 15, 15> &init_cov,
                                  ExtendedPoseRepresentation &pose_rep);

  /**
   * @brief Generate a perturbation for the IMU state
   */
  Eigen::Matrix<double, 15, 1> generateIMUPerturbation();

  /**
   * @brief Generates relative pose measurements given a set of true poses
   */
  std::vector<RelativePoseMessage>
  generateRelativePoseMeasurements(const std::vector<IMUType> &imu_states);

  void reset();

  /**
   * @brief Will generate points in the fov of the specified camera
   * @param R_GtoI Orientation of the IMU pose
   * @param p_IinG Position of the IMU pose
   * @param camid Camera id of the camera sensor we want to project into
   * @param[out] feats Map we will append new features to
   * @param numpts Number of points we should generate
   */
  void generatePoints(const Eigen::Matrix3d &R_GtoI,
                      const Eigen::Vector3d &p_IinG, size_t camid,
                      std::unordered_map<size_t, Eigen::Vector3d> &feats,
                      int numpts);

  /**
   * @brief Projects the passed map features into the desired camera frame.
   * @param R_GtoI Orientation of the IMU pose
   * @param p_IinG Position of the IMU pose
   * @param camid Camera id of the camera sensor we want to project into
   * @param feats Our set of 3d features
   * @return True distorted raw image measurements and their ids for the
   * specified camera
   */
  std::vector<std::pair<size_t, Eigen::VectorXf>>
  projectPointCloud(const Eigen::Matrix3d &R_GtoI,
                    const Eigen::Vector3d &p_IinG, int camid,
                    const std::unordered_map<size_t, Eigen::Vector3d> &feats);

  /**
   * @brief Takes in the calibration parameters and perturbs them if necessary.
   */
  void perturbCalibrationParameters(std::shared_ptr<VinsConfig> &params);

  std::vector<Eigen::Vector3d> getSlamFeatures() {
    std::vector<Eigen::Vector3d> map_vec;
    for (const auto &pair : slam_featmap) {
      map_vec.push_back(pair.second);
    }
    return map_vec;
  }

  std::unordered_map<size_t, Eigen::Vector3d> getSlamMap() {
    return slam_featmap;
  }

protected:
  // The simulation parameters
  // This allows us to perturb the calibration parameters,
  // noise parameters, etc.
  SimConfig params;

  // The groundtruth trajectory data
  std::vector<Eigen::VectorXd> traj_data;

  // B-Spline for the IMU trajectory
  std::shared_ptr<ov_core::BsplineSE3> spline;

  // Map of 3D features
  size_t id_map = 0;
  std::unordered_map<size_t, Eigen::Vector3d> featmap;

  // Features that we will generate for 3D SLAM
  size_t id_slam_features = 0;
  std::unordered_map<size_t, Eigen::Vector3d> slam_featmap;

  /// If our simulation is running
  bool is_running;

  // Mersenne twister PRNG for measurements
  std::mt19937 gen_meas_imu;
  std::vector<std::mt19937> gen_meas_cams;
  std::mt19937 gen_state_init;
  std::mt19937 gen_state_perturb;
  std::mt19937 gen_gps;
  std::mt19937 gen_relative_pose_meas;
  std::mt19937 gen_3d_slam_feature_meas;

  /// Timestamps
  /// Current timestamp of the system
  double timestamp;

  /// Last time we had an IMU reading
  double timestamp_last_imu;

  /// Last time we had an CAMERA reading
  double timestamp_last_cam;

  /// Our running acceleration bias
  Eigen::Vector3d true_bias_accel = Eigen::Vector3d::Zero();

  /// Our running gyroscope bias
  Eigen::Vector3d true_bias_gyro = Eigen::Vector3d::Zero();

  // Our history of true biases
  bool has_skipped_first_bias = false;
  std::vector<double> hist_true_bias_time;
  std::vector<Eigen::Vector3d> hist_true_bias_accel;
  std::vector<Eigen::Vector3d> hist_true_bias_gyro;

  // Last sensor simulation times
  double timestamp_last_gps;
  double timestamp_last_relative_feat_meas;

  /**
   * @brief Generates the groundtruth 3D SLAM features
   */
  void generateGroundtruthSlamFeatures();
};

// /** Helper functions used to generate measurements and initial state
// estimates*/ std::vector<RelativePoseMessage>
// generateRelativePoseMeasurements(
//     const std::vector<IMUType> &imu_states, std::mt19937 &gen, const
//     VinsConfig &config);

// // Generate quaadric estimates by perturbing groundtruth
// std::map<int, QuadricType> generateInitialQuadricEstimates(
//     Eigen::Matrix<double, 9, 9> &init_cov, std::map<int, QuadricType>
//     &quadrics, std::mt19937 &gen_init_state, const VinsConfig
//     &estimator_params_);