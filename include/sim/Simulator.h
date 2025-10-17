#pragma once

#include <random>
#include <unordered_map>

#include "sim/SimConfig.h"
#include "sim/BsplineSE3.h"
#include "utils/SensorData.h"

/**
 * @brief A simple holder for an IMU state
 */
struct IMUState {
  double timestamp;
  Eigen::Matrix3d attitude;
  Eigen::Vector3d gyro_bias;
  Eigen::Vector3d accel_bias;
  Eigen::Vector3d velocity;
  Eigen::Vector3d position;
};

/**
 * @brief A simulator class to generate IMU and 3D SLAM feature measurements
 */
class Simulator {
public:
  /**
   * @brief Initializes the simulator with the given configuration and a
   * trajectory path.
   */
  Simulator(SimConfig &sim_config, const std::string &imu_traj_path);

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
  bool getState(double desired_time, IMUState &imustate);

  /**
   * @brief Gets the next IMU measurement if available
   */
  bool getNextImu(double &time_imu, Eigen::Vector3d &wm, Eigen::Vector3d &am);

  /**
   * @brief Gets the next set of 3D SLAM features if available
   *
   * NOTE: This will update the camera timestamp, and is meant to be called for
   * simulations of SLAM systems that use 3D features measurements.
   */
  bool getNextRelativeFeatures(
      double &time_cam, std::vector<std::pair<size_t, Eigen::Vector3d>> &feats);

  // /**
  //  * @brief Generates the initial IMU state.
  //  */
  // IMUType generateInitialImuState(IMUType x0,
  //                                 Eigen::Matrix<double, 15, 15> &init_cov,
  //                                 ExtendedPoseRepresentation &pose_rep);
  
  /**
   * @brief Generate a perturbation for the IMU state
   */
  Eigen::Matrix<double, 15, 1> generateIMUPerturbation();

  void reset();

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

  /**
   * @brief Returns the simulation configuration parameters
   */
  SimConfig getSimConfig() const { return params; }

protected:
  /**
   * @brief Generates the groundtruth 3D SLAM features
   */
  void generateGroundtruthSlamFeatures();
  
  // The simulation parameters
  // This allows us to perturb the calibration parameters,
  // noise parameters, etc.
  SimConfig params;

  // The groundtruth trajectory data
  std::vector<Eigen::VectorXd> traj_data;

  // B-Spline for the IMU trajectory
  std::shared_ptr<ov_core::BsplineSE3> spline;

  // Features that we will generate for 3D SLAM
  size_t id_slam_features = 0;
  std::unordered_map<size_t, Eigen::Vector3d> slam_featmap;

  /// If our simulation is running
  bool is_running;

  // Mersenne twister PRNG for measurements
  std::mt19937 gen_meas_imu;
  std::mt19937 gen_state_init;
  std::mt19937 gen_state_perturb;
  std::mt19937 gen_slam_feat_meas;

  /// Timestamps
  /// Current timestamp of the system
  double timestamp;

  /// Last time we had an IMU reading
  double timestamp_last_imu;

  /// Last time we had an feature measurement reading
  double timestamp_last_feat;

  /// Our running IMU biases
  Eigen::Vector3d true_bias_accel = Eigen::Vector3d::Zero();
  Eigen::Vector3d true_bias_gyro = Eigen::Vector3d::Zero();

  // Our history of true biases
  bool has_skipped_first_bias = false;
  std::vector<double> hist_true_bias_time;
  std::vector<Eigen::Vector3d> hist_true_bias_accel;
  std::vector<Eigen::Vector3d> hist_true_bias_gyro;
};


/**
 * @brief This will load the trajectory into memory (space separated)
 * @param path Path to the trajectory file that we want to read in.
 * @param traj_data Will be filled with groundtruth states (timestamp(s), q_GtoI, p_IinG)
 */ 
void load_simulated_trajectory(std::string path, std::vector<Eigen::VectorXd> &traj_data);