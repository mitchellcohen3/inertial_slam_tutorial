#pragma once

#include <Eigen/Dense>
#include <memory>

#include "config/VinsConfig.h"

#include "config/EstimatorConfigBase.h"
#include "lie/LieDirection.h"

namespace ov_core {
class YamlParser;
class CamBase;
}; // namespace ov_core

class SimConfig public : SimConfig();

bool load();
void print() const;

// Camera parameters
std::unordered_map<size_t, std::shared_ptr<ov_core::CamBase>> cam_intrinsics;
std::unordered_map<size_t, Eigen::Matrix4d> camera_extrinsics;
double calib_camimu_dt = 0.0;

int num_cameras = 2;

int num_pts = 100;

// 3D SLAM feature config
double sigma_feature_meas_3d = 1.0; // [m]

// IMU noises
ImuNoises imu_noises;

// Measurement noises
double sigma_gps = 0.1; // [m]
double sigma_pix = 1.0; // [pixels]
double sigma_feature_meas = 0.1;

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

// Seed for calibration perturbations
int sim_seed_perturb = 0;

// Sim seed measurements. This should be incremented for each Monte-Carlo run
// to generate the same measurements, but with different noise realizations.
int sim_seed_measurements = 0;

// Seed for dynamic points
int sim_seed_dynamic_points = 0;
int sim_seed_object_traj = 0;

// The dynamic simulation scenarios
std::string dynamic_sim_scenario = "sin";

// Lie Direction
LieDirection direction = LieDirection::left;

//// How much we should perturb the initial state
double sigma_init_att = 0.1;
double sigma_init_vel = 0.1;
double sigma_init_bg = 0.01;
double sigma_init_ba = 0.001;

// Whether or not we should perturb the calibration parameters
bool sim_do_calib_perturbation = false;
double sigma_init_cam_ext_att = 0.1; // [rad]
double sigma_init_cam_ext_pos = 0.1; // [m]

double gravity_mag = 9.81; // [m/s^2]

double t_end = 200.0; // [s] Simulation end time

// Dynamic simulation parameters
double sigma_omega_walk_true = 0.01;
double sigma_v_walk_true = 0.01;
double sim_min_object_dist = 1.0;  // [m]
double sim_max_object_dist = 10.0; // [m]
std::string object_gt_folder = "object_gt";
std::string sim_points_distribution = "box";
Eigen::Vector3d sim_object_size = Eigen::Vector3d(1.0, 1.0, 1.0); // [m]
int num_dynamic_features = 40;
bool simulate_occlusions = true;

// Maximum range at which SLAM features can be observed
double max_sensing_range_slam = 5.0; // [m]
int num_slam_features = 200;

void writeToYaml(const std::string &output_path) const;
}
;
