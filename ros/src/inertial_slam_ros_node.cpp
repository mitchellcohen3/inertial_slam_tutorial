#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <glog/logging.h>

#include "estimator/EKFSlamEstimator.h"
#include "estimator/ISAMSlamEstimator.h"
#include "estimator/SlamEstimatorBase.h"
#include "sim/SimConfig.h"
#include "sim/Simulator.h"
#include "utils/SensorData.h"
#include "utils/Utility.h"

static geometry_msgs::PoseStamped getRosPose(double t, const Eigen::Matrix3d &R,
                                           const Eigen::Vector3d &p) {
  geometry_msgs::PoseStamped ps;
  ps.header.stamp = ros::Time(t);
  ps.header.frame_id = "world";
  ps.pose.position.x = p.x();
  ps.pose.position.y = p.y();
  ps.pose.position.z = p.z();
  Eigen::Quaterniond q(R);
  q.normalize();
  ps.pose.orientation.w = q.w();
  ps.pose.orientation.x = q.x();
  ps.pose.orientation.y = q.y();
  ps.pose.orientation.z = q.z();
  return ps;
}

static sensor_msgs::PointCloud2
getRosCloud(double t, const std::vector<Eigen::Vector3d> &landmarks) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.stamp = ros::Time(t);
  cloud.header.frame_id = "world";
  cloud.height = 1;
  cloud.width = landmarks.size();
  cloud.is_dense = true;
  cloud.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(landmarks.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  for (const auto &pt : landmarks) {
    *iter_x = static_cast<float>(pt.x());
    *iter_y = static_cast<float>(pt.y());
    *iter_z = static_cast<float>(pt.z());
    ++iter_x; ++iter_y; ++iter_z;
  }
  return cloud;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ekf_slam_node");
  ros::NodeHandle nh("~");

  google::InitGoogleLogging(argv[0]);
  FLAGS_colorlogtostderr = true;
  FLAGS_alsologtostderr = true;

  std::string config_path, traj_path, estimator_type;
  double realtime_factor;
  nh.param<std::string>("config_path", config_path, "");
  nh.param<std::string>("trajectory_path", traj_path, "");
  nh.param<std::string>("estimator", estimator_type, "ekf");

  LOG(INFO) << "Config path: " << config_path;
  LOG(INFO) << "Trajectory path: " << traj_path;
  LOG(INFO) << "Estimator type: " << estimator_type;

  if (config_path.empty() || traj_path.empty()) {
    LOG(ERROR) << "Config path and trajectory path must be provided.";
    return 1;
  }

  // Setup our publsihers and TF broadcaster
  auto est_path_pub =
      nh.advertise<nav_msgs::Path>("estimated_path", 1, true);
  auto gt_path_pub =
      nh.advertise<nav_msgs::Path>("ground_truth_path", 1, /*latch=*/true);
  auto est_landmark_pub = nh.advertise<sensor_msgs::PointCloud2>(
      "estimated_landmarks", 1, /*latch=*/true);
  auto gt_landmark_pub = nh.advertise<sensor_msgs::PointCloud2>(
      "ground_truth_landmarks", 1, /*latch=*/true);
  auto pose_pub =
      nh.advertise<geometry_msgs::PoseStamped>("current_pose", 1, /*latch=*/true);

  tf2_ros::TransformBroadcaster tf_broadcaster;

  SimConfig config;
  config.load(config_path);
  config.print();

  auto sim = std::make_shared<Simulator>(config, traj_path);

  // Publish groundtruth landmarks a single time
  auto gt_landmarks = sim->getSlamFeatures();
  auto gt_lm_msg = getRosCloud(0.0, gt_landmarks);
  gt_landmark_pub.publish(gt_lm_msg);

  // Build estimator
  std::shared_ptr<SlamEstimatorBase> estimator;
  if (estimator_type == "ekf") {
    estimator = std::make_shared<EKFSlamEstimator>(EstimatorConfig{});
  } else if (estimator_type == "isam2") {
    KinematicsConfig kin;
    kin.gravity_mag = config.gravity_mag;
    kin.gravity = Eigen::Vector3d(0, 0, -config.gravity_mag);
    kin.imu_noises = config.imu_noises;
    estimator = std::make_shared<ISAMSlamEstimator>(kin);
  } else {
    ROS_ERROR_STREAM("Unknown estimator type: " << estimator_type
                                                << ". Use 'ekf' or 'isam2'.");
    return 1;
  }

  // Initialize estimator at the first IMU step (mirrors ekf_slam_example.cpp)
  double dt = 1.0 / config.sim_freq_imu;
  double next_imu_time = sim->currentTimestamp() + dt;
  double end_time = sim->currentTimestamp() + config.t_end;

  Eigen::Matrix<double, 15, 15> init_cov =
      Eigen::Matrix<double, 15, 15>::Identity() * 1e-7;
  IMUState init_state;
  sim->getState(next_imu_time, init_state);
  Eigen::Matrix<double, 5, 5> nav_state =
      Eigen::Matrix<double, 5, 5>::Identity();
  nav_state.block<3, 3>(0, 0) = init_state.attitude;
  nav_state.block<3, 1>(0, 3) = init_state.velocity;
  nav_state.block<3, 1>(0, 4) = init_state.position;
  estimator->initializeIMUState(init_state.timestamp, nav_state,
                                init_state.gyro_bias, init_state.accel_bias,
                                init_cov);

  nav_msgs::Path est_path, gt_path;
  est_path.header.frame_id = gt_path.header.frame_id = "world";

  double buffer_timefeat = -1;
  std::vector<RelativeFeatureMessage> buffer_rel_feat;

  ROS_INFO("Starting simulation loop...");
  while (sim->ok() && ros::ok()) {
    if (sim->currentTimestamp() > end_time) {
      ROS_INFO("Reached simulation end time.");
      break;
    }

    ImuMessage imu_meas;
    if (sim->getNextImu(imu_meas.timestamp, imu_meas.gyro, imu_meas.accel)) {
      estimator->inputIMU(imu_meas);
    }

    double time_feat;
    std::vector<std::pair<size_t, Eigen::Vector3d>> feat_meas;
    if (sim->getNextRelativeFeatures(time_feat, feat_meas)) {
      std::vector<RelativeFeatureMessage> rel_feat_meas;
      for (const auto &feat : feat_meas) {
        RelativeFeatureMessage msg;
        msg.timestamp = time_feat;
        msg.feature_id = feat.first;
        msg.meas = feat.second;
        const double cov_scale = config.noise_active
                                     ? config.sigma_feature_meas_3d * config.sigma_feature_meas_3d
                                     : 1e-5;
        msg.covariance = Eigen::Matrix3d::Identity() * cov_scale;
        rel_feat_meas.push_back(msg);
      }

      if (buffer_timefeat != -1) {
        estimator->inputRelativeFeatureMeasurements(buffer_rel_feat,
                                                    buffer_timefeat);
      }
      buffer_timefeat = time_feat;
      buffer_rel_feat = rel_feat_meas;

      double t = estimator->getEstimateTime();
      NavStateEstimate est = estimator->getLatestState();

      // Current estimated pose
      auto pose_msg = getRosPose(t, est.attitude, est.position);
      pose_pub.publish(pose_msg);

      // Estimated trajectory
      est_path.header.stamp = pose_msg.header.stamp;
      est_path.poses.push_back(pose_msg);
      est_path_pub.publish(est_path);

      // Ground-truth trajectory
      IMUState gt_state;
      if (sim->getState(t, gt_state)) {
        auto gt_pose = getRosPose(t, gt_state.attitude, gt_state.position);
        gt_path.header.stamp = gt_pose.header.stamp;
        gt_path.poses.push_back(gt_pose);
        gt_path_pub.publish(gt_path);
      }

      // Estimated landmarks
      auto est_landmarks = estimator->getEstimatedMap();
      if (!est_landmarks.empty()) {
        auto est_lm_msg = getRosCloud(t, est_landmarks);
        est_landmark_pub.publish(est_lm_msg);
      }

      // Publish our frame transformation
      geometry_msgs::TransformStamped tf;
      tf.header.stamp = pose_msg.header.stamp;
      tf.header.frame_id = "world";
      tf.child_frame_id = "body";
      tf.transform.translation.x = est.position.x();
      tf.transform.translation.y = est.position.y();
      tf.transform.translation.z = est.position.z();
      Eigen::Quaterniond q(est.attitude);
      q.normalize();
      tf.transform.rotation.w = q.w();
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf_broadcaster.sendTransform(tf);
    }
    // sleep for small amount of time to allow ROS callbacks to run and for RViz to update (if open)
    // ros::Duration(0.001).sleep();
    ros::spinOnce();
  }

  LOG(INFO) << "Simulation complete!";
  return 0;
}
