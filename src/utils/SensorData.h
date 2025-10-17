#pragma once

#include "quadrics/Box2D.h"
#include "quadrics/DualConic.h"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

struct ImuMessage {

  double timestamp;
  Eigen::Vector3d gyro;
  Eigen::Vector3d accel;

  ImuMessage(double timestamp, Eigen::Vector3d gyro, Eigen::Vector3d accel)
      : timestamp(timestamp), gyro(gyro), accel(accel) {}

  ImuMessage() {
    timestamp = 0.0;
    gyro = Eigen::Vector3d::Zero();
    accel = Eigen::Vector3d::Zero();
  }

  // Sort function to allow sorting of STL containers
  bool operator<(const ImuMessage &other) const {
    return timestamp < other.timestamp;
  }
};

struct GpsMessage {
  double timestamp;
  Eigen::Vector3d meas;
  Eigen::Matrix3d covariance;

  GpsMessage() {
    timestamp = 0.0;
    meas = Eigen::Vector3d::Zero();
    covariance = Eigen::Matrix3d::Zero();
  }

  GpsMessage(double timestamp, const Eigen::Vector3d &meas,
             const Eigen::Matrix3d &cov)
      : timestamp(timestamp), meas(meas), covariance(cov) {}

  // Sort function to allow sorting of STL containers
  bool operator<(const GpsMessage &other) const {
    return timestamp < other.timestamp;
  }
};

struct BoundingBoxMessage {
  // Class name
  std::string class_name;

  double score;

  // If we additionally have a conic measurement
  bool has_conic = false;
  DualConic dual_conic;

  // Timestamp of reading
  double timestamp;

  // Bounding box data
  Box2D bbox;

  // Camera IDs of each of the images collected
  std::vector<int> sensor_ids;

  // Unique quadric ID of this measurement
  // NOTE: This assumes that the data association problem is solved
  size_t feature_id;

  // Measurement covariance
  Eigen::Matrix4d covariance;
  double sigma;
};

// class Detection {
// public:
//   // Class name
//   std::string class_name;
//   // Detection score
//   double score;
//   // Bounding box measurement
//   Box2D bbox;

//   // If we have a conic measurement
//   bool has_conic = false;
//   DualConic dual_conic;

//   // Measurement covariance
//   Eigen::Matrix4d covariance;

//   double stamp;
// };

struct RelativeFeatureMessage {
  // Timestamp of reading
  double timestamp;

  // Relative position measurement
  Eigen::Vector3d meas;

  // Unique ID of this measurement
  size_t feature_id;

  // Measurement covariance
  Eigen::Matrix3d covariance;

  RelativeFeatureMessage() {
    timestamp = 0.0;
    meas = Eigen::Vector3d::Zero();
    feature_id = 0;
    covariance = Eigen::Matrix3d::Zero();
  }

  RelativeFeatureMessage(double timestamp, const Eigen::Vector3d &meas,
                         size_t feature_id, const Eigen::Matrix3d &cov)
      : timestamp(timestamp), meas(meas), feature_id(feature_id),
        covariance(cov) {}
};

/**
 *
 */
struct FeatureMessage {
  double timestamp;

  // ID of the camera that this feature measurement was taken from
  size_t cam_id;

  // UV coordinates for this feature message
  Eigen::VectorXf uv;

  // Normalized UV coordinates
  Eigen::VectorXf uv_norm;

  // Unique ID of this measurement
  size_t feature_id;
};

struct DynamicFeatureMessage {

  DynamicFeatureMessage() {
    timestamp = 0.0;
    cam_id = 0;
    r_pc_c = Eigen::Vector3d::Zero();
    uv = Eigen::VectorXf::Zero(2);
    uv_norm = Eigen::VectorXf::Zero(2);
    feature_id = 0;
    object_id = 0;
  }

  DynamicFeatureMessage(double timestamp_, int cam_id_,
                        const Eigen::Vector3d &position_,
                        const Eigen::VectorXf &uv_,
                        const Eigen::VectorXf &uv_norm_, size_t feature_id_,
                        size_t object_id_)
      : timestamp(timestamp_), cam_id(cam_id_), r_pc_c(position_), uv(uv_),
        uv_norm(uv_norm_), feature_id(feature_id_), object_id(object_id_) {}

  // Timestamp of message
  double timestamp;

  // ID of the camera that this feature message was taken from
  int cam_id;

  // Feature position resolved in the camera frame.
  // This would be, for example, if we had depth measurements
  // from RGB-D for each feature as well.
  Eigen::Vector3d r_pc_c;

  // UV coordinates for this feature message
  Eigen::VectorXf uv;

  // Normalized UV coordinates
  Eigen::VectorXf uv_norm;

  // Unique ID of this measurement
  size_t feature_id;

  // Unique ID of the object
  size_t object_id;
};

struct SimulatedCameraMessage {
  //  Timestamp
  double timestamp;
  // Vector of camera IDs
  std::vector<int> camids;
  // Vector of features, each feature is a vector of pairs of size_t and
  // Eigen::VectorXf;
  std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> feats;
};

/**
 * @brief A message that contains simulated visual measurements to a single
 * object with a particular ID.
 */
struct SimulatedObjectMessage {

  using RelativeFeatureMeasurements =
      std::vector<std::pair<size_t, Eigen::Vector3d>>;
  using RelativeFeatureMeasurementsByCamera =
      std::vector<RelativeFeatureMeasurements>;

  SimulatedObjectMessage(
      double timestamp_, const std::vector<int> &camids_,
      const std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>>
          &feats_,
      const RelativeFeatureMeasurementsByCamera &feats_3d_, size_t object_id_)
      : timestamp(timestamp_), camids(camids_), feats(feats_),
        feats_3d(feats_3d_), object_id(object_id_) {}

  // Timestamp
  double timestamp;
  // Vector of camera IDs
  std::vector<int> camids;
  // Vector of features, where each feature is a vector of pairs of size_t and
  // Eigen::VectorXf. The size_t in the pair is the feature ID.
  std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> feats;

  // 3D features, where each feature is a pair of size_t and Eigen::Vector3d.
  RelativeFeatureMeasurementsByCamera feats_3d;

  // Object ID of these measurements
  size_t object_id;
};

struct CameraMessage {

  CameraMessage() { boxes = {}; }
  // Timestamp of the reading
  double timestamp;

  // Camera IDs for each of the images collected
  std::vector<int> sensor_ids;

  // Raw image we have collected for each camera
  std::vector<cv::Mat> images;

  /// Tracking masks for each camera we have
  std::vector<cv::Mat> masks;

  // Bounding boxes for each camera
  std::vector<std::vector<BoundingBoxMessage>> boxes;

  cv::Mat stereo_optical_flow;   // left_frame to right_frame
  cv::Mat temporal_optical_flow; // prev_left_frame to left_frame

  // / Sort function to allow for using of STL containers
  bool operator<(const CameraMessage &other) const {
    if (timestamp == other.timestamp) {
      int id = *std::min_element(sensor_ids.begin(), sensor_ids.end());
      int id_other =
          *std::min_element(other.sensor_ids.begin(), other.sensor_ids.end());
      return id < id_other;
    } else {
      return timestamp < other.timestamp;
    }
  }
};

struct RelativePoseMessage {
  Eigen::Matrix4d value;
  Eigen::Matrix<double, 6, 6> covariance;
  std::vector<double> stamps;
};

struct RelativeObjectPoseMessage {
  double timestamp;
  Eigen::Matrix4d value;
  size_t object_id;
  Eigen::Matrix<double, 6, 6> covariance;

  RelativeObjectPoseMessage(double timestamp_, const Eigen::Matrix4d &value_,
                            size_t object_id_,
                            const Eigen::Matrix<double, 6, 6> &covariance_)
      : timestamp(timestamp_), value(value_), object_id(object_id_),
        covariance(covariance_) {}
};

struct ObjectPointMessage {
  double timestamp;
  Eigen::Vector3d value; // r_oz_b, position of the object in the body frame
  size_t object_id;
  Eigen::Matrix3d covariance;
  size_t point_id; // ID of the point

  ObjectPointMessage(double timestamp_, const Eigen::Vector3d &value_,
                     size_t object_id_, const Eigen::Matrix3d &covariance_,
                     size_t point_id_)
      : timestamp(timestamp_), value(value_), object_id(object_id_),
        covariance(covariance_), point_id(point_id_) {}
};