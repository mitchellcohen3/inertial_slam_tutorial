#include "ImuPropagator.h"
#include "lieutils/SE23.h"
#include "lieutils/SO3.h"

#include "utils/Utils.h"
#include "utils/colors.h"
#include "utils/utility.h"

#include "imu/IMUHelper.h"

#include <glog/logging.h>
#include <iostream>
#include <string.h>

void discretizeSystem(const Eigen::MatrixXd &A_ct, const Eigen::MatrixXd &L_ct,
                      const Eigen::MatrixXd &Q_ct, double dt,
                      Eigen::MatrixXd &A_d, Eigen::MatrixXd &Q_d) {
  // Compute discrete-time A matrix
  Eigen::MatrixXd A_dt = A_ct * dt;
  Eigen::MatrixXd A_dt_square = A_dt * A_dt;
  Eigen::MatrixXd A_dt_cube = A_dt_square * A_dt;

  A_d = Eigen::MatrixXd::Identity(A_ct.rows(), A_ct.cols()) + A_dt +
        0.5 * A_dt_square + (1.0 / 6.0) * A_dt_cube;

  // Compute the discrete-time noise covariance
  // if (method == ceres_nav::DiscretizationMethod::TaylorSeries) {
  Eigen::Matrix<double, 15, 15> Q = L_ct * Q_ct * L_ct.transpose();
  Eigen::Matrix<double, 15, 15> first_term = Q * dt;
  Eigen::Matrix<double, 15, 15> second_term =
      (A_ct * Q + Q * A_ct.transpose()) * (dt * dt) / 2.0;
  Eigen::Matrix<double, 15, 15> third_term =
      (A_ct * A_ct * Q + 2.0 * A_ct * Q * A_ct.transpose() +
       Q * A_ct.transpose() * A_ct.transpose()) *
      (dt * dt * dt) / 6.0;
  Q_d = first_term + second_term + third_term;
  Q_d = 0.5 * (Q_d + Q_d.transpose());
}

void getUnbiasedImu(const ImuMessage &input,
                    const std::shared_ptr<IMUType> &state,
                    Eigen::Vector3d &unbiased_gyro,
                    Eigen::Vector3d &unbiased_accel) {
  // Extract the
  Eigen::Vector3d gyro_meas = input.gyro;
  Eigen::Vector3d accel_meas = input.accel;

  // Get the unbiased IMU measurement
  unbiased_gyro = gyro_meas - state->gyroBias();
  unbiased_accel = accel_meas - state->accelBias();
}

/*
 * Propagates the state forward one timestamp.
 */
void ImuPropagator::predict(std::shared_ptr<IMUType> state,
                            const Eigen::Vector3d &gyro,
                            const Eigen::Vector3d &accel, double dt) {
  Eigen::Vector3d unbiased_gyro = gyro - state->gyroBias();
  Eigen::Vector3d unbiased_accel = accel - state->accelBias();

  Eigen::Matrix<double, 5, 5> G = createGMatrix(config.gravity, dt);
  Eigen::Matrix<double, 5, 5> U =
      createUMatrix(unbiased_gyro, unbiased_accel, dt);

  Eigen::Matrix<double, 5, 5> prev_extended_pose = state->navState();
  Eigen::Matrix<double, 5, 5> next_extended_pose = G * prev_extended_pose * U;

  double new_stamp = state->stamp() + dt;
  state->setStamp(new_stamp);
  // Update the value for the extended pose
  state->setNavState(next_extended_pose);
}

std::vector<ImuMessage>
ImuPropagator::getIMUBetweenTimes(const std::vector<ImuMessage> &imu_data,
                                  const double &time0, const double &time1) {
  std::vector<ImuMessage> selected_data;
  for (size_t i = 0; i < imu_data.size(); i++) {
    if (imu_data.at(i).timestamp >= time0 && imu_data.at(i).timestamp < time1) {
      selected_data.push_back(imu_data.at(i));
    }
  }
  return selected_data;
}

std::vector<ImuMessage>
ImuPropagator::selectIMUData(const std::vector<ImuMessage> &imu_data,
                             const double &time0, const double &time1,
                             bool warn) {
  std::vector<ImuMessage> prop_data;

  // Ensure that we have enough measurements
  if (imu_data.empty()) {
    if (warn) {
      LOG(WARNING) << "No IMU measurements to propagate with!";
    }
    return prop_data;
  }

  // Loop through and find all the needed measurements to propagate with
  for (size_t i = 0; i < imu_data.size() - 1; i++) {
    // START OF INTEGARTION PERIOD
    // If the next timestamp is greater than our current state timestamp
    // And the current is not greater than it yet...
    // Then we should ``split'' our current IMU measurement
    if (imu_data.at(i + 1).timestamp > time0 &&
        imu_data.at(i).timestamp < time0) {
      ImuMessage data =
          interpolateIMUData(imu_data.at(i), imu_data.at(i + 1), time0);
      prop_data.push_back(data);
      continue;
    }

    // MIDDLE OF INTEGRAION PERIOD
    // If our imu measurement is right in the middle of our propagation period
    // Then add it
    if (imu_data.at(i).timestamp >= time0 &&
        imu_data.at(i + 1).timestamp <= time1) {
      prop_data.push_back(imu_data.at(i));
      continue;
    }

    // END OF THE INTEGRATION PERIOD
    // If the current timestamp is greater then our update time
    // We should just "split" the NEXT IMU measurement to the update time,
    // NOTE: we add the current time, and then the time at the end of the
    // interval (so we can get a dt) NOTE: we also break out of this loop, as
    // this is the last IMU measurement we need!
    if (imu_data.at(i + 1).timestamp > time1) {
      // If we have a very low frequency IMU then, we could have only recorded
      // the first integration (i.e. case 1) and nothing else In this case,
      // both the current IMU measurement and the next is greater than the
      // desired intepolation, thus we should just cut the current at the
      // desired time Else, we have hit CASE2 and this IMU measurement is not
      // past the desired propagation time, thus add the whole IMU reading
      if (imu_data.at(i).timestamp > time1 && i == 0) {
        // This case can happen if we don't have any imu data that has occured
        // before the startup time This means that either we have dropped IMU
        // data, or we have not gotten enough. In this case we can't propgate
        // forward in time, so there is not that much we can do.
        break;
      } else if (imu_data.at(i).timestamp > time1) {
        ImuMessage data =
            interpolateIMUData(imu_data.at(i - 1), imu_data.at(i), time1);
        prop_data.push_back(data);
        // PRINT_DEBUG("propagation #%d = CASE 3.1 = %.3f => %.3f\n", (int)i,
        //             imu_data.at(i).timestamp - prop_data.at(0).timestamp,
        //             imu_data.at(i).timestamp - time0);
      } else {
        prop_data.push_back(imu_data.at(i));
        // PRINT_DEBUG("propagation #%d = CASE 3.2 = %.3f => %.3f\n", (int)i,
        //             imu_data.at(i).timestamp - prop_data.at(0).timestamp,
        //             imu_data.at(i).timestamp - time0);
      }
      // If the added IMU message doesn't end exactly at the camera time
      // Then we need to add another one that is right at the ending time
      if (prop_data.at(prop_data.size() - 1).timestamp != time1) {
        ImuMessage data =
            interpolateIMUData(imu_data.at(i), imu_data.at(i + 1), time1);
        prop_data.push_back(data);
        // PRINT_DEBUG("propagation #%d = CASE 3.3 = %.3f => %.3f\n", (int)i,
        //             data.timestamp - prop_data.at(0).timestamp,
        //             data.timestamp - time0);
      }
      break;
    }
  }

  // Check that we have at least one measurement to propagate with
  if (prop_data.empty()) {
    if (warn) {
      LOG(WARNING) << "No IMU measurements to propagate with!";
    }
    return prop_data;
  }

  // If we did not reach the whole integration period
  // (i.e., the last inertial measurement we have is smaller then the time we
  // want to reach) Then we should just "stretch" the last measurement to be
  // the whole period
  // TODO: this really isn't that good of logic, we should fix this so the
  // above logic is exact!
  if (prop_data.at(prop_data.size() - 1).timestamp != time1) {
    if (warn) {
      LOG(WARNING) << "Missing inertial measurements to propagate with ("
                   << (time1 - imu_data.at(imu_data.size() - 1).timestamp)
                   << " sec missing)!";
    }
    ImuMessage data =
        interpolateIMUData(imu_data.at(imu_data.size() - 2),
                           imu_data.at(imu_data.size() - 1), time1);
    prop_data.push_back(data);
    // PRINT_DEBUG("propagation #%d = CASE 3.4 = %.3f => %.3f\n",
    // (int)(imu_data.size() - 2), data.timestamp - prop_data.at(0).timestamp,
    // data.timestamp - time0);
  }

  // Loop through and ensure we do not have any zero dt values
  // This would cause the noise covariance to be Infinity
  // TODO: we should actually fix this by properly implementing this function
  // and doing unit tests on it...
  for (size_t i = 0; i < prop_data.size() - 1; i++) {
    if (std::abs(prop_data.at(i + 1).timestamp - prop_data.at(i).timestamp) <
        1e-12) {
      if (warn) {
        LOG(WARNING) << "Zero Dt between IMU readings "
                     << prop_data.at(i).timestamp << " and "
                     << prop_data.at(i + 1).timestamp << "!";
      }
      prop_data.erase(prop_data.begin() + i);
      i--;
    }
  }

  // Check that we have at least one measurement to propagate with
  if (prop_data.size() < 2) {
    if (warn) {
      LOG(WARNING) << "No IMU measurements to propagate with!";
    }
    return prop_data;
  }

  // Success :D
  return prop_data;
}

ImuMessage ImuPropagator::interpolateIMUData(const ImuMessage &imu_data1,
                                             const ImuMessage &imu_data2,
                                             const double &time) {
  double t1 = imu_data1.timestamp;
  double t2 = imu_data2.timestamp;
  double alpha = (time - t1) / (t2 - t1);
  Eigen::Vector3d gyro = (1 - alpha) * imu_data1.gyro + alpha * imu_data2.gyro;
  Eigen::Vector3d accel =
      (1 - alpha) * imu_data1.accel + alpha * imu_data2.accel;
  ImuMessage interpolated_imu(time, gyro, accel);
  return interpolated_imu;
}

void ImuPropagator::predictWithJacobians(std::shared_ptr<IMUType> state,
                                         const Eigen::Vector3d &gyro,
                                         const Eigen::Vector3d &accel,
                                         double dt,
                                         Eigen::Matrix<double, 15, 15> &Ad,
                                         Eigen::Matrix<double, 15, 15> &Qd) {
  // Compute the Jacobians
  Eigen::Matrix<double, 15, 15> A_ct = Eigen::Matrix<double, 15, 15>::Zero();
  Eigen::Matrix<double, 15, 12> L_ct = Eigen::Matrix<double, 15, 12>::Zero();

  Eigen::Matrix3d C = state->attitude();
  Eigen::Vector3d v = state->velocity();
  Eigen::Vector3d r = state->position();
  Eigen::Vector3d g_a = config.gravity;

  if (state->direction() == LieDirection::left) {
    A_ct.block<3, 3>(0, 9) = -C;
    A_ct.block<3, 3>(3, 0) = SO3::cross(g_a);
    A_ct.block<3, 3>(3, 9) = -SO3::cross(v) * C;
    A_ct.block<3, 3>(3, 12) = -C;
    A_ct.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity();
    A_ct.block<3, 3>(6, 9) = -SO3::cross(r) * C;

    L_ct.block<3, 3>(0, 0) = C;
    L_ct.block<3, 3>(3, 0) = SO3::cross(v) * C;
    L_ct.block<3, 3>(3, 3) = C;
    L_ct.block<3, 3>(6, 3) = SO3::cross(r) * C;
    L_ct.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();
    L_ct.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();
  } else if (state->direction() == LieDirection::right) {
    LOG(ERROR) << "Right Jacobians not implemented yet!";
  }

  // Discretize the continuous-time Jacobians
  Eigen::Matrix<double, 12, 12> Q_ct = config.imu_noises.Q_ct;

  Eigen::MatrixXd Ad_tmp, Qd_tmp;
  discretizeSystem(A_ct, L_ct, Q_ct, dt, Ad_tmp, Qd_tmp);
  Ad = Ad_tmp;
  Qd = Qd_tmp;

  // Propagate the state forward
  predict(state, gyro, accel, dt);
}

void ImuPropagator::propagate(std::shared_ptr<IMUType> state,
                              double timestamp) {
  // Crash if the timestmap if the same as the state
  if (isDoubleWithinTolerance(state->stamp(), timestamp, 1e-12)) {
    LOG(ERROR) << "ImuPropagator::propagateImuState(): The timestamp of the "
                  "state is the same as the timestamp to propagate to!";
    std::exit(EXIT_FAILURE);
  }

  // Crash if we are trying to propagate backwards
  if (state->stamp() > timestamp) {
    LOG(ERROR) << "ImuPropagator::propagateImuState(): The timestamp of the "
                  "state is greater than the timestamp to propagate to!";
    std::exit(EXIT_FAILURE);
  }

  // Construct a vector of IMU messaages to use
  double time0 = state->stamp();
  double time1 = timestamp;
  std::vector<ImuMessage> prop_data;
  prop_data = selectIMUData(imu_data, time0, time1, true);

  // Propagate IMU data
  if (prop_data.size() > 1) {
    for (size_t i = 0; i < prop_data.size() - 1; i++) {
      double dt = prop_data.at(i + 1).timestamp - prop_data.at(i).timestamp;

      Eigen::Vector3d gyro_avg, acc_avg;
      if (config.average_meas) {
        gyro_avg = 0.5 * (prop_data.at(i).gyro + prop_data.at(i + 1).gyro);
        acc_avg = 0.5 * (prop_data.at(i).accel + prop_data.at(i + 1).accel);
      } else {
        gyro_avg = prop_data.at(i).gyro;
        acc_avg = prop_data.at(i).accel;
      }

      // Propagate state
      predict(state, gyro_avg, acc_avg, dt);
      // Propagate IMU increment
      imu_increment->propagate(dt, gyro_avg, acc_avg);
    }
  }
}