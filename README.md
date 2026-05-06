# Inertial SLAM Tutorial
This repo is intended for tutorial purposes and showcases a basic inertial SLAM example, where a robot moves in 3D space collecting gyroscope and accelerometer measurements, as well as measurements to unknown landmarks in the body frame. This repo contains implementations of two commonly used methods: the Extended Kalman Filter, and iSAM2, part of the GTSAM library. The implementation of the EKF is based on the OpenVINS library, and the solution using iSAM2 utilizes IMU preintegration. 

This repo contains the C++ library code for the EKF and iSAM2 implementations, as well as a ROS package that allows for visualization of the estimated trajectory and landmarks in RViz. 

## Dependencies
This code has the following dependencies and has been tested on Ubuntu 20.04:
- **CMake >= 3.10**
- **Eigen3 (>= 3.3)**
- **Boost**
- **glog** 
- **yaml-cpp**

The required dependencies can be installed using
```bash
sudo apt update
sudo apt install cmake libeigen3-dev libboost-all-dev libgoogle-glog-dev libyaml-cpp-dev
```

Additionally, the iSAM2 implementation requires GTSAM (tested with GTSAM 4.2), which can be installed from source by following the instructions [here](https://github.com/borglab/gtsam).

## Building with ROS
To build with ROS, clone the repo into a catkin ws and build the ROS package using
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone git@github.com:mitchellcohen3/inertial_slam_tutorial.git
cd ..
catkin build
source devel/setup.bash
```

The example can then be run using
```bash
roslaunch inertial_slam_tutorial ekf_slam_example.launch
```

## Building without ROS
To build without ros, clone the repo and build the library using CMake as follows:
```bash
git clone git@github.com:mitchellcohen3/inertial_slam_tutorial.git
cd inertial_slam_tutorial
mkdir build && cd build
cmake ..
make -j4
```

## Running the example
The main C++ example simulates a robot moving through 3D space along with noisy
IMU and landmark measurements, and outputs state estimates and covariances. The
example can either be run standalone or through the provided Python script,
which will also evaluate and plot the results. To run the example through
Python, first install the required Python dependencies and run the script as
```bash
pip install numpy matplotlib scipy
cd examples
python run_ekf_slam_example.py
```

## References
This repo incorporates code from
**[OpenVINS](https://github.com/rpng/open_vins)**, where the EKF implementation
is and B-Spline simulator are derived from. 

## TODO
- [ ] Add derivation of process and measurement model Jacobians
