# EKF SLAM Example
This repo contains a basic inertial SLAM example using the implementation of the
EKF from OpenVINS. This code is meant for tutorial purposes, to demonstrate how
to set up an EKF for a basic problem with varying state size. This repo also
showcases the evaluation of the 

## Dependencies
This code has the following dependencies
- **CMake >= 3.10**
- **Eigen3 (>= 3.3)**
- **Boost**
- **glog** 
- **yaml-cpp**

## Installation
This code has been tested on Ubuntu 20.04 and 22.04. The required dependencies
can be installed using
```bash
sudo apt update
sudo apt install cmake libeigen3-dev libboost-all-dev libgoogle-glog-dev libyaml-cpp-dev
```

To build, clone the repo and run CMake
```bash
cd ekf_slam_example
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
