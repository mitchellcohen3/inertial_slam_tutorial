#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CATKIN_WS="$(cd "$SCRIPT_DIR/../../.." && pwd)"

source $CATKIN_WS/devel/setup.bash

roslaunch inertial_slam_ros inertial_slam.launch \
  config_path:="$REPO_ROOT/config/slam_example_config.yaml" \
  traj_path:="$REPO_ROOT/examples/trajectories/euroc_mav/MH_01_easy.txt" \
  estimator:="${1:-ekf}" \
  realtime_factor:="${2:-1.0}"
