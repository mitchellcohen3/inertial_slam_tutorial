#include <glog/logging.h>

#include "sim/SimConfig.h"
#include "sim/Simulator.h"

#include "utils/SensorData.h"
#include "lieutils/LieDirection.h"

int main(int argc, char **argv) {
  // Initialize Google's logging library
  google::InitGoogleLogging(argv[0]);
  FLAGS_colorlogtostderr = true;
  FLAGS_alsologtostderr = true;
  FLAGS_v = 0;

  // Load command line argument
  std::string trajectory_path = "";
  if (argc > 1) {
    trajectory_path = std::string(argv[1]);
    LOG(INFO) << "Using trajectory path: " << trajectory_path;
  } else {
    LOG(ERROR) << "Please provide a trajectory path as the first argument.";
    return -1;
  }

  // Initialize the simulation
  SimConfig config;
  std::shared_ptr<Simulator> sim = std::make_shared<Simulator>(config, trajectory_path);

  // LOG(INFO) << "Simulation initialized.";
  return 0;
}