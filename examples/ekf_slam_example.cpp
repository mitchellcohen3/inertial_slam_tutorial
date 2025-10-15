#include <glog/logging.h>

int main(int argc, char **argv) {
  // Initialize Google's logging library
  google::InitGoogleLogging(argv[0]);
  FLAGS_colorlogtostderr = true;
  FLAGS_alsologtostderr = true;
  FLAGS_v = 0;
  LOG(INFO) << "Hello, World!";
  return 0;
}