cmake_minimum_required(VERSION 3.10)
project(inertial_slam_example)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(sources
    inertial_slam_example.cpp
)

# Create executable
add_executable(inertial_slam_example ${sources})
target_include_directories(inertial_slam_example PRIVATE include)

# Link the example to the main library and dependancies
target_link_libraries(inertial_slam_example inertial_slam ${Boost_LIBRARIES} ${GLOG_LIBRARIES})