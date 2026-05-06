cmake_minimum_required(VERSION 3.10)
project(inertial_slam_ros)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
)

set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/.." "${CMAKE_CURRENT_BINARY_DIR}/core")

catkin_package()

add_executable(inertial_slam_node src/inertial_slam_ros_node.cpp)

target_include_directories(inertial_slam_node PRIVATE ${catkin_INCLUDE_DIRS})

target_link_libraries(inertial_slam_node
  ${catkin_LIBRARIES}
  inertial_slam::inertial_slam
)

install(TARGETS inertial_slam_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)