cmake_minimum_required(VERSION 3.10)
project(inertial_slam)

SET(GCC_COVERAGE_LINK_FLAGS "-lstdc++fs")
set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS: "Build type: " ${CMAKE_BUILD_TYPE})

# Find Dependancies
find_package(Catch2 3 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time program_options) # for timing
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLOG REQUIRED IMPORTED_TARGET libglog)
find_package(yaml-cpp REQUIRED)
find_package(GTSAM REQUIRED)
message(STATUS "Found GTSAM version: ${GTSAM_VERSION}")

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
)

list(
    APPEND lib_sources
    src/ekf/EKFState.cpp
    src/ekf/EKFStateHelper.cpp
    src/sim/BsplineSE3.cpp
    src/sim/Simulator.cpp
    src/sim/SimConfig.cpp

    src/lieutils/SO3.cpp
    src/lieutils/SE3.cpp
    src/lieutils/SE23.cpp

    src/estimator/EKFSlamEstimator.cpp
    src/estimator/ISAMSlamEstimator.cpp
    src/estimator/ImuPropagator.cpp

    src/utils/Utility.cpp
    src/utils/FileAccess.cpp
)

# Declare the static library (no GTSAM dependency)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_library(${PROJECT_NAME} STATIC ${lib_sources})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link libraries
target_link_libraries(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen
    ${Boost_LIBRARIES}
    yaml-cpp
    gtsam
    PkgConfig::GLOG
)

option(BUILD_EXAMPLES "Build examples" ON)
if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

# Install targets
install(TARGETS ${PROJECT_NAME}
    EXPORT inertial_slam_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)