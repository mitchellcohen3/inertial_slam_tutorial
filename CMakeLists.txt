cmake_minimum_required(VERSION 3.10)
project(ekf_slam)

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
pkg_check_modules(GLOG REQUIRED libglog)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
)

list(
    APPEND lib_sources
    src/ekf/EKFState.cpp
    src/ekf/EKFStateHelper.cpp
    src/sim/BsplineSE3.cpp
)

# Declare the static library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_library(${PROJECT_NAME} STATIC ${lib_sources})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# Link libraries
add_subdirectory(examples)

# Install targets
install(TARGETS ${PROJECT_NAME}
    EXPORT ekf_slam_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)