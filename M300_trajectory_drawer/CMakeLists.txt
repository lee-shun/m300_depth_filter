cmake_minimum_required(VERSION 2.8)
project(trajectory_drawer)

set(CMAKE_CXX_FLAGS_DEBUG
    "$ENV{CXXFLAGS} -std=c++11 -O0 -Wall -ggdb -fopenmp -pthread")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++11 -O3 -fopenmp -pthread")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)

# Eigen
include_directories("/usr/include/eigen3")

# yaml
find_package(yaml-cpp REQUIRED)

set(THIRD_PARTY_LIBS
    yaml-cpp)

add_executable(local_pose_offset local_pose_offset.cpp)
