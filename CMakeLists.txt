cmake_minimum_required(VERSION 2.8.3)
project(cuda_test)

set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-8.0)
set(PCL_DIR /opt/pcl_1_8_1/share/pcl-1.8)
set(PCL_ROOT /opt/pcl_1_8_1)
set(CMAKE_CUDA_COMPILER /usr/local/cuda-8.0/bin/nvcc)

add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O2")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
  sensor_msgs
  pyrosmsg
)

find_package(CUDA 8.0 REQUIRED)
find_package(PCL 1.8.1 REQUIRED COMPONENTS
  cuda_filters
  cuda_io
  cuda_segmentations
  cuda_features
  cuda_sample_consensus
  cuda_common
  filters
  io
  common
)

include(FindCUDA)

catkin_package(
#  INCLUDE_DIRS include 
#  LIBRARIES cuda_test
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

link_directories( 
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

cuda_add_executable(pcl_cuda_node src/cuda_pcl.cpp)
cuda_add_executable(voxel_grid_cuda_node src/voxel_grid_cuda.cpp)
add_executable(voxel_grid_node src/voxel_grid.cpp)

target_link_libraries(pcl_cuda_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(voxel_grid_cuda_node 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(voxel_grid_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
