#include <ros/ros.h>
#include <pcl/cuda/time_cpu.h>
#include <pcl/cuda/time_gpu.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/smart_ptr/shared_ptr.hpp>

#include <functional>
#include <iostream>
#include <mutex>
#include <cmath>
#include <math.h>

#define _USE_MATH_DEFINES