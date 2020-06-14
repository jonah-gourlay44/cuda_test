#include <ros/ros.h>
#include <pcl/cuda/io/host_device.h>
#include <pcl/cuda/time_cpu.h>
#include <pcl/cuda/time_gpu.h>
#include <pcl/cuda/io/cloud_to_pcl.h>
#include <pcl/cuda/io/extract_indices.h>
#include <pcl/cuda/io/disparity_to_cloud.h>
#include <pcl/cuda/io/host_device.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/cuda/filters/voxel_grid.h>
#include <pcl/cuda/io/extract_indices.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/smart_ptr/shared_ptr.hpp>

#include <functional>
#include <iostream>
#include <mutex>
#include <cmath>
#include <math.h>

#define _USE_MATH_DEFINES
