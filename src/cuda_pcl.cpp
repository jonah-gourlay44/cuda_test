#include <ros/ros.h>
#include <pcl/cuda/io/host_device.h>
#include <pcl/cuda/features/normal_3d.h>
#include <pcl/cuda/time_cpu.h>
#include <pcl/cuda/time_gpu.h>
#include <pcl/cuda/io/cloud_to_pcl.h>
#include <pcl/cuda/io/extract_indices.h>
#include <pcl/cuda/io/disparity_to_cloud.h>
#include <pcl/cuda/io/host_device.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/smart_ptr/shared_ptr.hpp>

#include <functional>
#include <iostream>
#include <mutex>


using namespace pcl::cuda;

template <typename T>
using shared_ptr = boost::shared_ptr<T>;

class TestPCL
{
public:
    TestPCL( void ) 
    {   
        cloud_sub_ = nh_.subscribe( "cloud_in", 1,
				&TestPCL::cloudCallback<Device>,
				this );
    }

    
    template <template <typename> class Storage> void
    cloudCallback( const sensor_msgs::PointCloud2ConstPtr cloud )
    {   
       
        ROS_INFO("cloud callback");

        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        
        pcl::fromROSMsg( *cloud, pcl_cloud );

        PointCloudAOS<Host> data_host;
        data_host.points.resize (pcl_cloud.points.size());
        for (std::size_t i = 0; i < pcl_cloud.points.size (); ++i)
        {
            PointXYZRGB pt;
            pt.x = pcl_cloud.points[i].x;
            pt.y = pcl_cloud.points[i].y;
            pt.z = pcl_cloud.points[i].z;
    
            data_host.points[i] = pt;
        }
        data_host.width = pcl_cloud.width;
        data_host.height = pcl_cloud.height;
        data_host.is_dense = pcl_cloud.is_dense;
        typename PointCloudAOS<Storage>::Ptr data = toStorage<Host, Storage> (data_host);

        shared_ptr<typename Storage<float4>::type> normals;
        {
            ScopeTimeCPU time ("Normal Estimation");
            float focallength = 580/2.0;
            normals = computePointNormals<Storage, typename PointIterator<Storage,PointXYZRGB>::type > (data->points.begin (), data->points.end (), focallength, data, 0.05, 30);
        }
        
        std::lock_guard<std::mutex> l(m_mutex);
        normal_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        toPCL (*data, *normals, *normal_cloud);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    std::mutex m_mutex;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normal_cloud;
    DisparityToCloud d2c;
};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "simple_cuda_node");
    ros::NodeHandle n("~");

    TestPCL test;

    ros::Rate loop_rate(1);
    while(n.ok())
    {   
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

