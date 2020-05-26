#include "cuda_test/voxel_grid_include.h"

using namespace pcl;

template <typename T>
using shared_ptr = boost::shared_ptr<T>;

class VoxelGridCPU
{
public:
    VoxelGridCPU( void ) :
        nh_private_("~")
    {   
        cloud_sub_ = nh_.subscribe( "cloud_in", 1,
				&VoxelGridCPU::cloudCallback,
				this );

        pub_voxel_filt_ = nh_private_.advertise<sensor_msgs::PointCloud2>( "voxel_grid_filter", 1 );

        voxel_filter_.setLeafSize ( 0.5, 0.5, 0.5 );
    }

    void
    cloudCallback( const sensor_msgs::PointCloud2ConstPtr cloud )
    {   
        pcl::cuda::ScopeTimeCPU time_total ("Callback");

        PointCloud<PointXYZ> pcl_cloud;
        PointCloud<PointXYZ> out_cloud;
        
        fromROSMsg( *cloud, pcl_cloud );

        voxel_filter_.setInputCloud ( pcl_cloud.makeShared() );

        {
            pcl::cuda::ScopeTimeCPU filter_time ("filtering");
            voxel_filter_.filter ( out_cloud ); 
        }
        
        std::lock_guard<std::mutex> l(m_mutex);

        sensor_msgs::PointCloud2 ros_cloud;
        toROSMsg( out_cloud, ros_cloud );
        ros_cloud.header.frame_id = cloud->header.frame_id;
        pub_voxel_filt_.publish( ros_cloud );
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber cloud_sub_;
    ros::Publisher pub_voxel_filt_;
    std::mutex m_mutex;

    PointCloud<PointXYZ> out_cloud;
    VoxelGrid<PointXYZ> voxel_filter_;

};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "cuda_voxel_grid_node");
    ros::NodeHandle n("~");

    VoxelGridCPU test;

    ros::Rate loop_rate(1);
    while(n.ok())
    {   
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}