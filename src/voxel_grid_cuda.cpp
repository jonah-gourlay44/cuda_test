#include <cuda_test/voxel_grid_cuda.h>

using namespace pcl::cuda;

template <typename T>
using shared_ptr = boost::shared_ptr<T>;

template <template <typename> class Storage>
using PointCloud = PointCloudAOS<Storage>;

template <template <typename> class Storage>
class VoxelGridCuda
{
public:
    VoxelGridCuda( void ) :
        nh_private_("~")
    {   
        cloud_sub_ = nh_.subscribe( "cloud_in", 1,
				&VoxelGridCuda::cloudCallback,
				this );

        pub_voxel_filt_ = nh_private_.advertise<sensor_msgs::PointCloud2>( "voxel_grid_filter", 1 );

        voxel_filter_.setLeafSize ( 0.5, 0.5, 0.5 );
    }

    void
    cloudCallback( const sensor_msgs::PointCloud2ConstPtr cloud )
    {   
        ScopeTimeCPU time_total ("Callback");
        ROS_INFO("\ncloud callback");
        ros::Time start = ros::Time::now();

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

        PointCloudAOS<Host> filter_host;
        typename PointCloudAOS<Storage>::Ptr filter_cloud = toStorage<Host, Storage> (filter_host);
        voxel_filter_.setInputCloud ( data );

        {
            ScopeTimeCPU filter_time ("filtering");
            voxel_filter_.filter ( filter_cloud ); 
        }
        
        std::lock_guard<std::mutex> l(m_mutex);
        out_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
        toPCL (*filter_cloud, *out_cloud);

        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg( *out_cloud, ros_cloud );
        ros_cloud.header.frame_id = cloud->header.frame_id;
        pub_voxel_filt_.publish( ros_cloud );
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber cloud_sub_;
    ros::Publisher pub_voxel_filt_;
    std::mutex m_mutex;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud;
    VoxelGrid<Storage> voxel_filter_;

};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "cuda_voxel_grid_node");
    ros::NodeHandle n("~");

    VoxelGridCuda<Device> test;

    ros::Rate loop_rate(1);
    while(n.ok())
    {   
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}