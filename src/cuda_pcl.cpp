#include <cuda_test/pcl_cuda_include.h>

using namespace pcl::cuda;

template <typename T>
using shared_ptr = boost::shared_ptr<T>;

template <template <typename> class Storage>
class TestPCL
{
public:
    TestPCL( void ) 
    {   
        cloud_sub_ = nh_.subscribe( "cloud_in", 1,
				&TestPCL::cloudCallback,
				this );

        voxel_filter_.setLeafSize( 0.1, 0.1, 0.1 );
    }

    void
    cloudCallback( const sensor_msgs::PointCloud2ConstPtr cloud )
    {   
        ScopeTimeCPU time_total ("Callback");
        ROS_INFO("\ncloud callback");
        ros::Time start = ros::Time::now();

        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        
        pcl::fromROSMsg( *cloud, pcl_cloud );

        pcl::PointCloud<pcl::PointXYZ> pcl_cloud_voxel;
        voxel_filter_.setInputCloud( pcl_cloud.makeShared() );
        voxel_filter_.filter( pcl_cloud_voxel );

        PointCloudAOS<Host> data_host;
        data_host.points.resize (pcl_cloud_voxel.points.size());
        for (std::size_t i = 0; i < pcl_cloud_voxel.points.size (); ++i)
        {
            PointXYZRGB pt;
            pt.x = pcl_cloud_voxel.points[i].x;
            pt.y = pcl_cloud_voxel.points[i].y;
            pt.z = pcl_cloud_voxel.points[i].z;
            
            data_host.points[i] = pt;
        }
        data_host.width = pcl_cloud.width;
        data_host.height = pcl_cloud.height;
        data_host.is_dense = pcl_cloud.is_dense;
        typename PointCloudAOS<Storage>::Ptr data = toStorage<Host, Storage> (data_host);

        shared_ptr<typename Storage<float4>::type> normals;
        {
            ScopeTimeCPU time_normals ("Normal Estimation");
            normals = computeFastPointNormals<Storage> (data);
        }

        typename SampleConsensusModel1PointPlane<Storage>::Ptr model;

        model.reset(new SampleConsensusModel1PointPlane<Storage> (data));
        model->setNormals (normals);

        MultiRandomSampleConsensus<Storage> sac (model, 2000.0);
        sac.setMinimumCoverage (0.9);
        sac.setMaximumBatches (5);
        sac.setIterationsPerBatch (1000);

        {
            ScopeTimeCPU time_compute ("Plane Computation");
            sac.computeModel (0);
            std::vector<typename SampleConsensusModel1PointPlane<Storage>::IndicesPtr> planes;
            typename Storage<int>::type region_mask;
            markInliers<Storage> (data, region_mask, planes);
            thrust::host_vector<int> regions_host;
            std::copy (regions_host.begin (), regions_host.end (), std::ostream_iterator<int>(std::cerr, " "));
            planes = sac.getAllInliers ();
            std::vector<int> planes_inlier_counts = sac.getAllInlierCounts ();
            std::vector<float4> coeffs = sac.getAllModelCoefficients ();
            std::vector<float3> centroids = sac.getAllModelCentroids ();
            ROS_INFO_STREAM("\nFound " << planes.size () << " planes");
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
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;

};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "simple_cuda_node");
    ros::NodeHandle n("~");

    TestPCL<Device> test;

    ros::Rate loop_rate(1);
    while(n.ok())
    {   
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

