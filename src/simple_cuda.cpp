#include <ros/ros.h>

void cudamain();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_cuda_node");
    ros::NodeHandle n("~");

    ros::Rate loop_rate(1);
    while(n.ok())
    {   
        cudamain();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}