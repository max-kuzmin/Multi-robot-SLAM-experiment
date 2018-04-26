#include <node_base.cpp>
#include <sensor_msgs/LaserScan.h>
#include <mrpt_rbpf_slam/ObservationWithTransform.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "range_ctrl_observation");

    NodeBase<mrpt_rbpf_slam::ObservationWithTransform> node;
    node.loop();

    return 0;
}
