#include <node_base.cpp>
#include <sensor_msgs/LaserScan.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "range_ctrl_laser_scan");

    NodeBase<sensor_msgs::LaserScan> node;
    node.loop();

    return 0;
}
