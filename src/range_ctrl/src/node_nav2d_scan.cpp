#include <node_base.cpp>
#include <nav2d_msgs/LocalizedScan.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "range_ctrl_nav2d_scan");

    NodeBase<nav2d_msgs::LocalizedScan> node;
    node.loop();

    return 0;
}
