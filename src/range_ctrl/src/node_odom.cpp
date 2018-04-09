#include <node_base.cpp>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "range_ctrl_odom");

    NodeBase<nav_msgs::Odometry> node;
    node.loop();

    return 0;
}
