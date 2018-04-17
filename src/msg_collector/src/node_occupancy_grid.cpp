#include <node_base.cpp>
#include <nav_msgs/OccupancyGrid.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "range_ctrl_occupancy_grid");

    NodeBase<nav_msgs::OccupancyGrid> node;
    node.loop();

    return 0;
}
