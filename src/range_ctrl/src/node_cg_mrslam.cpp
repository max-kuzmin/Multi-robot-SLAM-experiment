#include <node_base.cpp>
#include "cg_mrslam/SLAM.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "range_ctrl_cg_mrslam");

    NodeBase<cg_mrslam::SLAM> node;
    node.loop();

    return 0;
}
