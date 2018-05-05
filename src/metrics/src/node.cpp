#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <iostream>
#include <fstream>
#include <tf/transform_datatypes.h>


geometry_msgs::Pose ground_pose;

void ground_truth_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    ground_pose = msg->pose.pose;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "metrics");

    ros::NodeHandle node;
    sleep(5);
    
    std::string tf_prefix;
    node.param("tf_prefix", tf_prefix, tf_prefix);
    if (tf_prefix.length()>0) tf_prefix+="/";
    
    ros::Subscriber ground_truth_sub = node.subscribe<nav_msgs::Odometry>(
                "ground_truth", 100, ground_truth_callback);

    ros::Rate loop_rate(1);
   
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    std::ofstream deviation_file;
    deviation_file.open(node.param<std::string>("tf_prefix", "robot") + "_rmse.csv");
    
    
    while (ros::ok())
    {
        try
        {
            //transform from robot to map coordinates = pose
            geometry_msgs::TransformStamped ts = tfBuffer.lookupTransform(tf_prefix+"map", tf_prefix+"base_footprint", ros::Time(0));
            geometry_msgs::Pose estimated_pose;
            estimated_pose.orientation = ts.transform.rotation;
            estimated_pose.position.x = ts.transform.translation.x;
            estimated_pose.position.y = ts.transform.translation.y;
            estimated_pose.position.z = ts.transform.translation.z;
            
            double dev_x = abs(estimated_pose.position.x - ground_pose.position.x);
            double dev_y = abs(estimated_pose.position.y - ground_pose.position.y);
            double rmse = sqrt(dev_x*dev_x + dev_y*dev_y);
            ROS_INFO_STREAM(rmse);
            deviation_file << dev_x << "; " << dev_y << "; "
                           << rmse << std::endl;
        }
        catch (tf2::TransformException& ex)
        {
            ROS_INFO("%s", ex.what());
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    deviation_file.close();
    return 0;
}
