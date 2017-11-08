#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_to_pose");

    ros::NodeHandle node;
    std::string tf_prefix;
    node.param("tf_prefix", tf_prefix, tf_prefix);
    if (tf_prefix.length()>0) tf_prefix+="/";

    ros::Rate loop_rate(10);

    ros::Publisher pub = node.advertise<geometry_msgs::PoseStamped>("pose", 10);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    while (ros::ok())
    {
        try
        {
            //transform from robot to map coordinates made by gmapping
            geometry_msgs::TransformStamped ts = tfBuffer.lookupTransform(tf_prefix+"map", tf_prefix+"base_footprint", ros::Time(0));
            geometry_msgs::PoseStamped msg;
            msg.pose.orientation = ts.transform.rotation;
            msg.pose.position.x = ts.transform.translation.x;
            msg.pose.position.y = ts.transform.translation.y;
            msg.pose.position.z = ts.transform.translation.z;
            pub.publish(msg);
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();

        }

        loop_rate.sleep();
    }


    return 0;
}
