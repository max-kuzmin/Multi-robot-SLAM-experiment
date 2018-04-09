#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <string>

using namespace std;

template<class T> //msg type
class NodeBase
{
 public:

  vector< vector<ros::Publisher> > pubs;
  vector< vector<ros::Subscriber> > subs;

  string base_ns = "robot";
  int range;
  int r_count;
  int r_id;
  vector<string> topics;

  void split(const string& s, char delim,vector<string>& v) {
      int i = 0;
      int pos = s.find(delim);
      while (pos != string::npos) {
        v.push_back(s.substr(i, pos-i));
        i = ++pos;
        pos = s.find(delim, pos);

        if (pos == string::npos)
           v.push_back(s.substr(i, s.length()));
      }
  }

  void route(T& msg, ros::Publisher& pb){
      pb.publish(msg);
  }

  NodeBase() {
      ros::NodeHandle node;

      node.param("base_ns", base_ns, base_ns);
      node.param("range", range, 5);
      node.param("r_count", r_count, 2);
      node.param("r_id", r_id, 0);

      string tempTopics = "";
      node.param("topics", tempTopics, tempTopics);
      split(tempTopics, ' ', topics);

      for (int k=0; k < r_count; k++) {
          vector<ros::Subscriber> rsubs;
          vector<ros::Publisher> rpubs;
          for (int i=0; i < topics.size() && k!= r_id; i++) {

              ros::Publisher pb = node.advertise<T>("/" + base_ns + to_string(k) + "/_" + topics[i], 1);
              rpubs.push_back(pb);

              ros::Subscriber sb; /*= node.subscribe<T>("/" + base_ns + to_string(k) + "/" + topics[i], 1, bind(&NodeBase::route, this, _1, pb));*/
              rsubs.push_back(sb);
          }

          pubs.push_back(rpubs);
          subs.push_back(rsubs);
      }
  }

  int loop()
  {
      tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener tfListener(tfBuffer);
      ros::Rate loop_rate(10);

      while (ros::ok())
      {
          /*try
          {
              //transform from robot to map coordinates made by gmapping
              geometry_msgs::TransformStamped ts = tfBuffer.lookupTransform("map", "base_footprint", ros::Time(0));
              geometry_msgs::PoseStamped msg;
              msg.pose.orientation = ts.transform.rotation;
              msg.pose.position.x = ts.transform.translation.x;
              msg.pose.position.y = ts.transform.translation.y;
              msg.pose.position.z = ts.transform.translation.z;
              msg.header.stamp = ros::Time(0);
              msg.header.frame_id = tf_prefix+"map";
              pub.publish(msg);
          }
          catch (tf2::TransformException& ex)
          {
              ROS_WARN("%s", ex.what());
              ros::Duration(1.0).sleep();

          }*/

          loop_rate.sleep();
      }


      return 0;
  }

};

