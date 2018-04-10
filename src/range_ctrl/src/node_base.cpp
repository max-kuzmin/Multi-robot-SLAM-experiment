#include <ros/ros.h>
#include <vector>
#include <string>
#include <nav_msgs/Odometry.h>
#include <boost/algorithm/string.hpp>

using namespace std;

template<class T> //msg type
class NodeBase
{
 public:

  ros::NodeHandle node = ros::NodeHandle("~");
  vector< vector<ros::Publisher> > pubs;
  vector< vector<ros::Subscriber> > subs;

  vector<ros::Subscriber> ground_truth_subs;

  string base_ns = "robot";
  string ground_truth_topic = "ground_truth";
  int max_range;
  int r_count;
  int r_id;
  int delta = 0.5;
  vector<string> topics;
  vector<geometry_msgs::Point> positions;
  vector<bool> are_subs_enabled;

  void route(T msg, ros::Publisher& pb){
      pb.publish(msg);
  }

  void watch_range(nav_msgs::Odometry::ConstPtr msg, int robot_id) {
      positions[robot_id] = msg->pose.pose.position;
  }

  NodeBase() {
      //get parameters
      node.param("base_ns", base_ns, base_ns);
      node.param("ground_truth_topic", ground_truth_topic, ground_truth_topic);
      node.param("range", max_range, 5);
      node.param("r_count", r_count, 2);
      node.param("r_id", r_id, 0);

      //get list of topics
      string tempTopics = "";
      node.param("topics", tempTopics, tempTopics);
      boost::split(topics, tempTopics, boost::is_any_of("\t "));

      //Show topics in log
      string message = "";
      for (int i=0; i< topics.size(); i++)
          message+=topics[i] + " ";
      ROS_INFO_STREAM("# Topics to retranslate: " << message);

      for (int k=0; k < r_count; k++) {
          vector<ros::Subscriber> rsubs;
          vector<ros::Publisher> rpubs;
          for (int i=0; i < topics.size() && k!= r_id; i++) {

              //create publishers
              ros::Publisher pb = node.advertise<T>("/" + base_ns + to_string(k) + "/_" + topics[i], 1);
              rpubs.push_back(pb);
          }

          pubs.push_back(rpubs);
          rsubs.resize(topics.size());
          subs.push_back(rsubs);

          geometry_msgs::Point p; p.x=k*1000;
          positions.push_back(p);
          are_subs_enabled.push_back(false);

          //subscripe to ground truth
          ground_truth_subs.push_back(node.subscribe<nav_msgs::Odometry>(
                                          "/" + base_ns + to_string(k) + "/" + ground_truth_topic, 1,
                                          boost::bind(&NodeBase<T>::watch_range, this, _1, k)));
      }
  }

  int loop()
  {
      if (topics.size()==0) return 0;

      ros::Rate loop_rate(10);
      while (ros::ok())
      {
          for (int k=0; k < r_count; k++) {
              if (k == r_id) continue;

              double rng = sqrt(pow(positions[k].x-positions[r_id].x, 2)
                                + pow(positions[k].y-positions[r_id].y, 2));

              //create subscribers
              if (rng < (max_range-delta) && !are_subs_enabled[k]) {
                  for (int i=0; i < topics.size(); i++) {
                      boost::function<void (T)> fun1( boost::bind(&NodeBase::route, this, _1, pubs[k][i]) );
                      ros::Subscriber sb = node.subscribe<T>("/" + base_ns + to_string(k) + "/" + topics[i], 1, fun1);
                      subs[k][i] = sb;
                  }
                  are_subs_enabled[k] = true;
                  ROS_INFO("# Establishing connection between robots %d and %d", r_id, k);
              }

              //shutdown subscribers
              else if (rng > (max_range+delta) && are_subs_enabled[k]) {
                  for (int i = 0; i < subs[k].size(); i++) {
                      subs[k][i].shutdown();
                  }
                  are_subs_enabled[k] = false;
                  ROS_INFO("# Shutting down connection between robots %d and %d", r_id, k);
              }

          }


          loop_rate.sleep();
          ros::spinOnce();
      }

      return 0;
  }

};

