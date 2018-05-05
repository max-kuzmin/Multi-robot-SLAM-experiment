#include <ros/ros.h>
#include <vector>
#include <string>
#include <nav_msgs/Odometry.h>
#include <boost/algorithm/string.hpp>
#include <fstream>

using namespace std;

template<class T> //msg type
class NodeBase
{
 public:
  double full_time = 0;
  vector<double> connection_time;
  std::ofstream times_file;
  double step = 0.1;

  ros::NodeHandle node = ros::NodeHandle("~");
  vector< vector<ros::Publisher> > pubs;
  vector< vector<ros::Subscriber> > subs;

  vector<ros::Subscriber> ground_truth_subs;

  string base_ns = "robot";
  string ground_truth_topic = "ground_truth";
  bool retrans_own_topics;
  int max_range;
  int r_count;
  int r_id;
  int delta = 0.5;
  vector<string> input_topics, output_topics;
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
      node.param("retrans_own_topics", retrans_own_topics, false);

      //get list of in topics
      string tempTopics = "";
      node.param("input_topics", tempTopics, tempTopics);
      boost::split(input_topics, tempTopics, boost::is_any_of("\t "));

      //get list of out topics
      tempTopics = "";
      node.param("output_topics", tempTopics, tempTopics);
      boost::split(output_topics, tempTopics, boost::is_any_of("\t "));

      if (input_topics.size() != output_topics.size()) {
          ROS_ERROR_STREAM("# Number of input and output topics not the same");
          return;
      }

      //Show topics in log
      string message = "";
      for (int i=0; i< input_topics.size(); i++)
          message+=input_topics[i] + "->" + output_topics[i] + " ";
      ROS_INFO_STREAM("# Topics to retranslate: " << message);

      for (int k=0; k < r_count; k++) {
          vector<ros::Subscriber> rsubs;
          vector<ros::Publisher> rpubs;
          for (int i=0; i < output_topics.size() && k!= r_id; i++) {
              //create publishers
              ros::Publisher pb = node.advertise<T>("/" + base_ns + to_string(k) + "/" + output_topics[i], 100);
              rpubs.push_back(pb);
          }

          pubs.push_back(rpubs);
          rsubs.resize(input_topics.size());
          subs.push_back(rsubs);

          geometry_msgs::Point p; p.x=k*1000;
          positions.push_back(p);
          are_subs_enabled.push_back(false);

          //subscribe to ground truth
          ground_truth_subs.push_back(node.subscribe<nav_msgs::Odometry>(
                                          "/" + base_ns + to_string(k) + "/" + ground_truth_topic, 100,
                                          boost::bind(&NodeBase<T>::watch_range, this, _1, k)));
      }

      //retranslate own topics
      for (int i=0; i < output_topics.size() && retrans_own_topics; i++) {
          //publishers
          ros::Publisher pb = node.advertise<T>("/" + base_ns + to_string(r_id) + "/" + output_topics[i], 100);
          pubs[r_id].push_back(pb);
          //subscribers
          boost::function<void (T)> fun1( boost::bind(&NodeBase::route, this, _1, pubs[r_id][i]) );
          ros::Subscriber sb = node.subscribe<T>("/" + base_ns + to_string(r_id) + "/" + input_topics[i], 100, fun1);
          subs[r_id][i] = sb;
      }
      
      connection_time.resize(r_count);
      times_file.open(base_ns + to_string(r_id) + "_times.txt");
  }

  int loop()
  {
      if (input_topics.size()==0 || input_topics.size() != output_topics.size()) return 0;

      ros::Rate loop_rate(10);
      while (ros::ok())
      {
          string output = "";
          for (int k=0; k < r_count; k++) {
              if (k == r_id) continue;

              double rng = sqrt(pow(positions[k].x-positions[r_id].x, 2)
                                + pow(positions[k].y-positions[r_id].y, 2));

              //create subscribers
              if (rng < (max_range-delta) && !are_subs_enabled[k]) {
                  for (int i=0; i < input_topics.size(); i++) {
                      boost::function<void (T)> fun1( boost::bind(&NodeBase::route, this, _1, pubs[k][i]) );
                      ros::Subscriber sb = node.subscribe<T>("/" + base_ns + to_string(k) + "/" + input_topics[i], 100, fun1);
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
            
              if (are_subs_enabled[k])
                  connection_time[k]+=step;
              output += to_string(connection_time[k]) + "; ";
          }
          
          full_time+=step;
          output += to_string((int)full_time);
          if ((int)(full_time*10)%10 == 0)
             times_file << output << endl;

          loop_rate.sleep();
          ros::spinOnce();
      }

      times_file.close();
      return 0;
  }

};

