#include <ros/ros.h>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include <queue>
#include <boost/thread/mutex.hpp>

using namespace std;

template<class T> //msg type
class NodeBase
{
 public:

  ros::NodeHandle node = ros::NodeHandle("~");
  vector< vector<ros::Publisher> > pubs;
  vector<ros::Subscriber> subs;
  double min_msg_interval; //in seconds
  int r_count, l_rate;
  string ns;
  vector<string> input_topics, output_topics;

  vector< vector<bool> > has_subscribers;
  vector< vector<queue<T>> > msgs; //queue for every publisher for every topic
  vector< vector<boost::mutex*> > queue_mx; //mutexes for msgs queues

  vector<ros::Time> lastTimes; //times of last msg for every topic

  int printMsgCount = 100;

  void check_subs(int t, int r) {
      has_subscribers[t][r] = (pubs[t][r].getNumSubscribers() > 0);
  }

  //push new msg in queues, if previous was more than min_msg_interval before
  void collect(T msg, int t){
      if (ros::Time::now()-lastTimes[t] > ros::Duration(min_msg_interval)) {
          //add msg to every msgs queue of topic
          for (int r=0; r<r_count; r++) {
              boost::mutex::scoped_lock lock(*queue_mx[t][r]);
              lastTimes[t] = ros::Time::now();
              msgs[t][r].push(msg);
              check_subs(t, r);
              if (msgs[t][r].size()>=printMsgCount && msgs[t][r].size()%printMsgCount==0)
              ROS_INFO_STREAM("'" << input_topics[t] << r << "' has "
                              << msgs[t][r].size() << " msgs, publish = " << has_subscribers[t][r]);
          }
      }
  }

  NodeBase() {
      //get parameters
      node.param("min_msg_interval", min_msg_interval, 0.1);
      node.param("r_count", r_count, 2);
      node.param("loop_rate", l_rate, 50);
      node.param<string>("ns", ns, "robot0");
      printMsgCount = 10/min_msg_interval;

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
      for (int t=0; t< input_topics.size(); t++)
          for (int r=0; r< r_count; r++)
              message+=input_topics[t] + "->" + output_topics[t] + to_string(r) + " ";
      ROS_INFO_STREAM("# Topics to collect: " << message);

      msgs.resize(output_topics.size());
      queue_mx.resize(output_topics.size());
      has_subscribers.resize(output_topics.size());
      lastTimes.resize(output_topics.size());

      for (int t=0; t < output_topics.size(); t++) {
          //create publishers for every robot for topic i
          vector<ros::Publisher> topicPubs;
          for (int r=0; r< r_count; r++) {
            ros::Publisher pb = node.advertise<T>("/" +ns + "/" + output_topics[t]+to_string(r), 100);
            topicPubs.push_back(pb);

            queue_mx[t].push_back(new boost::mutex());
          }
          pubs.push_back(topicPubs);

          //set sizes
          msgs[t].resize(r_count);
          //queue_mx[t].resize(r_count);
          has_subscribers[t].resize(r_count);

          //create subscriber for topic i
          boost::function<void (T)> fun1( boost::bind(&NodeBase::collect, this, _1, t) );
          ros::Subscriber sb = node.subscribe<T>("/" +ns + "/" + input_topics[t], 100, fun1);
          subs.push_back(sb);
      }
  }

  int loop()
  {
      if (input_topics.size()==0 || input_topics.size() != output_topics.size()) return 0;

      ros::Rate loop_rate(l_rate);
      while (ros::ok())
      {
          for (int t=0; t< output_topics.size(); t++) {
              for (int r=0; r< r_count; r++) {
                  //if publisher has subscribers and there are msgs in queue
                  if (has_subscribers[t][r] && msgs[t][r].size() > 0) {
                      //lock mutex of current msgs queue
                      boost::mutex::scoped_lock lock(*queue_mx[t][r]);
                      msgs[t][r].front().header.stamp=ros::Time::now(); //set current time
                      pubs[t][r].publish(msgs[t][r].front());
                      msgs[t][r].pop();
                  }
                  check_subs(t, r);
              }
          }

          loop_rate.sleep();
          ros::spinOnce();
      }

      return 0;
  }

};

