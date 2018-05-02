export ROS_IP=192.168.217.128;
source /opt/ros/kinetic/setup.bash;
rosservice call /gazebo/unpause_physics '{}'

ssh max@192.168.217.131 "
export ROS_IP=192.168.217.131;
source /opt/ros/kinetic/setup.bash;
rosservice call /gazebo/unpause_physics '{}'"

ssh max@192.168.217.132 "
export ROS_IP=192.168.217.132;
source /opt/ros/kinetic/setup.bash;
rosservice call /gazebo/unpause_physics '{}'"