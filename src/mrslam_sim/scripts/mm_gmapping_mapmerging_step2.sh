export ROS_IP=192.168.217.128;
source /opt/ros/kinetic/setup.bash;
rosservice call /gazebo/unpause_physics '{}'

ssh max@192.168.217.129 "
export ROS_IP=192.168.217.129;
source /opt/ros/kinetic/setup.bash;
rosservice call /gazebo/unpause_physics '{}'"

ssh max@192.168.217.130 "
export ROS_IP=192.168.217.130;
source /opt/ros/kinetic/setup.bash;
rosservice call /gazebo/unpause_physics '{}'"