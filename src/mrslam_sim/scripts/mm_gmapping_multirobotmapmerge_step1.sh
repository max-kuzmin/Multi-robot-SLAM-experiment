export ROS_IP=192.168.217.128;
source /opt/ros/kinetic/setup.bash;
source /home/max/Desktop/ROS-SLAM-playground/devel/setup.bash;
gnome-terminal -e 'roslaunch mrslam_sim mm_gmapping_multirobotmapmerge_step1.launch x:=0 y:=0 robot_name:=robot1'

ssh max@192.168.217.131 "
export DISPLAY=:0;
export ROS_IP=192.168.217.131;
source /opt/ros/kinetic/setup.bash;
source /home/max/Desktop/ROS-SLAM-playground/devel/setup.bash;
gnome-terminal -e 'roslaunch mrslam_sim mm_gmapping_multirobotmapmerge_step1.launch x:=1 y:=1 robot_name:=robot2'"

ssh max@192.168.217.132 "
export DISPLAY=:0;
export ROS_IP=192.168.217.132;
source /opt/ros/kinetic/setup.bash;
source /home/max/Desktop/ROS-SLAM-playground/devel/setup.bash;
gnome-terminal -e 'roslaunch mrslam_sim mm_gmapping_multirobotmapmerge_step1.launch x:=-1 y:=-1 robot_name:=robot3'"
