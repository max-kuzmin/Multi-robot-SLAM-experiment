source /opt/ros/kinetic/setup.bash;
source devel/setup.bash;
export ROS_IP=192.168.218.1
export ROS_MASTER_URI=http://192.168.218.1:11110
export GAZEBO_MASTER_URI=http://192.168.218.1:11100
#gnome-terminal -e 'roscore -p 11110'
#sleep 3
gnome-terminal -e 'roslaunch -p 11110 mrslam_sim ms_mrpt_graphslam2d_step1.launch x:=4 y:=-5 r_id:=0 r_count:=2'
sleep 7
gnome-terminal -e 'roslaunch -p 11110 mrslam_sim ms_mrpt_graphslam2d_step2.launch r_id:=0'
