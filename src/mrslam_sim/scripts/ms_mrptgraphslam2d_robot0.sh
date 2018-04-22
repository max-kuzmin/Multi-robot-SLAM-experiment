source /opt/ros/kinetic/setup.bash;
source devel/setup.bash;
export ROS_MASTER_URI=http://localhost:11110
gnome-terminal -e 'roscore -p 11110'
sleep 3
gnome-terminal -e 'roslaunch -p 11110 mrslam_sim ms_mrptgraphslam2d_step1.launch x:=4 y:=-5 r_id:=0 r_count:=2'
sleep 7
gnome-terminal -e 'roslaunch -p 11110 mrslam_sim ms_mrptgraphslam2d_step2.launch'
