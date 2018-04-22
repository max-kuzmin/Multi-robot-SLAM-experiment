source /opt/ros/kinetic/setup.bash;
source devel/setup.bash;
export ROS_MASTER_URI=http://localhost:11111
gnome-terminal -e 'roscore -p 11111'
sleep 3
gnome-terminal -e 'roslaunch -p 11111 mrslam_sim ms_mrptgraphslam2d_step1.launch x:=4 y:=2 r_id:=1 r_count:=2'
sleep 7
gnome-terminal -e 'roslaunch -p 11111 mrslam_sim ms_mrptgraphslam2d_step2.launch'
