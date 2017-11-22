# ROS-SLAM-playground

This simulation uses:
* Robot - Turtlebot 3
* Navigation - move_base
* SLAM - gmapping
* Simulator - Gazebo
* World - willowgarage.world

##### Working simulations:
* Multi robot simulation with map_merging algorithm. Requires init positions of robots. Merged map used by move_base.
```
roslaunch mrslam_sim multi_gmapping_merge.launch
```

* Multi robot simulation with map_merging algorithm. The same as previous put can be ran on different hosts. Connection between hosts established with ad-hoc wifi network. On every host run:
```
sudo ip link set wlan0 down
sudo iwconfig wlan0 mode ad-hoc
sudo iwconfig wlan0 channel 4
sudo iwconfig wlan0 essid 'ROS_NET'
sudo iwconfig wlan0 key 1234267895
sudo ip link set wlan0 up
sudo ip addr add <your local ip>/16 dev wlan0
sudo route add -net 224.0.0.0 netmask 224.0.0.0 wlan0

export ROS_IP=<your local ip>
export ROS_MASTER_URI=http://<your local ip>:<port>
```
It's important to run simulation at the same moment on every host, because Gazebo has some problems with synchronisation. Use names for robot1 and robot2:
```
roslaunch mrslam_sim multimaster_gmapping_merge.launch x:=0 y:=0 robot_name:=robot1
```
