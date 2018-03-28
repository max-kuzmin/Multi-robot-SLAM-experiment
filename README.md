# ROS-SLAM-playground

These simulations use:
* Robot - Turtlebot 3
* Navigation - move_base
* Simulator - Gazebo
* World - willowgarage.world

##### Working simulations:
1.1. Multi-robot simulation with [map_merging](https://github.com/yzrobot/map_merging) algorithm. Requires init positions of robots. Merged map used by move_base.
```
roslaunch mrslam_sim m_gmapping_mapmerging.launch
```

1.2. Multi-robot simulation with map_merging algorithm. The same as previous, but can be ran on different hosts with separate copies of gazebo. Robots connections made with [multimaster_fkie](https://github.com/fkie/multimaster_fkie) package. Connections between hosts established with ad-hoc wifi network (run `adhoc_net.sh` for setup). To run simulation setup ssh keys and then execute on main computer:
```
mm_gmapping_mapmerging_step1.sh
mm_gmapping_mapmerging_step2.sh
mm_gmapping_mapmerging_step3.sh
```
Launch of simulation separated on 3 steps, because copies of Gazebo can not be synchronised another way.

2.1. Multi-robot simulation with [multirobot_map_merge](https://github.com/hrnr/m-explore) algorithm. Uses algoritms from OpenCV 3 for maps merging. Does not require initial positions of robots. Runs on one master-process.
```
roslaunch mrslam_sim m_gmapping_multirobotmapmerge.launch
```

2.2. TODO
