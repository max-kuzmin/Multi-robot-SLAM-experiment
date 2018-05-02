# Multi-robot SLAM simulation experiment

Collection of multi-robot SLAM algorithms for ROS

Simulation enviroment:
* Robot - [Turtlebot 3](https://github.com/ROBOTIS-GIT/turtlebot3)
* Navigation stack - [move_base](https://github.com/ros-planning/navigation)
* Simulator - Gazebo
* World model - willowgarage.world

Main package ``mrslam_sim`` contains all required launch files. Look inside those files for additional info. 
Some simulations require modified versions of packages. So, it's recommended to use my forks of this repositories.

Package ``range_ctrl`` simulates connection interruptions between robots with specified range, if simulation runned inside one master-process.

Package ``msg_collector`` collects messages from specific for every algorithm ROS topics, when connection between two robots lost. 
When connection established again, it sends all saved messages. It's very useful addition for those algorithms, for which data lose is critical.

##### Smulations:
1.1. Simple map merging algoritm [map_merging](https://github.com/MaxGsomGsom/map_merging). Just superimposes all maps. 
[G-mapping](https://github.com/ros-perception/slam_gmapping) used as base SLAM.
```
roslaunch mrslam_sim m_gmapping_mapmerging.launch
```

1.2. The same as previous, but can be ran on different hosts with separate copies of gazebo and ROS. 
Robots connections provided by [multimaster_fkie](https://github.com/fkie/multimaster_fkie) package. 
Connections between hosts can established with ad-hoc WIFI network. Run `mrslam_sim/scripts/adhoc_net.sh` on every host to setup WIFI adapters.
To run simulation setup ssh keys from main computer to secondary (to allow connection without password). Then execute on main computer:
```
mm_gmapping_mapmerging_step1.sh
mm_gmapping_mapmerging_step2.sh
mm_gmapping_mapmerging_step3.sh
```
Launch of simulation separated on 3 steps, because of some problems with time syncronization.

2.1. Map merging algorithm [m-explore/map_merge](https://github.com/MaxGsomGsom/m-explore) based on OpenCV library matchers. 
Does not require knowledge of initial positions of robots. [G-mapping](https://github.com/ros-perception/slam_gmapping) used as base SLAM.
```
roslaunch mrslam_sim m_gmapping_mexplore_mapmerge.launch
```

2.2. TODO
