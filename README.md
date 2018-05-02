# Multi-robot SLAM simulation experiment

Collection of multi-robot SLAM algorithms for ROS

##### Description
Simulation enviroment:
* Robot - [**Turtlebot 3**](https://github.com/ROBOTIS-GIT/turtlebot3)
* Navigation stack - [**move_base**](https://github.com/ros-planning/navigation)
* Simulator - **Gazebo**
* World model - **willowgarage.world**

Main package **mrslam_sim** contains all required launch files. Look inside those files for additional info. 
Some simulations require modified versions of packages. So, it's recommended to use my forks of this repositories.

Package **range_ctrl** simulates connection interruptions between robots with specified range, if simulation runned inside one master-process.

Package **msg_collector** collects messages from specific for every algorithm ROS topics, when connection between two robots lost. 
When connection established again, it sends all saved messages. It's very useful addition for those algorithms, for which data lose is critical.

##### Installation
Install **GCC 7** as default compiler, older versions aren't supported by some packages.
Also install some dependences from sources:
* [**MRPT 1.9.9**](https://github.com/MRPT/mrpt)
* [**G2O**](https://github.com/RainerKuemmerle/g2o)
* [**Obviously**](https://github.com/autonohm/obviously)

Then run:
```
git clone https://github.com/MaxGsomGsom/Multi-robot-SLAM-experiment
cd Multi-robot-SLAM-experiment
rosdep install --from-paths . --ignore-src --rosdistro=kinetic
catkin_make
```

##### Smulations
1.1. Simple map merging algoritm [**map_merging**](https://github.com/MaxGsomGsom/map_merging). Just superimposes all maps. 
[**G-mapping**](https://github.com/ros-perception/slam_gmapping) used as base SLAM.
```
roslaunch mrslam_sim m_gmapping_mapmerging.launch
```

1.2. The same as 1.1. but can be run on different hosts with separate copies of gazebo and ROS. 
Robots connections provided by [**multimaster_fkie**](https://github.com/fkie/multimaster_fkie) package. 
Connections between hosts can established with ad-hoc WIFI network.
Run `src/mrslam_sim/scripts/adhoc_net.sh` on every host to setup WIFI adapters in Ad-hoc mode.
Setup SSH keys from main computer to secondary (`src/mrslam_sim/scripts/install_ssh_keys.sh`) to allow connection without password.
Don't forget to enter correct IP's in scripts.
Launch of simulation separated on 3 steps, because of some problems with time syncronization. You should execute it on main computer:
```
./src/mrslam_sim/scripts/mm_gmapping_mapmerging_step1.sh
./src/mrslam_sim/scripts/mm_gmapping_mapmerging_step2.sh
./src/mrslam_sim/scripts/mm_gmapping_mapmerging_step3.sh
```

2.1. Map merging algorithm [**m-explore/map_merge**](https://github.com/MaxGsomGsom/m-explore) based on OpenCV library matchers. 
Does not require knowledge of initial positions of robots. [**G-mapping**](https://github.com/ros-perception/slam_gmapping) used as base SLAM.
```
roslaunch mrslam_sim m_gmapping_mexplore_mapmerge.launch
```

2.2. The same as 2.1. but can be run on different hosts with separate copies of gazebo and ROS. 
Robots connections provided by [**multimaster_fkie**](https://github.com/fkie/multimaster_fkie) package. 
Connections between hosts can established with ad-hoc WIFI network.
Run `src/mrslam_sim/scripts/adhoc_net.sh` on every host to setup WIFI adapters in Ad-hoc mode.
Setup SSH keys from main computer to secondary (`src/mrslam_sim/scripts/install_ssh_keys.sh`) to allow connection without password.
Don't forget to enter correct IP's in scripts.
Launch of simulation separated on 3 steps, because of some problems with time syncronization. You should execute it on main computer:
```
./src/mrslam_sim/scripts/mm_gmapping_mexplore_mapmerge_step1.sh
./src/mrslam_sim/scripts/mm_gmapping_mexplore_mapmerge_step2.sh
./src/mrslam_sim/scripts/mm_gmapping_mexplore_mapmerge_step3.sh
```
