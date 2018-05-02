# Multi-robot SLAM simulation experiment

Collection of multi-robot SLAM algorithms for ROS

##### Description
Simulation enviroment:
* Robot - [**Turtlebot 3**](https://github.com/ROBOTIS-GIT/turtlebot3)
* Navigation stack - [**move_base**](https://github.com/ros-planning/navigation)
* Simulator - **Gazebo**
* World model - **willowgarage.world**

All algorithms use LIDAR as sensor (with or without odometry) and grid-maps.

Main package **mrslam_sim** contains all required launch files. Look inside those files for additional info. 
Some simulations require modified versions of packages. So, it's recommended to use my forks of this repositories.

Package **range_ctrl** simulates connection interruptions between robots with specified range, if simulation runned inside one master-process. 
Default connection range is 10 meters.

Package **msg_collector** collects messages from specific for every algorithm ROS topics, when connection between two robots lost. 
When connection established again, it sends all saved messages. It's very useful addition for those algorithms, for which data lose is critical.

You can control robots inside RViz by selecting green arrows on top panel and pointing on the map.

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
[**G-mapping**](https://github.com/ros-perception/slam_gmapping) used as base SLAM (Rao-Blackwellised particle filter).
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
Does not require knowledge of initial positions of robots. [**G-mapping**](https://github.com/ros-perception/slam_gmapping) used as base SLAM (Rao-Blackwellised particle filter).
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

3. Map merging algorithm [**m-explore/map_merge**](https://github.com/MaxGsomGsom/m-explore) based on OpenCV library matchers. 
Does not require knowledge of initial positions of robots. [**Hector-SLAM**](https://github.com/tu-darmstadt-ros-pkg/hector_slam) used as base SLAM (scan-matching).
```
roslaunch mrslam_sim m_hectormapping_mexplore_mapmerge.launch
```

4. Graph-SLAM algorithm based on **G2O** framework [**cg_mrslam**](https://github.com/MaxGsomGsom/cg_mrslam). Merges overlaped parts of map online based on condensed laser scans. 
After executing it is possible to merge full maps. To perform it first merge output graphs (look for *.g2o files in working dir) with **g2o_merger** (part of cg_mrslam package). 
Then open resulting file in **g2o_viewer** (part of G2O framework) and optimize it.
```
roslaunch mrslam_sim m_cgmrslam.launch
```

5. Graph-SLAM algorithm based on **Karto** mapping library [**nav2d_karto**](https://github.com/MaxGsomGsom/navigation_2d). 
Performs full map merging online based on raw laser scans. Initial positions of robots must be known.
```
roslaunch mrslam_sim m_nav2dkarto.launch
```

6. ICP-based no-odometry SLAM [**ohm_tsd_slam**](https://github.com/MaxGsomGsom/ohm_tsd_slam). 
Performs full map merging online based on raw laser scans. Initial positions of robots must be known.
```
roslaunch mrslam_sim m_ohmtsdslam.launch
```

7. Graph-SLAM algorithm based on **MRPT** framework [**mrpt_graphslam_2d**](https://github.com/MaxGsomGsom/mrpt_slam). 
Merges parts of maps by requesting chains of graph nodes of other robots. 
Integrated with **multimaster_fkie**, so it is not possible to run simulation inside one master process.
To run it on one PC you need setup virtual network adapters in you system in `/etc/network/interfaces` 
(you can find example of config in `src/mrslam_sim/scripts/interfaces`) and restart networking service. Don't forget to enter correct IP's in scripts.
Then run launch script for every robot simultaneously. Robots should start from one position because map-matching dont work for now (some bugs).
```
./src/mrslam_sim/scripts/ms_mrpt_graphslam2d_robot0.sh & 
./src/mrslam_sim/scripts/ms_mrpt_graphslam2d_robot1.sh
```

8. Rao-Blackwellised particle filter SLAM algorithm based on **MRPT** framework [**mrpt_rbpf_slam**](https://github.com/MaxGsomGsom/mrpt_slam). 
Performs full map merging online based on raw localized laser scans. Initial positions of robots must be known.
```
roslaunch mrslam_sim m_mrpt_rbpf.launch
```

9. ICP-based no-odometry SLAM algorithm based on **MRPT** framework [**mrpt_icp_slam_2d**](https://github.com/MaxGsomGsom/mrpt_slam). 
Performs full map merging online based on raw localized laser scans. Initial positions of robots must be known.
```
roslaunch mrslam_sim m_mrpt_icp2d.launch
```
