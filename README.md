# ROS-SLAM-playground

This simulation using:
* Robot - Turtlebot 3
* Navigation - move_base
* SLAM - gmapping
* Simulator - Gazebo
* World - willowgarage.world

How to run:
* Multi robot simulation with map_merging algorithm. Requires init positions of robots. Merged map used by move_base.
```roslaunch mrslam_sim multi_gmapping_merge.launch```

