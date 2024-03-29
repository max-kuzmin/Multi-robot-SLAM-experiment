cmake_minimum_required(VERSION 2.8.3)
project(range_ctrl)

set (CMAKE_CXX_STANDARD 11)
set (CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp nav_msgs cg_mrslam sensor_msgs nav2d_msgs mrpt_rbpf_slam
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES range_ctrl
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  src
)

add_executable(${PROJECT_NAME}_odom src/node_odom.cpp)
add_executable(${PROJECT_NAME}_cg_mrslam src/node_cg_mrslam.cpp)
add_executable(${PROJECT_NAME}_occupancy_grid src/node_occupancy_grid.cpp)
add_executable(${PROJECT_NAME}_laser_scan src/node_laser_scan.cpp)
add_executable(${PROJECT_NAME}_nav2d_scan src/node_nav2d_scan.cpp)
add_executable(${PROJECT_NAME}_observation src/node_observation.cpp)

target_link_libraries(${PROJECT_NAME}_odom
   ${catkin_LIBRARIES}
 )
target_link_libraries(${PROJECT_NAME}_cg_mrslam
   ${catkin_LIBRARIES}
 )
target_link_libraries(${PROJECT_NAME}_occupancy_grid
   ${catkin_LIBRARIES}
 )
target_link_libraries(${PROJECT_NAME}_laser_scan
   ${catkin_LIBRARIES}
 )

target_link_libraries(${PROJECT_NAME}_nav2d_scan
   ${catkin_LIBRARIES}
 )

target_link_libraries(${PROJECT_NAME}_observation
   ${catkin_LIBRARIES}
 )
