cmake_minimum_required(VERSION 2.8.3)
project(msg_collector)

set (CMAKE_CXX_STANDARD 11)
set (CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp nav_msgs sensor_msgs nav2d_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES msg_collector
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  src
)

add_executable(${PROJECT_NAME}_odom src/node_odom.cpp)
add_executable(${PROJECT_NAME}_occupancy_grid src/node_occupancy_grid.cpp)
add_executable(${PROJECT_NAME}_laser_scan src/node_laser_scan.cpp)
add_executable(${PROJECT_NAME}_nav2d_scan src/node_nav2d_scan.cpp)

target_link_libraries(${PROJECT_NAME}_odom
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
