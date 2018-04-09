cmake_minimum_required(VERSION 2.8.3)
project(range_ctrl)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp tf2_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES range_ctrl
  CATKIN_DEPENDS roscpp tf2_ros
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  src
)

add_executable(${PROJECT_NAME}_odom src/node_odom.cpp)

target_link_libraries(${PROJECT_NAME}_odom
   ${catkin_LIBRARIES}
 )
