cmake_minimum_required(VERSION 2.8.3)
project(map_to_pose)
find_package(catkin REQUIRED COMPONENTS
  roscpp tf2_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map_to_pose
  CATKIN_DEPENDS roscpp tf2_ros
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/node.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )
