# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(map_merging)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs nav_msgs tf)

# Set the default path for built executables to the "bin" directory.
set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# Set the name to use for the executable.
set (BINNAME map_merging)

# Set the source files to use with the executable.
set (SRCS ${SRCS} src/map_merging.cpp)

# Set the directories where include files can be found.
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

# Build the executable that will be used to run this node.
add_executable (${BINNAME} ${SRCS})
target_link_libraries(${BINNAME} ${catkin_LIBRARIES})

# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roscpp geometry_msgs nav_msgs tf
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include 
    LIBRARIES # TODO
)
