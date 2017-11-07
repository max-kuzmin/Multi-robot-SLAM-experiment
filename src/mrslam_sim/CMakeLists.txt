cmake_minimum_required(VERSION 2.8.3)
project(mrslam_sim)
find_package(catkin REQUIRED)
catkin_package()

include_directories()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
