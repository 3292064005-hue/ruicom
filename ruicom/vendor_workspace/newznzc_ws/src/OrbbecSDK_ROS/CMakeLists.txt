cmake_minimum_required(VERSION 3.0.2)
project(OrbbecSDK_ROS)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY .
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)
