cmake_minimum_required(VERSION 3.0.2)
project(ruikang_recon_baseline)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  cv_bridge
  image_transport
  move_base_msgs
  tf2_ros
  message_generation
)

add_message_files(
  FILES
  Detection.msg
  DetectionArray.msg
  ZoneCapture.msg
  MissionState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs sensor_msgs actionlib actionlib_msgs cv_bridge image_transport move_base_msgs tf2_ros message_runtime
)

catkin_install_python(PROGRAMS
  scripts/vision_counter_node.py
  scripts/mission_manager_node.py
  scripts/mission_recorder_node.py
  scripts/cmd_safety_mux_node.py
  scripts/synthetic_scene_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest QUIET)
  if(rostest_FOUND)
    add_rostest(tests/demo_profile_smoke.test)
  endif()
endif()
