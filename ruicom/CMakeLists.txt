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
  std_srvs
  message_generation
)

add_message_files(
  FILES
  Detection.msg
  DetectionArray.msg
  ZoneCapture.msg
  ZoneCaptureDynamic.msg
  MissionState.msg
  HealthState.msg
  FrameRegionCounts.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs sensor_msgs actionlib actionlib_msgs cv_bridge image_transport move_base_msgs tf2_ros std_srvs message_runtime
)

catkin_install_python(PROGRAMS
  scripts/vision_counter_node.py
  scripts/mission_manager_node.py
  scripts/mission_recorder_node.py
  scripts/system_manager_node.py
  scripts/cmd_safety_mux_node.py
  scripts/synthetic_scene_publisher.py
  scripts/fake_move_base_action_server.py
  scripts/fake_detection_array_publisher.py
  scripts/fake_base_feedback_publisher.py
  scripts/fake_twist_publisher.py
  scripts/platform_bridge_node.py
  tests/demo_pipeline_smoke.py
  tests/baseline_smoke.py
  tests/baseline_contract_smoke.py
  tests/baseline_deploy_smoke.py
  tests/baseline_deploy_feedback_timeout_smoke.py
  tests/baseline_integration_smoke.py
  tests/dynamic_schema_integration_smoke.py
  tests/dynamic_schema_mismatch_smoke.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(tests/demo_profile_smoke.test)
  add_rostest(tests/baseline_profile_smoke.test)
  add_rostest(tests/baseline_contract_smoke.test)
  add_rostest(tests/baseline_deploy_smoke.test)
  add_rostest(tests/baseline_deploy_feedback_timeout_smoke.test)
  add_rostest(tests/baseline_integration_smoke.test)
  add_rostest(tests/baseline_integration_namespace_smoke.test)
  add_rostest(tests/dynamic_schema_integration_smoke.test)
  add_rostest(tests/dynamic_schema_integration_namespace_smoke.test)
  add_rostest(tests/dynamic_schema_mismatch_smoke.test)
endif()
