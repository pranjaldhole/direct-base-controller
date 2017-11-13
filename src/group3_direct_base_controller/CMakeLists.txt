cmake_minimum_required(VERSION 2.8.3)
project(group3_direct_base_controller)

find_package(catkin REQUIRED
    COMPONENTS
    roslint
    roscpp
    rospy
    std_msgs
    geometry_msgs
    dynamic_reconfigure
    mcr_manipulation_msgs
    mcr_monitoring_msgs
    topic_tools
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  ros/config/ComponentWisePoseErrorMonitor.cfg
)

catkin_package(
  INCLUDE_DIRS
    ros/include
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    tf
    dynamic_reconfigure
    mcr_monitoring_msgs
    mcr_manipulation_msgs
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(component_wise_pose_error_monitor_node
  ros/src/mcr_direct_base_controller_ros/component_wise_pose_error_monitor_node.cpp
)

add_dependencies(component_wise_pose_error_monitor_node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(component_wise_pose_error_monitor_node
  ${catkin_LIBRARIES}
)

### INSTALLS
install(PROGRAMS
  ros/scripts/direct_base_controller_coordinator_node
  ros/scripts/transform_to_pose_converter_node
  ros/scripts/component_wise_pose_error_calculator_node
  ros/scripts/twist_controller_node
  ros/scripts/twist_limiter_node
  ros/scripts/twist_synchronizer_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS
    component_wise_pose_error_monitor_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(DIRECTORY ros/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config
)

### TESTS
roslint_python(
  ros/scripts/component_wise_pose_error_calculator_node
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
  add_rostest(ros/launch/component_wise_pose_error_calculator.test)
  catkin_add_nosetests(ros/test/component_wise_pose_error_calculator_unittest.py)
  add_rostest(ros/launch/twist_controller.test)
  add_rostest(ros/launch/twist_limiter.test)
  catkin_add_nosetests(common/test/limiter_unittest.py)
  add_rostest(ros/launch/twist_synchronizer.test)
  catkin_add_nosetests(ros/test/twist_synchronizer_unittest.py)
endif()
