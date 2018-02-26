cmake_minimum_required(VERSION 2.8.3)
project(flc_teleoperation)

find_package(catkin REQUIRED COMPONENTS
  joy
  rospy
  std_msgs
  message_generation
  actionlib
  actionlib_msgs
  genmsg
  geometry_msgs
)

add_action_files(
    FILES
    FLC.action
)

catkin_python_setup()

generate_messages(
   DEPENDENCIES
   actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    ${PACKAGE_DEPS}
    message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_install_python(
  PROGRAMS nodes/flc nodes/flc_as
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
