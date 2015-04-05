cmake_minimum_required(VERSION 2.8.3)
project(tag_renderer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  tf
  cv_bridge
)

add_message_files(
  FILES
  TagPose.msg
)

add_service_files(
  FILES
  SetSceneViewport.srv
  SetTagSource.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ftag2_core
  CATKIN_DEPENDS roscpp rospy message_runtime std_msgs geometry_msgs sensor_msgs tf cv_bridge
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_library(tag_renderer
#  src/${PROJECT_NAME}/tag_renderer.cpp
#)

#add_executable(tester src/test.cpp)
#target_link_libraries(tester
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES}
#  ${GLUT_LIBRARIES}
#  ${OpenGL_LIBRARIES}
#)
