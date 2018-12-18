cmake_minimum_required(VERSION 2.8.3)
project(flock_simulator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  Request.msg
  FlockState.msg
  FlockCommand.msg
  DuckieState.msg
  DuckieCommand.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES flock_simulator
  CATKIN_DEPENDS rospy std_msgs message_runtime geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
