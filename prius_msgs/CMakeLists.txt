cmake_minimum_required(VERSION 2.8.12)
project(prius_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  My_Trajectory.msg
)

add_message_files(
  FILES
  My_Trajectory_Point.msg
)
add_message_files(
  FILES
  VehicleInfo.msg
)
add_message_files(
  FILES
  DimensionInfo.msg
)
add_message_files(
  FILES
  LateralData.msg
)
add_message_files(
  FILES
  LongitudinalData.msg
)

add_message_files(
  FILES
  Control.msg
)

add_message_files(
  FILES
  Augmented_My_Trajectory_Point.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
