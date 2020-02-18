cmake_minimum_required(VERSION 2.8.3)
project(arka_manipulator)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  message_generation
)


add_message_files(
  FILES
  ManipulatorData.msg
  ManipulatorStatus.msg
)

add_service_files(
  FILES
  SetEnabled.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/ManipulatorJoy.cfg
)


catkin_package(
  CATKIN_DEPENDS message_runtime
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)

