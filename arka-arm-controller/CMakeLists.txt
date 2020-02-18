cmake_minimum_required(VERSION 2.8.3)
project(arka_arm_controller)


find_package(catkin REQUIRED COMPONENTS
  control_msgs
  roscpp
  rospy
  sensor_msgs
)



catkin_package(

)


include_directories(

  ${catkin_INCLUDE_DIRS}
)


