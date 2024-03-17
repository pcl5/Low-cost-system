cmake_minimum_required(VERSION 3.0.2)
project(livox2sensor)

set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  tf2_ros
  pcl_conversions
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES livox2sensor
#  CATKIN_DEPENDS roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



add_executable(livox2sensor src/livox2sensor.cpp)
target_link_libraries(livox2sensor ${catkin_LIBRARIES})

