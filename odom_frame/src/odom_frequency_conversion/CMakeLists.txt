cmake_minimum_required(VERSION 3.0.2)
project(odom_frequency_conversion)

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -ggdb")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  mavros_msgs
  roslib
  geometry_msgs
)

find_package(Eigen3 REQUIRED)
# find_package(gflags REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})


generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp std_msgs
 DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${gflags_INCLUDE_DIRS}
)

add_executable(odom_frequency_conversion
  src/odom_frequency_conversion.cpp)
target_link_libraries(odom_frequency_conversion
  Eigen3::Eigen
  ${catkin_LIBRARIES})
