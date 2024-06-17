cmake_minimum_required(VERSION 3.0.2)
project(virtual_pos)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_DEBUG "-g3 -Wall -Wextra -fsanitize=address,undefined -static-libasan")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  mavros
  std_msgs
  mavros_msgs
  geometry_msgs
)

find_package(Curses REQUIRED)

include_directories(${CURSES_INCLUDE_DIR})

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

catkin_package(
  INCLUDE_DIRS include
  #  LIBRARIES virtual_rc
  #  CATKIN_DEPENDS roscpp
  #  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(virtual_rc 
  SHARED
  src/virtual_rc/keyboard_reader.cpp
  src/virtual_rc/rc_control_publisher.cpp)
target_link_libraries(virtual_rc
  ${catkin_LIBRARIES})

add_library(socket 
  SHARED
  src/socket/heartbeat_sender.cpp
  src/socket/heartbeat_receiver.cpp)
target_link_libraries(socket
  ${catkin_LIBRARIES})

add_library(net
  SHARED
  src/net/net_quality.cpp
  src/net/wifi_quality.cpp)
target_link_libraries(socket
  ${catkin_LIBRARIES})

add_library(virtual_pos
  SHARED
  src/virtual_pos/virtual_pos_publisher.cpp)
target_link_libraries(virtual_pos
  ${catkin_LIBRARIES})

# =================================
# =================================

add_executable(rc_control
  src/rc_control.cpp)
target_link_libraries(rc_control
  virtual_rc
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
)

add_executable(mix_control
  src/mix_control.cpp)
target_link_libraries(mix_control
  virtual_rc
  virtual_pos
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
)

add_executable(heartbeat_send
  src/heartbeat_send.cpp)
target_link_libraries(heartbeat_send
  socket
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
)

add_executable(heartbeat_receive
  src/heartbeat_receive.cpp)
target_link_libraries(heartbeat_receive
  socket
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
)

add_executable(netquality_pub
  src/netquality_pub.cpp)
target_link_libraries(netquality_pub
  net
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
)

add_executable(wifiquality_pub
  src/wifiquality_pub.cpp)
target_link_libraries(wifiquality_pub
  net
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
)