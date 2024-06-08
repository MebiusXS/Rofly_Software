cmake_minimum_required(VERSION 2.8.3)
project(fast_lio)

SET(CMAKE_BUILD_TYPE "Debug")

ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenCV  REQUIRED)
#添加OpenCV头文件
include_directories(${OpenCV_INCLUDE_DIRS})
 
#显示OpenCV_INCLUDE_DIRS的值
message(${OpenCV_INCLUDE_DIRS})

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver
  message_generation
  eigen_conversions
  cv_bridge

)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
  ${PROJECT_SOURCE_DIR}/include
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include)

add_message_files(
  FILES
  Pose6D.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

# add_library( plan_env 
#     src/grid_map.cpp 
#     src/raycast.cpp
#     src/obj_predictor.cpp 
#     )
# target_link_libraries( plan_env
#     ${catkin_LIBRARIES} 
#     ${PCL_LIBRARIES}
#     ${OpenCV_LIBS}
#     ) 

#     add_executable(obj_generator
#     src/obj_generator.cpp 
# )

# add_executable(tobe_grid
#     src/tobe_grid.cpp 
#     src/grid_map.cpp 
#     src/raycast.cpp
# )
# target_link_libraries(obj_generator 
#     ${catkin_LIBRARIES}
#     )
# target_link_libraries(tobe_grid
#     ${catkin_LIBRARIES}
#     ${PCL_LIBRARIES}
#     ${OpenCV_LIBS}
#     )
add_executable(fastlio_mapping src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
target_link_libraries(fastlio_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES}  ${OpenCV_LIBS})
target_include_directories(fastlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})

add_executable(pointCloud_clipping src/pointCloud_clipping.cpp)
target_link_libraries(pointCloud_clipping ${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${OpenCV_LIBS})
# target_link_directories(pointCloud_clipping ${catkin_LIBRARIES} ${PCL_LIBRARIES})