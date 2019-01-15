project(roborts_costmap)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
find_package(Eigen3 REQUIRED)
find_package(ProtoBuf REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  pcl_conversions
  pcl_ros
  roborts_common
  laser_geometry
  sensor_msgs
  geometry_msgs
  )

find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS
  include
)


#proto file
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")

rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  CostmapProtoSrcs
  CostmapProtoHeaders
  ${ProtoFiles}
  )

include_directories(include/costmap proto/)
aux_source_directory(src/. SRC_LIST)
list(REMOVE_ITEM SRC_LIST "src/test_costmap.cpp")

#lib project
add_library(roborts_costmap
  SHARED
  ${SRC_LIST}
  ${CostmapProtoHeaders}
  ${CostmapProtoSrcs}
  )

target_include_directories(roborts_costmap
  PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

target_link_libraries(roborts_costmap
  PUBLIC
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  )

#test project
add_executable(test_costmap src/test_costmap.cpp)

target_include_directories(test_costmap
  PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

target_link_libraries(test_costmap
  roborts_costmap
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  )

list(APPEND catkin_LIBRARIES roborts_costmap)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )


