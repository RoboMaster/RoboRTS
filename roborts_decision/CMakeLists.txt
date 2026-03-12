project(roborts_decision)
cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  geometry_msgs
  actionlib
  roborts_common
  roborts_msgs
  roborts_costmap
)

find_package(Eigen3 REQUIRED)
find_package(ProtoBuf REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Generate proto files
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  DecisionProtoSrc
  DecisionProtoHds
  ${ProtoFiles}
)

catkin_package(
  CATKIN_DEPENDS roscpp tf nav_msgs geometry_msgs actionlib
                 roborts_common roborts_msgs roborts_costmap
)

add_executable(behavior_test_node
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  example_behavior/escape_behavior.h
  example_behavior/back_boot_area_behavior.h
  example_behavior/chase_behavior.h
  example_behavior/patrol_behavior.h
  example_behavior/search_behavior.h
  example_behavior/goal_behavior.h
  behavior_test.cpp
  executor/chassis_executor.cpp
  executor/gimbal_executor.cpp
)

target_link_libraries(behavior_test_node
  PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)

add_dependencies(behavior_test_node
  roborts_msgs_generate_messages)
