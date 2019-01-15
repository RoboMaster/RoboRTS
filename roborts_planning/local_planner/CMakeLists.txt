cmake_minimum_required(VERSION 3.0)
project(local_planner)


set(EXTERNAL_INCLUDE_DIRS
  ${EIGEN3_INCLUDE_DIR}
  ${SUITESPARSE_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  )
set(EXTERNAL_LIBS
  ${SUITESPARSE_LIBRARIES}
  ${G2O_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  roborts_costmap
  )

include_directories(${EXTERNAL_INCLUDE_DIRS} include/ timed_elastic_band/include/)

add_subdirectory(timed_elastic_band)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/proto
  LocalPlanProtoSrc
  LocalPlanProtoHds
  ${ProtoFiles}
  )

aux_source_directory(src/.  SRC_LIST)

#local planner node
add_executable(${PROJECT_NAME}_node
  ${LocalPlanProtoSrc}
  ${LocalPlanProtoHds}
  src/local_planner_node.cpp
  src/local_visualization.cpp
  src/obstacles.cpp
  src/odom_info.cpp
  src/robot_position_cost.cpp
  )

target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
  planning::timed_elastic_band
  ${EXTERNAL_LIBS}
  )
add_dependencies(${PROJECT_NAME}_node
  roborts_msgs_generate_messages)

#local palnner test
add_executable(teb_test
  ${LocalPlanProtoSrc}
  ${LocalPlanProtoHds}
  src/teb_test.cpp
  src/local_visualization.cpp
  src/obstacles.cpp
  src/odom_info.cpp
  src/robot_position_cost.cpp
  )

target_link_libraries(teb_test
  PRIVATE
  planning::timed_elastic_band
  ${EXTERNAL_LIBS}
  )
add_dependencies(teb_test
  roborts_msgs_generate_messages)

#vel converter
add_executable(vel_converter
  src/vel_converter.cpp
  )

target_link_libraries(vel_converter
  PRIVATE
  ${EXTERNAL_LIBS}
  )
add_dependencies(vel_converter
  roborts_msgs_generate_messages)
