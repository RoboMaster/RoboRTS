cmake_minimum_required(VERSION 3.0)
project(local_planner)

add_subdirectory(timed_elastic_band)


set(EXTERNAL_INCLUDE_DIRS
  ${EIGEN3_INCLUDE_DIR}
  ${SUITESPARSE_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
  )
set(EXTERNAL_LIBS
  ${SUITESPARSE_LIBRARIES}
  ${G2O_LIBRARIES}
  )

include_directories(${EXTERNAL_INCLUDE_DIRS} )

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto LocalPlanProtoSrc LocalPlanProtoHds ${ProtoFiles})

add_executable(${PROJECT_NAME}_node
  ${LocalPlanProtoSrc}
  ${LocalPlanProtoHds}
  local_planner_node.cpp
  local_visualization.cpp
  obstacles.cpp
  odom_info.cpp
  robot_position_cost.cpp
  )

add_executable(${PROJECT_NAME}_test
  ${LocalPlanProtoSrc}
  ${LocalPlanProtoHds}
  test_optim_node.cpp
  local_visualization.cpp
  obstacles.cpp
  odom_info.cpp
  robot_position_cost.cpp
  )

target_link_libraries(${PROJECT_NAME}_test
  PRIVATE
  common::log
  common::io
  common::rrts
  perception::map::costmap
  planning::timed_elastic_band
  ${catkin_LIBRARIES}
  ${GLOG_LIBRARY}
  ${EXTERNAL_LIBS}
  )

target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
  common::log
  common::io
  common::rrts
  perception::map::costmap
  planning::timed_elastic_band
  ${catkin_LIBRARIES}
  ${GLOG_LIBRARY}
  ${EXTERNAL_LIBS}
  )

#local_client
add_executable(${PROJECT_NAME}_client
  local_client.cpp
  )

target_link_libraries(${PROJECT_NAME}_client
  PRIVATE
  common::log
  ${catkin_LIBRARIES}
  ${GLOG_LIBRARY}
  )
