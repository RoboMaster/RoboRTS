project(global_planner)

add_subdirectory(a_star_planner)
set(CMAKE_BUILD_TYPE Release)
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto GlobalPlannerProtoSrc GlobalPlannerProtoHds ${ProtoFiles})

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
  ${GlobalPlannerProtoSrc}
  ${GlobalPlannerProtoHds}
  global_planner_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
  planning::global_planner::a_star_planner
  roborts_costmap
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GLOG_LIBRARY}
)
add_dependencies(${PROJECT_NAME}_node
  roborts_msgs_generate_messages)

add_executable(${PROJECT_NAME}_test
  global_planner_test.cpp)
target_link_libraries(${PROJECT_NAME}_test
  PRIVATE
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_test
  roborts_msgs_generate_messages)


