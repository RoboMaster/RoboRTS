project(global_planner)

add_subdirectory(a_star_planner)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto GlobalPlannerProtoSrc GlobalPlannerProtoHds ${ProtoFiles})

add_executable(${PROJECT_NAME}_node
  ${GlobalPlannerProtoSrc}
  ${GlobalPlannerProtoHds}
  global_planner_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
    planning::global_planner::a_star_planner
    perception::map::costmap
    common::main_interface
    common::log
    common::io
    common::rrts
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${GLOG_LIBRARY}
)

add_executable(${PROJECT_NAME}_client
  global_planner_client.cpp)
target_link_libraries(${PROJECT_NAME}_client
  PRIVATE
  common::log
  ${catkin_LIBRARIES}
)



