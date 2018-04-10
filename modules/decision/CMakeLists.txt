project(decision)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto DecisionProtoSrc DecisionProtoHds ${ProtoFiles})

add_executable(behavior_decision_sim_node behavior_tree/behavior_decision_sim_node.cpp
  ${DecisionProtoSrc}
  ${DecisionProtoHds})
target_link_libraries(behavior_decision_sim_node
  PRIVATE
  common::io
  common::log
  ${catkin_LIBRARIES}
)
