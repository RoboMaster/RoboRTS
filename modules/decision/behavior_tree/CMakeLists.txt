project(behavior_tree)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto DecisionProtoSrc DecisionProtoHds ${ProtoFiles})

add_library(behavior_tree_config
  SHARED
  ${DecisionProtoSrc}
  ${DecisionProtoHds}
)
target_link_libraries(behavior_tree_config
  PUBLIC
  ${PROTOBUF_LIBRARIES}
)
add_library(behavior_tree INTERFACE)
target_sources(behavior_tree
  INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/behavior_node.h
  ${CMAKE_CURRENT_SOURCE_DIR}/behavior_tree.h
  ${CMAKE_CURRENT_SOURCE_DIR}/blackboard.h)
target_link_libraries(behavior_tree
  INTERFACE
  common::io
  common::log
  behavior_tree_config
  ${catkin_LIBRARIES}
)
