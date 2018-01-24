project(decision)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto DecisionProtoSrc DecisionProtoHds ${ProtoFiles})

add_executable(${PROJECT_NAME}_node
  ${DecisionProtoSrc}
  ${DecisionProtoHds}
  decision_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
    common::io
    common::log
    ${catkin_LIBRARIES}
)
