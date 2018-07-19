project(self_check)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto SelfCheckProtoSrc SelfCheckProtoHds ${ProtoFiles})

add_executable(self_check_node
  ${SelfCheckProtoSrc}
  ${SelfCheckProtoHds}
  self_check_node.cpp
  self_check_node.h
)
target_include_directories(self_check_node
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(self_check_node
  PRIVATE
  common::rrts
  common::io
  common::log
  driver::camera_param
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${GLOG_LIBRARY}
)

