project(color_detection)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto ColorDetectionProtoSrc ColorDetectionProtoHds ${ProtoFiles})

add_executable(${PROJECT_NAME}_node
  ${ColorDetectionProtoSrc}
  ${ColorDetectionProtoHds}
  color_detection_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
  detection::image_buffer

  driver::camera_param
  common::main_interface
  common::log
  common::io
  common::rrts
  ${catkin_LIBRARIES}
  ${GLOG_LIBRARY}
  ${OpenCV_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}_node
  PRIVATE
  ${OpenCV_INCLUDE_DIRECTORIES}
)
#armor_detection_client_test
add_executable(${PROJECT_NAME}_client
  color_detection_client.cpp
)

target_link_libraries(${PROJECT_NAME}_client
  PRIVATE
  common::log
  ${catkin_LIBRARIES}
  ${GLOG_LIBRARY}
)
