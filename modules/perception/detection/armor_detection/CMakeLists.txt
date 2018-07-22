project(armor_detection)

add_subdirectory(constraint_set)
add_subdirectory(clustering)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto ArmorDetectionProtoSrc ArmorDetectionProtoHds ${ProtoFiles})

add_executable(${PROJECT_NAME}_node
  ${ArmorDetectionProtoSrc}
  ${ArmorDetectionProtoHds}
  armor_detection_base.cpp
  armor_detection_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
  detection::constraint_set
  detection::image_buffer

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
  armor_detection_client.cpp
)

target_link_libraries(${PROJECT_NAME}_client
  PRIVATE
  common::log
  ${catkin_LIBRARIES}
  ${GLOG_LIBRARY}
)
