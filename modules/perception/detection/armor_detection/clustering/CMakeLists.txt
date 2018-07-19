project(clustering)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto ClusteringProtoSrc ClusteringProtoHds ${ProtoFiles})

add_executable(clustering
  ${ClusteringProtoSrc}
  ${ClusteringProtoHds}
  clustering.cpp
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC

  common::main_interface
  common::log
  common::io
  common::rrts
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${catkin_INCLUDE_DIRS}
)
